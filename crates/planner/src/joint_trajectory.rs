use kinematics::{
    ElbowConfiguration, JointAngles,
    KinematicsError::{self},
    Scara,
};
use trajectory::{LineSegment, TrajectoryPoints};

use crate::{JointLimits, PlannerError, ValidatedTrajectory};

#[derive(Debug, Clone)]
pub struct JointTrajectory<'a> {
    scara: &'a Scara,
    trajectory_points: TrajectoryPoints<'a>,
    configuration: ElbowConfiguration,
    is_finished: bool,
    previous_angle: Option<JointAngles>,
    limits: Option<&'a JointLimits>,
    max_joint_step: Option<f64>,
    max_point_distance: f64,
    has_started: bool,
}

impl<'a> JointTrajectory<'a> {
    pub fn new(
        scara: &'a Scara,
        line: &'a LineSegment,
        max_point_distance: f64,
        configuration: ElbowConfiguration,
    ) -> Result<Self, PlannerError> {
        let trajectory_points = line
            .points(max_point_distance)
            .map_err(PlannerError::Trajectory)?;

        if !scara
            .is_segment_reachable(
                trajectory_points.get_segment().start,
                trajectory_points.get_segment().end,
            )
            .map_err(PlannerError::Kinematics)?
        {
            return Err(PlannerError::SegmentUnreachable);
        }

        Ok(Self {
            scara,
            trajectory_points,
            configuration,
            is_finished: false,
            previous_angle: None,
            limits: None,
            max_joint_step: None,
            max_point_distance,
            has_started: false,
        })
    }

    pub fn with_limits(mut self, joint_limits: &'a JointLimits) -> Result<Self, PlannerError> {
        if let Some(previous) = self.previous_angle
            && !joint_limits.contains(previous)?
        {
            return Err(PlannerError::AngleOutOfLimits);
        }

        self.limits = Some(joint_limits);
        Ok(self)
    }

    pub fn with_initial_position(
        mut self,
        initial_position: JointAngles,
        max_point_distance_tolerance: f64,
        max_angular_tolerance: f64,
    ) -> Result<Self, PlannerError> {
        let validated_angle = self.validate_limits(initial_position)?;

        let initial_position_point = self
            .scara
            .forward(initial_position)
            .map_err(PlannerError::Kinematics)?;

        if !initial_position_point
            .is_close_to_point(
                self.trajectory_points.get_segment().start,
                max_point_distance_tolerance,
            )
            .map_err(PlannerError::Kinematics)?
        {
            return Err(PlannerError::InitialPositionNotInStart);
        }

        let expected_angle = self
            .scara
            .inverse(
                self.trajectory_points.get_segment().start,
                self.configuration,
            )
            .map_err(PlannerError::Kinematics)?;

        let aligned = Self::unwrap_joint_angles(initial_position, expected_angle)?;

        if !aligned
            .is_close_to_joint(initial_position, max_angular_tolerance)
            .map_err(PlannerError::Kinematics)?
        {
            return Err(PlannerError::InitialConfigurationMismatch);
        }

        self.previous_angle = Some(validated_angle);

        Ok(self)
    }

    pub fn with_max_joint_step(mut self, max_joint_step: f64) -> Result<Self, PlannerError> {
        if !max_joint_step.is_finite() || max_joint_step <= 0.0 {
            return Err(PlannerError::InvalidMaxJointStep);
        }

        self.max_joint_step = Some(max_joint_step);

        Ok(self)
    }

    pub fn validate(self) -> Result<(), PlannerError> {
        if self.has_started {
            return Err(PlannerError::TrajectoryAlreadyStarted);
        }

        for n in self {
            n?;
        }

        Ok(())
    }

    pub fn into_validated(self) -> Result<ValidatedTrajectory<'a>, PlannerError> {
        self.clone().validate()?;

        Ok(ValidatedTrajectory::new(self))
    }

    pub fn into_validated_with_refinement(
        mut self,
        max_refinements: usize,
    ) -> Result<ValidatedTrajectory<'a>, PlannerError> {
        if self.has_started {
            return Err(PlannerError::TrajectoryAlreadyStarted);
        }

        for refinement in 0..=max_refinements {
            match self.clone().validate() {
                Ok(_) => return Ok(ValidatedTrajectory::new(self)),
                Err(PlannerError::MaxJointStepExceeded) => {}
                Err(error) => return Err(error),
            }

            if refinement == max_refinements {
                break;
            }

            let next_point_distance = self.max_point_distance / 2.0;

            if !next_point_distance.is_finite()
                || next_point_distance <= 0.0
                || next_point_distance > self.max_point_distance
            {
                return Err(PlannerError::Kinematics(KinematicsError::NumericalFailure));
            }

            self.max_point_distance = next_point_distance;

            self.trajectory_points = self
                .trajectory_points
                .get_segment()
                .points(self.max_point_distance)
                .map_err(PlannerError::Trajectory)?;
        }

        Err(PlannerError::MaxRefinementAttemptsReached)
    }

    fn unwrap_angle(previous: f64, next: f64) -> Result<f64, PlannerError> {
        if !previous.is_finite() || !next.is_finite() {
            return Err(PlannerError::AngleNonFinite);
        }

        let result = previous + libm::atan2(libm::sin(next - previous), libm::cos(next - previous));

        if !result.is_finite() {
            return Err(PlannerError::AngleNonFinite);
        }

        Ok(result)
    }

    fn unwrap_joint_angles(
        previous: JointAngles,
        next: JointAngles,
    ) -> Result<JointAngles, PlannerError> {
        JointAngles::new(
            Self::unwrap_angle(previous.first_theta(), next.first_theta())?,
            Self::unwrap_angle(previous.second_theta(), next.second_theta())?,
        )
        .map_err(PlannerError::Kinematics)
    }

    fn validate_limits(&self, angle: JointAngles) -> Result<JointAngles, PlannerError> {
        let in_range = match self.limits {
            Some(limits) => limits.contains(angle),
            None => return Ok(angle),
        }?;

        if !in_range {
            return Err(PlannerError::AngleOutOfLimits);
        }

        Ok(angle)
    }

    fn validate_max_joint_step(&self, next: JointAngles) -> Result<JointAngles, PlannerError> {
        let Some(previous) = self.previous_angle else {
            return Ok(next);
        };

        let Some(joint_step) = self.max_joint_step else {
            return Ok(next);
        };

        if !previous
            .is_close_to_joint(next, joint_step)
            .map_err(PlannerError::Kinematics)?
        {
            return Err(PlannerError::MaxJointStepExceeded);
        }

        Ok(next)
    }
}

impl<'a> Iterator for JointTrajectory<'a> {
    type Item = Result<JointAngles, PlannerError>;

    fn next(&mut self) -> Option<Self::Item> {
        if !self.has_started {
            self.has_started = true;
        }

        if self.is_finished {
            return None;
        }

        let Some(point) = self.trajectory_points.next() else {
            self.is_finished = true;
            return None;
        };

        let result = point.map_err(PlannerError::Trajectory).and_then(|point| {
            self.scara
                .inverse(point, self.configuration)
                .map_err(PlannerError::Kinematics)
                .and_then(|next| match self.previous_angle {
                    Some(previous) => Self::unwrap_joint_angles(previous, next),
                    None => Ok(next),
                })
                .and_then(|angle| self.validate_max_joint_step(angle))
                .and_then(|angle| self.validate_limits(angle))
        });

        match result {
            Ok(ok) => self.previous_angle = Some(ok),
            Err(_) => self.is_finished = true,
        }

        Some(result)
    }
}

#[cfg(test)]
#[path = "../tests/unit/joint_trajectory.rs"]
mod tests;
