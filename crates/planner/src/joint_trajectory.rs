use kinematics::{ElbowConfiguration, JointAngles, Scara};
use trajectory::{LineSegment, TrajectoryPoints};

use crate::{JointLimits, PlannerError};

#[derive(Debug)]
pub struct JointTrajectory<'a> {
    scara: &'a Scara,
    trajectory_points: TrajectoryPoints<'a>,
    configuration: ElbowConfiguration,
    is_finished: bool,
    previous_angle: Option<JointAngles>,
    limits: Option<&'a JointLimits>,
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

        Ok(Self {
            scara,
            trajectory_points,
            configuration,
            is_finished: false,
            previous_angle: None,
            limits: None,
            has_started: false,
        })
    }

    pub fn with_limits(mut self, joint_limits: &'a JointLimits) -> Result<Self, PlannerError> {
        if let Some(previus) = self.previous_angle
            && !joint_limits.contains(previus)?
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

    pub fn validate(self) -> Result<(), PlannerError> {
        if self.has_started {
            return Err(PlannerError::TrajectoryAlreadyStarted);
        }

        for n in self {
            n?;
        }

        Ok(())
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
