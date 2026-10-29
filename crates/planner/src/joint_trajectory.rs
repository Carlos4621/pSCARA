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
        })
    }

    pub fn with_limits(mut self, joint_limits: &'a JointLimits) -> Self {
        self.limits = Some(joint_limits);
        self
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
        Ok(JointAngles {
            first_theta: Self::unwrap_angle(previous.first_theta, next.first_theta)?,
            second_theta: Self::unwrap_angle(previous.second_theta, next.second_theta)?,
        })
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
