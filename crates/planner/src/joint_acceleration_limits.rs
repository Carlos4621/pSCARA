use crate::{JointVelocityLimits, MotionProfile, PlannerError};
use kinematics::JointAngles;

#[derive(Debug)]
pub struct JointAccelerationLimits {
    first_joint_max_acceleration: f64,
    second_joint_max_acceleration: f64,
}

impl JointAccelerationLimits {
    pub fn new(
        first_joint_max_acceleration: f64,
        second_joint_max_acceleration: f64,
    ) -> Result<Self, PlannerError> {
        utilities::validate_non_zero_finite_positive(first_joint_max_acceleration)
            .map_err(|_| PlannerError::InvalidJointAcceleration)?;
        utilities::validate_non_zero_finite_positive(second_joint_max_acceleration)
            .map_err(|_| PlannerError::InvalidJointAcceleration)?;

        Ok(Self {
            first_joint_max_acceleration,
            second_joint_max_acceleration,
        })
    }

    pub fn min_duration_between(
        &self,
        start: JointAngles,
        end: JointAngles,
        velocity_limits: &JointVelocityLimits,
    ) -> Result<f64, PlannerError> {
        let max_duration = Self::duration_between_angles(
            start.first_theta(),
            end.first_theta(),
            velocity_limits.first_joint_max_velocity(),
            self.first_joint_max_acceleration,
        )?
        .max(Self::duration_between_angles(
            start.second_theta(),
            end.second_theta(),
            velocity_limits.second_joint_max_velocity(),
            self.second_joint_max_acceleration,
        )?);

        utilities::validate_finite_positive(max_duration)
            .map_err(|_| PlannerError::NumericalFailure)?;

        Ok(max_duration)
    }

    pub fn first_joint_max_acceleration(&self) -> f64 {
        self.first_joint_max_acceleration
    }

    pub fn second_joint_max_acceleration(&self) -> f64 {
        self.second_joint_max_acceleration
    }

    fn duration_between_angles(
        start: f64,
        end: f64,
        max_velocity: f64,
        max_acceleration: f64,
    ) -> Result<f64, PlannerError> {
        Ok(MotionProfile::new(start, end, max_acceleration, max_velocity)?.total_time())
    }
}
