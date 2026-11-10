use kinematics::JointAngles;

use crate::{JointVelocityLimits, PlannerError};

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
        if !first_joint_max_acceleration.is_finite()
            || !second_joint_max_acceleration.is_finite()
            || first_joint_max_acceleration <= 0.0
            || second_joint_max_acceleration <= 0.0
        {
            return Err(PlannerError::InvalidJointAcceleration);
        }

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

        if !max_duration.is_finite() {
            return Err(PlannerError::NumericalFailure);
        }

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
        let delta_theta = (end - start).abs();

        if delta_theta == 0.0 {
            return Ok(delta_theta);
        }

        let critic_delta = max_velocity * max_velocity / max_acceleration;

        if delta_theta < critic_delta {
            Ok(libm::sqrt(delta_theta / max_acceleration) * 2.0)
        } else {
            Ok(delta_theta / max_velocity + max_velocity / max_acceleration)
        }
    }
}
