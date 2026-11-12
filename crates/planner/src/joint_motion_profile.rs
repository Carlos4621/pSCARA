use kinematics::JointAngles;

use crate::{JointAccelerationLimits, JointVelocityLimits, MotionProfile, PlannerError};

#[derive(Debug)]
pub struct JointMotionProfile {
    first_profile: MotionProfile,
    second_profile: MotionProfile,
    total_time: f64,
}

impl JointMotionProfile {
    pub fn new(
        start_angle: JointAngles,
        end_angle: JointAngles,
        velocity_limits: &JointVelocityLimits,
        acceleration_limits: &JointAccelerationLimits,
    ) -> Result<Self, PlannerError> {
        let first_profile = MotionProfile::new(
            start_angle.first_theta(),
            end_angle.first_theta(),
            acceleration_limits.first_joint_max_acceleration(),
            velocity_limits.first_joint_max_velocity(),
        )?;

        let second_profile = MotionProfile::new(
            start_angle.second_theta(),
            end_angle.second_theta(),
            acceleration_limits.second_joint_max_acceleration(),
            velocity_limits.second_joint_max_velocity(),
        )?;

        let total_time = first_profile.total_time().max(second_profile.total_time());

        Ok(Self {
            first_profile,
            second_profile,
            total_time,
        })
    }

    pub fn position_at(&self, time: f64) -> Result<JointAngles, PlannerError> {
        JointAngles::new(
            self.first_profile
                .position_at_scaled(time, self.total_time)?,
            self.second_profile
                .position_at_scaled(time, self.total_time)?,
        )
        .map_err(PlannerError::Kinematics)
    }

    pub fn velocity_at(&self, time: f64) -> Result<(f64, f64), PlannerError> {
        Ok((
            self.first_profile
                .velocity_at_scaled(time, self.total_time)?,
            self.second_profile
                .velocity_at_scaled(time, self.total_time)?,
        ))
    }

    pub fn acceleration_at(&self, time: f64) -> Result<(f64, f64), PlannerError> {
        Ok((
            self.first_profile
                .acceleration_at_scaled(time, self.total_time)?,
            self.second_profile
                .acceleration_at_scaled(time, self.total_time)?,
        ))
    }

    pub fn total_time(&self) -> f64 {
        self.total_time
    }
}
