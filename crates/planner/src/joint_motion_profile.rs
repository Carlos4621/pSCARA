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
        if !time.is_finite() || time > self.total_time || time < 0.0 {
            return Err(PlannerError::InvalidTime);
        }

        // Garantizamos final exacto
        if time == self.total_time {
            return JointAngles::new(
                self.first_profile
                    .position_at(self.first_profile.total_time())?,
                self.second_profile
                    .position_at(self.second_profile.total_time())?,
            )
            .map_err(PlannerError::Kinematics);
        }

        let progress = time / self.total_time;

        let first_time = progress * self.first_profile.total_time();
        let second_time = progress * self.second_profile.total_time();

        let first_theta = self.first_profile.position_at(first_time)?;
        let second_theta = self.second_profile.position_at(second_time)?;

        JointAngles::new(first_theta, second_theta).map_err(PlannerError::Kinematics)
    }

    pub fn total_time(&self) -> f64 {
        self.total_time
    }
}
