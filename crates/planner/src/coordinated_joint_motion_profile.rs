use kinematics::JointAngles;

use crate::{JointAccelerationLimits, JointVelocityLimits, MotionProfile, PlannerError};

#[derive(Debug)]
pub struct CoordinatedJointMotionProfile {
    start: JointAngles,
    end: JointAngles,

    first_displacement: f64,
    second_displacement: f64,

    progress_profile: MotionProfile,
}

impl CoordinatedJointMotionProfile {
    pub fn new(
        start: JointAngles,
        end: JointAngles,
        velocity_limits: &JointVelocityLimits,
        acceleration_limits: &JointAccelerationLimits,
        initial_progress_speed: f64,
        final_progress_speed: f64,
    ) -> Result<Self, PlannerError> {
        if start.first_theta() == end.first_theta() && start.second_theta() == end.second_theta() {
            return Err(PlannerError::ZeroDisplacementMotion);
        }

        let first_displacement = end.first_theta() - start.first_theta();
        let second_displacement = end.second_theta() - start.second_theta();

        utilities::validate_finite(first_displacement)
            .map_err(|_| PlannerError::NumericalFailure)?;
        utilities::validate_finite(second_displacement)
            .map_err(|_| PlannerError::NumericalFailure)?;

        let (max_velocity, max_acceleration) = MotionProfile::calculate_progress_limits(
            start,
            end,
            velocity_limits,
            acceleration_limits,
        )?;

        let progress = MotionProfile::with_boundary_speeds(
            0.0,
            1.0,
            max_acceleration,
            max_velocity,
            initial_progress_speed,
            final_progress_speed,
        )?;

        Ok(Self {
            start,
            end,
            first_displacement,
            second_displacement,
            progress_profile: progress,
        })
    }

    pub fn position_at(&self, time: f64) -> Result<JointAngles, PlannerError> {
        utilities::validate_finite_positive(time).map_err(|_| PlannerError::InvalidTime)?;

        if time == 0.0 {
            return Ok(self.start);
        }

        if time == self.progress_profile.total_time() {
            return Ok(self.end);
        }

        let progress = self.progress_profile.position_at(time)?;

        let first_theta = self.start.first_theta() + progress * self.first_displacement;
        let second_theta = self.start.second_theta() + progress * self.second_displacement;

        utilities::validate_finite(first_theta).map_err(|_| PlannerError::NumericalFailure)?;
        utilities::validate_finite(second_theta).map_err(|_| PlannerError::NumericalFailure)?;

        JointAngles::new(first_theta, second_theta).map_err(PlannerError::Kinematics)
    }

    pub fn velocity_at(&self, time: f64) -> Result<(f64, f64), PlannerError> {
        utilities::validate_finite_positive(time).map_err(|_| PlannerError::InvalidTime)?;

        let progress = self.progress_profile.velocity_at(time)?;

        let first_velocity = self.first_displacement * progress;
        let second_velocity = self.second_displacement * progress;

        utilities::validate_finite(first_velocity).map_err(|_| PlannerError::NumericalFailure)?;
        utilities::validate_finite(second_velocity).map_err(|_| PlannerError::NumericalFailure)?;

        Ok((first_velocity, second_velocity))
    }

    pub fn acceleration_at(&self, time: f64) -> Result<(f64, f64), PlannerError> {
        utilities::validate_finite_positive(time).map_err(|_| PlannerError::InvalidTime)?;

        let progress = self.progress_profile.acceleration_at(time)?;

        let first_acceleration = self.first_displacement * progress;
        let second_acceleration = self.second_displacement * progress;

        utilities::validate_finite(first_acceleration)
            .map_err(|_| PlannerError::NumericalFailure)?;
        utilities::validate_finite(second_acceleration)
            .map_err(|_| PlannerError::NumericalFailure)?;

        Ok((first_acceleration, second_acceleration))
    }

    pub fn total_time(&self) -> f64 {
        self.progress_profile.total_time()
    }
}
