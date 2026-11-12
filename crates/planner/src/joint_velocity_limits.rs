use kinematics::JointAngles;

use crate::PlannerError;

#[derive(Debug, Clone, Copy)]
pub struct JointVelocityLimits {
    first_joint_max_velocity: f64,
    second_joint_max_velocity: f64,
}

impl JointVelocityLimits {
    pub fn new(
        first_joint_max_velocity: f64,
        second_joint_max_velocity: f64,
    ) -> Result<Self, PlannerError> {
        utilities::validate_non_zero_finite_positive(first_joint_max_velocity)
            .map_err(|_| PlannerError::InvalidJointVelocity)?;
        utilities::validate_non_zero_finite_positive(second_joint_max_velocity)
            .map_err(|_| PlannerError::InvalidJointVelocity)?;

        Ok(Self {
            first_joint_max_velocity,
            second_joint_max_velocity,
        })
    }

    pub fn min_duration_between(
        &self,
        start: JointAngles,
        end: JointAngles,
    ) -> Result<f64, PlannerError> {
        let first_delta = (end.first_theta() - start.first_theta()).abs();
        let second_delta = (end.second_theta() - start.second_theta()).abs();

        if first_delta == 0.0 && second_delta == 0.0 {
            return Ok(0.0); // El único caso válido para cero
        }

        utilities::validate_finite(first_delta).map_err(|_| PlannerError::NumericalFailure)?;
        utilities::validate_finite(second_delta).map_err(|_| PlannerError::NumericalFailure)?;

        let first_joint_time = first_delta / self.first_joint_max_velocity;
        let second_joint_time = second_delta / self.second_joint_max_velocity;
        let max_time = first_joint_time.max(second_joint_time);

        utilities::validate_non_zero_finite_positive(max_time)
            .map_err(|_| PlannerError::NumericalFailure)?;

        Ok(max_time)
    }

    pub fn first_joint_max_velocity(&self) -> f64 {
        self.first_joint_max_velocity
    }

    pub fn second_joint_max_velocity(&self) -> f64 {
        self.second_joint_max_velocity
    }
}
