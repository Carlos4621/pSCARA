use kinematics::JointAngles;

use crate::PlannerError;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotorSteps {
    pub first: i64,
    pub second: i64,
}

#[derive(Debug, Clone, Copy)]
pub struct JointToStepsConverter {
    first_steps_per_radian: f64,
    second_steps_per_radian: f64,
}

impl JointToStepsConverter {
    pub fn new(
        first_steps_per_radian: f64,
        second_steps_per_radian: f64,
    ) -> Result<Self, PlannerError> {
        utilities::validate_non_zero_finite_positive(first_steps_per_radian)
            .map_err(|_| PlannerError::InvalidSteps)?;
        utilities::validate_non_zero_finite_positive(second_steps_per_radian)
            .map_err(|_| PlannerError::InvalidSteps)?;

        Ok(Self {
            first_steps_per_radian,
            second_steps_per_radian,
        })
    }

    pub fn convert(&self, angles: JointAngles) -> Result<MotorSteps, PlannerError> {
        let first = libm::round(angles.first_theta() * self.first_steps_per_radian);
        let second = libm::round(angles.second_theta() * self.second_steps_per_radian);

        if !Self::in_i64_range(first) || !Self::in_i64_range(second) {
            return Err(PlannerError::NumericalFailure);
        }

        Ok(MotorSteps {
            first: first as i64,
            second: second as i64,
        })
    }

    /// Convierte magnitudes de velocidad en rad/s a frecuencias en micropasos/s.
    pub fn peak_step_frequencies(
        &self,
        joint_velocities: (f64, f64),
    ) -> Result<(f64, f64), PlannerError> {
        let (first_velocity, second_velocity) = joint_velocities;
        utilities::validate_finite_positive(first_velocity)
            .map_err(|_| PlannerError::InvalidJointVelocity)?;
        utilities::validate_finite_positive(second_velocity)
            .map_err(|_| PlannerError::InvalidJointVelocity)?;

        let first_frequency = first_velocity.abs() * self.first_steps_per_radian;
        let second_frequency = second_velocity.abs() * self.second_steps_per_radian;
        utilities::validate_finite(first_frequency).map_err(|_| PlannerError::NumericalFailure)?;
        utilities::validate_finite(second_frequency).map_err(|_| PlannerError::NumericalFailure)?;

        Ok((first_frequency, second_frequency))
    }

    fn in_i64_range(value: f64) -> bool {
        value.is_finite() && value >= i64::MIN as f64 && value < i64::MAX as f64
    }
}
