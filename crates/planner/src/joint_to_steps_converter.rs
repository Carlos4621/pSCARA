use kinematics::JointAngles;

use crate::PlannerError;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotorSteps {
    pub first: i64,
    pub second: i64,
}

#[derive(Debug)]
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

    fn in_i64_range(value: f64) -> bool {
        value.is_finite() && value >= i64::MIN as f64 && value < i64::MAX as f64
    }
}
