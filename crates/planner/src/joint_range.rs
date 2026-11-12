use crate::PlannerError;

#[derive(Debug, Clone, Copy)]
pub struct JointRange {
    min_angle: f64,
    max_angle: f64,
}

impl JointRange {
    pub fn new(min_angle: f64, max_angle: f64) -> Result<Self, PlannerError> {
        utilities::validate_finite(min_angle).map_err(|_| PlannerError::AngleNonFinite)?;
        utilities::validate_finite(max_angle).map_err(|_| PlannerError::AngleNonFinite)?;

        if min_angle > max_angle {
            return Err(PlannerError::MinGreaterThanMax);
        }

        Ok(Self {
            min_angle,
            max_angle,
        })
    }

    pub fn contains(&self, angle: f64) -> Result<bool, PlannerError> {
        utilities::validate_finite(angle).map_err(|_| PlannerError::AngleNonFinite)?;

        Ok((self.min_angle..=self.max_angle).contains(&angle))
    }
}

#[cfg(test)]
#[path = "../tests/unit/joint_range.rs"]
mod tests;
