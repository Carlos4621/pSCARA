use core::f64;

use crate::PlannerError;

#[derive(Debug)]
pub struct JointRange {
    min_angle: f64,
    max_angle: f64,
}

impl JointRange {
    pub fn new(min_angle: f64, max_angle: f64) -> Result<Self, PlannerError> {
        if !min_angle.is_finite() || !max_angle.is_finite() {
            return Err(PlannerError::AngleNonFinite);
        }

        if min_angle > max_angle {
            return Err(PlannerError::MinGreaterThanMax);
        }

        Ok(Self {
            min_angle,
            max_angle,
        })
    }

    pub fn contains(&self, angle: f64) -> Result<bool, PlannerError> {
        if !angle.is_finite() {
            return Err(PlannerError::AngleNonFinite);
        }

        Ok((self.min_angle..=self.max_angle).contains(&angle))
    }
}

#[cfg(test)]
#[path = "../tests/unit/joint_range.rs"]
mod tests;
