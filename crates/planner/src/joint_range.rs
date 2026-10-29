use core::f64;

use crate::PlannerError;

#[derive(Debug)]
pub struct JointRange {
    min_angle: f64,
    max_angle: f64,
}

impl JointRange {
    pub fn new(min_angle: f64, max_angle: f64) -> Result<Self, PlannerError> {
        let min_angle = Self::normalize_angle(min_angle)?;
        let max_angle = Self::normalize_angle(max_angle)?;

        if min_angle > max_angle {
            return Err(PlannerError::MinGreaterThanMax);
        }

        Ok(Self {
            min_angle,
            max_angle,
        })
    }

    pub fn contains(&self, angle: f64) -> Result<bool, PlannerError> {
        let angle = Self::normalize_angle(angle)?;

        Ok(angle >= self.min_angle && angle <= self.max_angle)
    }

    fn normalize_angle(angle: f64) -> Result<f64, PlannerError> {
        if !angle.is_finite() {
            return Err(PlannerError::AngleNonFinite);
        }

        Ok(libm::remainder(angle, f64::consts::TAU))
    }
}

#[cfg(test)]
#[path = "../tests/unit/joint_range.rs"]
mod tests;
