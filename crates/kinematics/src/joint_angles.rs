use crate::{KinematicsError, Point};

#[derive(Debug, Clone, Copy)]
pub struct JointAngles {
    angles: Point,
}

impl JointAngles {
    pub fn new(first_theta: f64, second_theta: f64) -> Result<Self, KinematicsError> {
        Ok(Self {
            angles: Point::new(first_theta, second_theta)?,
        })
    }

    pub fn first_theta(&self) -> f64 {
        self.angles.x()
    }

    pub fn second_theta(&self) -> f64 {
        self.angles.y()
    }

    pub fn is_close_to_joint(
        &self,
        other: JointAngles,
        max_tolerance: f64,
    ) -> Result<bool, KinematicsError> {
        utilities::validate_finite_positive(max_tolerance)
            .map_err(|_| KinematicsError::InvalidTolerance)?;

        Ok(
            (self.first_theta() - other.first_theta()).abs() <= max_tolerance
                && (self.second_theta() - other.second_theta()).abs() <= max_tolerance,
        )
    }
}
