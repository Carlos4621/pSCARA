use crate::KinematicsError;

#[derive(Debug, Clone, Copy)]
pub struct JointAngles {
    first_theta: f64,
    second_theta: f64,
}

impl JointAngles {
    pub fn new(first_theta: f64, second_theta: f64) -> Result<Self, KinematicsError> {
        if !first_theta.is_finite() || !second_theta.is_finite() {
            return Err(KinematicsError::InvalidAngle);
        }

        Ok(Self {
            first_theta,
            second_theta,
        })
    }

    pub fn first_theta(&self) -> f64 {
        self.first_theta
    }

    pub fn second_theta(&self) -> f64 {
        self.second_theta
    }
}
