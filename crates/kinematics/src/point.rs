use crate::KinematicsError;

#[derive(Debug, Clone, Copy)]
pub struct Point {
    x: f64,
    y: f64,
}

impl Point {
    pub fn new(x: f64, y: f64) -> Result<Self, KinematicsError> {
        if !x.is_finite() || !y.is_finite() {
            return Err(KinematicsError::InvalidPoint);
        }

        Ok(Self { x, y })
    }

    pub fn distance_from_origin(&self) -> f64 {
        libm::hypot(self.x, self.y)
    }

    pub fn distance_to_point(&self, other: Point) -> Result<f64, KinematicsError> {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let distance = libm::hypot(dx, dy);

        if !distance.is_finite() {
            return Err(KinematicsError::NumericalFailure);
        }

        Ok(distance)
    }

    pub fn x(&self) -> f64 {
        self.x
    }

    pub fn y(&self) -> f64 {
        self.y
    }
}
