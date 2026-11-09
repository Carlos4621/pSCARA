use core::ops::{Add, Mul, Sub};

use crate::KinematicsError;

#[derive(Debug, Clone, Copy, PartialEq)]
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

    pub fn is_close_to_point(
        &self,
        other: Point,
        max_tolerance: f64,
    ) -> Result<bool, KinematicsError> {
        if !max_tolerance.is_finite() || max_tolerance < 0.0 {
            return Err(KinematicsError::InvalidTolerance);
        }

        Ok(self.distance_to_point(other)?.abs() <= max_tolerance)
    }

    pub fn x(&self) -> f64 {
        self.x
    }

    pub fn y(&self) -> f64 {
        self.y
    }

    pub fn scale(self, scale: f64) -> Result<Self, KinematicsError> {
        Self::new(self.x * scale, self.y * scale)
    }
}

impl Add for Point {
    type Output = Result<Self, KinematicsError>;

    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl Sub for Point {
    type Output = Result<Self, KinematicsError>;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl Mul for Point {
    type Output = Result<f64, KinematicsError>;

    fn mul(self, rhs: Self) -> Self::Output {
        let result = self.x * rhs.x + self.y * rhs.y;

        if !result.is_finite() {
            return Err(KinematicsError::NumericalFailure);
        }

        Ok(result)
    }
}
