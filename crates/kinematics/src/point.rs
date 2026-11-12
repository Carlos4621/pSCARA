use core::ops::{Add, Mul, Neg, Sub};

use crate::KinematicsError;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    x: f64,
    y: f64,
}

impl Point {
    pub fn new(x: f64, y: f64) -> Result<Self, KinematicsError> {
        utilities::validate_finite(x).map_err(|_| KinematicsError::InvalidPoint)?;
        utilities::validate_finite(y).map_err(|_| KinematicsError::InvalidPoint)?;

        Ok(Self { x, y })
    }

    pub fn distance_from_origin(&self) -> Result<f64, KinematicsError> {
        let result = libm::hypot(self.x, self.y);

        utilities::validate_finite_positive(result)
            .map_err(|_| KinematicsError::NumericalFailure)?;

        Ok(result)
    }

    pub fn distance_to_point(&self, other: Point) -> Result<f64, KinematicsError> {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let distance = libm::hypot(dx, dy);

        utilities::validate_finite_positive(distance)
            .map_err(|_| KinematicsError::NumericalFailure)?;

        Ok(distance)
    }

    pub fn is_close_to_point(
        &self,
        other: Point,
        max_tolerance: f64,
    ) -> Result<bool, KinematicsError> {
        utilities::validate_finite_positive(max_tolerance)
            .map_err(|_| KinematicsError::InvalidTolerance)?;

        Ok(self.distance_to_point(other)? <= max_tolerance)
    }

    pub fn x(&self) -> f64 {
        self.x
    }

    pub fn y(&self) -> f64 {
        self.y
    }

    pub fn scale(self, scale: f64) -> Result<Self, KinematicsError> {
        utilities::validate_finite(scale).map_err(|_| KinematicsError::InvalidScale)?;

        Self::new(self.x * scale, self.y * scale).map_err(|_| KinematicsError::NumericalFailure)
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

        utilities::validate_finite(result).map_err(|_| KinematicsError::NumericalFailure)?;

        Ok(result)
    }
}

impl Neg for Point {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self {
            x: -self.x,
            y: -self.y,
        }
    }
}
