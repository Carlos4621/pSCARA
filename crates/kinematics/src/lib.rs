#![no_std]

mod point;
pub use point::Point;

#[derive(Debug, Clone, Copy)]
pub struct JointAngles {
    pub first_theta: f64,
    pub second_theta: f64,
}

#[derive(Debug)]
pub enum KinematicsError {
    UnreachablePoint,
    ArmWithInvalidLength,
}

#[derive(Debug)]
pub enum ElbowConfiguration {
    Positive,
    Negative,
}

#[derive(Debug)]
struct NormalizedGeometry {
    point: Point,
    first_arm_length: f64,
    second_arm_length: f64,
}

pub struct Scara {
    first_arm_length: f64,
    second_arm_length: f64,
}

impl Scara {
    const MAX_NUMERICAL_TOLERANCE_FACTOR : f64 = 64.0;

    pub fn new(first_arm_length: f64, second_arm_length: f64) -> Result<Self, KinematicsError> {
        if !Self::is_valid_arm_length(first_arm_length)
            || !Self::is_valid_arm_length(second_arm_length)
        {
            return Err(KinematicsError::ArmWithInvalidLength);
        }

        Ok(Self {
            first_arm_length,
            second_arm_length,
        })
    }

    pub fn forward(&self, angles: JointAngles) -> Point {
        let second_arm_absolute_angle = angles.first_theta + angles.second_theta;

        let x = self.first_arm_length * libm::cos(angles.first_theta)
            + self.second_arm_length * libm::cos(second_arm_absolute_angle);

        let y = self.first_arm_length * libm::sin(angles.first_theta)
            + self.second_arm_length * libm::sin(second_arm_absolute_angle);

        Point { x, y }
    }

    pub fn inverse(&self, point: Point, configuration : ElbowConfiguration) -> Result<JointAngles, KinematicsError> {
        if !self.is_reachable_point(point) {
            return Err(KinematicsError::UnreachablePoint);
        }

        let normalized = Self::normalize_geometry(point, self.first_arm_length, self.second_arm_length);

        let numerator = normalized.point.x * normalized.point.x + normalized.point.y * normalized.point.y
            - normalized.first_arm_length * normalized.first_arm_length
            - normalized.second_arm_length * normalized.second_arm_length;

        let denominator = 2.0 * normalized.first_arm_length * normalized.second_arm_length;

        let cos_theta2 = (numerator / denominator).clamp(-1.0, 1.0);

        let second_theta = match configuration {
            ElbowConfiguration::Positive => libm::acos(cos_theta2),
            ElbowConfiguration::Negative => -libm::acos(cos_theta2),
        };

        let alpha = libm::atan2(point.y, point.x);
        let beta = libm::atan2(
            normalized.second_arm_length * libm::sin(second_theta),
            normalized.first_arm_length + normalized.second_arm_length * libm::cos(second_theta),
        );

        let first_theta = alpha - beta;

        Ok(JointAngles {
            first_theta,
            second_theta,
        })
    }

    const fn is_valid_arm_length(length: f64) -> bool {
        length.is_finite() && length > 0.0
    }

    fn is_reachable_point(&self, point: Point) -> bool {
        let euclides_distance = point.distance_from_origin();

        let boundary_tolerance = self.max_tolerance_distance();

        euclides_distance.is_finite()
            && (euclides_distance <= self.max_reachable_distance() + boundary_tolerance)
            && (euclides_distance >= self.min_reachable_distance() - boundary_tolerance)
    }

    const fn max_reachable_distance(&self) -> f64 {
        self.first_arm_length + self.second_arm_length
    }

    const fn min_reachable_distance(&self) -> f64 {
        (self.first_arm_length - self.second_arm_length).abs()
    }

    fn max_tolerance_distance(&self) -> f64 {
        Self::MAX_NUMERICAL_TOLERANCE_FACTOR * f64::EPSILON * self.max_reachable_distance()
    }

    fn normalize_geometry(point: Point, first_arm_length: f64, second_arm_length: f64) -> NormalizedGeometry {
        let normalize_factor = first_arm_length.max(second_arm_length).max(point.x.abs()).max(point.y.abs());

        NormalizedGeometry {
            point: Point { x: point.x / normalize_factor, y: point.y / normalize_factor },
            first_arm_length: first_arm_length / normalize_factor,
            second_arm_length: second_arm_length / normalize_factor,
        }
    }
}
