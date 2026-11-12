use crate::{JointAngles, KinematicsError, Point};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ElbowConfiguration {
    Positive,
    Negative,
}

#[derive(Debug)]
pub struct Scara {
    first_arm_length: f64,
    second_arm_length: f64,
}

#[derive(Debug, Clone, Copy)]
struct NormalizedGeometry {
    // One common scale bounds both arm lengths and all supplied coordinates by 1.
    scale: f64,
    first_arm_length: f64,
    second_arm_length: f64,
}

impl NormalizedGeometry {
    fn normalize_point(&self, point: Point) -> Result<Point, KinematicsError> {
        Point::new(point.x() / self.scale, point.y() / self.scale)
    }

    fn reachable_bounds(&self) -> (f64, f64) {
        // Unlike the original distances, the normalized outer radius is <= 2.
        let outer = self.first_arm_length + self.second_arm_length;
        let inner = (self.first_arm_length - self.second_arm_length).abs();
        let tolerance = Scara::MAX_NUMERICAL_TOLERANCE_FACTOR * f64::EPSILON * outer;
        (inner - tolerance, outer + tolerance)
    }

    fn is_reachable(&self, normalized_point: Point) -> bool {
        let radius = libm::hypot(normalized_point.x(), normalized_point.y());
        let (inner, outer) = self.reachable_bounds();
        (inner..=outer).contains(&radius)
    }
}

impl Scara {
    const MAX_NUMERICAL_TOLERANCE_FACTOR: f64 = 64.0;

    pub fn new(first_arm_length: f64, second_arm_length: f64) -> Result<Self, KinematicsError> {
        utilities::validate_non_zero_finite_positive(first_arm_length)
            .map_err(|_| KinematicsError::InvalidArmLength)?;
        utilities::validate_non_zero_finite_positive(second_arm_length)
            .map_err(|_| KinematicsError::InvalidArmLength)?;

        Ok(Self {
            first_arm_length,
            second_arm_length,
        })
    }

    pub fn forward(&self, angles: JointAngles) -> Result<Point, KinematicsError> {
        let second_arm_absolute_angle = angles.first_theta() + angles.second_theta();

        let x = self.first_arm_length * libm::cos(angles.first_theta())
            + self.second_arm_length * libm::cos(second_arm_absolute_angle);

        let y = self.first_arm_length * libm::sin(angles.first_theta())
            + self.second_arm_length * libm::sin(second_arm_absolute_angle);

        Point::new(x, y).map_err(|_| KinematicsError::NumericalFailure)
    }

    pub fn inverse(
        &self,
        point: Point,
        configuration: ElbowConfiguration,
    ) -> Result<JointAngles, KinematicsError> {
        let normalized = self.normalize_geometry(&[point]);
        let normalized_point = normalized.normalize_point(point)?;

        if !normalized.is_reachable(normalized_point) {
            return Err(KinematicsError::UnreachablePoint);
        }

        let numerator = normalized_point.x() * normalized_point.x()
            + normalized_point.y() * normalized_point.y()
            - normalized.first_arm_length * normalized.first_arm_length
            - normalized.second_arm_length * normalized.second_arm_length;

        let denominator = 2.0 * normalized.first_arm_length * normalized.second_arm_length;

        utilities::validate_non_zero_finite(denominator)
            .map_err(|_| KinematicsError::NumericalFailure)?;

        let cos_theta2 = (numerator / denominator).clamp(-1.0, 1.0);

        let second_theta = match configuration {
            ElbowConfiguration::Positive => libm::acos(cos_theta2),
            ElbowConfiguration::Negative => -libm::acos(cos_theta2),
        };

        let alpha = libm::atan2(point.y(), point.x());
        let beta = libm::atan2(
            normalized.second_arm_length * libm::sin(second_theta),
            normalized.first_arm_length + normalized.second_arm_length * libm::cos(second_theta),
        );

        let first_theta = alpha - beta;

        JointAngles::new(first_theta, second_theta).map_err(|_| KinematicsError::NumericalFailure)
    }

    pub fn is_segment_reachable(
        &self,
        first_point: Point,
        second_point: Point,
    ) -> Result<bool, KinematicsError> {
        let normalized = self.normalize_geometry(&[first_point, second_point]);
        let a = normalized.normalize_point(first_point)?;
        let b = normalized.normalize_point(second_point)?;

        if !normalized.is_reachable(a) || !normalized.is_reachable(b) {
            return Ok(false);
        }

        if first_point == second_point {
            return Ok(true);
        }

        let d = (b - a)?;

        let d_length = d.distance_from_origin()?;

        utilities::validate_non_zero_finite(d_length)
            .map_err(|_| KinematicsError::NumericalFailure)?;

        let u = Point::new(d.x() / d_length, d.y() / d_length)?;

        let h = (-a * u)?;

        let t = (h / d_length).clamp(0.0, 1.0);

        let p = (a + (d.scale(t)?))?;

        let min_r = p.distance_from_origin()?;

        let (inner, _) = normalized.reachable_bounds();
        Ok(min_r >= inner)
    }

    fn normalize_geometry(&self, points: &[Point]) -> NormalizedGeometry {
        // Include absolute coordinates so every normalized component is <= 1
        // in magnitude, even for unreachable points in negative quadrants.
        let scale = points.iter().fold(
            self.first_arm_length.max(self.second_arm_length),
            |scale, point| scale.max(point.x().abs()).max(point.y().abs()),
        );

        NormalizedGeometry {
            scale,
            first_arm_length: self.first_arm_length / scale,
            second_arm_length: self.second_arm_length / scale,
        }
    }
}
