pub use kinematics::Point;

use crate::{TrajectoryError, TrajectoryPoints};

#[derive(Debug)]
pub struct LineSegment {
    pub start: Point,
    pub end: Point,
}

impl LineSegment {
    const MAX_SEGMENTS: usize = usize::MAX;

    pub fn point_at(&self, t: f64) -> Result<Point, TrajectoryError> {
        Self::validate_t(t)?;

        if t == 0.0 {
            return Ok(self.start);
        }

        if t == 1.0 {
            return Ok(self.end);
        }

        let x = self.start.x + (self.end.x - self.start.x) * t;
        let y = self.start.y + (self.end.y - self.start.y) * t;

        if !x.is_finite() || !y.is_finite() {
            return Err(TrajectoryError::PointOutOfSegment);
        }

        Ok(Point { x, y })
    }

    pub fn length(&self) -> Result<f64, TrajectoryError> {
        let dx = self.start.x - self.end.x;

        let dy = self.start.y - self.end.y;

        let distance = libm::hypot(dx, dy);

        if !distance.is_finite() {
            return Err(TrajectoryError::SegmentNonFinite);
        }

        Ok(distance)
    }

    pub fn points(&self, max_point_distance: f64) -> Result<TrajectoryPoints<'_>, TrajectoryError> {
        if !max_point_distance.is_finite() || max_point_distance <= 0.0 {
            return Err(TrajectoryError::InvalidMaxPointDitance);
        }

        let length = self.length()?;

        let mut segments = libm::ceil(length / max_point_distance).max(1.0);

        if !segments.is_finite() || segments >= Self::MAX_SEGMENTS as f64 {
            return Err(TrajectoryError::TooManySegments);
        }

        // Corregir un posible redondeo hacia abajo.
        if segments * max_point_distance < length {
            segments += 1.0;
        }

        if !segments.is_finite() || segments >= Self::MAX_SEGMENTS as f64 {
            return Err(TrajectoryError::TooManySegments);
        }

        Ok(TrajectoryPoints::new(self, segments as usize))
    }

    fn validate_t(t: f64) -> Result<f64, TrajectoryError> {
        if !t.is_finite() || !(0.0..=1.0).contains(&t) {
            return Err(TrajectoryError::TOutOfBounds);
        }

        Ok(t)
    }
}
