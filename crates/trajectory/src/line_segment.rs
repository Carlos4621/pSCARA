pub use kinematics::Point;

use crate::{TrajectoryError, TrajectoryPoints};

#[derive(Debug, Clone, Copy)]
pub struct LineSegment {
    pub start: Point,
    pub end: Point,
}

impl LineSegment {
    pub const MAX_SEGMENTS: usize = usize::MAX;

    pub fn point_at(&self, t: f64) -> Result<Point, TrajectoryError> {
        Self::validate_t(t)?;

        if t == 0.0 {
            return Ok(self.start);
        }

        if t == 1.0 {
            return Ok(self.end);
        }

        let x = self.start.x() + (self.end.x() - self.start.x()) * t;
        let y = self.start.y() + (self.end.y() - self.start.y()) * t;

        Point::new(x, y).map_err(|_| TrajectoryError::SegmentNonFinite)
    }

    pub fn length(&self) -> Result<f64, TrajectoryError> {
        self.start.distance_to_point(self.end).map_err(|_| TrajectoryError::SegmentNonFinite)
    }

    pub fn points(&self, max_point_distance: f64) -> Result<TrajectoryPoints<'_>, TrajectoryError> {
        if !max_point_distance.is_finite() || max_point_distance <= 0.0 {
            return Err(TrajectoryError::InvalidMaxPointDitance);
        }

        let length = self.length()?;

        let mut segments = libm::ceil(length / max_point_distance).max(1.0);

        Self::validate_segments_number(segments)?;

        // Corregir un posible redondeo hacia abajo.
        if segments * max_point_distance < length {
            segments += 1.0;
        }

        Self::validate_segments_number(segments)?;

        Ok(TrajectoryPoints::new(self, segments as usize))
    }

    fn validate_t(t: f64) -> Result<(), TrajectoryError> {
        if !t.is_finite() || !(0.0..=1.0).contains(&t) {
            return Err(TrajectoryError::TOutOfBounds);
        }

        Ok(())
    }

    fn validate_segments_number(segments: f64) -> Result<(), TrajectoryError> {
        if !segments.is_finite() || segments >= Self::MAX_SEGMENTS as f64 {
            return Err(TrajectoryError::TooManySegments);
        }

        Ok(())
    }
}
