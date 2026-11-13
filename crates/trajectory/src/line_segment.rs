pub use kinematics::Point;

use crate::{TrajectoryError, TrajectoryPoints};

#[derive(Debug, Clone, Copy)]
pub struct LineSegment {
    start: Point,
    end: Point,
    length: f64,
}

impl LineSegment {
    pub const MAX_SEGMENTS: usize = usize::MAX;

    pub fn new(start: Point, end: Point) -> Result<Self, TrajectoryError> {
        let length = start
            .distance_to_point(end)
            .map_err(|_| TrajectoryError::SegmentNonFinite)?;

        Ok(Self { start, end, length })
    }

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

    pub fn length(&self) -> f64 {
        self.length
    }

    pub fn start(&self) -> Point {
        self.start
    }

    pub fn end(&self) -> Point {
        self.end
    }

    pub fn points(&self, max_point_distance: f64) -> Result<TrajectoryPoints<'_>, TrajectoryError> {
        utilities::validate_non_zero_finite_positive(max_point_distance)
            .map_err(|_| TrajectoryError::InvalidMaxPointDistance)?;

        let mut segments = libm::ceil(self.length / max_point_distance).max(1.0);

        Self::validate_segments_number(segments)?;

        // Corregir un posible redondeo hacia abajo.
        if segments * max_point_distance < self.length {
            segments += 1.0;
        }

        Self::validate_segments_number(segments)?;

        Ok(TrajectoryPoints::new(
            self,
            segments as usize,
            max_point_distance,
        ))
    }

    fn validate_t(t: f64) -> Result<(), TrajectoryError> {
        utilities::validate_finite(t).map_err(|_| TrajectoryError::TOutOfBounds)?;

        if !(0.0..=1.0).contains(&t) {
            return Err(TrajectoryError::TOutOfBounds);
        }

        Ok(())
    }

    fn validate_segments_number(segments: f64) -> Result<(), TrajectoryError> {
        utilities::validate_finite_positive(segments)
            .map_err(|_| TrajectoryError::InvalidSegmentNumber)?;

        if segments >= Self::MAX_SEGMENTS as f64 {
            return Err(TrajectoryError::TooManySegments);
        }

        Ok(())
    }
}
