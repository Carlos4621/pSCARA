pub use kinematics::Point;

use crate::TrajectoryError;

pub struct LineSegment {
    pub start : Point,
    pub end : Point,
}

impl LineSegment {
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
            return Err(TrajectoryError::PointOutOfSegment)
        }

        Ok(Point { x, y })
    }

    fn validate_t(t: f64) -> Result<f64, TrajectoryError> {
        if !t.is_finite() || !(0.0..=1.0).contains(&t) {
            return Err(TrajectoryError::TOutOfBounds);
        }

        Ok(t)
    }
}