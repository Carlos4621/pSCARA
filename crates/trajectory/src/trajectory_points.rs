use kinematics::Point;
use crate::LineSegment;
use crate::TrajectoryError;

pub struct TrajectoryPoints<'a> {
    segment: &'a LineSegment,
    actual_index: usize,
    max_index: usize,
    is_completed: bool,
}

impl<'a> TrajectoryPoints<'a> {
    pub(crate) fn new(segment: &'a LineSegment, max_index: usize) -> Self {
        Self { segment, actual_index: 0, max_index, is_completed: false }
    }
}

impl Iterator for TrajectoryPoints<'_> {
    type Item = Result<Point, TrajectoryError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.is_completed {
            return None;
        }

        let t = if self.actual_index == self.max_index {
            self.is_completed = true;
            1.0
        } else if self.actual_index == 0 {
            0.0
        }  else {
            (self.actual_index as f64) / (self.max_index as f64)
        };

        self.actual_index = self.actual_index.wrapping_add(1);

        match self.segment.point_at(t) {
            Ok(ok) => Some(Ok(ok)),
            Err(err) => Some(Err(err)),
        }
    }
}