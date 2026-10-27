#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrajectoryError {
    TOutOfBounds,
    PointOutOfSegment,
    SegmentNonFinite,
    InvalidMaxPointDitance,
    TooManySegments,
}
