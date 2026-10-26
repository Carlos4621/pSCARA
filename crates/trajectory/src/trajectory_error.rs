pub enum TrajectoryError {
    TOutOfBounds,
    PointOutOfSegment,
    SegmentNonFinite,
    InvalidMaxPointDitance,
    MaxPointDistanteTooBig
}