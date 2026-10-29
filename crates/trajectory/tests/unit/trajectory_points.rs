use super::*;

#[test]
fn next_stops_permanently_after_a_point_calculation_error() {
    let segment = LineSegment {
        start: Point::new(-f64::MAX, 0.0).unwrap(),
        end: Point::new(f64::MAX, 0.0).unwrap(),
    };
    let mut points = TrajectoryPoints::new(&segment, 2);

    assert!(matches!(points.next(), Some(Ok(point)) if point.x() == -f64::MAX && point.y() == 0.0));
    assert!(matches!(
        points.next(),
        Some(Err(TrajectoryError::SegmentNonFinite))
    ));
    assert!(points.next().is_none());
    assert!(points.next().is_none());
}
