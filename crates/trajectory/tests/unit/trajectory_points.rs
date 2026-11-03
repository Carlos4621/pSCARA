use super::*;

#[test]
fn constructor_with_zero_max_index_yields_only_the_end() {
    let segment = LineSegment {
        start: Point::new(-8.0, 4.0).unwrap(),
        end: Point::new(8.0, -4.0).unwrap(),
    };
    let mut points = TrajectoryPoints::new(&segment, 0);

    assert!(matches!(
        points.next(),
        Some(Ok(point))
            if point.x() == segment.end.x() && point.y() == segment.end.y()
    ));
    assert!(points.next().is_none());
    assert!(points.next().is_none());
}

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
