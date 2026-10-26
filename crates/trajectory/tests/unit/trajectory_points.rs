use super::*;

#[test]
fn next_stops_permanently_after_a_point_calculation_error() {
    let segment = LineSegment {
        start: Point {
            x: -f64::MAX,
            y: 0.0,
        },
        end: Point {
            x: f64::MAX,
            y: 0.0,
        },
    };
    let mut points = TrajectoryPoints::new(&segment, 2);

    assert!(matches!(points.next(), Some(Ok(point)) if point.x == -f64::MAX && point.y == 0.0));
    assert!(matches!(
        points.next(),
        Some(Err(TrajectoryError::PointOutOfSegment))
    ));
    assert!(points.next().is_none());
    assert!(points.next().is_none());
}
