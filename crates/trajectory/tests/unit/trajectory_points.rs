use super::*;

#[test]
fn constructor_with_zero_max_index_yields_only_the_end() {
    let segment = LineSegment::new(
        Point::new(-8.0, 4.0).unwrap(),
        Point::new(8.0, -4.0).unwrap(),
    )
    .unwrap();
    let mut points = TrajectoryPoints::new(&segment, 0, f64::MAX);

    assert!(matches!(
        points.next(),
        Some(Ok(point))
            if point == segment.end()
    ));
    assert!(points.next().is_none());
    assert!(points.next().is_none());
}

#[test]
fn next_stops_permanently_after_an_invalid_internal_parameter() {
    let segment =
        LineSegment::new(Point::new(0.0, 0.0).unwrap(), Point::new(2.0, 0.0).unwrap()).unwrap();
    let mut points = TrajectoryPoints::new(&segment, 2, 1.0);

    assert_eq!(points.next(), Some(Ok(segment.start())));
    // Fault injection is internal: validated public segments no longer allow
    // the overflowing endpoint difference used by the previous regression.
    points.actual_index = 3;
    assert!(matches!(
        points.next(),
        Some(Err(TrajectoryError::TOutOfBounds))
    ));
    assert!(points.next().is_none());
    assert!(points.next().is_none());
}

#[test]
fn the_last_index_can_wrap_without_restarting_the_completed_iterator() {
    let segment =
        LineSegment::new(Point::new(0.0, 0.0).unwrap(), Point::new(1.0, 0.0).unwrap()).unwrap();
    let mut points = TrajectoryPoints::new(&segment, usize::MAX, 1.0);
    points.actual_index = usize::MAX;
    assert_eq!(points.next(), Some(Ok(segment.end())));
    assert_eq!(points.actual_index, 0);
    assert!(points.is_completed);
    assert!(points.next().is_none());
    assert!(points.next().is_none());
}

#[test]
fn last_point_tracks_every_successful_sample_including_the_first() {
    let segment =
        LineSegment::new(Point::new(0.0, 0.0).unwrap(), Point::new(4.0, 0.0).unwrap()).unwrap();
    let mut points = TrajectoryPoints::new(&segment, 4, 1.0);
    assert_eq!(points.last_point, None);
    for index in 0..=4 {
        let expected = Point::new(f64::from(index), 0.0).unwrap();
        assert_eq!(points.next(), Some(Ok(expected)));
        assert_eq!(points.last_point, Some(expected));
    }
    assert!(points.next().is_none());
    assert_eq!(points.last_point, Some(segment.end()));
}

#[test]
fn a_failed_separation_check_preserves_the_previous_point_and_terminates() {
    let segment =
        LineSegment::new(Point::new(0.0, 0.0).unwrap(), Point::new(2.0, 0.0).unwrap()).unwrap();
    let mut points = TrajectoryPoints::new(&segment, 2, 0.5);
    assert_eq!(points.next(), Some(Ok(segment.start())));
    // Seed the history explicitly to isolate the failure branch from history tracking.
    points.last_point = Some(segment.start());
    assert_eq!(points.next(), Some(Err(TrajectoryError::PrecissionLost)));
    assert_eq!(points.last_point, Some(segment.start()));
    assert!(points.is_completed);
    assert!(points.next().is_none());
    assert!(points.next().is_none());
}

#[test]
fn invalid_internal_separation_is_mapped_to_a_terminal_precision_error() {
    let segment =
        LineSegment::new(Point::new(0.0, 0.0).unwrap(), Point::new(1.0, 0.0).unwrap()).unwrap();
    for maximum in [-1.0, f64::NAN, f64::INFINITY] {
        // The public points() method rejects these values before construction.
        let mut points = TrajectoryPoints::new(&segment, 1, maximum);
        assert_eq!(points.next(), Some(Ok(segment.start())));
        points.last_point = Some(segment.start());
        assert_eq!(points.next(), Some(Err(TrajectoryError::PrecissionLost)));
        assert!(points.next().is_none());
        assert!(points.next().is_none());
    }
}
