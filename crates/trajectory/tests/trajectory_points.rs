use kinematics::Point;
use trajectory::{LineSegment, TrajectoryError};

fn segment(start: (f64, f64), end: (f64, f64)) -> LineSegment {
    LineSegment::new(point(start.0, start.1), point(end.0, end.1))
        .expect("test segment must have a finite length")
}

fn point(x: f64, y: f64) -> Point {
    Point::new(x, y).unwrap_or_else(|_| panic!("invalid test point ({x}, {y})"))
}

fn assert_point_eq(actual: Point, expected: Point) {
    assert_eq!(
        actual.x(),
        expected.x(),
        "x: expected {expected:?}, got {actual:?}"
    );
    assert_eq!(
        actual.y(),
        expected.y(),
        "y: expected {expected:?}, got {actual:?}"
    );
}

fn collect_points(line: &LineSegment, max_point_distance: f64) -> Vec<Point> {
    let iterator = match line.points(max_point_distance) {
        Ok(iterator) => iterator,
        Err(_) => panic!("points({max_point_distance}) unexpectedly returned an error"),
    };

    iterator
        .map(|result| match result {
            Ok(point) => point,
            Err(_) => panic!("trajectory iterator unexpectedly returned an error"),
        })
        .collect()
}

fn distance(a: Point, b: Point) -> f64 {
    libm::hypot(a.x() - b.x(), a.y() - b.y())
}

#[test]
fn points_matches_known_exact_sequences() {
    for (start, end, maximum, expected_x) in [
        ((0.0, 0.0), (10.0, 0.0), 3.0, vec![0.0, 2.5, 5.0, 7.5, 10.0]),
        ((0.0, 0.0), (10.0, 0.0), 2.5, vec![0.0, 2.5, 5.0, 7.5, 10.0]),
        ((0.0, 0.0), (10.0, 0.0), 10.0, vec![0.0, 10.0]),
        ((0.0, 0.0), (10.0, 0.0), 20.0, vec![0.0, 10.0]),
        ((10.0, 0.0), (0.0, 0.0), 3.0, vec![10.0, 7.5, 5.0, 2.5, 0.0]),
    ] {
        let actual = collect_points(&segment(start, end), maximum);
        assert_eq!(actual.len(), expected_x.len());
        for (actual_point, expected_x) in actual.into_iter().zip(expected_x) {
            assert_point_eq(actual_point, point(expected_x, 0.0));
        }
    }
}

#[test]
fn points_includes_exact_endpoints_and_is_fused_after_completion() {
    let line = segment((-7.0, 11.0), (13.0, -5.0));
    let mut points = line
        .points(6.0)
        .unwrap_or_else(|_| panic!("valid trajectory rejected"));

    assert_point_eq(
        points
            .next()
            .unwrap()
            .unwrap_or_else(|_| panic!("start rejected")),
        line.start(),
    );

    let mut last = line.start();
    for result in points.by_ref() {
        last = result.unwrap_or_else(|_| panic!("valid trajectory point rejected"));
    }
    assert_point_eq(last, line.end());
    assert!(points.next().is_none());
    assert!(points.next().is_none());
}

#[test]
fn points_yields_start_and_end_for_degenerate_segments_at_every_finite_scale() {
    for coordinate in [
        -f64::MAX,
        -1.0,
        -f64::from_bits(1),
        -0.0,
        0.0,
        f64::from_bits(1),
        1.0,
        f64::MAX,
    ] {
        let line = segment((coordinate, -coordinate), (coordinate, -coordinate));
        for maximum in [f64::from_bits(1), 1.0, f64::MAX] {
            let actual = collect_points(&line, maximum);
            assert_eq!(actual.len(), 2);
            assert_point_eq(actual[0], line.start());
            assert_point_eq(actual[1], line.end());
        }
    }
}

#[test]
fn points_rejects_every_non_positive_or_non_finite_maximum_distance() {
    let line = segment((0.0, 0.0), (1.0, 1.0));
    for maximum in [
        f64::NEG_INFINITY,
        -f64::MAX,
        -1.0,
        -f64::from_bits(1),
        -0.0,
        0.0,
        f64::NAN,
        -f64::NAN,
        f64::INFINITY,
    ] {
        assert!(matches!(
            line.points(maximum),
            Err(TrajectoryError::InvalidMaxPointDistance)
        ));
    }
}

#[test]
fn sampling_does_not_change_the_cached_length_or_segment_endpoints() {
    let line = segment((-3.0, 7.0), (9.0, -9.0));
    let length = line.length();
    let start = line.start();
    let end = line.end();
    let mut samples = line.points(2.0).unwrap();
    assert_eq!(samples.next(), Some(Ok(start)));
    assert!(samples.next().unwrap().is_ok());
    assert_eq!(line.length(), length);
    assert_eq!(line.start(), start);
    assert_eq!(line.end(), end);
}

#[test]
fn points_matches_point_at_and_respects_the_maximum_distance_on_a_grid() {
    let coordinates = [-8.0, -1.0, 0.0, 3.0, 8.0];
    for sx in coordinates {
        for sy in coordinates {
            for ex in coordinates {
                for ey in coordinates {
                    let line = segment((sx, sy), (ex, ey));
                    let length = line.length();
                    for maximum in [0.125, 0.5, 1.0, 2.0, 16.0] {
                        let segment_count = (libm::ceil(length / maximum) as usize).max(1);
                        let mut points = line.points(maximum).unwrap();
                        let mut previous = None;
                        for index in 0..=segment_count {
                            let t = index as f64 / segment_count as f64;
                            let candidate = line.point_at(t).unwrap();
                            let exceeds_limit =
                                previous.is_some_and(|last| distance(last, candidate) > maximum);
                            if exceeds_limit {
                                assert_eq!(
                                    points.next(),
                                    Some(Err(TrajectoryError::PrecissionLost))
                                );
                                break;
                            }
                            assert_eq!(points.next(), Some(Ok(candidate)));
                            previous = Some(candidate);
                        }
                        // Both normal completion and the first precision error are terminal.
                        assert!(points.next().is_none());
                        assert!(points.next().is_none());
                    }
                }
            }
        }
    }
}

#[test]
fn points_handles_subnormal_and_largest_finite_representable_scales() {
    let unit = f64::from_bits(1);
    let subnormal = segment((0.0, 0.0), (4.0 * unit, 0.0));
    let actual = collect_points(&subnormal, unit);
    assert_eq!(actual.len(), 5);
    for (index, actual_point) in actual.into_iter().enumerate() {
        assert_point_eq(actual_point, point(index as f64 * unit, 0.0));
    }

    let largest = segment((0.0, 0.0), (f64::MAX, 0.0));
    let actual = collect_points(&largest, f64::MAX);
    assert_eq!(actual.len(), 2);
    assert_point_eq(actual[0], largest.start());
    assert_point_eq(actual[1], largest.end());
}

#[test]
fn points_handles_values_immediately_adjacent_to_division_boundaries() {
    let line = segment((0.0, 0.0), (1.0, 0.0));
    let below_one = f64::from_bits(1.0_f64.to_bits() - 1);
    let above_one = f64::from_bits(1.0_f64.to_bits() + 1);
    let below_half = f64::from_bits(0.5_f64.to_bits() - 1);
    let above_half = f64::from_bits(0.5_f64.to_bits() + 1);

    for (maximum, expected_points) in [
        (below_one, 3),
        (1.0, 2),
        (above_one, 2),
        (below_half, 4),
        (0.5, 3),
        (above_half, 3),
    ] {
        let actual = collect_points(&line, maximum);
        assert_eq!(
            actual.len(),
            expected_points,
            "unexpected point count for maximum {maximum}"
        );
        assert_point_eq(actual[0], line.start());
        assert_point_eq(*actual.last().unwrap(), line.end());
        for pair in actual.windows(2) {
            assert!(distance(pair[0], pair[1]) <= maximum);
        }
    }
}

#[test]
fn points_adds_a_segment_when_rounded_division_underestimates_the_required_count() {
    let length = f64::from_bits(0x0b7b_9d85_8392_0fd1);
    let maximum = f64::from_bits(0x0b62_6903_ad0c_0a8b);

    // Floating-point division rounds the quotient down to exactly three even
    // though multiplying back proves that three intervals are insufficient.
    assert_eq!(length / maximum, 3.0);
    assert!(3.0 * maximum < length);

    let actual = collect_points(&segment((0.0, 0.0), (length, 0.0)), maximum);
    assert_eq!(actual.len(), 5);
    for pair in actual.windows(2) {
        assert!(distance(pair[0], pair[1]) <= maximum);
    }
}

#[test]
fn points_rejects_requested_resolutions_that_cannot_fit_in_usize() {
    let line = segment((0.0, 0.0), (1.0, 0.0));

    // The first division overflows to infinity; the second remains finite but
    // requests strictly more intervals than usize can represent.
    let finite_overflow_resolution = 1.0 / ((usize::MAX as f64) * 2.0);
    assert!(matches!(
        line.points(f64::from_bits(1)),
        Err(TrajectoryError::InvalidSegmentNumber)
    ));
    assert!(matches!(
        line.points(finite_overflow_resolution),
        Err(TrajectoryError::TooManySegments)
    ));
}

#[test]
fn get_segment_returns_the_borrowed_segment() {
    let line = segment((-3.0, 7.0), (11.0, -13.0));
    let points = line
        .points(f64::MAX)
        .unwrap_or_else(|_| panic!("valid trajectory rejected"));

    assert!(core::ptr::eq(points.get_segment(), &line));
}

#[test]
fn max_segments_matches_the_largest_representable_index() {
    assert_eq!(LineSegment::MAX_SEGMENTS, usize::MAX);
}

#[test]
fn cloned_iterators_preserve_partial_progress_and_finish_independently() {
    let line = segment((0.0, 0.0), (4.0, 0.0));
    let mut original = line.points(1.0).unwrap();
    assert_eq!(original.next(), Some(Ok(line.start())));
    assert_eq!(original.next(), Some(Ok(point(1.0, 0.0))));

    let mut cloned = original.clone();
    assert!(core::ptr::eq(original.get_segment(), cloned.get_segment()));
    assert_eq!(cloned.next(), Some(Ok(point(2.0, 0.0))));
    assert_eq!(original.next(), Some(Ok(point(2.0, 0.0))));
    let expected = vec![Ok(point(3.0, 0.0)), Ok(line.end())];
    assert_eq!(original.by_ref().collect::<Vec<_>>(), expected);
    assert_eq!(cloned.by_ref().collect::<Vec<_>>(), expected);
    for mut exhausted in [original, cloned] {
        assert!(exhausted.next().is_none());
        let mut copy = exhausted.clone();
        assert!(copy.next().is_none());
        assert!(copy.next().is_none());
    }
}

#[test]
fn points_checks_the_representable_neighbors_of_the_maximum_interval_count() {
    let limit = LineSegment::MAX_SEGMENTS as f64;
    let below = f64::from_bits(limit.to_bits() - 1);
    let accepted = segment((0.0, 0.0), (below, 0.0));
    let mut points = accepted.points(1.0).unwrap();
    // Check the boundary without traversing or allocating an enormous trajectory.
    assert_eq!(points.next(), Some(Ok(accepted.start())));
    assert_eq!(points.next(), Some(Ok(point(1.0, 0.0))));
    for length in [limit, f64::from_bits(limit.to_bits() + 1)] {
        let rejected = segment((0.0, 0.0), (length, 0.0));
        assert!(matches!(
            rejected.points(1.0),
            Err(TrajectoryError::TooManySegments)
        ));
    }
}

#[test]
fn precision_loss_in_either_coordinate_or_direction_is_reported_once() {
    for (start, end) in [
        ((1e16, 0.0), (1e16 + 4.0, 0.0)),
        ((1e16 + 4.0, 0.0), (1e16, 0.0)),
        ((0.0, -1e16), (0.0, -1e16 - 4.0)),
        ((0.0, -1e16 - 4.0), (0.0, -1e16)),
    ] {
        let line = segment(start, end);
        let mut points = line.points(1.0).unwrap();
        assert_eq!(points.next(), Some(Ok(line.start())));
        assert_eq!(points.next(), Some(Ok(line.start())));
        // The third candidate would jump by 2; report it without yielding that point.
        assert_eq!(
            points.next(),
            Some(Err(TrajectoryError::PrecissionLost)),
            "segment {line:?}"
        );
        assert!(points.next().is_none());
        assert!(points.next().is_none());
        let mut exhausted_clone = points.clone();
        assert!(exhausted_clone.next().is_none());
    }
}

#[test]
fn a_precision_loss_on_the_final_sample_is_delivered_before_termination() {
    let line = segment((0.0, 0.0), (1.0, 0.0));
    let maximum = 1.0 / 3.0;
    assert!(1.0 - 2.0 / 3.0 > maximum);
    let mut points = line.points(maximum).unwrap();
    for x in [0.0, 1.0 / 3.0, 2.0 / 3.0] {
        assert_eq!(points.next(), Some(Ok(point(x, 0.0))));
    }
    assert_eq!(points.next(), Some(Err(TrajectoryError::PrecissionLost)));
    assert!(points.next().is_none());
    assert!(points.next().is_none());
}

#[test]
fn cloning_before_a_precision_error_preserves_the_last_successful_point() {
    let line = segment((0.0, 0.0), (1.0, 0.0));
    let mut points = line.points(1.0 / 3.0).unwrap();
    for _ in 0..3 {
        assert!(points.next().unwrap().is_ok());
    }
    let cloned = points.clone();
    for mut iterator in [points, cloned] {
        assert_eq!(iterator.next(), Some(Err(TrajectoryError::PrecissionLost)));
        assert!(iterator.next().is_none());
        assert!(iterator.next().is_none());
    }
}
