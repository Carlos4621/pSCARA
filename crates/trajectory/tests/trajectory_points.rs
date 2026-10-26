use kinematics::Point;
use trajectory::{LineSegment, TrajectoryError};

fn segment(start: (f64, f64), end: (f64, f64)) -> LineSegment {
    LineSegment {
        start: Point {
            x: start.0,
            y: start.1,
        },
        end: Point { x: end.0, y: end.1 },
    }
}

fn assert_point_eq(actual: Point, expected: Point) {
    assert_eq!(
        actual.x, expected.x,
        "x: expected {expected:?}, got {actual:?}"
    );
    assert_eq!(
        actual.y, expected.y,
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
    libm::hypot(a.x - b.x, a.y - b.y)
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
        for (point, expected_x) in actual.into_iter().zip(expected_x) {
            assert_point_eq(
                point,
                Point {
                    x: expected_x,
                    y: 0.0,
                },
            );
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
        line.start,
    );

    let mut last = line.start;
    while let Some(result) = points.next() {
        last = result.unwrap_or_else(|_| panic!("valid trajectory point rejected"));
    }
    assert_point_eq(last, line.end);
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
            assert_point_eq(actual[0], line.start);
            assert_point_eq(actual[1], line.end);
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
            Err(TrajectoryError::InvalidMaxPointDitance)
        ));
    }
}

#[test]
fn points_propagates_invalid_segment_lengths_before_constructing_an_iterator() {
    for line in [
        segment((f64::NAN, 0.0), (0.0, 0.0)),
        segment((0.0, f64::INFINITY), (0.0, 0.0)),
        segment((-f64::MAX, 0.0), (f64::MAX, 0.0)),
        segment((0.0, 0.0), (f64::MAX, f64::MAX)),
    ] {
        assert!(matches!(
            line.points(1.0),
            Err(TrajectoryError::SegmentNonFinite)
        ));
    }
}

#[test]
fn points_matches_point_at_and_respects_the_maximum_distance_on_a_grid() {
    let coordinates = [-8.0, -1.0, 0.0, 3.0, 8.0];
    for sx in coordinates {
        for sy in coordinates {
            for ex in coordinates {
                for ey in coordinates {
                    let line = segment((sx, sy), (ex, ey));
                    let length = line
                        .length()
                        .unwrap_or_else(|_| panic!("finite length rejected"));
                    for maximum in [0.125, 0.5, 1.0, 2.0, 16.0] {
                        let actual = collect_points(&line, maximum);
                        let segment_count = (libm::ceil(length / maximum) as usize).max(1);

                        // There is always at least one interval, so start and end are emitted
                        // even when both endpoints represent the same point.
                        assert_eq!(actual.len(), segment_count + 1);
                        assert_point_eq(actual[0], line.start);
                        assert_point_eq(*actual.last().unwrap(), line.end);

                        for (index, point) in actual.iter().copied().enumerate() {
                            let t = index as f64 / segment_count as f64;
                            assert_point_eq(
                                point,
                                line.point_at(t).unwrap_or_else(|_| {
                                    panic!("point_at({t}) unexpectedly failed")
                                }),
                            );
                        }

                        for pair in actual.windows(2) {
                            let actual_distance = distance(pair[0], pair[1]);
                            let tolerance = 16.0 * f64::EPSILON * maximum.max(1.0);
                            assert!(
                                actual_distance <= maximum + tolerance,
                                "distance {actual_distance} exceeds maximum {maximum} for {actual:?}"
                            );
                        }
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
    for (index, point) in actual.into_iter().enumerate() {
        assert_point_eq(
            point,
            Point {
                x: index as f64 * unit,
                y: 0.0,
            },
        );
    }

    let largest = segment((0.0, 0.0), (f64::MAX, 0.0));
    let actual = collect_points(&largest, f64::MAX);
    assert_eq!(actual.len(), 2);
    assert_point_eq(actual[0], largest.start);
    assert_point_eq(actual[1], largest.end);
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
        assert_point_eq(actual[0], line.start);
        assert_point_eq(*actual.last().unwrap(), line.end);
        for pair in actual.windows(2) {
            assert!(distance(pair[0], pair[1]) <= maximum);
        }
    }
}

#[test]
fn points_rejects_requested_resolutions_that_cannot_fit_in_usize() {
    let line = segment((0.0, 0.0), (1.0, 0.0));

    // The first division overflows to infinity; the second remains finite but
    // requests strictly more intervals than usize can represent.
    let finite_overflow_resolution = 1.0 / ((usize::MAX as f64) * 2.0);
    for maximum in [f64::from_bits(1), finite_overflow_resolution] {
        assert!(matches!(
            line.points(maximum),
            Err(TrajectoryError::TooManySegments)
        ));
    }
}
