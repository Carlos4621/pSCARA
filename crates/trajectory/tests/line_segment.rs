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

fn point_at(line: &LineSegment, t: f64) -> Point {
    match line.point_at(t) {
        Ok(point) => point,
        Err(_) => panic!("point_at({t}) unexpectedly returned an error"),
    }
}

fn assert_t_out_of_bounds(result: Result<Point, TrajectoryError>) {
    assert!(matches!(result, Err(TrajectoryError::TOutOfBounds)));
}

fn assert_point_out_of_segment(result: Result<Point, TrajectoryError>) {
    assert!(matches!(result, Err(TrajectoryError::PointOutOfSegment)));
}

// Small integers and dyadic parameters keep the following oracles exact in f64.
// This avoids accepting a geometric error through an overly broad tolerance.
#[test]
fn point_at_matches_known_horizontal_vertical_and_diagonal_points() {
    for (start, end, midpoint) in [
        ((2.0, 3.0), (10.0, 3.0), (6.0, 3.0)),
        ((2.0, 3.0), (2.0, 11.0), (2.0, 7.0)),
        ((-8.0, -12.0), (8.0, 4.0), (0.0, -4.0)),
        ((8.0, 4.0), (-8.0, -12.0), (0.0, -4.0)),
        ((-12.0, 8.0), (-4.0, -8.0), (-8.0, 0.0)),
    ] {
        let line = segment(start, end);
        assert_point_eq(point_at(&line, 0.0), line.start);
        assert_point_eq(
            point_at(&line, 0.5),
            Point {
                x: midpoint.0,
                y: midpoint.1,
            },
        );
        assert_point_eq(point_at(&line, 1.0), line.end);
    }
    let line = segment((4.0, -8.0), (20.0, 24.0));
    assert_point_eq(point_at(&line, 0.25), Point { x: 8.0, y: 0.0 });
    assert_point_eq(point_at(&line, 0.75), Point { x: 16.0, y: 16.0 });
}

#[test]
fn point_at_matches_exact_weighted_points_on_a_coordinate_grid() {
    let coordinates = [-16.0, -3.0, 0.0, 5.0, 16.0];
    for sx in coordinates {
        for sy in coordinates {
            for ex in coordinates {
                for ey in coordinates {
                    let line = segment((sx, sy), (ex, ey));
                    for step in 0..=16 {
                        let t = f64::from(step) / 16.0;
                        // Independent weighted-endpoint oracle.
                        assert_point_eq(
                            point_at(&line, t),
                            Point {
                                x: ((16 - step) as f64 * sx + step as f64 * ex) / 16.0,
                                y: ((16 - step) as f64 * sy + step as f64 * ey) / 16.0,
                            },
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn point_at_rejects_parameters_outside_the_closed_unit_interval() {
    let line = segment((-4.0, 8.0), (12.0, -16.0));
    for t in [
        f64::NEG_INFINITY,
        -f64::MAX,
        -2.0,
        -f64::EPSILON,
        -f64::from_bits(1),
    ] {
        assert_t_out_of_bounds(line.point_at(t));
    }
    for t in [
        f64::from_bits(1.0_f64.to_bits() + 1),
        2.0,
        f64::MAX,
        f64::INFINITY,
    ] {
        assert_t_out_of_bounds(line.point_at(t));
    }

    assert_point_eq(point_at(&line, -0.0), line.start);
    assert_point_eq(point_at(&line, 0.0), line.start);
    assert_point_eq(point_at(&line, 1.0), line.end);
}

#[test]
fn point_at_handles_adjacent_floats_at_zero_midpoint_and_one() {
    let line = segment((0.0, 0.0), (1.0, -1.0));
    for t in [
        0.0,
        f64::from_bits(1),
        f64::MIN_POSITIVE,
        f64::EPSILON,
        f64::from_bits(0.5_f64.to_bits() - 1),
        0.5,
        f64::from_bits(0.5_f64.to_bits() + 1),
        f64::from_bits(1.0_f64.to_bits() - 1),
        1.0,
    ] {
        assert_point_eq(point_at(&line, t), Point { x: t, y: -t });
    }
}

#[test]
fn point_at_preserves_degenerate_segments_across_finite_scales() {
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
        for t in [0.0, 0.25, 0.5, 0.75, 1.0] {
            assert_point_eq(point_at(&line, t), line.start);
        }
        for t in [f64::NEG_INFINITY, -1.0, 2.0, f64::INFINITY] {
            assert_t_out_of_bounds(line.point_at(t));
        }
    }
}

#[test]
fn point_at_is_symmetric_when_endpoints_and_parameter_are_reversed() {
    for a in -8..=8 {
        for b in -8..=8 {
            let forward = segment((a as f64, b as f64), (b as f64, -(a as f64)));
            let reverse = segment((b as f64, -(a as f64)), (a as f64, b as f64));
            for step in 0..=16 {
                let t = step as f64 / 16.0;
                assert_point_eq(point_at(&forward, t), point_at(&reverse, 1.0 - t));
            }
        }
    }
}

#[test]
fn point_at_subdivision_agrees_with_the_original_segment() {
    let line = segment((-32.0, 16.0), (32.0, -48.0));
    for a in 0..=8 {
        for b in a..=8 {
            let a = a as f64 / 8.0;
            let b = b as f64 / 8.0;
            let subsegment = LineSegment {
                start: point_at(&line, a),
                end: point_at(&line, b),
            };
            for step in 0..=8 {
                let t = step as f64 / 8.0;
                assert_point_eq(
                    point_at(&subsegment, t),
                    point_at(&line, a * (1.0 - t) + b * t),
                );
            }
        }
    }
}

#[test]
fn point_at_stays_in_the_bounding_box_and_advances_monotonically() {
    for start in [-16.0_f64, 0.0, 16.0] {
        for end in [-16.0_f64, 0.0, 16.0] {
            let line = segment((start, end), (end, start));
            let mut previous = point_at(&line, 0.0);
            for step in 0..=256 {
                let p = point_at(&line, step as f64 / 256.0);
                assert!((start.min(end)..=start.max(end)).contains(&p.x));
                assert!((start.min(end)..=start.max(end)).contains(&p.y));
                if end >= start {
                    assert!(p.x >= previous.x && p.y <= previous.y);
                } else {
                    assert!(p.x <= previous.x && p.y >= previous.y);
                }
                previous = p;
            }
        }
    }
}

#[test]
fn point_at_commutes_with_translation_and_coordinate_swapping() {
    let line = segment((-16.0, 24.0), (32.0, -8.0));
    let translated = segment((112.0, -40.0), (160.0, -72.0));
    let swapped = segment((24.0, -16.0), (-8.0, 32.0));
    for step in 0..=32 {
        let t = step as f64 / 32.0;
        let p = point_at(&line, t);
        assert_point_eq(
            point_at(&translated, t),
            Point {
                x: p.x + 128.0,
                y: p.y - 64.0,
            },
        );
        assert_point_eq(point_at(&swapped, t), Point { x: p.y, y: p.x });
    }
}

#[test]
fn point_at_scales_over_binary_exponents_without_spurious_error() {
    // All values in this sweep remain normal; subnormals have a separate oracle.
    for exponent in -1000_i32..=1000 {
        let scale = f64::from_bits(((exponent + 1023) as u64) << 52);
        let line = segment((-0.5 * scale, 0.5 * scale), (0.5 * scale, -0.5 * scale));
        for step in 0..=16 {
            let coefficient = (step as f64 - 8.0) / 16.0;
            assert_point_eq(
                point_at(&line, step as f64 / 16.0),
                Point {
                    x: coefficient * scale,
                    y: -coefficient * scale,
                },
            );
        }
    }
}

#[test]
fn point_at_interpolates_representable_subnormal_points() {
    let unit = f64::from_bits(1);
    let line = segment((-16.0 * unit, 16.0 * unit), (16.0 * unit, -16.0 * unit));
    for step in 0..=32 {
        let expected = (step as f64 - 16.0) * unit;
        assert_point_eq(
            point_at(&line, step as f64 / 32.0),
            Point {
                x: expected,
                y: -expected,
            },
        );
    }
}

#[test]
fn point_at_interpolates_neighboring_values_near_the_largest_finite_coordinate() {
    let last_bits = f64::MAX.to_bits();
    let first = f64::from_bits(last_bits - 8);
    let line = segment((first, -first), (f64::MAX, -f64::MAX));
    for step in 0..=8 {
        let expected = f64::from_bits(last_bits - 8 + step);
        assert_point_eq(
            point_at(&line, step as f64 / 8.0),
            Point {
                x: expected,
                y: -expected,
            },
        );
    }
}

#[test]
fn point_at_crosses_the_subnormal_to_normal_boundary() {
    let line = segment(
        (0.0, f64::MIN_POSITIVE * 2.0),
        (f64::MIN_POSITIVE * 2.0, 0.0),
    );
    for step in 0..=32 {
        let units = (1_u64 << 48) * step;
        assert_point_eq(
            point_at(&line, step as f64 / 32.0),
            Point {
                x: f64::from_bits(units),
                y: f64::from_bits((1_u64 << 53) - units),
            },
        );
    }
}

#[test]
fn point_at_rejects_nan_parameters_including_on_a_degenerate_segment() {
    for line in [
        segment((1.0, 2.0), (3.0, 4.0)),
        segment((1.0, 2.0), (1.0, 2.0)),
    ] {
        for t in [f64::NAN, -f64::NAN] {
            assert_t_out_of_bounds(line.point_at(t));
        }
    }
}

#[test]
fn point_at_returns_exact_endpoints_before_checking_their_coordinates() {
    let line = segment((f64::NAN, f64::NEG_INFINITY), (f64::INFINITY, -f64::NAN));

    let start = point_at(&line, 0.0);
    assert!(start.x.is_nan());
    assert_eq!(start.y, f64::NEG_INFINITY);

    let end = point_at(&line, 1.0);
    assert_eq!(end.x, f64::INFINITY);
    assert!(end.y.is_nan());
}

#[test]
fn point_at_rejects_non_finite_interior_results() {
    for line in [
        segment((f64::NAN, 0.0), (8.0, 16.0)),
        segment((8.0, 0.0), (f64::NAN, 16.0)),
        segment((0.0, f64::NAN), (16.0, 8.0)),
        segment((0.0, 8.0), (16.0, f64::NAN)),
        segment((f64::NEG_INFINITY, 0.0), (8.0, 16.0)),
        segment((8.0, 0.0), (f64::INFINITY, 16.0)),
    ] {
        assert_point_out_of_segment(line.point_at(0.5));
    }
}

#[test]
fn point_at_returns_extreme_finite_endpoints_without_interpolating() {
    let line = segment((-f64::MAX, f64::MAX), (f64::MAX, -f64::MAX));
    assert_point_eq(point_at(&line, 0.0), line.start);
    assert_point_eq(point_at(&line, 1.0), line.end);
}

#[test]
fn point_at_reports_overflow_for_interior_points_at_opposite_finite_extremes() {
    let line = segment((-f64::MAX, f64::MAX), (f64::MAX, -f64::MAX));
    for t in [0.25, 0.5, 0.75] {
        assert_point_out_of_segment(line.point_at(t));
    }
}

#[test]
fn point_at_preserves_a_small_end_coordinate_after_large_start_coordinates() {
    for start in [1e16, -1e16] {
        let line = segment((start, -start), (1.0, -1.0));
        assert_point_eq(point_at(&line, 1.0), line.end);
    }
}

#[test]
fn point_at_validates_t_before_extreme_endpoint_arithmetic() {
    let line = segment((-f64::MAX, f64::MAX), (f64::MAX, -f64::MAX));
    for t in [f64::NEG_INFINITY, -1.0, 2.0, f64::INFINITY, f64::NAN] {
        assert_t_out_of_bounds(line.point_at(t));
    }
}
