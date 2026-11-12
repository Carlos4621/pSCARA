use kinematics::{KinematicsError, Point, Scara};

fn point(x: f64, y: f64) -> Point {
    Point::new(x, y).expect("test coordinates must be finite")
}

fn robot(first_arm: f64, second_arm: f64) -> Scara {
    Scara::new(first_arm, second_arm).expect("test arm lengths must be valid")
}

fn assert_in_both_directions(scara: &Scara, start: Point, end: Point, expected: bool) {
    for (a, b) in [(start, end), (end, start)] {
        assert_eq!(
            scara.is_segment_reachable(a, b),
            Ok(expected),
            "robot {scara:?}, segment {a:?} -> {b:?}"
        );
    }
}

#[test]
fn chord_wholly_inside_the_annulus_is_reachable_in_either_direction() {
    let scara = robot(100.0, 60.0); // Reachable radii: 40 through 160.
    let start = point(100.0, 0.0);
    let end = point(0.0, 100.0);
    // The chord's nearest point has radius sqrt(5_000), greater than 40.

    assert_eq!(scara.is_segment_reachable(start, end), Ok(true));
    assert_eq!(scara.is_segment_reachable(end, start), Ok(true));
}

#[test]
fn chord_crossing_the_inner_hole_is_unreachable_even_with_outer_boundary_endpoints() {
    let scara = robot(100.0, 60.0);
    let start = point(160.0, 0.0);
    let end = point(-160.0, 0.0);
    // Both endpoints are reachable, but the midpoint lies at the origin.

    assert_eq!(scara.is_segment_reachable(start, end), Ok(false));
    assert_eq!(scara.is_segment_reachable(end, start), Ok(false));
}

#[test]
fn tangent_to_inner_boundary_is_reachable_but_nearby_crossing_is_not() {
    let scara = robot(100.0, 60.0);

    assert_eq!(
        scara.is_segment_reachable(point(40.0, -100.0), point(40.0, 100.0)),
        Ok(true)
    );
    assert_eq!(
        scara.is_segment_reachable(point(39.0, -100.0), point(39.0, 100.0)),
        Ok(false)
    );
}

#[test]
fn outer_boundary_endpoints_do_not_require_the_chord_to_stay_on_the_boundary() {
    let scara = robot(100.0, 60.0);
    // Its midpoint has radius sqrt(12_800), safely outside the inner hole.

    assert_eq!(
        scara.is_segment_reachable(point(160.0, 0.0), point(0.0, 160.0)),
        Ok(true)
    );
}

#[test]
fn an_unreachable_endpoint_makes_the_whole_segment_unreachable() {
    let scara = robot(100.0, 60.0);

    for outside in [point(39.0, 0.0), point(161.0, 0.0)] {
        assert_eq!(
            scara.is_segment_reachable(outside, point(100.0, 0.0)),
            Ok(false)
        );
        assert_eq!(
            scara.is_segment_reachable(point(100.0, 0.0), outside),
            Ok(false)
        );
    }
}

#[test]
fn zero_length_segment_has_the_reachability_of_its_endpoint() {
    let scara = robot(100.0, 60.0);

    for reachable in [point(40.0, 0.0), point(100.0, 0.0), point(160.0, 0.0)] {
        assert_eq!(scara.is_segment_reachable(reachable, reachable), Ok(true));
    }
    assert_eq!(
        scara.is_segment_reachable(point(0.0, 0.0), point(0.0, 0.0)),
        Ok(false)
    );
}

#[test]
fn a_longer_second_arm_keeps_the_same_annular_segment_geometry() {
    let scara = robot(60.0, 100.0);
    assert_in_both_directions(&scara, point(40.0, 0.0), point(160.0, 0.0), true);
    assert_in_both_directions(&scara, point(-160.0, 0.0), point(160.0, 0.0), false);
}

#[test]
fn equal_arms_have_no_inner_hole_so_a_diameter_is_reachable() {
    let scara = robot(100.0, 100.0);

    assert_eq!(
        scara.is_segment_reachable(point(100.0, 0.0), point(-100.0, 0.0)),
        Ok(true)
    );
}

#[test]
fn normalized_chord_is_reachable_when_raw_squared_coordinates_underflow() {
    let scale = 1e-200;
    let scara = robot(scale, scale);

    assert_eq!(
        scara.is_segment_reachable(point(scale, 0.0), point(0.0, scale)),
        Ok(true)
    );
}

#[test]
fn normalized_chord_is_reachable_when_raw_squared_coordinates_overflow() {
    let scale = 1e200;
    let scara = robot(scale, scale);

    assert_eq!(
        scara.is_segment_reachable(point(scale, 0.0), point(0.0, scale)),
        Ok(true)
    );
}

#[test]
fn axial_geometry_is_preserved_from_the_smallest_subnormal_to_near_maximum() {
    for scale in [
        f64::from_bits(1),
        f64::from_bits(16),
        f64::from_bits(f64::MIN_POSITIVE.to_bits() - 1),
        f64::MIN_POSITIVE,
        1e-300,
        1e-200,
        1.0,
        1e200,
        1e300,
        f64::MAX / 8.0,
    ] {
        for (l1, l2) in [(3.0 * scale, scale), (scale, 3.0 * scale)] {
            let scara = robot(l1, l2); // Inner radius 2s, outer radius 4s.
            for (a, b, expected) in [
                ((2.0, -1.0), (2.0, 1.0), true),  // Inner tangent.
                ((0.0, -3.0), (0.0, 3.0), false), // Both endpoints reachable; crosses hole.
                ((2.0, 0.0), (4.0, 0.0), true),   // Both inclusive boundaries.
                ((3.0, 0.0), (5.0, 0.0), false),  // Outside outer boundary.
                ((1.0, 0.0), (3.0, 0.0), false),  // Inside inner boundary.
                ((2.0, 0.0), (2.0, 0.0), true),   // Degenerate inner boundary.
                ((4.0, 0.0), (4.0, 0.0), true),   // Degenerate outer boundary.
                ((0.0, 0.0), (0.0, 0.0), false),
            ] {
                assert_in_both_directions(
                    &scara,
                    point(a.0 * scale, a.1 * scale),
                    point(b.0 * scale, b.1 * scale),
                    expected,
                );
            }
        }
    }
}

// Exact integer oracle for the annulus [2, 4]. With integer coordinates this
// compares squared distances as rational numbers, without any floating-point
// projection, square root, or copy of the implementation's tolerance logic.
fn integer_segment_is_reachable(a: (i64, i64), b: (i64, i64)) -> bool {
    let squared_norm = |p: (i64, i64)| p.0 * p.0 + p.1 * p.1;
    if !(4..=16).contains(&squared_norm(a)) || !(4..=16).contains(&squared_norm(b)) {
        return false;
    }

    let d = (b.0 - a.0, b.1 - a.1);
    let length_squared = squared_norm(d);
    let projection_numerator = -(a.0 * d.0 + a.1 * d.1);
    if projection_numerator <= 0 || projection_numerator >= length_squared {
        return true; // The nearest point is a previously validated endpoint.
    }

    let determinant = a.0 * b.1 - a.1 * b.0;
    determinant * determinant >= 4 * length_squared
}

#[test]
fn integer_grid_matches_exact_geometry_across_extreme_binary_scales() {
    for exponent in [-1022, -1000, -700, 0, 700, 1000, 1020] {
        let scale = 2.0_f64.powi(exponent);
        let scara = robot(3.0 * scale, scale);
        for ax in -5..=5 {
            for ay in -5..=5 {
                for bx in -5..=5 {
                    for by in -5..=5 {
                        let expected = integer_segment_is_reachable((ax, ay), (bx, by));
                        assert_eq!(
                            scara.is_segment_reachable(
                                point(ax as f64 * scale, ay as f64 * scale),
                                point(bx as f64 * scale, by as f64 * scale),
                            ),
                            Ok(expected),
                            "exponent {exponent}, segment ({ax}, {ay}) -> ({bx}, {by})"
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn tiny_normalized_segments_remain_reachable_when_their_squared_length_is_zero() {
    for scale in [1e-100, 1.0, 1e100, f64::MAX / 4.0] {
        let scara = robot(2.0 * scale, scale);
        let offset = scale * 1e-200;
        let normalized_length = (2.0 * offset) / (2.0 * scale);
        assert!(normalized_length > 0.0);
        assert_eq!(normalized_length * normalized_length, 0.0);

        // The closest point is respectively the midpoint, the first endpoint,
        // and the second endpoint; the unit direction must work in each case.
        for (y1, y2) in [(-offset, offset), (0.0, offset), (-offset, 0.0)] {
            assert_in_both_directions(&scara, point(scale, y1), point(scale, y2), true);
        }
    }
}

#[test]
fn a_subnormal_normalized_direction_is_not_mistaken_for_a_zero_length_segment() {
    let scara = robot(f64::MAX / 2.0, f64::MAX / 4.0);
    let normalized_delta = 1.0 / (f64::MAX / 2.0);
    assert!(normalized_delta > 0.0 && normalized_delta < f64::MIN_POSITIVE);
    assert_eq!(normalized_delta * normalized_delta, 0.0);

    assert_in_both_directions(
        &scara,
        point(f64::MAX / 2.0, -0.5),
        point(f64::MAX / 2.0, 0.5),
        true,
    );
}

#[test]
fn normalization_reports_numerical_failure_if_distinct_endpoints_collapse() {
    let scara = robot(f64::MAX / 2.0, f64::MAX / 4.0);
    let a = point(f64::MAX / 2.0, 0.0);
    let b = point(f64::MAX / 2.0, f64::from_bits(1));
    assert_ne!(a, b);
    for (start, end) in [(a, b), (b, a)] {
        assert_eq!(
            scara.is_segment_reachable(start, end),
            Err(KinematicsError::NumericalFailure)
        );
    }
}

#[test]
fn adjacent_representable_endpoints_are_reachable_at_small_and_large_scales() {
    for scale in [f64::from_bits(1), f64::MIN_POSITIVE, 1e-300, 1.0, 1e300] {
        let next = f64::from_bits(scale.to_bits() + 1);
        assert_in_both_directions(
            &robot(scale, scale),
            point(scale, 0.0),
            point(next, 0.0),
            true,
        );
    }
}

#[test]
fn inner_and_outer_roundoff_tolerances_scale_with_the_robot() {
    for exponent in [-1000, 0, 1000] {
        let scale = 2.0_f64.powi(exponent);
        let scara = robot(3.0 * scale, scale);
        // Both offsets are relative to s; the documented roundoff allowance is
        // 64 * EPSILON * (3s + s) = 256 * EPSILON * s.
        for (offset, accepted) in [(32.0 * f64::EPSILON, true), (1024.0 * f64::EPSILON, false)] {
            let inner_x = (2.0 - offset) * scale;
            assert_in_both_directions(
                &scara,
                point(inner_x, -scale),
                point(inner_x, scale),
                accepted,
            );
            for outside in [point(inner_x, 0.0), point((4.0 + offset) * scale, 0.0)] {
                assert_in_both_directions(&scara, outside, point(3.0 * scale, 0.0), accepted);
            }
        }
    }
}

#[test]
fn projection_onto_the_supporting_line_is_clamped_to_the_segment() {
    for scale in [1e-300, 1.0, 1e300] {
        let scara = robot(3.0 * scale, scale);
        // The infinite line passes through the hole, but the segment does not.
        assert_in_both_directions(
            &scara,
            point(2.0 * scale, 0.0),
            point(3.0 * scale, 0.0),
            true,
        );
    }
}

#[test]
fn a_maximum_diameter_is_reachable_even_when_raw_endpoint_subtraction_overflows() {
    assert!((f64::MAX - (-f64::MAX)).is_infinite());
    assert_in_both_directions(
        &robot(f64::MAX, f64::MAX),
        point(f64::MAX, 0.0),
        point(-f64::MAX, 0.0),
        true,
    );
}

#[test]
fn unequal_arms_keep_a_finite_inner_boundary_when_their_sum_overflows() {
    assert!((f64::MAX + f64::MAX / 2.0).is_infinite());
    for (l1, l2) in [(f64::MAX, f64::MAX / 2.0), (f64::MAX / 2.0, f64::MAX)] {
        let scara = robot(l1, l2);
        for (x, reachable) in [(f64::MAX / 2.0, true), (f64::MAX / 4.0, false)] {
            assert_in_both_directions(
                &scara,
                point(x, -f64::MAX / 2.0),
                point(x, f64::MAX / 2.0),
                reachable,
            );
        }
    }
}

#[test]
fn extremely_disparate_arms_still_reject_a_chord_crossing_the_inner_hole() {
    for (l1, l2) in [(f64::MAX, f64::from_bits(1)), (f64::from_bits(1), f64::MAX)] {
        assert_in_both_directions(
            &robot(l1, l2),
            point(f64::MAX, 0.0),
            point(0.0, f64::MAX),
            false,
        );
    }
}

#[test]
fn signed_zero_endpoints_have_identical_reachability() {
    for x in [-0.0, 0.0] {
        for y in [-0.0, 0.0] {
            for scale in [f64::from_bits(1), 1.0, f64::MAX] {
                let origin = point(x, y);
                assert_in_both_directions(&robot(scale, scale), origin, point(0.0, 0.0), true);
            }
        }
    }
}

#[test]
fn regression_degenerate_origin_is_outside_the_hole_even_when_arm_sum_overflows() {
    let scara = robot(f64::MAX, f64::MAX / 2.0);
    // The inner radius is MAX/2. An overflowing outer radius must not erase it.
    assert_in_both_directions(&scara, point(0.0, 0.0), point(0.0, 0.0), false);
}

#[test]
fn regression_reachable_diagonal_is_not_rejected_when_its_radius_overflows() {
    let scara = robot(f64::MAX, f64::MAX);
    // The whole segment lies in the square [0, MAX]^2, inside radius 2*MAX.
    // Its endpoint radius sqrt(2)*MAX is not representable as f64.
    assert_in_both_directions(
        &scara,
        point(f64::MAX, f64::MAX),
        point(f64::MAX, 0.0),
        true,
    );
}

#[test]
fn a_maximum_finite_outer_radius_and_its_tolerance_preserve_segment_geometry() {
    let arm = f64::MAX / 2.0;
    let outer = arm + arm;
    assert!(outer.is_finite());
    assert!((outer + 64.0 * f64::EPSILON * outer).is_infinite());

    let scara = robot(arm, arm);
    assert_in_both_directions(&scara, point(outer, 0.0), point(-outer, 0.0), true);
    assert_in_both_directions(&scara, point(arm, arm), point(-arm, -arm), true);
    assert_in_both_directions(&scara, point(outer, outer), point(0.0, 0.0), false);
}
