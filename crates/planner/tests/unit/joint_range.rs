use super::*;
use core::f64::consts::{PI, TAU};

fn assert_range(min_angle: f64, max_angle: f64, expected_min: f64, expected_max: f64) {
    let range = JointRange::new(min_angle, max_angle).expect("valid range should be accepted");
    assert_eq!(range.min_angle, expected_min);
    assert_eq!(range.max_angle, expected_max);
}

#[test]
fn new_accepts_equal_finite_limits_at_every_representative_scale() {
    for angle in [
        -f64::MAX,
        -f64::MIN_POSITIVE,
        -f64::from_bits(1),
        -0.0,
        0.0,
        f64::from_bits(1),
        f64::MIN_POSITIVE,
        f64::MAX,
    ] {
        let range = JointRange::new(angle, angle).expect("equal finite limits should be valid");
        assert!(range.min_angle.is_finite());
        assert!(range.max_angle.is_finite());
        assert_eq!(range.min_angle, libm::remainder(angle, TAU));
        assert_eq!(range.max_angle, libm::remainder(angle, TAU));
    }
}

#[test]
fn new_accepts_ordered_finite_limits_including_adjacent_values() {
    let one = 1.0_f64;
    let next_after_one = f64::from_bits(one.to_bits() + 1);

    for (min_angle, max_angle) in [
        (-f64::MAX, f64::MAX),
        (-f64::MIN_POSITIVE, f64::MIN_POSITIVE),
        (-f64::from_bits(1), f64::from_bits(1)),
        (-PI, PI),
        (one, next_after_one),
    ] {
        let range = JointRange::new(min_angle, max_angle)
            .expect("ordered finite limits should be accepted");
        assert!(range.min_angle.is_finite());
        assert!(range.max_angle.is_finite());
    }
}

#[test]
fn new_rejects_a_non_finite_minimum_or_maximum() {
    for invalid in [f64::NEG_INFINITY, f64::INFINITY, f64::NAN, -f64::NAN] {
        for (min_angle, max_angle) in [(invalid, 0.0), (0.0, invalid), (invalid, invalid)] {
            assert!(matches!(
                JointRange::new(min_angle, max_angle),
                Err(PlannerError::AngleNonFinite)
            ));
        }
    }
}

#[test]
fn non_finite_validation_takes_precedence_over_limit_ordering() {
    for (min_angle, max_angle) in [
        (f64::INFINITY, f64::NEG_INFINITY),
        (f64::NAN, -f64::MAX),
        (f64::MAX, f64::NAN),
    ] {
        assert!(matches!(
            JointRange::new(min_angle, max_angle),
            Err(PlannerError::AngleNonFinite)
        ));
    }
}

#[test]
fn new_rejects_every_representative_inverted_finite_range() {
    let one = 1.0_f64;
    let before_one = f64::from_bits(one.to_bits() - 1);

    for (min_angle, max_angle) in [
        (f64::from_bits(1), 0.0),
        (0.0, -f64::from_bits(1)),
        (one, before_one),
        (PI, -PI),
        (f64::MAX, -f64::MAX),
    ] {
        assert!(matches!(
            JointRange::new(min_angle, max_angle),
            Err(PlannerError::MinGreaterThanMax)
        ));
    }
}

#[test]
fn angles_are_reduced_before_ordering_is_validated_to_one_turn() {
    assert_range(TAU, PI, 0.0, PI);
}

#[test]
fn accepted_limits_are_reduced_with_ieee_remainder() {
    for (min_angle, max_angle) in [
        (-2.0 * TAU - 0.5, -2.0 * TAU - 0.25),
        (-PI, PI),
        (TAU + 0.25, TAU + 0.5),
        (10.0 * TAU + 1.0, 10.0 * TAU + 2.0),
    ] {
        assert_range(
            min_angle,
            max_angle,
            libm::remainder(min_angle, TAU),
            libm::remainder(max_angle, TAU),
        );
    }
}
