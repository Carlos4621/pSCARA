use super::*;
use core::f64::consts::{PI, TAU};

fn assert_range_preserved(min_angle: f64, max_angle: f64) -> JointRange {
    let range = JointRange::new(min_angle, max_angle).expect("valid range should be accepted");
    assert_eq!(range.min_angle.to_bits(), min_angle.to_bits());
    assert_eq!(range.max_angle.to_bits(), max_angle.to_bits());
    range
}

#[test]
fn new_preserves_ordered_finite_limits_without_normalizing_them() {
    let one = 1.0_f64;
    let next_after_one = f64::from_bits(one.to_bits() + 1);

    for (min_angle, max_angle) in [
        (-f64::MAX, f64::MAX),
        (-f64::MIN_POSITIVE, f64::MIN_POSITIVE),
        (-f64::from_bits(1), f64::from_bits(1)),
        (-PI, PI),
        (one, next_after_one),
        (TAU + 0.25, 10.0 * TAU + 2.0),
    ] {
        assert_range_preserved(min_angle, max_angle);
    }
}

#[test]
fn new_accepts_equal_finite_limits_at_representative_scales() {
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
        assert_range_preserved(angle, angle);
    }
}

#[test]
fn new_rejects_inverted_finite_limits() {
    let one = 1.0_f64;
    let before_one = f64::from_bits(one.to_bits() - 1);

    for (min_angle, max_angle) in [
        (f64::from_bits(1), 0.0),
        (0.0, -f64::from_bits(1)),
        (one, before_one),
        (PI, -PI),
        (TAU, PI),
        (f64::MAX, -f64::MAX),
    ] {
        assert_eq!(
            JointRange::new(min_angle, max_angle).unwrap_err(),
            PlannerError::MinGreaterThanMax
        );
    }
}

#[test]
fn new_rejects_a_non_finite_minimum_or_maximum() {
    for invalid in [f64::NEG_INFINITY, f64::INFINITY, f64::NAN, -f64::NAN] {
        for (min_angle, max_angle) in [(invalid, 0.0), (0.0, invalid), (invalid, invalid)] {
            assert_eq!(
                JointRange::new(min_angle, max_angle).unwrap_err(),
                PlannerError::AngleNonFinite
            );
        }
    }
}

#[test]
fn non_finite_validation_takes_precedence_over_limit_ordering() {
    for (min_angle, max_angle) in [
        (f64::INFINITY, f64::NEG_INFINITY),
        (f64::INFINITY, f64::MAX),
        (-f64::MAX, f64::NEG_INFINITY),
        (f64::NAN, -f64::MAX),
        (f64::MAX, f64::NAN),
    ] {
        assert_eq!(
            JointRange::new(min_angle, max_angle).unwrap_err(),
            PlannerError::AngleNonFinite
        );
    }
}

#[test]
fn contains_is_inclusive_and_distinguishes_adjacent_values() {
    let lower = 1.0_f64;
    let upper = f64::from_bits(lower.to_bits() + 2);
    let inside = f64::from_bits(lower.to_bits() + 1);
    let below = f64::from_bits(lower.to_bits() - 1);
    let above = f64::from_bits(upper.to_bits() + 1);
    let range = JointRange::new(lower, upper).unwrap();

    for angle in [lower, inside, upper] {
        assert_eq!(range.contains(angle), Ok(true));
    }
    for angle in [below, above, -f64::MAX, f64::MAX] {
        assert_eq!(range.contains(angle), Ok(false));
    }
}

#[test]
fn contains_does_not_reduce_angles_to_one_turn() {
    let range = JointRange::new(TAU + 0.25, TAU + 0.5).unwrap();

    assert_eq!(range.contains(TAU + 0.25), Ok(true));
    assert_eq!(range.contains(TAU + 0.5), Ok(true));
    assert_eq!(range.contains(0.25), Ok(false));
    assert_eq!(range.contains(0.5), Ok(false));
}

#[test]
fn contains_rejects_every_non_finite_query() {
    for range in [
        JointRange::new(-PI, PI).unwrap(),
        JointRange::new(-f64::MAX, f64::MAX).unwrap(),
        JointRange::new(0.0, 0.0).unwrap(),
    ] {
        for angle in [f64::NEG_INFINITY, f64::INFINITY, f64::NAN, -f64::NAN] {
            assert_eq!(range.contains(angle), Err(PlannerError::AngleNonFinite));
        }
    }
}
