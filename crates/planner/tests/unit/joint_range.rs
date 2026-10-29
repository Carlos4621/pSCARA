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
fn new_orders_infinite_limits_by_the_same_comparison_as_finite_limits() {
    for (min_angle, max_angle) in [
        (f64::NEG_INFINITY, f64::NEG_INFINITY),
        (f64::NEG_INFINITY, 0.0),
        (f64::NEG_INFINITY, f64::INFINITY),
        (0.0, f64::INFINITY),
        (f64::INFINITY, f64::INFINITY),
    ] {
        assert_range_preserved(min_angle, max_angle);
    }

    for (min_angle, max_angle) in [
        (f64::INFINITY, f64::NEG_INFINITY),
        (f64::INFINITY, f64::MAX),
        (-f64::MAX, f64::NEG_INFINITY),
    ] {
        assert_eq!(
            JointRange::new(min_angle, max_angle).unwrap_err(),
            PlannerError::MinGreaterThanMax
        );
    }
}

#[test]
fn new_accepts_nan_limits_because_they_are_unordered() {
    for (min_angle, max_angle) in [
        (f64::NAN, 0.0),
        (0.0, f64::NAN),
        (f64::NAN, f64::NAN),
        (-f64::NAN, f64::NAN),
    ] {
        let range = JointRange::new(min_angle, max_angle)
            .expect("NaN does not satisfy the min > max rejection condition");
        assert_eq!(range.min_angle.to_bits(), min_angle.to_bits());
        assert_eq!(range.max_angle.to_bits(), max_angle.to_bits());
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
        JointRange::new(f64::NEG_INFINITY, f64::INFINITY).unwrap(),
        JointRange::new(f64::NAN, f64::NAN).unwrap(),
    ] {
        for angle in [f64::NEG_INFINITY, f64::INFINITY, f64::NAN, -f64::NAN] {
            assert_eq!(range.contains(angle), Err(PlannerError::AngleNonFinite));
        }
    }
}

#[test]
fn contains_supports_infinite_bounds_for_finite_queries() {
    let unbounded = JointRange::new(f64::NEG_INFINITY, f64::INFINITY).unwrap();
    for angle in [-f64::MAX, -0.0, 0.0, f64::MAX] {
        assert_eq!(unbounded.contains(angle), Ok(true));
    }

    let positive_infinity = JointRange::new(f64::INFINITY, f64::INFINITY).unwrap();
    let negative_infinity = JointRange::new(f64::NEG_INFINITY, f64::NEG_INFINITY).unwrap();
    for angle in [-f64::MAX, 0.0, f64::MAX] {
        assert_eq!(positive_infinity.contains(angle), Ok(false));
        assert_eq!(negative_infinity.contains(angle), Ok(false));
    }
}

#[test]
fn contains_returns_false_when_either_limit_is_nan() {
    for range in [
        JointRange::new(f64::NAN, 1.0).unwrap(),
        JointRange::new(-1.0, f64::NAN).unwrap(),
        JointRange::new(f64::NAN, f64::NAN).unwrap(),
    ] {
        for angle in [-f64::MAX, -1.0, 0.0, 1.0, f64::MAX] {
            assert_eq!(range.contains(angle), Ok(false));
        }
    }
}
