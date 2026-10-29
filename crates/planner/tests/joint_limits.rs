use core::f64::consts::{PI, TAU};

use kinematics::JointAngles;
use planner::{JointLimits, JointRange, PlannerError};

fn range(min_angle: f64, max_angle: f64) -> JointRange {
    JointRange::new(min_angle, max_angle).expect("test range should be valid")
}

fn angles(first_theta: f64, second_theta: f64) -> JointAngles {
    JointAngles {
        first_theta,
        second_theta,
    }
}

#[test]
fn contains_requires_both_joint_angles_to_be_in_their_independent_ranges() {
    let limits = JointLimits::new(range(-1.0, 1.0), range(10.0, 20.0));

    assert_eq!(limits.contains(angles(0.0, 15.0)), Ok(true));
    assert_eq!(limits.contains(angles(-2.0, 15.0)), Ok(false));
    assert_eq!(limits.contains(angles(2.0, 15.0)), Ok(false));
    assert_eq!(limits.contains(angles(0.0, 9.0)), Ok(false));
    assert_eq!(limits.contains(angles(0.0, 21.0)), Ok(false));
    assert_eq!(limits.contains(angles(-2.0, 9.0)), Ok(false));
    assert_eq!(limits.contains(angles(2.0, 21.0)), Ok(false));
}

#[test]
fn contains_includes_every_combination_of_range_boundaries() {
    let limits = JointLimits::new(range(-PI, PI), range(-1.0, 1.0));

    for first_theta in [-PI, PI] {
        for second_theta in [-1.0, 1.0] {
            assert_eq!(limits.contains(angles(first_theta, second_theta)), Ok(true));
        }
    }
}

#[test]
fn contains_supports_degenerate_ranges_and_adjacent_finite_values() {
    let first = 1.0_f64;
    let second = -1.0_f64;
    let limits = JointLimits::new(range(first, first), range(second, second));

    assert_eq!(limits.contains(angles(first, second)), Ok(true));
    assert_eq!(
        limits.contains(angles(f64::from_bits(first.to_bits() - 1), second)),
        Ok(false)
    );
    assert_eq!(
        limits.contains(angles(first, f64::from_bits(second.to_bits() - 1))),
        Ok(false)
    );
}

#[test]
fn contains_handles_the_largest_and_smallest_finite_angles() {
    let limits = JointLimits::new(
        range(-f64::MAX, f64::MAX),
        range(-f64::MIN_POSITIVE, f64::MIN_POSITIVE),
    );

    for first_theta in [-f64::MAX, -0.0, 0.0, f64::MAX] {
        for second_theta in [
            -f64::MIN_POSITIVE,
            -f64::from_bits(1),
            0.0,
            f64::from_bits(1),
            f64::MIN_POSITIVE,
        ] {
            assert_eq!(limits.contains(angles(first_theta, second_theta)), Ok(true));
        }
    }
}

#[test]
fn contains_does_not_reduce_either_joint_angle_to_one_turn() {
    let limits = JointLimits::new(
        range(TAU + 0.25, TAU + 0.5),
        range(-2.0 * TAU - 0.5, -2.0 * TAU - 0.25),
    );

    assert_eq!(
        limits.contains(angles(TAU + 0.25, -2.0 * TAU - 0.25)),
        Ok(true)
    );
    assert_eq!(limits.contains(angles(0.25, -0.25)), Ok(false));
}

#[test]
fn contains_rejects_a_non_finite_first_angle() {
    let limits = JointLimits::new(range(-1.0, 1.0), range(-1.0, 1.0));

    for invalid in [f64::NEG_INFINITY, f64::INFINITY, f64::NAN, -f64::NAN] {
        for second_theta in [-f64::MAX, 0.0, f64::MAX, f64::NAN] {
            assert_eq!(
                limits.contains(angles(invalid, second_theta)),
                Err(PlannerError::AngleNonFinite)
            );
        }
    }
}

#[test]
fn contains_rejects_a_non_finite_second_angle_when_the_first_is_inside() {
    let limits = JointLimits::new(range(-1.0, 1.0), range(-1.0, 1.0));

    for first_theta in [-1.0, 0.0, 1.0] {
        for invalid in [f64::NEG_INFINITY, f64::INFINITY, f64::NAN, -f64::NAN] {
            assert_eq!(
                limits.contains(angles(first_theta, invalid)),
                Err(PlannerError::AngleNonFinite)
            );
        }
    }
}

#[test]
fn contains_rejects_a_non_finite_second_angle_even_when_the_first_is_outside() {
    let limits = JointLimits::new(range(-1.0, 1.0), range(-1.0, 1.0));

    for first_theta in [-f64::MAX, -2.0, 2.0, f64::MAX] {
        for invalid in [f64::NEG_INFINITY, f64::INFINITY, f64::NAN, -f64::NAN] {
            assert_eq!(
                limits.contains(angles(first_theta, invalid)),
                Err(PlannerError::AngleNonFinite)
            );
        }
    }
}
