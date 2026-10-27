use super::*;
use core::f64::consts::{PI, TAU};

const TOLERANCE: f64 = 1e-12;

fn assert_close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() <= TOLERANCE,
        "expected {expected}, got {actual}"
    );
}

fn unwrap_angle(actual: f64, next: f64) -> f64 {
    JointTrajectory::unwrap_angle(actual, next).expect("finite angles should unwrap")
}

#[test]
fn unwrap_angle_keeps_values_that_already_use_the_nearest_turn() {
    for (actual, next) in [
        (-PI, -PI),
        (-2.0, -1.0),
        (-0.0, 0.0),
        (0.0, f64::from_bits(1)),
        (1.0, 2.0),
        (PI, PI),
    ] {
        assert_close(unwrap_angle(actual, next), next);
    }
}

#[test]
fn unwrap_angle_crosses_both_sides_of_the_pi_discontinuity_continuously() {
    let margin = 0.125;

    assert_close(unwrap_angle(PI - margin, -PI + margin), PI + margin);
    assert_close(unwrap_angle(-PI + margin, PI - margin), -PI - margin);
}

#[test]
fn unwrap_angle_removes_any_number_of_complete_turns() {
    for actual in [-10.25, -PI, -0.25, 0.0, 0.25, PI, 10.25] {
        for offset in [-2.5, -1.0, -0.25, 0.0, 0.25, 1.0, 2.5] {
            let nearest = actual + offset;
            for turns in [-1_000.0, -10.0, -1.0, 0.0, 1.0, 10.0, 1_000.0] {
                assert_close(unwrap_angle(actual, nearest + turns * TAU), nearest);
            }
        }
    }
}

#[test]
fn unwrap_angle_returns_an_equivalent_angle_with_a_bounded_step() {
    let values = [-100.0, -10.0, -PI, -0.1, 0.0, 0.1, PI, 10.0, 100.0];

    for actual in values {
        for next in values {
            let result = unwrap_angle(actual, next);
            assert!((result - actual).abs() <= PI);
            assert_close(libm::sin(result), libm::sin(next));
            assert_close(libm::cos(result), libm::cos(next));
        }
    }
}

#[test]
fn unwrap_angle_preserves_equal_extreme_and_subnormal_values() {
    for value in [
        -f64::MAX,
        -f64::MIN_POSITIVE,
        -f64::from_bits(1),
        f64::from_bits(1),
        f64::MIN_POSITIVE,
        f64::MAX,
    ] {
        assert_eq!(unwrap_angle(value, value), value);
    }
}

#[test]
fn unwrap_angle_rejects_every_non_finite_operand() {
    for invalid in [f64::NEG_INFINITY, f64::INFINITY, f64::NAN, -f64::NAN] {
        assert_eq!(
            JointTrajectory::unwrap_angle(invalid, 0.0),
            Err(PlannerError::AngleNonFinite)
        );
        assert_eq!(
            JointTrajectory::unwrap_angle(0.0, invalid),
            Err(PlannerError::AngleNonFinite)
        );
        assert_eq!(
            JointTrajectory::unwrap_angle(invalid, invalid),
            Err(PlannerError::AngleNonFinite)
        );
    }
}

#[test]
fn unwrap_angle_rejects_a_non_finite_difference_between_finite_operands() {
    for (actual, next) in [(f64::MAX, -f64::MAX), (-f64::MAX, f64::MAX)] {
        assert_eq!(
            JointTrajectory::unwrap_angle(actual, next),
            Err(PlannerError::AngleNonFinite)
        );
    }
}

#[test]
fn unwrap_joint_angles_unwraps_both_joints_independently() {
    let margin = 0.125;
    let actual = JointAngles {
        first_theta: PI - margin,
        second_theta: -PI + margin,
    };
    let next = JointAngles {
        first_theta: -PI + margin,
        second_theta: PI - margin,
    };

    let result = JointTrajectory::unwrap_joint_angles(actual, next).unwrap();
    assert_close(result.first_theta, PI + margin);
    assert_close(result.second_theta, -PI - margin);
}

#[test]
fn unwrap_joint_angles_rejects_non_finite_components_in_either_sample() {
    let finite = JointAngles {
        first_theta: 0.25,
        second_theta: -0.5,
    };

    for invalid in [f64::NEG_INFINITY, f64::INFINITY, f64::NAN] {
        for invalid_angles in [
            JointAngles {
                first_theta: invalid,
                second_theta: finite.second_theta,
            },
            JointAngles {
                first_theta: finite.first_theta,
                second_theta: invalid,
            },
        ] {
            assert!(matches!(
                JointTrajectory::unwrap_joint_angles(invalid_angles, finite),
                Err(PlannerError::AngleNonFinite)
            ));
            assert!(matches!(
                JointTrajectory::unwrap_joint_angles(finite, invalid_angles),
                Err(PlannerError::AngleNonFinite)
            ));
        }
    }
}
