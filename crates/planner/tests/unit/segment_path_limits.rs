use crate::joint_path_velocity_planner::segment_path_limits;
use crate::{JointAccelerationLimits, JointVelocityLimits, MotionProfile, PlannerError};
use kinematics::JointAngles;

fn limits(
    start: (f64, f64),
    end: (f64, f64),
    velocity: (f64, f64),
    acceleration: (f64, f64),
) -> Result<(f64, f64, f64), PlannerError> {
    segment_path_limits(
        JointAngles::new(start.0, start.1).unwrap(),
        JointAngles::new(end.0, end.1).unwrap(),
        &JointVelocityLimits::new(velocity.0, velocity.1).unwrap(),
        &JointAccelerationLimits::new(acceleration.0, acceleration.1).unwrap(),
    )
}

#[test]
fn first_joint_only_ignores_the_stationary_joint_limits() {
    let tiny = f64::from_bits(1);
    assert_eq!(
        limits((1.0, 5.0), (3.0, 5.0), (6.0, tiny), (8.0, tiny)),
        Ok((2.0, 6.0, 8.0))
    );
}

#[test]
fn second_joint_only_ignores_the_stationary_joint_limits() {
    let tiny = f64::from_bits(1);
    assert_eq!(
        limits((5.0, 1.0), (5.0, 5.0), (tiny, 7.0), (tiny, 9.0)),
        Ok((4.0, 7.0, 9.0))
    );
}

#[test]
fn path_length_is_the_largest_absolute_joint_displacement() {
    assert_eq!(
        limits((0.0, 0.0), (3.0, 4.0), (100.0, 100.0), (200.0, 200.0)),
        Ok((4.0, 100.0, 200.0))
    );
}

#[test]
fn a_shorter_second_joint_can_limit_path_velocity() {
    assert_eq!(
        limits((0.0, 0.0), (4.0, 2.0), (20.0, 3.0), (8.0, 20.0)),
        Ok((4.0, 6.0, 8.0))
    );
}

#[test]
fn a_shorter_second_joint_can_limit_path_acceleration() {
    assert_eq!(
        limits((0.0, 0.0), (4.0, 2.0), (20.0, 50.0), (20.0, 3.0)),
        Ok((4.0, 20.0, 6.0))
    );
}

#[test]
fn velocity_and_acceleration_can_be_limited_by_different_joints() {
    for (velocity, acceleration, expected) in [
        ((20.0, 3.0), (5.0, 20.0), (4.0, 6.0, 5.0)),
        ((5.0, 20.0), (20.0, 3.0), (4.0, 5.0, 6.0)),
    ] {
        assert_eq!(
            limits((0.0, 0.0), (4.0, 2.0), velocity, acceleration),
            Ok(expected)
        );
    }
}

#[test]
fn equal_joint_constraints_are_inclusive() {
    assert_eq!(
        limits((0.0, 0.0), (4.0, 2.0), (8.0, 4.0), (12.0, 6.0)),
        Ok((4.0, 8.0, 12.0))
    );
}

#[test]
fn zero_displacement_is_rejected_at_all_finite_scales() {
    for point in [(0.0, -0.0), (3.0, -7.0), (f64::MAX, -f64::MAX)] {
        assert_eq!(
            limits(point, point, (1.0, 2.0), (3.0, 4.0)),
            Err(PlannerError::ZeroDisplacementMotion)
        );
    }
}

#[test]
fn signed_zero_is_stationary_for_either_joint() {
    for zero in [0.0, -0.0] {
        assert_eq!(
            limits((0.0, zero), (2.0, -zero), (6.0, 1.0), (8.0, 1.0)),
            Ok((2.0, 6.0, 8.0))
        );
        assert_eq!(
            limits((zero, 0.0), (-zero, 2.0), (1.0, 6.0), (1.0, 8.0)),
            Ok((2.0, 6.0, 8.0))
        );
    }
}

#[test]
fn reflection_reversal_and_translation_preserve_path_limits() {
    for first_sign in [-1.0, 1.0] {
        for second_sign in [-1.0, 1.0] {
            for start in [(0.0, 0.0), (32.0, -64.0)] {
                let end = (start.0 + first_sign * 4.0, start.1 + second_sign * 2.0);
                for (from, to) in [(start, end), (end, start)] {
                    assert_eq!(
                        limits(from, to, (6.0, 100.0), (8.0, 100.0)),
                        Ok((4.0, 6.0, 8.0))
                    );
                }
            }
        }
    }
}

#[test]
fn swapping_joints_and_their_constraints_preserves_the_result() {
    let original = limits((0.0, 0.0), (4.0, 2.0), (8.0, 3.0), (20.0, 4.0));
    let swapped = limits((0.0, 0.0), (2.0, 4.0), (3.0, 8.0), (4.0, 20.0));
    assert_eq!(original, swapped);
    assert_eq!(original, Ok((4.0, 6.0, 8.0)));
}

#[test]
fn scaling_displacements_changes_length_but_preserves_path_limits() {
    // q(s) = start + delta*s/length: scaling both delta and length
    // preserves the joint-to-path velocity and acceleration ratios.
    for scale in [
        f64::from_bits(1),
        f64::MIN_POSITIVE,
        1e-200,
        0.25,
        1.0,
        1e200,
        1e306,
    ] {
        assert_eq!(
            limits(
                (0.0, 0.0),
                (4.0 * scale, 2.0 * scale),
                (6.0, 100.0),
                (8.0, 100.0)
            ),
            Ok((4.0 * scale, 6.0, 8.0))
        );
    }
}

#[test]
fn scaling_joint_constraints_scales_path_limits() {
    for scale in [
        f64::from_bits(1),
        f64::MIN_POSITIVE,
        1e-200,
        1.0,
        1e200,
        1e300,
    ] {
        assert_eq!(
            limits(
                (0.0, 0.0),
                (4.0, 2.0),
                (6.0 * scale, 100.0 * scale),
                (8.0 * scale, 100.0 * scale),
            ),
            Ok((4.0, 6.0 * scale, 8.0 * scale))
        );
    }
}

#[test]
fn overflowing_products_do_not_destroy_representable_caps() {
    assert!((f64::MAX * (f64::MAX / 4.0)).is_infinite());
    assert_eq!(
        limits(
            (0.0, 0.0),
            (f64::MAX, f64::MAX / 2.0),
            (f64::MAX / 4.0, f64::MAX / 2.0),
            (f64::MAX / 8.0, f64::MAX),
        ),
        Ok((f64::MAX, f64::MAX / 4.0, f64::MAX / 8.0))
    );
}

#[test]
fn underflowing_products_preserve_subnormal_caps() {
    let tiny = f64::from_bits(1);
    assert_eq!((6.0 * tiny) * (4.0 * tiny), 0.0);
    assert_eq!(
        limits(
            (0.0, 0.0),
            (4.0 * tiny, 2.0 * tiny),
            (6.0 * tiny, 100.0 * tiny),
            (8.0 * tiny, 100.0 * tiny)
        ),
        Ok((4.0 * tiny, 6.0 * tiny, 8.0 * tiny))
    );
}

#[test]
fn overflowing_nonbinding_caps_do_not_replace_finite_joint_constraints() {
    assert_eq!(
        limits(
            (0.0, 0.0),
            (2.0, f64::from_bits(1)),
            (6.0, f64::MAX),
            (8.0, f64::MAX)
        ),
        Ok((2.0, 6.0, 8.0))
    );
}

#[test]
fn an_extremely_short_moving_joint_can_still_be_the_bottleneck() {
    assert_eq!(
        limits(
            (0.0, 0.0),
            (1.0, libm::scalbn(1.0, -1000)),
            (4.0, libm::scalbn(1.0, -1001)),
            (8.0, libm::scalbn(1.0, -1002)),
        ),
        Ok((1.0, 0.5, 0.25))
    );
}

#[test]
fn largest_finite_path_length_does_not_require_a_representable_euclidean_norm() {
    assert_eq!(
        limits((0.0, 0.0), (f64::MAX, f64::MAX), (2.0, 1.0), (4.0, 2.0)),
        Ok((f64::MAX, 1.0, 2.0))
    );
}

#[test]
fn first_joint_displacement_overflow_is_rejected_in_both_directions() {
    for sign in [-1.0, 1.0] {
        assert_eq!(
            limits(
                (sign * f64::MAX, 0.0),
                (-sign * f64::MAX, 1.0),
                (1.0, 1.0),
                (1.0, 1.0)
            ),
            Err(PlannerError::NumericalFailure)
        );
    }
}

#[test]
fn second_joint_displacement_overflow_is_rejected_in_both_directions() {
    for sign in [-1.0, 1.0] {
        assert_eq!(
            limits(
                (0.0, sign * f64::MAX),
                (1.0, -sign * f64::MAX),
                (1.0, 1.0),
                (1.0, 1.0)
            ),
            Err(PlannerError::NumericalFailure)
        );
    }
}

#[test]
fn differences_between_adjacent_large_angles_are_preserved() {
    let previous = f64::MAX.next_down();
    let distance = f64::MAX - previous;
    assert_eq!(
        limits(
            (previous, -f64::MAX),
            (f64::MAX, -previous),
            (6.0, 3.0),
            (8.0, 4.0)
        ),
        Ok((distance, 3.0, 4.0))
    );
}

#[test]
fn joint_angles_are_not_reduced_modulo_a_revolution() {
    assert_eq!(
        limits((0.0, 0.0), (16.0, 0.0), (6.0, 1.0), (8.0, 1.0)),
        Ok((16.0, 6.0, 8.0))
    );
}

#[test]
fn independent_integer_grid_oracle_checks_both_joints() {
    let mut mismatches = 0;
    let mut first_mismatch = None;
    for first in -3_i32..=3 {
        for second in -3_i32..=3 {
            for velocity in [(1.0, 4.0), (4.0, 1.0), (2.0, 2.0)] {
                for acceleration in [(1.0, 8.0), (8.0, 1.0), (4.0, 4.0)] {
                    let delta = (f64::from(first), f64::from(second));
                    let length = delta.0.abs().max(delta.1.abs());
                    // Compute each joint's fraction of path length directly;
                    // avoid the production product_ratio helper in this oracle.
                    let expected = if length == 0.0 {
                        Err(PlannerError::ZeroDisplacementMotion)
                    } else {
                        let fractions = (delta.0.abs() / length, delta.1.abs() / length);
                        Ok((
                            length,
                            (velocity.0 / fractions.0).min(velocity.1 / fractions.1),
                            (acceleration.0 / fractions.0).min(acceleration.1 / fractions.1),
                        ))
                    };
                    let actual = limits((0.0, 0.0), delta, velocity, acceleration);
                    // Moderate rational inputs need a relative comparison:
                    // the independently evaluated expressions may round differently.
                    let agrees = match (actual, expected) {
                        (Err(actual), Err(expected)) => actual == expected,
                        (Ok(actual), Ok(expected)) => {
                            actual.0 == expected.0
                                && (actual.1 - expected.1).abs() <= 8.0 * f64::EPSILON * expected.1
                                && (actual.2 - expected.2).abs() <= 8.0 * f64::EPSILON * expected.2
                        }
                        _ => false,
                    };
                    if !agrees {
                        mismatches += 1;
                        first_mismatch.get_or_insert((
                            delta,
                            velocity,
                            acceleration,
                            actual,
                            expected,
                        ));
                    }
                }
            }
        }
    }
    assert_eq!(mismatches, 0, "first mismatch: {first_mismatch:?}");
}

#[test]
fn scalar_path_profile_respects_both_joint_velocity_and_acceleration_limits() {
    let delta: (f64, f64) = (4.0, 2.0);
    let velocity = (20.0, 3.0);
    let acceleration = (20.0, 4.0);
    let (length, path_velocity, path_acceleration) =
        limits((0.0, 0.0), delta, velocity, acceleration).unwrap();
    let profile = MotionProfile::new(0.0, length, path_acceleration, path_velocity).unwrap();
    for step in 0..=100 {
        let time = profile.total_time() * f64::from(step) / 100.0;
        let speed = profile.velocity_at(time).unwrap().abs();
        let accel = profile.acceleration_at(time).unwrap().abs();
        for (joint_delta, max_speed, max_accel) in [
            (delta.0, velocity.0, acceleration.0),
            (delta.1, velocity.1, acceleration.1),
        ] {
            let fraction = joint_delta.abs() / length;
            assert!(speed * fraction <= max_speed * (1.0 + 8.0 * f64::EPSILON));
            assert!(accel * fraction <= max_accel * (1.0 + 8.0 * f64::EPSILON));
        }
    }
}
