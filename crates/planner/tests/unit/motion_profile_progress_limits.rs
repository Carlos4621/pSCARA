use super::*;

fn limits(
    start: (f64, f64),
    end: (f64, f64),
    velocity: (f64, f64),
    acceleration: (f64, f64),
) -> Result<(f64, f64), PlannerError> {
    MotionProfile::calculate_progress_limits(
        JointAngles::new(start.0, start.1).unwrap(),
        JointAngles::new(end.0, end.1).unwrap(),
        &JointVelocityLimits::new(velocity.0, velocity.1).unwrap(),
        &JointAccelerationLimits::new(acceleration.0, acceleration.1).unwrap(),
    )
}

fn close(actual: f64, expected: f64) {
    assert!(
        actual.is_finite() && actual > 0.0,
        "invalid limit: {actual:e}"
    );
    assert!(
        actual == expected || (actual - expected).abs() <= 8.0 * f64::EPSILON * expected,
        "expected {expected:e}, got {actual:e}"
    );
}

#[test]
fn progress_velocity_and_acceleration_can_be_limited_by_different_joints() {
    // q(s) = start + delta*s: |q'| = |delta|*s', |q''| = |delta|*s''.
    assert_eq!(
        limits((0.0, 0.0), (2.0, 4.0), (6.0, 20.0), (14.0, 8.0)),
        Ok((3.0, 2.0))
    );
    assert_eq!(
        limits((0.0, 0.0), (2.0, 4.0), (10.0, 12.0), (4.0, 28.0)),
        Ok((3.0, 2.0))
    );
}

#[test]
fn each_joint_can_limit_both_quantities_and_ties_are_inclusive() {
    for (velocity, acceleration) in [
        ((6.0, 20.0), (4.0, 28.0)),
        ((10.0, 12.0), (14.0, 8.0)),
        ((6.0, 12.0), (4.0, 8.0)),
    ] {
        assert_eq!(
            limits((0.0, 0.0), (2.0, 4.0), velocity, acceleration),
            Ok((3.0, 2.0))
        );
    }
}

#[test]
fn stationary_joint_imposes_no_constraint_even_with_the_smallest_limits() {
    let tiny = f64::from_bits(1);
    assert_eq!(
        limits((5.0, -3.0), (7.0, -3.0), (6.0, tiny), (4.0, tiny)),
        Ok((3.0, 2.0))
    );
    assert_eq!(
        limits((5.0, -3.0), (5.0, 1.0), (tiny, 12.0), (tiny, 8.0)),
        Ok((3.0, 2.0))
    );
}

#[test]
fn direction_reversal_and_translation_preserve_progress_limits() {
    for first_sign in [-1.0, 1.0] {
        for second_sign in [-1.0, 1.0] {
            for start in [(0.0, 0.0), (32.0, -64.0)] {
                let end = (start.0 + first_sign * 2.0, start.1 + second_sign * 4.0);
                for (from, to) in [(start, end), (end, start)] {
                    assert_eq!(limits(from, to, (6.0, 20.0), (14.0, 8.0)), Ok((3.0, 2.0)));
                }
            }
        }
    }
}

#[test]
fn angular_displacement_is_not_wrapped_modulo_a_revolution() {
    assert_eq!(
        limits((0.0, 0.0), (16.0, -32.0), (48.0, 160.0), (112.0, 64.0)),
        Ok((3.0, 2.0))
    );
}

#[test]
fn swapping_joints_and_their_limits_preserves_the_result() {
    for (start, end, velocity, acceleration) in [
        ((3.0, -2.0), (5.0, -6.0), (6.0, 20.0), (14.0, 8.0)),
        (
            (0.0, 0.0),
            (1e-200, 1e200),
            (1e-200, 1e200),
            (2e-200, 2e200),
        ),
        ((0.0, 0.0), (f64::from_bits(1), 2.0), (1.0, 6.0), (1.0, 4.0)),
    ] {
        let swap = |pair: (f64, f64)| (pair.1, pair.0);
        assert_eq!(
            limits(start, end, velocity, acceleration),
            limits(swap(start), swap(end), swap(velocity), swap(acceleration))
        );
    }
}

#[test]
fn scaling_displacements_inversely_scales_progress_limits() {
    for scale in [1e-200, 0.25, 1.0, 16.0, 1e200] {
        let (velocity, acceleration) = limits(
            (0.0, 0.0),
            (2.0 * scale, 4.0 * scale),
            (6.0, 20.0),
            (14.0, 8.0),
        )
        .unwrap();
        close(velocity, 3.0 / scale);
        close(acceleration, 2.0 / scale);
    }
}

#[test]
fn common_scaling_preserves_limits_across_extreme_finite_scales() {
    for scale in [
        f64::from_bits(1),
        f64::MIN_POSITIVE,
        1e-200,
        1.0,
        1e200,
        1e306,
    ] {
        let (velocity, acceleration) = limits(
            (0.0, 0.0),
            (2.0 * scale, 4.0 * scale),
            (6.0 * scale, 20.0 * scale),
            (14.0 * scale, 8.0 * scale),
        )
        .unwrap();
        close(velocity, 3.0);
        close(acceleration, 2.0);
    }
}

#[test]
fn signed_zero_displacements_are_treated_as_stationary() {
    for zero in [0.0, -0.0] {
        assert_eq!(
            limits((zero, 0.0), (-zero, 4.0), (1.0, 12.0), (1.0, 8.0)),
            Ok((3.0, 2.0))
        );
        assert_eq!(
            limits((0.0, zero), (2.0, -zero), (6.0, 1.0), (4.0, 1.0)),
            Ok((3.0, 2.0))
        );
    }
}

#[test]
fn no_motion_has_no_finite_progress_limit() {
    // With no moving joint both constraints are unbounded. This helper has no
    // stationary-profile sentinel and rejects nonfinite output limits.
    for point in [(0.0, -0.0), (3.0, -7.0), (f64::MAX, -f64::MAX)] {
        assert_eq!(
            limits(point, point, (1.0, 2.0), (3.0, 4.0)),
            Err(PlannerError::NumericalFailure)
        );
    }
}

#[test]
fn overflowed_displacement_is_rejected_for_either_joint_and_direction() {
    for sign in [-1.0, 1.0] {
        let large = sign * f64::MAX;
        for (start, end) in [
            ((large, 0.0), (-large, 1.0)),
            ((0.0, large), (1.0, -large)),
            ((large, large), (-large, -large)),
        ] {
            assert_eq!(
                limits(start, end, (1.0, 1.0), (1.0, 1.0)),
                Err(PlannerError::NumericalFailure)
            );
        }
    }
}

#[test]
fn an_overflowing_ratio_is_nonbinding_when_the_other_joint_has_a_finite_limit() {
    let tiny = f64::from_bits(1);
    assert_eq!(
        limits((0.0, 0.0), (tiny, 2.0), (1.0, 6.0), (1.0, 4.0)),
        Ok((3.0, 2.0))
    );
    assert_eq!(
        limits((0.0, 0.0), (2.0, tiny), (6.0, 1.0), (4.0, 1.0)),
        Ok((3.0, 2.0))
    );
}

#[test]
fn an_unrepresentable_final_limit_is_rejected_for_either_quantity() {
    let tiny = f64::from_bits(1);
    for displacement in [(tiny, 0.0), (0.0, tiny), (tiny, tiny)] {
        for (velocity, acceleration) in [
            ((1.0, 1.0), (tiny, tiny)),
            ((tiny, tiny), (1.0, 1.0)),
            ((1.0, 1.0), (1.0, 1.0)),
        ] {
            assert_eq!(
                limits((0.0, 0.0), displacement, velocity, acceleration),
                Err(PlannerError::NumericalFailure)
            );
        }
    }
}

#[test]
fn smallest_positive_and_largest_finite_progress_limits_are_preserved() {
    for expected in [f64::from_bits(1), f64::MIN_POSITIVE, 1.0, f64::MAX] {
        for displacement in [(1.0, 0.0), (0.0, 1.0), (1.0, 1.0)] {
            assert_eq!(
                limits(
                    (0.0, 0.0),
                    displacement,
                    (expected, expected),
                    (expected, expected)
                ),
                Ok((expected, expected))
            );
        }
    }
}

#[test]
fn first_joint_velocity_underflow_is_rejected() {
    assert_eq!(
        limits((0.0, 0.0), (2.0, 1.0), (f64::from_bits(1), 1.0), (2.0, 1.0)),
        Err(PlannerError::NumericalFailure)
    );
}

#[test]
fn second_joint_velocity_underflow_is_rejected() {
    assert_eq!(
        limits((0.0, 0.0), (1.0, 2.0), (1.0, f64::from_bits(1)), (1.0, 2.0)),
        Err(PlannerError::NumericalFailure)
    );
}

#[test]
fn first_joint_acceleration_underflow_is_rejected() {
    assert_eq!(
        limits((0.0, 0.0), (2.0, 1.0), (2.0, 1.0), (f64::from_bits(1), 1.0)),
        Err(PlannerError::NumericalFailure)
    );
}

#[test]
fn second_joint_acceleration_underflow_is_rejected() {
    assert_eq!(
        limits((0.0, 0.0), (1.0, 2.0), (1.0, 2.0), (1.0, f64::from_bits(1))),
        Err(PlannerError::NumericalFailure)
    );
}

#[test]
fn progress_profile_respects_joint_limits_throughout_the_motion() {
    for (start, end) in [
        ((1.0, -2.0), (3.0, -6.0)),
        ((3.0, -6.0), (1.0, -2.0)),
        ((0.0, 0.0), (0.0, 4.0)),
        ((0.0, 0.0), (2.0, 0.0)),
    ] {
        for (velocity, acceleration) in [((6.0, 20.0), (14.0, 8.0)), ((0.5, 1.0), (4.0, 8.0))] {
            let (max_velocity, max_acceleration) =
                limits(start, end, velocity, acceleration).unwrap();
            let profile = MotionProfile::new(0.0, 1.0, max_acceleration, max_velocity).unwrap();
            assert_eq!(profile.position_at(0.0), Ok(0.0));
            assert_eq!(profile.position_at(profile.total_time()), Ok(1.0));
            let mut previous = 0.0;
            for index in 0..=64 {
                let time = profile.total_time() * (f64::from(index) / 64.0);
                let progress = profile.position_at(time).unwrap();
                assert!(progress >= previous && progress <= 1.0);
                previous = progress;
                for (delta, velocity_limit, acceleration_limit) in [
                    (end.0 - start.0, velocity.0, acceleration.0),
                    (end.1 - start.1, velocity.1, acceleration.1),
                ] {
                    let joint_velocity = delta * profile.velocity_at(time).unwrap();
                    let joint_acceleration = delta * profile.acceleration_at(time).unwrap();
                    assert!(joint_velocity.abs() <= velocity_limit * (1.0 + 8.0 * f64::EPSILON));
                    assert!(
                        joint_acceleration.abs() <= acceleration_limit * (1.0 + 8.0 * f64::EPSILON)
                    );
                }
            }
        }
    }
}
