use super::limit_reachable_speed;
use crate::{MotionProfile, PlannerError};

fn close(actual: f64, expected: f64) {
    assert!(actual.is_finite() && actual >= 0.0, "{actual:e}");
    assert!(
        actual == expected || (actual - expected).abs() <= 4.0 * f64::EPSILON * expected,
        "expected {expected:e}, got {actual:e}"
    );
}

#[test]
fn analytic_constant_acceleration_cases() {
    for (initial, acceleration, distance, expected) in [
        (0.0, 2.0, 4.0, 4.0),
        (3.0, 2.0, 4.0, 5.0),
        (5.0, 6.0, 12.0, 13.0),
        (8.0, 9.0, 2.0, 10.0),
        (0.0, 0.5, 1.0, 1.0),
    ] {
        assert_eq!(
            limit_reachable_speed(initial, acceleration, distance, f64::MAX),
            Ok(expected)
        );
    }
}

#[test]
fn irrational_reachable_speed_matches_independent_moderate_scale_formula() {
    for initial in [0.0, 0.1, 1.0, 3.0, 100.0] {
        for acceleration in [0.125, 1.0, 3.0, 25.0] {
            for distance in [0.25, 1.0, 7.0, 100.0] {
                let expected = libm::sqrt(initial * initial + 2.0 * acceleration * distance);
                close(
                    limit_reachable_speed(initial, acceleration, distance, f64::MAX).unwrap(),
                    expected,
                );
            }
        }
    }
}

#[test]
fn speed_cap_is_inclusive_at_adjacent_representable_values() {
    for cap in [0.0, f64::from_bits(1), 2.0, 5.0_f64.next_down(), 5.0] {
        assert_eq!(limit_reachable_speed(3.0, 2.0, 4.0, cap), Ok(cap));
    }
    for cap in [5.0_f64.next_up(), 10.0, f64::MAX] {
        assert_eq!(limit_reachable_speed(3.0, 2.0, 4.0, cap), Ok(5.0));
    }
}

#[test]
fn a_lower_cap_limits_the_propagated_bound_even_below_initial_speed() {
    // This helper propagates an upper bound; it does not check braking distance.
    assert_eq!(limit_reachable_speed(5.0, 2.0, 4.0, 3.0), Ok(3.0));
    assert_eq!(limit_reachable_speed(5.0, 2.0, 0.0, 3.0), Ok(3.0));
}

#[test]
fn zero_distance_preserves_initial_speed_without_squaring_it() {
    for initial in [
        0.0,
        f64::from_bits(1),
        f64::MIN_POSITIVE,
        1.0,
        1e200,
        f64::MAX,
    ] {
        for acceleration in [f64::from_bits(1), 1.0, f64::MAX] {
            for zero in [0.0, -0.0] {
                assert_eq!(
                    limit_reachable_speed(initial, acceleration, zero, f64::MAX),
                    Ok(initial)
                );
            }
        }
    }
}

#[test]
fn zero_acceleration_preserves_initial_speed_across_finite_distances() {
    for initial in [0.0, f64::from_bits(1), 1.0, f64::MAX] {
        for distance in [0.0, f64::from_bits(1), 1.0, f64::MAX] {
            for zero in [0.0, -0.0] {
                assert_eq!(
                    limit_reachable_speed(initial, zero, distance, f64::MAX),
                    Ok(initial)
                );
            }
        }
    }
}

#[test]
fn signed_zero_inputs_produce_nonnegative_zero_speed() {
    for initial in [0.0, -0.0] {
        for acceleration in [0.0, -0.0] {
            for distance in [0.0, -0.0] {
                let speed = limit_reachable_speed(initial, acceleration, distance, 1.0).unwrap();
                assert_eq!(speed.to_bits(), 0.0_f64.to_bits());
            }
        }
    }
}

#[test]
fn overflowing_acceleration_distance_product_retains_finite_square_root() {
    // 2*a*d overflows, but its square root is representable.
    assert!((2.0_f64 * 1e200 * 1e200).is_infinite());
    close(
        limit_reachable_speed(0.0, 1e200, 1e200, f64::MAX).unwrap(),
        core::f64::consts::SQRT_2 * 1e200,
    );
    close(
        limit_reachable_speed(0.0, f64::MAX, 0.25, f64::MAX).unwrap(),
        libm::sqrt(f64::MAX / 2.0),
    );
}

#[test]
fn underflowing_acceleration_distance_product_retains_finite_square_root() {
    assert_eq!(2.0_f64 * 1e-200 * 1e-200, 0.0);
    close(
        limit_reachable_speed(0.0, 1e-200, 1e-200, 1.0).unwrap(),
        core::f64::consts::SQRT_2 * 1e-200,
    );
}

#[test]
fn hypot_does_not_square_large_or_tiny_initial_speeds_directly() {
    for exponent in [-1000, -500, 0, 500, 1000] {
        let scale = libm::scalbn(1.0, exponent);
        assert_eq!(
            limit_reachable_speed(3.0 * scale, 8.0 * scale, scale, f64::MAX),
            Ok(5.0 * scale)
        );
    }
}

#[test]
fn subnormal_inputs_and_outputs_are_not_rounded_to_zero_prematurely() {
    let tiny = f64::from_bits(1);
    assert_eq!(limit_reachable_speed(0.0, tiny, tiny, 1.0), Ok(tiny));
    assert_eq!(
        limit_reachable_speed(3.0 * tiny, 8.0 * tiny, tiny, 1.0),
        Ok(5.0 * tiny)
    );
    assert_eq!(limit_reachable_speed(0.0, 1.0, 1.0, tiny), Ok(tiny));
}

#[test]
fn swapping_acceleration_and_distance_preserves_the_speed() {
    for (initial, acceleration, distance, cap) in [
        (3.0, 2.0, 4.0, 10.0),
        (1e200, 1e300, 1e-300, f64::MAX),
        (0.0, f64::from_bits(1), f64::MAX, 1.0),
        (0.0, 1e200, 1e200, 1e100),
    ] {
        assert_eq!(
            limit_reachable_speed(initial, acceleration, distance, cap),
            limit_reachable_speed(initial, distance, acceleration, cap)
        );
    }
}

#[test]
fn reciprocal_scaling_of_acceleration_and_distance_preserves_speed() {
    for exponent in [-1000, -500, 0, 500, 1000] {
        assert_eq!(
            limit_reachable_speed(
                3.0,
                libm::scalbn(2.0, exponent),
                libm::scalbn(4.0, -exponent),
                10.0,
            ),
            Ok(5.0)
        );
    }
}

#[test]
fn increasing_each_nonnegative_input_cannot_lower_the_speed_bound() {
    let values = [0.0, f64::from_bits(1), 1e-200, 0.25, 1.0, 10.0, 1e200];
    for index in 0..4 {
        let mut previous = 0.0;
        for value in values {
            let mut args = [1.0, 1.0, 1.0, 1e300];
            args[index] = value;
            let speed = limit_reachable_speed(args[0], args[1], args[2], args[3]).unwrap();
            assert!(speed >= previous, "input {index}, value {value:e}");
            previous = speed;
        }
    }
}

#[test]
fn nonfinite_arguments_are_rejected_in_every_position() {
    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        for index in 0..4 {
            let mut args = [0.0, 1.0, 1.0, 1.0];
            args[index] = invalid;
            assert_eq!(
                limit_reachable_speed(args[0], args[1], args[2], args[3]),
                Err(PlannerError::NumericalFailure),
                "argument {index}, value {invalid:?}"
            );
        }
    }
}

#[test]
fn negative_arguments_are_rejected_in_every_position() {
    for invalid in [-f64::from_bits(1), -1.0, -f64::MAX] {
        for index in 0..4 {
            let mut args = [0.0, 1.0, 1.0, 1.0];
            args[index] = invalid;
            assert_eq!(
                limit_reachable_speed(args[0], args[1], args[2], args[3]),
                Err(PlannerError::NumericalFailure),
                "argument {index}, value {invalid:e}"
            );
        }
    }
}

#[test]
fn invalid_arguments_are_not_hidden_by_zero_motion_or_a_zero_cap() {
    for baseline in [[0.0; 4], [3.0, 2.0, 0.0, 0.0], [f64::MAX; 4]] {
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -1.0] {
            for index in 0..4 {
                let mut args = baseline;
                args[index] = invalid;
                assert_eq!(
                    limit_reachable_speed(args[0], args[1], args[2], args[3]),
                    Err(PlannerError::NumericalFailure),
                    "argument {index}, args={args:?}"
                );
            }
        }
    }
}

#[test]
fn signed_zero_is_accepted_in_every_argument() {
    for zero in [0.0, -0.0] {
        for (index, expected) in [4.0, 3.0, 3.0, 0.0].into_iter().enumerate() {
            let mut args = [3.0, 2.0, 4.0, 10.0];
            args[index] = zero;
            assert_eq!(
                limit_reachable_speed(args[0], args[1], args[2], args[3]),
                Ok(expected),
                "argument {index}, zero={zero:?}"
            );
        }
    }
}

#[test]
fn square_root_overflow_is_capped_to_a_finite_speed() {
    let theoretical = crate::profile_math::sqrt_product_ratio(&[2.0, f64::MAX, f64::MAX], &[]);
    assert!(theoretical.is_infinite());
    for cap in [
        0.0,
        -0.0,
        f64::from_bits(1),
        f64::MIN_POSITIVE,
        1.0,
        f64::MAX / 2.0,
        f64::MAX.next_down(),
        f64::MAX,
    ] {
        assert_eq!(limit_reachable_speed(0.0, f64::MAX, f64::MAX, cap), Ok(cap));
    }
}

#[test]
fn hypot_overflow_is_capped_even_when_the_square_root_is_finite() {
    for (initial, acceleration, distance) in [
        (f64::MAX, f64::MAX / 2.0, f64::MAX / 2.0),
        (f64::MAX / 2.0, f64::MAX / 2.0, f64::MAX),
    ] {
        let ramp_speed =
            crate::profile_math::sqrt_product_ratio(&[2.0, acceleration, distance], &[]);
        assert!(ramp_speed.is_finite());
        assert!(libm::hypot(initial, ramp_speed).is_infinite());
        for cap in [0.0, f64::from_bits(1), 1.0, f64::MAX / 2.0, f64::MAX] {
            assert_eq!(
                limit_reachable_speed(initial, acceleration, distance, cap),
                Ok(cap)
            );
        }
    }
}

#[test]
fn analytic_results_compose_with_acceleration_and_braking_profiles() {
    for (initial, acceleration, distance) in [(0.0, 2.0, 4.0), (3.0, 2.0, 4.0)] {
        let final_speed = limit_reachable_speed(initial, acceleration, distance, 10.0).unwrap();
        for (from, to) in [(initial, final_speed), (final_speed, initial)] {
            let profile =
                MotionProfile::with_boundary_speeds(0.0, distance, acceleration, 10.0, from, to)
                    .unwrap();
            assert_eq!(profile.velocity_at(0.0), Ok(from));
            assert_eq!(profile.velocity_at(profile.total_time()), Ok(to));
        }
    }
}

#[test]
fn rounded_reachable_speed_must_be_accepted_as_a_profile_boundary() {
    // sqrt(2) rounds upward. A propagated reachable speed must still be
    // accepted by the consumer's acceleration-distance feasibility check.
    let speed = limit_reachable_speed(0.0, 1.0, 1.0, 10.0).unwrap();
    assert_eq!(speed, core::f64::consts::SQRT_2.next_down());
    let profile = MotionProfile::with_boundary_speeds(0.0, 1.0, 1.0, 10.0, 0.0, speed);
    assert!(profile.is_ok(), "speed={speed:.17e}, profile={profile:?}");
}

#[test]
fn reachable_bounds_compose_with_profiles_for_nonzero_initial_speeds() {
    for initial in [0.0, 0.1, 1.0, 3.0, 10.0] {
        for acceleration in [0.125, 0.5, 1.0, 3.0, 25.0] {
            for distance in [0.125, 1.0, 7.0, 100.0] {
                let speed = limit_reachable_speed(initial, acceleration, distance, 100.0).unwrap();
                assert!(speed >= initial && speed <= 100.0);
                for (from, to) in [(initial, speed), (speed, initial)] {
                    let profile = MotionProfile::with_boundary_speeds(
                        0.0,
                        distance,
                        acceleration,
                        100.0,
                        from,
                        to,
                    );
                    assert!(
                        profile.is_ok(),
                        "vi={initial:e}, a={acceleration:e}, d={distance:e}, \
                         from={from:e}, to={to:e}, profile={profile:?}"
                    );
                }
            }
        }
    }
}

#[test]
fn rounding_adjustments_stay_bounded_across_extreme_scales() {
    for exponent in [-1000, -500, 0, 500, 1000] {
        let scale = libm::scalbn(1.0, exponent);
        for initial in [0.0, scale, 3.0 * scale] {
            let theoretical = libm::hypot(initial, core::f64::consts::SQRT_2 * scale);
            let speed = limit_reachable_speed(initial, scale, scale, f64::MAX).unwrap();
            assert!(speed >= initial && speed <= theoretical);
            assert!(theoretical.to_bits() - speed.to_bits() <= 4);
            for (from, to) in [(initial, speed), (speed, initial)] {
                let profile =
                    MotionProfile::with_boundary_speeds(0.0, scale, scale, f64::MAX, from, to);
                assert!(profile.is_ok(), "exponent={exponent}, profile={profile:?}");
            }
        }
    }
}

#[test]
fn caps_below_the_feasible_boundary_are_not_lowered_unnecessarily() {
    let theoretical = core::f64::consts::SQRT_2;
    for cap in [0.0, 1.0, theoretical.next_down(), theoretical, 10.0] {
        let speed = limit_reachable_speed(0.0, 1.0, 1.0, cap).unwrap();
        assert_eq!(speed, cap.min(theoretical.next_down()));
    }
}
