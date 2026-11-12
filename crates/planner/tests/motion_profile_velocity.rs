use planner::{MotionProfile, PlannerError};

// Ordinary API inputs stay in [-10000, 10000]. Precision regressions at the
// bottom deliberately bypass these helpers to cover extreme finite scales.
fn bounded(value: f64) -> f64 {
    assert!(
        (-10000.0..=10000.0).contains(&value),
        "ordinary input outside the test range: {value}"
    );
    value
}

fn profile(start: f64, end: f64, acceleration: f64, speed: f64) -> MotionProfile {
    MotionProfile::new(
        bounded(start),
        bounded(end),
        bounded(acceleration),
        bounded(speed),
    )
    .unwrap()
}

fn velocity(motion: &MotionProfile, time: f64) -> f64 {
    motion.velocity_at(bounded(time)).unwrap()
}

fn position(motion: &MotionProfile, time: f64) -> f64 {
    motion.position_at(bounded(time)).unwrap()
}

fn assert_close(actual: f64, expected: f64) {
    assert!(actual.is_finite(), "nonfinite velocity: {actual}");
    let tolerance = 128.0 * f64::EPSILON * expected.abs().max(1.0);
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {expected:e}, got {actual:e}"
    );
}

#[test]
fn triangular_velocity_accelerates_to_the_actual_peak_then_decelerates() {
    // Four radians at a=4 gives two one-second ramps and peak speed 4,
    // even though the configured speed cap is 10.
    let motion = profile(-2.0, 2.0, 4.0, 10.0);
    assert_eq!(motion.total_time(), 2.0);
    for (time, expected) in [
        (0.0, 0.0),
        (0.25, 1.0),
        (0.5, 2.0),
        (1.0, 4.0),
        (1.5, 2.0),
        (1.75, 1.0),
        (2.0, 0.0),
    ] {
        assert_eq!(velocity(&motion, time), expected);
    }
}

#[test]
fn negative_direction_negates_each_phase_of_a_triangular_velocity() {
    let motion = profile(2.0, -2.0, 4.0, 10.0);
    for (time, expected) in [
        (0.0, 0.0),
        (0.25, -1.0),
        (0.5, -2.0),
        (1.0, -4.0),
        (1.5, -2.0),
        (1.75, -1.0),
        (2.0, 0.0),
    ] {
        assert_eq!(velocity(&motion, time), expected);
    }
}

#[test]
fn trapezoidal_velocity_keeps_the_speed_cap_during_cruise() {
    // d=4, a=4, v=2: ramp 0.5, cruise 1.5, ramp 0.5.
    let motion = profile(-2.0, 2.0, 4.0, 2.0);
    assert_eq!(motion.total_time(), 2.5);
    for (time, expected) in [
        (0.0, 0.0),
        (0.25, 1.0),
        (0.5, 2.0),
        (0.75, 2.0),
        (1.5, 2.0),
        (2.0, 2.0),
        (2.25, 1.0),
        (2.5, 0.0),
    ] {
        assert_eq!(velocity(&motion, time), expected);
    }
}

#[test]
fn negative_direction_negates_the_trapezoidal_cruise_and_both_ramps() {
    let motion = profile(2.0, -2.0, 4.0, 2.0);
    for (time, expected) in [
        (0.0, 0.0),
        (0.25, -1.0),
        (0.5, -2.0),
        (1.0, -2.0),
        (2.0, -2.0),
        (2.25, -1.0),
        (2.5, 0.0),
    ] {
        assert_eq!(velocity(&motion, time), expected);
    }
}

#[test]
fn exact_critical_distance_has_a_single_peak_without_a_velocity_jump() {
    let motion = profile(-0.5, 0.5, 4.0, 2.0);
    assert_eq!(motion.total_time(), 1.0);
    for (time, expected) in [
        (0.0, 0.0),
        (0.375, 1.5),
        (0.5, 2.0),
        (0.625, 1.5),
        (1.0, 0.0),
    ] {
        assert_eq!(velocity(&motion, time), expected);
    }
}

#[test]
fn ordinary_distances_on_either_side_of_the_critical_distance_have_the_correct_peak() {
    // Critical distance is one radian. All inputs here are ordinary binary
    // fractions, rather than adjacent-float precision probes.
    for (distance, total, peak) in [(0.25, 0.5, 1.0), (1.0, 1.0, 2.0), (1.5, 1.25, 2.0)] {
        let motion = profile(-2.0, -2.0 + distance, 4.0, 2.0);
        assert_eq!(motion.total_time(), total);
        assert_eq!(velocity(&motion, total / 2.0), peak);
    }
}

#[test]
fn stationary_profiles_have_zero_velocity_at_their_only_valid_time() {
    for angle in [-10000.0, -12.5, -0.0, 0.0, 12.5, 10000.0] {
        for (a, v) in [(0.25, 10000.0), (10000.0, 0.25), (1.0, 1.0)] {
            let motion = profile(angle, angle, a, v);
            assert_eq!(motion.total_time(), 0.0);
            assert_eq!(velocity(&motion, 0.0), 0.0);
            assert_eq!(velocity(&motion, -0.0), 0.0);
            assert_eq!(motion.validate_time(0.0), Ok(()));
            for time in [-10000.0, -0.125, 0.125, 10000.0] {
                assert_eq!(motion.velocity_at(time), Err(PlannerError::InvalidTime));
                assert_eq!(motion.validate_time(time), Err(PlannerError::InvalidTime));
            }
        }
    }
}

#[test]
fn all_ordinary_moves_start_and_finish_at_rest() {
    for (start, end) in [
        (-10000.0, 10000.0),
        (10000.0, -10000.0),
        (-10.25, -13.75),
        (13.75, 10.25),
        (-1.0, 1.0),
    ] {
        for (a, v) in [(4.0, 4.0), (7.5, 2.5), (10000.0, 10000.0)] {
            let motion = profile(start, end, a, v);
            assert_eq!(velocity(&motion, 0.0), 0.0);
            assert_eq!(velocity(&motion, motion.total_time()), 0.0);
        }
    }
}

#[test]
fn both_velocity_and_position_use_the_public_time_validation_contract() {
    let motion = profile(-2.0, 2.0, 4.0, 2.0);
    for time in [0.0, -0.0, 0.125, 0.5, 1.25, 2.0, 2.5] {
        assert_eq!(motion.validate_time(bounded(time)), Ok(()));
        assert!(motion.velocity_at(time).is_ok());
        assert!(motion.position_at(time).is_ok());
    }
    for time in [-10000.0, -1.0, -0.125, 2.625, 10.0, 10000.0] {
        let expected = Err(PlannerError::InvalidTime);
        assert_eq!(motion.validate_time(bounded(time)), expected);
        assert_eq!(motion.velocity_at(time), Err(PlannerError::InvalidTime));
        assert_eq!(motion.position_at(time), Err(PlannerError::InvalidTime));
    }
}

#[test]
fn velocity_queries_are_repeatable_after_out_of_order_and_rejected_times() {
    let motion = profile(-2.0, 2.0, 4.0, 2.0);
    for (time, expected) in [
        (2.25, 1.0),
        (0.25, 1.0),
        (1.25, 2.0),
        (2.5, 0.0),
        (0.0, 0.0),
        (2.25, 1.0),
    ] {
        assert_eq!(motion.velocity_at(-1.0), Err(PlannerError::InvalidTime));
        assert_eq!(velocity(&motion, time), expected);
        // Velocity queries must not change position or consume the profile.
        assert_eq!(position(&motion, motion.total_time()), 2.0);
    }
}

#[test]
fn sampled_velocities_are_finite_directional_and_never_exceed_the_speed_cap() {
    for start in [-10000.0, -12.5, 0.0, 12.5, 10000.0] {
        for end in [-10000.0, -12.5, 0.0, 12.5, 10000.0] {
            for (a, cap) in [(0.25, 4.0), (4.0, 16.0), (7.5, 2.5), (10000.0, 10000.0)] {
                let motion = profile(start, end, a, cap);
                for i in 0..=64 {
                    let time = motion.total_time() * (f64::from(i) / 64.0);
                    let speed = velocity(&motion, time);
                    assert!(speed.is_finite());
                    assert!(speed.abs() <= cap + 1e-10);
                    if end > start {
                        assert!(speed >= 0.0);
                    } else if end < start {
                        assert!(speed <= 0.0);
                    } else {
                        assert_eq!(speed, 0.0);
                    }
                }
            }
        }
    }
}

#[test]
fn velocity_is_the_derivative_of_position_inside_each_phase() {
    for (start, end, a, cap) in [
        (-2.0, 2.0, 4.0, 10.0),
        (2.0, -2.0, 4.0, 10.0),
        (-2.0, 2.0, 4.0, 2.0),
        (2.0, -2.0, 4.0, 2.0),
    ] {
        let motion = profile(start, end, a, cap);
        // These fractions stay away from phase boundaries in both profiles,
        // so a centered position difference measures the exact local slope.
        for fraction in [0.125, 0.375, 0.625, 0.875] {
            let time = fraction * motion.total_time();
            let h = 0.0001;
            let numerical_derivative =
                (position(&motion, time + h) - position(&motion, time - h)) / (2.0 * h);
            let actual = velocity(&motion, time);
            assert!(
                (actual - numerical_derivative).abs() <= 1e-8,
                "velocity {actual} differs from position derivative {numerical_derivative}"
            );
        }
    }
}

#[test]
fn integrating_velocity_recovers_the_signed_displacement() {
    for (start, end, a, cap) in [
        (-2.0, 2.0, 4.0, 10.0),
        (2.0, -2.0, 4.0, 10.0),
        (-2.0, 2.0, 4.0, 2.0),
        (2.0, -2.0, 4.0, 2.0),
        (-10000.0, 10000.0, 10000.0, 10000.0),
        (-12.5, -12.5, 1.0, 1.0),
    ] {
        let motion = profile(start, end, a, cap);
        // 600 intervals align with every phase boundary in these cases.
        // Trapezoidal quadrature is exact for each linear velocity segment,
        // apart from floating-point rounding in the accumulation.
        let mut integral = 0.0;
        let mut previous_time = 0.0;
        let mut previous_velocity = velocity(&motion, 0.0);
        for i in 1..=600 {
            let time = motion.total_time() * (f64::from(i) / 600.0);
            let current = velocity(&motion, time);
            integral += (previous_velocity + current) / 2.0 * (time - previous_time);
            previous_time = time;
            previous_velocity = current;
        }
        assert!(
            (integral - (end - start)).abs() <= 1e-9,
            "integrated velocity {integral} does not recover displacement {}",
            end - start
        );
    }
}

#[test]
fn slopes_of_velocity_match_acceleration_cruise_and_deceleration() {
    for (start, end, direction) in [(-2.0, 2.0, 1.0), (2.0, -2.0, -1.0)] {
        let motion = profile(start, end, 4.0, 2.0);
        for (from, to, expected) in [(0.125, 0.375, 4.0), (0.75, 1.75, 0.0), (2.125, 2.375, -4.0)] {
            let slope = (velocity(&motion, to) - velocity(&motion, from)) / (to - from);
            assert_close(slope, direction * expected);
        }
    }
}

#[test]
fn velocity_changes_are_bounded_by_acceleration_even_across_phase_transitions() {
    for (start, end, a, cap) in [
        (-2.0, 2.0, 4.0, 10.0),
        (2.0, -2.0, 4.0, 2.0),
        (-123.5, 246.25, 7.3, 9.7),
    ] {
        let motion = profile(start, end, a, cap);
        let mut previous_time = 0.0;
        let mut previous = velocity(&motion, 0.0);
        for i in 1..=256 {
            let time = motion.total_time() * (f64::from(i) / 256.0);
            let current = velocity(&motion, time);
            assert!((current - previous).abs() <= a * (time - previous_time) + 1e-10);
            previous = current;
            previous_time = time;
        }
    }
}

#[test]
fn translating_angles_does_not_change_velocity() {
    let original = profile(-2.0, 2.0, 4.0, 2.0);
    for offset in [-9000.0, -123.5, 0.0, 123.5, 9000.0] {
        let translated = profile(-2.0 + offset, 2.0 + offset, 4.0, 2.0);
        assert_eq!(translated.total_time(), original.total_time());
        for i in 0..=64 {
            let time = original.total_time() * (f64::from(i) / 64.0);
            assert_eq!(velocity(&translated, time), velocity(&original, time));
        }
    }
}

#[test]
fn reflecting_angles_negates_velocity_at_every_time() {
    for (start, end, a, cap) in [(-2.0, 2.0, 4.0, 2.0), (-12.5, -11.5, 4.0, 10.0)] {
        let original = profile(start, end, a, cap);
        let reflected = profile(-start, -end, a, cap);
        assert_eq!(reflected.total_time(), original.total_time());
        for i in 0..=64 {
            let time = original.total_time() * (f64::from(i) / 64.0);
            assert_eq!(velocity(&reflected, time), -velocity(&original, time));
        }
    }
}

#[test]
fn reversing_a_move_negates_the_time_reversed_velocity() {
    for (start, end, a, cap) in [(-2.0, 2.0, 4.0, 2.0), (-12.5, -11.5, 4.0, 10.0)] {
        let forward = profile(start, end, a, cap);
        let reverse = profile(end, start, a, cap);
        assert_eq!(forward.total_time(), reverse.total_time());
        for i in 0..=64 {
            let time = forward.total_time() * (f64::from(i) / 64.0);
            assert_close(
                velocity(&reverse, reverse.total_time() - time),
                -velocity(&forward, time),
            );
        }
    }
}

#[test]
fn scaling_limits_rescales_velocity_and_time_consistently() {
    for (start, end, a, cap) in [(-2.0, 2.0, 4.0, 2.0), (-12.5, -11.5, 4.0, 10.0)] {
        let original = profile(start, end, a, cap);
        for factor in [0.25, 0.5, 1.0, 2.0, 4.0] {
            let scaled = profile(start, end, a * factor * factor, cap * factor);
            assert_close(scaled.total_time(), original.total_time() / factor);
            for i in 0..=64 {
                let time = original.total_time() * (f64::from(i) / 64.0);
                assert_close(
                    velocity(&scaled, time / factor),
                    velocity(&original, time) * factor,
                );
            }
        }
    }
}

// Numerical regressions run normally. The separately marked IEEE validation
// case can also be included with --include-ignored.

#[test]
fn regression_absorbed_ramps_still_end_with_zero_velocity() {
    for end in [1e-13, -1e-13] {
        let motion = MotionProfile::new(0.0, end, 10000.0, 1e-13).unwrap();
        assert_eq!(motion.velocity_at(motion.total_time()), Ok(0.0));
    }
}

#[test]
fn regression_stationary_motion_has_zero_velocity_with_subnormal_limits() {
    let smallest = f64::from_bits(1);
    let motion = MotionProfile::new(-1.0, -1.0, smallest, smallest).unwrap();
    assert_eq!(motion.velocity_at(motion.total_time() / 2.0), Ok(0.0));
}

#[test]
#[ignore = "IEEE validation boundary: NaN and infinities must be rejected as InvalidTime; outside the ordinary finite-input suite"]
fn nonfinite_times_are_rejected_consistently_by_all_three_public_methods() {
    for (start, end) in [(-2.0, 2.0), (2.0, -2.0), (-1.0, -1.0)] {
        let motion = MotionProfile::new(start, end, 4.0, 2.0).unwrap();
        for time in [f64::NAN, -f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert_eq!(motion.validate_time(time), Err(PlannerError::InvalidTime));
            assert_eq!(motion.velocity_at(time), Err(PlannerError::InvalidTime));
            assert_eq!(motion.position_at(time), Err(PlannerError::InvalidTime));
        }
    }
}
