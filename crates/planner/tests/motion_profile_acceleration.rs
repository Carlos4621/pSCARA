use planner::{MotionProfile, PlannerError};

fn bounded(value: f64) -> f64 {
    assert!((-10000.0..=10000.0).contains(&value));
    value
}

fn profile(start: f64, end: f64, a: f64, v: f64) -> MotionProfile {
    MotionProfile::new(bounded(start), bounded(end), bounded(a), bounded(v)).unwrap()
}

fn acceleration(motion: &MotionProfile, time: f64) -> f64 {
    motion.acceleration_at(bounded(time)).unwrap()
}

fn assert_close(actual: f64, expected: f64) {
    assert!(actual.is_finite());
    assert!(
        (actual - expected).abs() <= 1e-10 + 128.0 * f64::EPSILON * expected.abs(),
        "expected {expected:e}, got {actual:e}"
    );
}

// Acceleration jumps at phase transitions, so its derivative-based value
// is not unique there. The current API assigns each boundary to the phase
// ending there: +a at the triangular peak, 0 at the end of cruise, and -a
// at total_time for a positive move. Tests document that convention rather
// than assuming a separate zero acceleration at the moving endpoint.
#[test]
fn triangular_acceleration_changes_sign_at_the_peak_in_both_directions() {
    for (start, end, direction) in [(-2.0, 2.0, 1.0), (2.0, -2.0, -1.0)] {
        let motion = profile(start, end, 4.0, 10.0);
        assert_eq!(motion.total_time(), 2.0);
        for (time, magnitude) in [
            (0.0, 4.0),
            (0.25, 4.0),
            (1.0, 4.0),
            (1.25, -4.0),
            (1.75, -4.0),
            (2.0, -4.0),
        ] {
            assert_eq!(acceleration(&motion, time), direction * magnitude);
        }
    }
}

#[test]
fn trapezoidal_acceleration_has_two_signed_ramps_and_zero_cruise() {
    for (start, end, direction) in [(-2.0, 2.0, 1.0), (2.0, -2.0, -1.0)] {
        let motion = profile(start, end, 4.0, 2.0);
        assert_eq!(motion.total_time(), 2.5);
        for (time, magnitude) in [
            (0.0, 4.0),
            (0.25, 4.0),
            (0.5, 4.0),
            (0.625, 0.0),
            (1.5, 0.0),
            (2.0, 0.0),
            (2.125, -4.0),
            (2.5, -4.0),
        ] {
            assert_eq!(acceleration(&motion, time), direction * magnitude);
        }
    }
}

#[test]
fn exact_critical_distance_has_no_spurious_cruise_acceleration() {
    let motion = profile(-0.5, 0.5, 4.0, 2.0);
    assert_eq!(motion.total_time(), 1.0);
    for (time, expected) in [
        (0.0, 4.0),
        (0.375, 4.0),
        (0.5, 4.0),
        (0.625, -4.0),
        (1.0, -4.0),
    ] {
        assert_eq!(acceleration(&motion, time), expected);
    }
}

#[test]
fn stationary_profile_has_zero_acceleration_at_its_only_valid_time() {
    // Ordinary logical regression: keep active even if it fails. There is
    // no floating-point extreme here; a fixed position has zero derivatives.
    for angle in [-10000.0, -12.5, 0.0, 12.5, 10000.0] {
        let motion = profile(angle, angle, 4.0, 2.0);
        assert_eq!(motion.total_time(), 0.0);
        assert_eq!(motion.position_at(0.0), Ok(angle));
        assert_eq!(motion.velocity_at(0.0), Ok(0.0));
        assert_eq!(motion.acceleration_at(0.0), Ok(0.0));
        assert_eq!(motion.acceleration_at(-0.0), Ok(0.0));
    }
}

#[test]
fn time_validation_is_shared_with_position_and_velocity() {
    for (start, end) in [(-2.0, 2.0), (2.0, -2.0), (-1.0, -1.0)] {
        let motion = profile(start, end, 4.0, 2.0);
        for fraction in [0.0, 0.125, 0.5, 0.875, 1.0] {
            let time = bounded(motion.total_time() * fraction);
            assert!(motion.acceleration_at(time).is_ok());
            assert_eq!(motion.validate_time(time), Ok(()));
        }
        for time in [-10000.0, -0.125, motion.total_time() + 0.125, 10000.0] {
            assert_eq!(
                motion.acceleration_at(bounded(time)),
                Err(PlannerError::InvalidTime)
            );
            assert_eq!(motion.velocity_at(time), Err(PlannerError::InvalidTime));
            assert_eq!(motion.position_at(time), Err(PlannerError::InvalidTime));
        }
    }
}

#[test]
fn acceleration_is_the_derivative_of_velocity_inside_every_phase() {
    for (start, end, cap) in [
        (-2.0, 2.0, 10.0),
        (2.0, -2.0, 10.0),
        (-2.0, 2.0, 2.0),
        (2.0, -2.0, 2.0),
    ] {
        let motion = profile(start, end, 4.0, cap);
        for fraction in [0.125, 0.375, 0.625, 0.875] {
            let time = motion.total_time() * fraction;
            let h = 0.0001;
            let before = motion.velocity_at(bounded(time - h)).unwrap();
            let after = motion.velocity_at(bounded(time + h)).unwrap();
            assert_close(acceleration(&motion, time), (after - before) / (2.0 * h));
        }
    }
}

#[test]
fn acceleration_is_the_curvature_of_position_inside_every_phase() {
    for (start, end, cap) in [
        (-2.0, 2.0, 10.0),
        (2.0, -2.0, 10.0),
        (-2.0, 2.0, 2.0),
        (2.0, -2.0, 2.0),
    ] {
        let motion = profile(start, end, 4.0, cap);
        for fraction in [0.125, 0.375, 0.625, 0.875] {
            let time = motion.total_time() * fraction;
            let h = 0.01;
            let before = motion.position_at(bounded(time - h)).unwrap();
            let center = motion.position_at(bounded(time)).unwrap();
            let after = motion.position_at(bounded(time + h)).unwrap();
            let curvature = (after - 2.0 * center + before) / (h * h);
            assert!((acceleration(&motion, time) - curvature).abs() <= 1e-8);
        }
    }
}

#[test]
fn integrating_acceleration_recovers_velocity_throughout_the_move() {
    for (start, end, a, cap) in [
        (-2.0, 2.0, 4.0, 10.0),
        (2.0, -2.0, 4.0, 10.0),
        (-2.0, 2.0, 4.0, 2.0),
        (2.0, -2.0, 4.0, 2.0),
        (-10000.0, 10000.0, 10000.0, 10000.0),
    ] {
        let motion = profile(start, end, a, cap);
        let mut integral = 0.0;
        let mut previous_time = 0.0;
        // These profiles have phase boundaries aligned with 600 intervals.
        // Midpoint quadrature avoids assigning weight to acceleration jumps.
        for i in 1..=600 {
            let time = motion.total_time() * (f64::from(i) / 600.0);
            let midpoint = (previous_time + time) / 2.0;
            integral += acceleration(&motion, midpoint) * (time - previous_time);
            assert!((integral - motion.velocity_at(bounded(time)).unwrap()).abs() <= 1e-8);
            previous_time = time;
        }
    }
}

#[test]
fn reflection_and_translation_preserve_the_expected_acceleration() {
    let original = profile(-2.0, 2.0, 4.0, 2.0);
    let reflected = profile(2.0, -2.0, 4.0, 2.0);
    for offset in [-9000.0, -123.5, 0.0, 123.5, 9000.0] {
        let translated = profile(-2.0 + offset, 2.0 + offset, 4.0, 2.0);
        for i in 0..=64 {
            let time = original.total_time() * (f64::from(i) / 64.0);
            let expected = acceleration(&original, time);
            assert_eq!(acceleration(&translated, time), expected);
            assert_eq!(acceleration(&reflected, time), -expected);
        }
    }
}

#[test]
fn scaling_limits_scales_acceleration_by_the_square_of_the_speed_factor() {
    for cap in [2.0, 10.0] {
        let original = profile(-2.0, 2.0, 4.0, cap);
        for factor in [0.25, 0.5, 1.0, 2.0, 4.0] {
            let scaled = profile(-2.0, 2.0, 4.0 * factor * factor, cap * factor);
            assert_close(scaled.total_time(), original.total_time() / factor);
            for fraction in [0.125, 0.375, 0.625, 0.875] {
                let time = original.total_time() * fraction;
                assert_close(
                    acceleration(&scaled, time / factor),
                    acceleration(&original, time) * factor * factor,
                );
            }
        }
    }
}

#[test]
fn reversing_time_preserves_acceleration_away_from_its_discontinuities() {
    for cap in [2.0, 10.0] {
        let forward = profile(-2.0, 2.0, 4.0, cap);
        let reverse = profile(2.0, -2.0, 4.0, cap);
        // Reversing time swaps left/right limits, so transition instants are
        // deliberately excluded from this symmetry check.
        for fraction in [0.125, 0.375, 0.625, 0.875] {
            let time = forward.total_time() * fraction;
            assert_eq!(
                acceleration(&forward, time),
                acceleration(&reverse, reverse.total_time() - time)
            );
        }
    }
}

#[test]
fn acceleration_queries_are_repeatable_after_rejected_and_out_of_order_times() {
    let motion = profile(-2.0, 2.0, 4.0, 2.0);
    for (time, expected) in [
        (2.25, -4.0),
        (0.25, 4.0),
        (1.25, 0.0),
        (2.5, -4.0),
        (0.0, 4.0),
        (2.25, -4.0),
    ] {
        assert_eq!(motion.acceleration_at(-1.0), Err(PlannerError::InvalidTime));
        assert_eq!(acceleration(&motion, time), expected);
        assert_eq!(motion.position_at(2.5), Ok(2.0));
        assert_eq!(motion.velocity_at(2.5), Ok(0.0));
    }
}

// IEEE boundary checks are ignored as requested. They verify expected
// discontinuity/validation behavior, not a claimed implementation defect.
#[test]
#[ignore = "IEEE transition boundary: probes adjacent f64 instants around the discontinuous triangular peak; checks the documented left-phase convention"]
fn adjacent_float_times_straddle_the_triangular_acceleration_jump() {
    let motion = MotionProfile::new(-2.0, 2.0, 4.0, 10.0).unwrap();
    let below = f64::from_bits(1.0_f64.to_bits() - 1);
    let above = f64::from_bits(1.0_f64.to_bits() + 1);
    assert_eq!(motion.acceleration_at(below), Ok(4.0));
    assert_eq!(motion.acceleration_at(1.0), Ok(4.0));
    assert_eq!(motion.acceleration_at(above), Ok(-4.0));
}

#[test]
#[ignore = "IEEE validation boundary: NaN and infinities must return InvalidTime, including for a stationary profile"]
fn nonfinite_times_are_rejected_by_acceleration_queries() {
    for (start, end) in [(-2.0, 2.0), (2.0, -2.0), (-1.0, -1.0)] {
        let motion = MotionProfile::new(start, end, 4.0, 2.0).unwrap();
        for time in [f64::NAN, -f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert_eq!(motion.acceleration_at(time), Err(PlannerError::InvalidTime));
        }
    }
}
