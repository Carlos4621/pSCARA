use core::f64::consts::TAU;

use kinematics::JointAngles;
use planner::{JointAccelerationLimits, JointVelocityLimits, MotionProfile, PlannerError};

// The original cases use [0, 1000]. signed_profile extends ordinary angles
// to [-1000, 1000]; positive limits and valid query times remain in [0, 1000].
// Invalid negative limits and times are passed directly in rejection tests.
fn bounded(value: f64) -> f64 {
    assert!(
        (0.0..=1000.0).contains(&value),
        "out-of-scope input: {value}"
    );
    value
}

fn profile(start: f64, end: f64, acceleration: f64, velocity: f64) -> MotionProfile {
    MotionProfile::new(
        bounded(start),
        bounded(end),
        bounded(acceleration),
        bounded(velocity),
    )
    .unwrap()
}

fn signed_profile(start: f64, end: f64, acceleration: f64, velocity: f64) -> MotionProfile {
    assert!((-1000.0..=1000.0).contains(&start));
    assert!((-1000.0..=1000.0).contains(&end));
    MotionProfile::new(start, end, bounded(acceleration), bounded(velocity)).unwrap()
}

fn position(profile: &MotionProfile, time: f64) -> f64 {
    profile.position_at(bounded(time)).unwrap()
}

fn assert_close(actual: f64, expected: f64) {
    assert!(actual.is_finite(), "nonfinite result: {actual}");
    let tolerance = 64.0 * f64::EPSILON * expected.abs().max(1.0);
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {expected:.17e}, got {actual:.17e} (tolerance {tolerance:e})"
    );
}

#[test]
fn new_accepts_positive_limits_and_angles_across_the_requested_range() {
    for start in [0.0, 0.25, 37.5, 1000.0] {
        for end in [0.0, 0.25, 37.5, 1000.0] {
            for acceleration in [0.25, 1.0, 7.5, 1000.0] {
                for velocity in [0.25, 1.0, 7.5, 1000.0] {
                    let _ = profile(start, end, acceleration, velocity);
                }
            }
        }
    }
}

#[test]
fn zero_acceleration_is_rejected() {
    for velocity in [0.25, 1.0, 1000.0] {
        assert_eq!(
            MotionProfile::new(10.0, 20.0, 0.0, velocity).unwrap_err(),
            PlannerError::InvalidMaxAcceleration
        );
    }
}

#[test]
fn zero_velocity_is_rejected() {
    for acceleration in [0.25, 1.0, 1000.0] {
        assert_eq!(
            MotionProfile::new(10.0, 20.0, acceleration, 0.0).unwrap_err(),
            PlannerError::InvalidMaxVelocity
        );
    }
}

#[test]
fn invalid_acceleration_takes_precedence_when_both_limits_are_zero() {
    assert_eq!(
        MotionProfile::new(10.0, 20.0, 0.0, 0.0).unwrap_err(),
        PlannerError::InvalidMaxAcceleration
    );
}

#[test]
fn stationary_profiles_have_zero_duration_and_preserve_the_position() {
    for angle in [0.0, 0.125, 123.75, 1000.0] {
        for (a, v) in [(0.25, 1000.0), (1000.0, 0.25), (1.0, 1.0)] {
            let motion = profile(angle, angle, a, v);
            assert_eq!(motion.total_time(), 0.0);
            assert_eq!(position(&motion, 0.0), angle);
            assert_eq!(motion.position_at(0.25), Err(PlannerError::InvalidTime));
        }
    }
}

#[test]
fn time_zero_preserves_the_start_for_both_directions_and_nonunit_distances() {
    for (start, end) in [(0.0, 1000.0), (1000.0, 0.0), (10.25, 13.75), (13.75, 10.25)] {
        assert_eq!(position(&profile(start, end, 2.5, 1.75), 0.0), start);
    }
}

#[test]
fn forward_unit_triangle_matches_integrated_acceleration_at_every_phase() {
    let motion = profile(10.0, 11.0, 4.0, 4.0);
    assert_close(motion.total_time(), 1.0);
    for (time, expected) in [
        (0.0, 10.0),
        (0.25, 10.125),
        (0.5, 10.5),
        (0.75, 10.875),
        (1.0, 11.0),
    ] {
        assert_close(position(&motion, time), expected);
    }
}

#[test]
fn reverse_unit_triangle_moves_towards_the_smaller_angle() {
    let motion = profile(11.0, 10.0, 4.0, 4.0);
    assert_close(motion.total_time(), 1.0);
    for (time, expected) in [
        (0.0, 11.0),
        (0.25, 10.875),
        (0.5, 10.5),
        (0.75, 10.125),
        (1.0, 10.0),
    ] {
        assert_close(position(&motion, time), expected);
    }
}

#[test]
fn forward_unit_trapezoid_matches_both_ramps_and_the_cruise() {
    let motion = profile(10.0, 11.0, 4.0, 1.0);
    assert_close(motion.total_time(), 1.25);
    for (time, expected) in [
        (0.0, 10.0),
        (0.125, 10.03125),
        (0.25, 10.125),
        (0.5, 10.375),
        (1.0, 10.875),
        (1.125, 10.96875),
        (1.25, 11.0),
    ] {
        assert_close(position(&motion, time), expected);
    }
}

#[test]
fn reverse_unit_trapezoid_matches_both_ramps_and_the_cruise() {
    let motion = profile(11.0, 10.0, 4.0, 1.0);
    assert_close(motion.total_time(), 1.25);
    for (time, expected) in [
        (0.0, 11.0),
        (0.125, 10.96875),
        (0.25, 10.875),
        (0.5, 10.625),
        (1.0, 10.125),
        (1.125, 10.03125),
        (1.25, 10.0),
    ] {
        assert_close(position(&motion, time), expected);
    }
}

#[test]
fn forward_nonunit_distance_reaches_the_requested_endpoint() {
    let motion = profile(10.0, 14.0, 4.0, 4.0);
    assert_close(position(&motion, motion.total_time()), 14.0);
}

#[test]
fn reverse_nonunit_distance_reaches_the_requested_endpoint() {
    let motion = profile(14.0, 10.0, 4.0, 4.0);
    assert_close(position(&motion, motion.total_time()), 10.0);
}

#[test]
fn upper_angle_boundary_reaches_one_thousand_without_overshooting() {
    let motion = profile(0.0, 1000.0, 1000.0, 1000.0);
    assert_close(motion.total_time(), 2.0);
    assert_close(position(&motion, 1.0), 500.0);
    assert_close(position(&motion, 2.0), 1000.0);
}

#[test]
fn ordinary_inputs_select_a_trapezoid_when_the_speed_cap_is_reached() {
    // d=1, a=16, v=2: critical distance is 0.25. Ramps take
    // 0.125 each, and cruising for the remaining 0.75 takes 0.375.
    let motion = profile(10.0, 11.0, 16.0, 2.0);
    assert_close(motion.total_time(), 0.625);
}

#[test]
fn ordinary_inputs_select_a_triangle_when_the_speed_cap_is_not_reached() {
    // d=18, a=2, v=8: ramp time 3, peak speed 6, no cruise.
    let motion = profile(10.0, 28.0, 2.0, 8.0);
    assert_close(motion.total_time(), 6.0);
}

#[test]
fn exact_critical_distance_has_two_ramps_and_no_cruise() {
    let motion = profile(10.0, 11.0, 4.0, 2.0);
    assert_close(motion.total_time(), 1.0);
    assert_close(position(&motion, 0.5), 10.5);
    assert_close(position(&motion, 1.0), 11.0);
}

#[test]
fn distances_on_both_sides_of_the_critical_distance_use_the_correct_duration() {
    // a=4, v=2 gives critical distance 1. These are ordinary,
    // exactly representable inputs, rather than adjacent-float probes.
    for (distance, expected) in [(0.25, 0.5), (1.0, 1.0), (1.5, 1.25)] {
        assert_close(
            profile(10.0, 10.0 + distance, 4.0, 2.0).total_time(),
            expected,
        );
    }
}

#[test]
fn fractional_nonboundary_inputs_have_the_analytical_trapezoidal_duration() {
    // d/v + v/a = 3.5/1.75 + 1.75/2.5 = 2.7.
    assert_close(profile(10.25, 13.75, 2.5, 1.75).total_time(), 2.7);
}

#[test]
fn time_after_the_endpoint_is_rejected_within_the_requested_input_range() {
    let motion = profile(10.0, 11.0, 4.0, 4.0);
    assert!(motion.position_at(bounded(motion.total_time())).is_ok());
    for time in [1.125, 10.0, 1000.0] {
        assert_eq!(motion.position_at(time), Err(PlannerError::InvalidTime));
    }
}

#[test]
fn queries_are_repeatable_and_do_not_require_increasing_time() {
    let motion = profile(10.0, 11.0, 4.0, 4.0);
    for (time, expected) in [
        (0.75, 10.875),
        (0.25, 10.125),
        (1.0, 11.0),
        (0.0, 10.0),
        (0.75, 10.875),
    ] {
        assert_close(position(&motion, time), expected);
    }
}

#[test]
fn duration_is_unchanged_by_translation_or_reversal() {
    let expected = profile(0.0, 18.0, 2.0, 8.0).total_time();
    for (start, end) in [(10.0, 28.0), (28.0, 10.0), (982.0, 1000.0), (1000.0, 982.0)] {
        assert_close(profile(start, end, 2.0, 8.0).total_time(), expected);
    }
}

#[test]
fn positions_remain_between_the_endpoints_throughout_the_move() {
    for (start, end) in [(10.0, 14.0), (14.0, 10.0), (125.5, 150.75), (150.75, 125.5)] {
        let motion = profile(start, end, 4.0, 4.0);
        for sample in 0..=32 {
            let time = motion.total_time() * f64::from(sample) / 32.0;
            let angle = position(&motion, time);
            assert!(
                (start.min(end)..=start.max(end)).contains(&angle),
                "angle {angle} at {time} escaped [{start}, {end}]"
            );
        }
    }
}

#[test]
fn a_forward_profile_cannot_jump_backwards_at_a_phase_transition() {
    // d=8, a=2, v=8: the triangular profile changes from acceleration
    // to deceleration after 2 seconds, with peak speed 4.
    let motion = profile(50.0, 58.0, 2.0, 8.0);
    let before = position(&motion, 1.999999);
    let after = position(&motion, 2.000001);
    assert!(
        after >= before,
        "position jumps backwards from {before} to {after}"
    );
    assert!(after - before <= 4.0 * 0.000002 + 1e-10);
}

#[test]
fn ordinary_inputs_do_not_exceed_the_maximum_velocity() {
    // Unit displacement isolates profile selection from distance scaling.
    let motion = profile(10.0, 11.0, 16.0, 2.0);
    let distance = position(&motion, 0.3) - position(&motion, 0.2);
    assert!(
        distance <= 2.0 * (0.3 - 0.2) + 1e-12,
        "travelled {distance} in 0.1 seconds with a speed cap of 2"
    );
}

#[test]
fn sampled_positions_respect_velocity_and_acceleration_limits_for_ordinary_inputs() {
    for (start, end, a, v) in [
        (10.0, 11.0, 4.0, 4.0),
        (11.0, 10.0, 4.0, 1.0),
        (25.0, 43.0, 2.0, 8.0),
        (43.0, 25.0, 2.0, 3.0),
        (10.25, 13.75, 2.5, 1.75),
        (500.0, 600.0, 7.3, 9.7),
        (-11.0, -10.0, 4.0, 4.0),
        (-10.0, -11.0, 4.0, 1.0),
        (-25.0, 43.0, 2.0, 8.0),
        (43.0, -25.0, 2.0, 3.0),
        (-600.0, -500.0, 7.3, 9.7),
    ] {
        let motion = signed_profile(start, end, a, v);
        let step = motion.total_time() / 128.0;
        let samples: Vec<_> = (0..=128)
            .map(|i| position(&motion, step * f64::from(i)))
            .collect();
        for pair in samples.windows(2) {
            let distance = (pair[1] - pair[0]).abs();
            assert!(
                distance <= v * step + 1e-10,
                "speed limit exceeded for {motion:?}"
            );
        }
        for triple in samples.windows(3) {
            // A bounded acceleration implies a bounded second difference,
            // including windows that straddle a transition between phases.
            let difference = (triple[2] - 2.0 * triple[1] + triple[0]).abs();
            assert!(
                difference <= a * step * step + 1e-10,
                "acceleration limit exceeded for {motion:?}"
            );
        }
    }
}

#[test]
fn triangular_profiles_do_not_gain_a_roundoff_cruise_phase() {
    // d=a=0.1 gives a one-second ramp and total duration exactly 2.
    // Reconstructing cruise from peak^2/a previously produced a small
    // negative cruise and changed the total to 1.9999999999999998.
    for (start, end) in [(0.0, 0.1), (0.0, -0.1), (-0.1, 0.0), (0.1, 0.0)] {
        let motion = signed_profile(start, end, 0.1, 1.0);
        assert_eq!(motion.total_time(), 2.0);
        assert_close(position(&motion, 1.0), (start + end) / 2.0);
        assert_eq!(position(&motion, 2.0), end);
    }
}

#[test]
fn final_position_is_exact_despite_cancellation_between_signed_angles() {
    for (start, end) in [
        (-1000.0, 0.1),
        (1000.0, -0.1),
        (-123.45, 67.89),
        (123.45, -67.89),
        (-10.25, -13.75),
    ] {
        let motion = signed_profile(start, end, 2.5, 4.0);
        // Exact endpoint identity is intentional here. A tolerance would
        // hide the cancellation that storing end_angle now prevents.
        assert_eq!(position(&motion, motion.total_time()), end);
    }
}

#[test]
fn negative_angles_follow_triangular_and_trapezoidal_profiles_in_both_directions() {
    for (velocity, duration, samples) in [
        (
            4.0,
            1.0,
            [
                (0.0, 0.0),
                (0.25, 0.125),
                (0.5, 0.5),
                (0.75, 0.875),
                (1.0, 1.0),
            ],
        ),
        (
            1.0,
            1.25,
            [
                (0.0, 0.0),
                (0.125, 0.03125),
                (0.5, 0.375),
                (1.125, 0.96875),
                (1.25, 1.0),
            ],
        ),
    ] {
        let forward = signed_profile(-11.0, -10.0, 4.0, velocity);
        let reverse = signed_profile(-10.0, -11.0, 4.0, velocity);
        assert_close(forward.total_time(), duration);
        assert_close(reverse.total_time(), duration);
        for (time, distance) in samples {
            assert_close(position(&forward, time), -11.0 + distance);
            assert_close(position(&reverse, time), -10.0 - distance);
        }
    }
}

#[test]
fn crossing_zero_is_continuous_in_both_directions() {
    let forward = signed_profile(-2.0, 2.0, 4.0, 4.0);
    let reverse = signed_profile(2.0, -2.0, 4.0, 4.0);
    for (time, expected) in [(0.0, -2.0), (0.5, -1.5), (1.0, 0.0), (1.5, 1.5), (2.0, 2.0)] {
        assert_close(position(&forward, time), expected);
        assert_close(position(&reverse, time), -expected);
    }
}

#[test]
fn negative_stationary_angles_have_zero_duration() {
    for angle in [-1000.0, -123.75, -0.125] {
        for (a, v) in [(0.25, 1000.0), (1000.0, 0.25), (1.0, 1.0)] {
            let motion = signed_profile(angle, angle, a, v);
            assert_eq!(motion.total_time(), 0.0);
            assert_eq!(position(&motion, 0.0), angle);
            assert_eq!(motion.position_at(0.25), Err(PlannerError::InvalidTime));
        }
    }
}

#[test]
fn signed_zero_endpoints_preserve_the_stored_final_angle() {
    for start in [-0.0, 0.0, -1.0, 1.0] {
        for end in [-0.0_f64, 0.0] {
            let motion = signed_profile(start, end, 4.0, 4.0);
            assert_eq!(
                position(&motion, motion.total_time()).to_bits(),
                end.to_bits()
            );
        }
    }
}

#[test]
fn negative_acceleration_is_rejected_for_moving_and_stationary_profiles() {
    for acceleration in [-1000.0, -7.5, -1.0, -0.125, -0.0] {
        for (start, end) in [(-10.0, -9.0), (-1.0, 1.0), (-10.0, -10.0)] {
            assert_eq!(
                MotionProfile::new(start, end, acceleration, 4.0).unwrap_err(),
                PlannerError::InvalidMaxAcceleration
            );
        }
    }
}

#[test]
fn negative_velocity_is_rejected_for_moving_and_stationary_profiles() {
    for velocity in [-1000.0, -7.5, -1.0, -0.125, -0.0] {
        for (start, end) in [(-10.0, -9.0), (-1.0, 1.0), (-10.0, -10.0)] {
            assert_eq!(
                MotionProfile::new(start, end, 4.0, velocity).unwrap_err(),
                PlannerError::InvalidMaxVelocity
            );
        }
    }
}

#[test]
fn invalid_acceleration_precedes_invalid_velocity_for_negative_limits() {
    for a in [-1000.0, -0.125, -0.0] {
        for v in [-1000.0, -0.125, -0.0] {
            assert_eq!(
                MotionProfile::new(-10.0, -9.0, a, v).unwrap_err(),
                PlannerError::InvalidMaxAcceleration
            );
        }
    }
}

#[test]
fn negative_times_are_rejected_without_affecting_later_queries() {
    for (start, end) in [(-11.0, -10.0), (1.0, -1.0), (-1.0, -1.0)] {
        let motion = signed_profile(start, end, 4.0, 4.0);
        for time in [-1000.0, -1.0, -0.125] {
            assert_eq!(motion.position_at(time), Err(PlannerError::InvalidTime));
            assert_eq!(position(&motion, 0.0), start);
            assert_eq!(position(&motion, motion.total_time()), end);
        }
        assert_eq!(motion.position_at(-0.0), Ok(start));
    }
}

#[test]
fn reflecting_both_angles_reflects_every_sample_without_changing_duration() {
    for (start, end, a, v) in [
        (10.0, 11.0, 4.0, 4.0),
        (-2.0, 2.0, 4.0, 1.0),
        (-25.0, -7.0, 2.0, 8.0),
    ] {
        let original = signed_profile(start, end, a, v);
        let reflected = signed_profile(-start, -end, a, v);
        assert_eq!(original.total_time(), reflected.total_time());
        for i in 0..=32 {
            let time = original.total_time() * (f64::from(i) / 32.0);
            assert_close(position(&reflected, time), -position(&original, time));
        }
    }
}

#[test]
fn translating_profiles_into_negative_angles_translates_every_sample() {
    let original = signed_profile(-6.0, 12.0, 2.0, 3.0);
    for offset in [-500.0, -20.0, 20.0, 500.0] {
        let translated = signed_profile(-6.0 + offset, 12.0 + offset, 2.0, 3.0);
        assert_eq!(original.total_time(), translated.total_time());
        for i in 0..=32 {
            let time = original.total_time() * (f64::from(i) / 32.0);
            assert_close(
                position(&translated, time),
                position(&original, time) + offset,
            );
        }
    }
}

#[test]
fn reversing_a_signed_move_reverses_its_time_sequence() {
    for (start, end, a, v) in [
        (-11.0, -10.0, 4.0, 4.0),
        (-2.0, 2.0, 4.0, 1.0),
        (-25.0, -7.0, 2.0, 8.0),
    ] {
        let forward = signed_profile(start, end, a, v);
        let reverse = signed_profile(end, start, a, v);
        let duration = forward.total_time();
        assert_eq!(duration, reverse.total_time());
        for i in 0..=32 {
            let time = duration * (f64::from(i) / 32.0);
            assert_close(
                position(&forward, time),
                position(&reverse, duration - time),
            );
        }
    }
}

#[test]
fn signed_multiple_turns_are_not_normalized_to_a_shorter_move() {
    let motion = signed_profile(-2.0 * TAU, 3.0 * TAU, TAU, TAU);
    assert_close(motion.total_time(), 6.0);
    assert_close(position(&motion, 3.0), TAU / 2.0);
    assert_eq!(position(&motion, 6.0), 3.0 * TAU);
}

#[test]
fn signed_profile_durations_agree_with_joint_acceleration_limits() {
    for start in [-1000.0, -7.5, -0.125, 0.0, 0.125, 7.5, 1000.0] {
        for end in [-1000.0, -7.5, -0.125, 0.0, 0.125, 7.5, 1000.0] {
            for a in [0.25, 4.0, 1000.0] {
                for v in [4.0, 16.0, 1000.0] {
                    let motion = signed_profile(start, end, a, v);
                    let expected = JointAccelerationLimits::new(a, 1.0)
                        .unwrap()
                        .min_duration_between(
                            JointAngles::new(start, 0.0).unwrap(),
                            JointAngles::new(end, 0.0).unwrap(),
                            &JointVelocityLimits::new(v, 1.0).unwrap(),
                        )
                        .unwrap();
                    assert_close(motion.total_time(), expected);
                }
            }
        }
    }
}

#[test]
fn scaling_speed_and_acceleration_scales_the_duration() {
    // Scale speed by k and acceleration by k^2. Physical duration
    // must be divided by k in both triangular and trapezoidal regimes.
    for factor in [0.5, 1.0, 2.0, 4.0, 8.0] {
        let motion = profile(10.0, 14.0, factor * factor, factor);
        assert_close(motion.total_time(), 5.0 / factor);
    }
}

// Build independent reference cases by integrating velocity areas.
// Returns distance, peak speed, total duration, and samples of displacement.
fn integrated_profile(a: f64, ramp: f64, cruise: f64) -> (f64, f64, f64, [(f64, f64); 5]) {
    let peak = a * ramp;
    let ramp_distance = peak * ramp / 2.0;
    let distance = 2.0 * ramp_distance + peak * cruise;
    let duration = 2.0 * ramp + cruise;
    (
        distance,
        peak,
        duration,
        [
            (0.0, 0.0),
            (ramp / 2.0, ramp_distance / 4.0),
            (ramp, ramp_distance),
            (ramp + cruise, ramp_distance + peak * cruise),
            (duration, distance),
        ],
    )
}

#[test]
fn constructed_velocity_profiles_recover_their_integrated_duration() {
    for a in [0.25, 1.0, 4.0] {
        for ramp in [0.25, 0.5, 1.0] {
            for cruise in [0.0, 0.5, 2.0] {
                let (distance, peak, duration, _) = integrated_profile(a, ramp, cruise);
                let cap = if cruise == 0.0 { 2.0 * peak } else { peak };
                let motion = profile(10.0, 10.0 + distance, a, cap);
                assert_close(motion.total_time(), duration);
            }
        }
    }
}

#[test]
fn constructed_velocity_profiles_recover_their_integrated_positions() {
    for a in [0.25, 1.0, 4.0] {
        for ramp in [0.25, 0.5, 1.0] {
            for cruise in [0.0, 0.5, 2.0] {
                let (distance, peak, _, samples) = integrated_profile(a, ramp, cruise);
                let cap = if cruise == 0.0 { 2.0 * peak } else { peak };
                let motion = profile(10.0, 10.0 + distance, a, cap);
                for (time, displacement) in samples {
                    assert_close(position(&motion, time), 10.0 + displacement);
                }
            }
        }
    }
}

// Extreme numerical regressions run normally after stabilizing profile creation
// and preserving exact endpoint samples.

#[test]
fn regression_underflowed_product_still_has_a_finite_triangular_duration() {
    let scale = 2.0_f64.powi(-600);
    let motion = MotionProfile::new(0.0, scale, scale, 1.0).unwrap();
    assert_close(motion.total_time(), 2.0);
}

#[test]
fn regression_underflowed_peak_preserves_the_start_at_time_zero() {
    let scale = 2.0_f64.powi(-600);
    let motion = MotionProfile::new(0.0, scale, scale, 1.0).unwrap();
    assert_eq!(motion.position_at(0.0), Ok(0.0));
}

#[test]
fn regression_overflowed_product_still_has_a_finite_triangular_duration() {
    let scale = 2.0_f64.powi(600);
    let motion = MotionProfile::new(0.0, scale, scale, scale).unwrap();
    assert_close(motion.total_time(), 2.0);
}

#[test]
fn regression_small_common_scale_preserves_a_triangular_duration() {
    let scale = 2.0_f64.powi(-600);
    let motion = MotionProfile::new(0.0, scale / 2.0, scale, scale).unwrap();
    assert_close(motion.total_time(), 2.0_f64.sqrt());
}

#[test]
fn regression_final_position_is_exact_even_when_ramp_times_are_absorbed() {
    let motion = MotionProfile::new(0.0, 1.0, 1.0, 1e-300).unwrap();
    assert_eq!(motion.position_at(motion.total_time()), Ok(1.0));
}

#[test]
fn regression_stationary_negative_angle_takes_zero_time_with_subnormal_limits() {
    let smallest = f64::from_bits(1);
    let motion = MotionProfile::new(-1000.0, -1000.0, smallest, smallest).unwrap();
    assert_eq!(motion.total_time(), 0.0);
}
