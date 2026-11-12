use kinematics::JointAngles;
use planner::{
    JointAccelerationLimits, JointMotionProfile, JointVelocityLimits, MotionProfile, PlannerError,
};

fn bounded(value: f64) -> f64 {
    assert!(
        (-10000.0..=10000.0).contains(&value),
        "ordinary input outside the test range: {value}"
    );
    value
}

// All ordinary API inputs, including calculated query times, are bounded.
// Ignored precision regressions below intentionally bypass these helpers.
fn make(
    start: [f64; 2],
    end: [f64; 2],
    speeds: [f64; 2],
    accelerations: [f64; 2],
) -> JointMotionProfile {
    JointMotionProfile::new(
        JointAngles::new(bounded(start[0]), bounded(start[1])).unwrap(),
        JointAngles::new(bounded(end[0]), bounded(end[1])).unwrap(),
        &JointVelocityLimits::new(bounded(speeds[0]), bounded(speeds[1])).unwrap(),
        &JointAccelerationLimits::new(bounded(accelerations[0]), bounded(accelerations[1]))
            .unwrap(),
    )
    .unwrap()
}

fn velocity(motion: &JointMotionProfile, time: f64) -> [f64; 2] {
    let result = motion.velocity_at(bounded(time)).unwrap();
    [result.0, result.1]
}

fn position(motion: &JointMotionProfile, time: f64) -> [f64; 2] {
    let result = motion.position_at(bounded(time)).unwrap();
    [result.first_theta(), result.second_theta()]
}

fn assert_close(actual: f64, expected: f64) {
    assert!(actual.is_finite());
    let tolerance = 128.0 * f64::EPSILON * expected.abs().max(1.0);
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {expected:e}, got {actual:e}"
    );
}

fn assert_velocity(actual: [f64; 2], expected: [f64; 2]) {
    for joint in 0..2 {
        assert_close(actual[joint], expected[joint]);
    }
}

type MotionCase = ([f64; 2], [f64; 2], [f64; 2], [f64; 2]);

const CASES: [MotionCase; 8] = [
    ([0.0, 0.0], [4.0, 1.0], [4.0, 4.0], [4.0, 4.0]),
    ([-2.0, 1.0], [2.0, 0.0], [4.0, 1.0], [4.0, 4.0]),
    ([0.0, 0.0], [4.0, 1.0], [4.0, 0.25], [4.0, 1.0]),
    (
        [-10000.0, 10000.0],
        [10000.0, -10000.0],
        [10000.0, 10000.0],
        [10000.0, 10000.0],
    ),
    ([-123.5, 246.25], [-7.25, -64.5], [7.5, 2.5], [2.5, 7.5]),
    ([12.5, -1.0], [12.5, 0.0], [10000.0, 4.0], [0.25, 4.0]),
    ([-1.0, 12.5], [0.0, 12.5], [4.0, 10000.0], [4.0, 0.25]),
    (
        [-10000.0, 10000.0],
        [-10000.0, 10000.0],
        [0.25, 10000.0],
        [10000.0, 0.25],
    ),
];

#[test]
fn synchronized_triangles_reduce_the_faster_axes_velocity() {
    let motion = make([0.0, 0.0], [4.0, 1.0], [4.0, 4.0], [4.0, 4.0]);
    assert_eq!(motion.total_time(), 2.0);
    // Axis 2 alone peaks at 2 in one second. Stretched to two seconds,
    // its peak is 1, so returning its unscaled local velocity would fail.
    for (time, expected) in [
        (0.0, [0.0, 0.0]),
        (0.5, [2.0, 0.5]),
        (1.0, [4.0, 1.0]),
        (1.5, [2.0, 0.5]),
        (2.0, [0.0, 0.0]),
    ] {
        assert_velocity(velocity(&motion, time), expected);
    }
}

#[test]
fn mixed_profiles_rescale_ramps_and_cruise_at_their_shared_global_times() {
    let motion = make([0.0, 0.0], [4.0, 1.0], [4.0, 1.0], [4.0, 4.0]);
    // Axis 2 has factor 1.25/2=0.625, global ramp duration 0.4,
    // peak speed 0.625 and global acceleration 4*0.625^2=1.5625.
    for (time, expected) in [
        (0.0, [0.0, 0.0]),
        (0.2, [0.8, 0.3125]),
        (0.4, [1.6, 0.625]),
        (1.0, [4.0, 0.625]),
        (1.6, [1.6, 0.625]),
        (1.8, [0.8, 0.3125]),
        (2.0, [0.0, 0.0]),
    ] {
        assert_velocity(velocity(&motion, time), expected);
    }
}

#[test]
fn either_axis_can_be_the_slower_one_and_keep_its_unscaled_speed() {
    let motion = make([0.0, 0.0], [4.0, 1.0], [4.0, 0.25], [4.0, 1.0]);
    assert_eq!(motion.total_time(), 4.25);
    assert_velocity(velocity(&motion, 2.125), [8.0 / 4.25, 0.25]);
    let swapped = make([0.0, 0.0], [1.0, 4.0], [0.25, 4.0], [1.0, 4.0]);
    assert_velocity(velocity(&swapped, 2.125), [0.25, 8.0 / 4.25]);
}

#[test]
fn equal_durations_leave_both_independent_velocities_unchanged() {
    let motion = make([-4.0, 1.0], [0.0, 0.0], [4.0, 1.0], [4.0, 1.0]);
    assert_eq!(motion.total_time(), 2.0);
    for (time, expected) in [
        (0.0, [0.0, 0.0]),
        (0.5, [2.0, -0.5]),
        (1.0, [4.0, -1.0]),
        (1.5, [2.0, -0.5]),
        (2.0, [0.0, 0.0]),
    ] {
        assert_velocity(velocity(&motion, time), expected);
    }
}

#[test]
fn opposite_directions_have_independent_signed_velocities() {
    let motion = make([-2.0, 1.0], [2.0, 0.0], [4.0, 4.0], [4.0, 4.0]);
    assert_velocity(velocity(&motion, 0.5), [2.0, -0.5]);
    assert_velocity(velocity(&motion, 1.0), [4.0, -1.0]);
    assert_velocity(velocity(&motion, 1.5), [2.0, -0.5]);
}

#[test]
fn stationary_axes_have_zero_velocity_during_the_entire_move() {
    for (start, end, speeds, accelerations) in [CASES[5], CASES[6]] {
        let motion = make(start, end, speeds, accelerations);
        assert_eq!(motion.total_time(), 1.0);
        for i in 0..=64 {
            let result = velocity(&motion, motion.total_time() * (f64::from(i) / 64.0));
            for joint in 0..2 {
                if start[joint] == end[joint] {
                    assert_eq!(result[joint], 0.0);
                }
            }
        }
    }
}

#[test]
fn zero_duration_returns_two_zeros_without_dividing_by_zero() {
    for at in [[-10000.0, 10000.0], [-12.5, -0.125], [-0.0, 0.0]] {
        let motion = make(at, at, [0.25, 10000.0], [10000.0, 0.25]);
        assert_eq!(motion.total_time(), 0.0);
        assert_eq!(velocity(&motion, 0.0), [0.0, 0.0]);
        assert_eq!(velocity(&motion, -0.0), [0.0, 0.0]);
        for time in [-10000.0, -0.125, 0.125, 10000.0] {
            assert_eq!(
                motion.velocity_at(bounded(time)),
                Err(PlannerError::InvalidTime)
            );
        }
    }
}

#[test]
fn every_ordinary_move_starts_and_finishes_with_both_axes_at_rest() {
    for (start, end, speeds, accelerations) in CASES {
        let motion = make(start, end, speeds, accelerations);
        assert_eq!(velocity(&motion, 0.0), [0.0, 0.0]);
        assert_eq!(velocity(&motion, motion.total_time()), [0.0, 0.0]);
    }
}

#[test]
fn finite_invalid_times_are_rejected_before_sampling_either_axis() {
    let motion = make([0.0, 0.0], [4.0, 1.0], [4.0, 4.0], [4.0, 4.0]);
    for time in [-10000.0, -1.0, -0.125, 2.125, 10.0, 10000.0] {
        assert_eq!(
            motion.velocity_at(bounded(time)),
            Err(PlannerError::InvalidTime)
        );
        assert_eq!(
            motion.position_at(time).unwrap_err(),
            PlannerError::InvalidTime
        );
    }
    assert_eq!(velocity(&motion, -0.0), [0.0, 0.0]);
    assert_eq!(velocity(&motion, 2.0), [0.0, 0.0]);
}

#[test]
fn queries_are_repeatable_after_out_of_order_and_invalid_times() {
    let motion = make([0.0, 0.0], [4.0, 1.0], [4.0, 4.0], [4.0, 4.0]);
    for (time, expected) in [
        (1.5, [2.0, 0.5]),
        (0.5, [2.0, 0.5]),
        (2.0, [0.0, 0.0]),
        (1.0, [4.0, 1.0]),
        (0.0, [0.0, 0.0]),
        (1.5, [2.0, 0.5]),
    ] {
        assert_eq!(motion.velocity_at(-1.0), Err(PlannerError::InvalidTime));
        assert_velocity(velocity(&motion, time), expected);
        assert_eq!(position(&motion, motion.total_time()), [4.0, 1.0]);
    }
}

#[test]
fn velocity_matches_the_derivative_of_synchronized_position_for_each_axis() {
    for (start, end, speeds, accelerations) in [CASES[0], CASES[1], CASES[2]] {
        let motion = make(start, end, speeds, accelerations);
        // Fractions avoid the known phase transitions in these profiles.
        for fraction in [0.125, 0.375, 0.625, 0.875] {
            let time = motion.total_time() * fraction;
            let h = 0.0001;
            let before = position(&motion, time - h);
            let after = position(&motion, time + h);
            let actual = velocity(&motion, time);
            for joint in 0..2 {
                let derivative = (after[joint] - before[joint]) / (2.0 * h);
                assert!(
                    (actual[joint] - derivative).abs() <= 1e-8,
                    "axis {joint}: velocity {} differs from position derivative {derivative}",
                    actual[joint]
                );
            }
        }
    }
}

#[test]
fn integrating_global_velocities_recovers_both_signed_displacements() {
    for (start, end, speeds, accelerations) in [CASES[0], CASES[1], CASES[2], CASES[3], CASES[7]] {
        let motion = make(start, end, speeds, accelerations);
        let mut integral = [0.0, 0.0];
        let mut previous_time = 0.0;
        let mut previous = velocity(&motion, 0.0);
        // 1020 intervals align with all shared phase transitions in these
        // cases (including 1/17 for the slow trapezoid). Thus quadrature is
        // exact for the linear velocity segments, apart from rounding.
        for i in 1..=1020 {
            let time = motion.total_time() * (f64::from(i) / 1020.0);
            let current = velocity(&motion, time);
            for joint in 0..2 {
                integral[joint] +=
                    (previous[joint] + current[joint]) / 2.0 * (time - previous_time);
            }
            previous_time = time;
            previous = current;
        }
        for joint in 0..2 {
            assert!(
                (integral[joint] - (end[joint] - start[joint])).abs() <= 1e-8,
                "axis {joint}: integrated displacement {} differs from {}",
                integral[joint],
                end[joint] - start[joint]
            );
        }
    }
}

#[test]
fn faster_axes_global_acceleration_includes_the_square_of_the_time_factor() {
    let motion = make([0.0, 0.0], [4.0, 1.0], [4.0, 1.0], [4.0, 4.0]);
    let from = velocity(&motion, 0.1);
    let to = velocity(&motion, 0.3);
    assert_close((to[0] - from[0]) / 0.2, 4.0);
    assert_close((to[1] - from[1]) / 0.2, 1.5625);
    let from = velocity(&motion, 1.7);
    let to = velocity(&motion, 1.9);
    assert_close((to[0] - from[0]) / 0.2, -4.0);
    assert_close((to[1] - from[1]) / 0.2, -1.5625);
}

#[test]
fn sampled_velocities_have_the_correct_sign_and_obey_each_joints_speed_cap() {
    for (start, end, speeds, accelerations) in CASES {
        let motion = make(start, end, speeds, accelerations);
        for i in 0..=128 {
            let values = velocity(&motion, motion.total_time() * (f64::from(i) / 128.0));
            for joint in 0..2 {
                assert!(values[joint].is_finite());
                assert!(values[joint].abs() <= speeds[joint] + 1e-9);
                if end[joint] > start[joint] {
                    assert!(values[joint] >= 0.0);
                } else if end[joint] < start[joint] {
                    assert!(values[joint] <= 0.0);
                } else {
                    assert_eq!(values[joint], 0.0);
                }
            }
        }
    }
}

#[test]
fn velocity_changes_obey_each_joints_acceleration_limit_across_phases() {
    for (start, end, speeds, accelerations) in CASES {
        let motion = make(start, end, speeds, accelerations);
        let mut previous_time = 0.0;
        let mut previous = velocity(&motion, 0.0);
        for i in 1..=128 {
            let time = motion.total_time() * (f64::from(i) / 128.0);
            let current = velocity(&motion, time);
            for joint in 0..2 {
                assert!(
                    (current[joint] - previous[joint]).abs()
                        <= accelerations[joint] * (time - previous_time) + 1e-9
                );
            }
            previous = current;
            previous_time = time;
        }
    }
}

#[test]
fn velocities_match_independent_profiles_with_the_chain_rule_time_factor() {
    for (start, end, speeds, accelerations) in CASES {
        let first = MotionProfile::new(start[0], end[0], accelerations[0], speeds[0]).unwrap();
        let second = MotionProfile::new(start[1], end[1], accelerations[1], speeds[1]).unwrap();
        let motion = make(start, end, speeds, accelerations);
        if motion.total_time() == 0.0 {
            continue;
        }
        for i in 0..=64 {
            let fraction = f64::from(i) / 64.0;
            let expected = [
                first
                    .velocity_at(bounded(fraction * first.total_time()))
                    .unwrap()
                    * (first.total_time() / motion.total_time()),
                second
                    .velocity_at(bounded(fraction * second.total_time()))
                    .unwrap()
                    * (second.total_time() / motion.total_time()),
            ];
            assert_velocity(velocity(&motion, fraction * motion.total_time()), expected);
        }
    }
}

#[test]
fn swapping_axes_and_limits_swaps_the_velocity_tuple() {
    let swap = |values: [f64; 2]| [values[1], values[0]];
    for (start, end, speeds, accelerations) in CASES {
        let original = make(start, end, speeds, accelerations);
        let swapped = make(swap(start), swap(end), swap(speeds), swap(accelerations));
        assert_eq!(original.total_time(), swapped.total_time());
        for i in 0..=64 {
            let time = original.total_time() * (f64::from(i) / 64.0);
            assert_velocity(velocity(&swapped, time), swap(velocity(&original, time)));
        }
    }
}

#[test]
fn translating_each_axis_independently_preserves_both_velocities() {
    let original = make([-2.0, 1.0], [2.0, 0.0], [4.0, 1.0], [4.0, 4.0]);
    for offsets in [[-9000.0, 9000.0], [123.5, -456.25], [0.0, 0.0]] {
        let translated = make(
            [-2.0 + offsets[0], 1.0 + offsets[1]],
            [2.0 + offsets[0], offsets[1]],
            [4.0, 1.0],
            [4.0, 4.0],
        );
        assert_eq!(original.total_time(), translated.total_time());
        for i in 0..=64 {
            let time = original.total_time() * (f64::from(i) / 64.0);
            assert_eq!(velocity(&translated, time), velocity(&original, time));
        }
    }
}

#[test]
fn reflecting_angles_negates_both_velocities_without_changing_duration() {
    let reflect = |values: [f64; 2]| [-values[0], -values[1]];
    for (start, end, speeds, accelerations) in CASES {
        let original = make(start, end, speeds, accelerations);
        let reflected = make(reflect(start), reflect(end), speeds, accelerations);
        assert_eq!(original.total_time(), reflected.total_time());
        for i in 0..=64 {
            let time = original.total_time() * (f64::from(i) / 64.0);
            assert_velocity(
                velocity(&reflected, time),
                reflect(velocity(&original, time)),
            );
        }
    }
}

#[test]
fn reversing_moves_negates_the_time_reversed_velocity() {
    for (start, end, speeds, accelerations) in CASES {
        let forward = make(start, end, speeds, accelerations);
        let reverse = make(end, start, speeds, accelerations);
        assert_eq!(forward.total_time(), reverse.total_time());
        for i in 0..=64 {
            let time = forward.total_time() * (f64::from(i) / 64.0);
            let expected = velocity(&forward, time);
            assert_velocity(
                velocity(&reverse, reverse.total_time() - time),
                [-expected[0], -expected[1]],
            );
        }
    }
}

#[test]
fn scaling_limits_rescales_global_velocity_and_shared_time() {
    for (start, end, speeds, accelerations) in [CASES[0], CASES[1], CASES[2]] {
        let original = make(start, end, speeds, accelerations);
        for factor in [0.25, 0.5, 1.0, 2.0, 4.0] {
            let scaled = make(
                start,
                end,
                [speeds[0] * factor, speeds[1] * factor],
                [
                    accelerations[0] * factor * factor,
                    accelerations[1] * factor * factor,
                ],
            );
            assert_close(scaled.total_time(), original.total_time() / factor);
            for i in 0..=64 {
                let time = original.total_time() * (f64::from(i) / 64.0);
                let expected = velocity(&original, time);
                assert_velocity(
                    velocity(&scaled, time / factor),
                    [expected[0] * factor, expected[1] * factor],
                );
            }
        }
    }
}

// Numerical regressions run normally. The separately marked IEEE validation
// case can also be included with --include-ignored.

fn extreme(end: [f64; 2], speeds: [f64; 2], accelerations: [f64; 2]) -> JointMotionProfile {
    JointMotionProfile::new(
        JointAngles::new(0.0, 0.0).unwrap(),
        JointAngles::new(end[0], end[1]).unwrap(),
        &JointVelocityLimits::new(speeds[0], speeds[1]).unwrap(),
        &JointAccelerationLimits::new(accelerations[0], accelerations[1]).unwrap(),
    )
    .unwrap()
}

#[test]
fn regression_tiny_time_factor_preserves_a_representable_global_velocity() {
    let motion = extreme(
        [2.0_f64.powi(-200), 1.0],
        [2.0_f64.powi(400), 2.0_f64.powi(-600)],
        [2.0_f64.powi(1000), 1.0],
    );
    let result = motion.velocity_at(motion.total_time() / 2.0).unwrap();
    // The first axis is triangular over the shared duration: peak=2*d/T.
    // Normalization prevents an absolute tolerance from hiding tiny values.
    assert_close(result.0 / 2.0_f64.powi(-799), 1.0);
}

#[test]
fn regression_tiny_progress_preserves_a_representable_ramp_velocity() {
    let speed = 2.0_f64.powi(-600);
    let motion = extreme([1.0, 0.0], [speed, 1.0], [1.0, 1.0]);
    let result = motion.velocity_at(speed).unwrap();
    assert_close(result.0 / speed, 1.0);
}

#[test]
fn regression_absorbed_scalar_ramps_still_end_with_both_axes_at_rest() {
    let motion = extreme([1e-13, 1.0], [1e-13, 4.0], [10000.0, 4.0]);
    assert_eq!(motion.velocity_at(motion.total_time()), Ok((0.0, 0.0)));
}

#[test]
fn regression_stationary_axis_keeps_zero_velocity_with_subnormal_limits() {
    let smallest = f64::from_bits(1);
    let motion = extreme([0.0, 1.0], [smallest, 4.0], [smallest, 4.0]);
    assert_eq!(
        motion.velocity_at(motion.total_time() / 2.0).unwrap().0,
        0.0
    );
}

#[test]
#[ignore = "IEEE validation boundary: NaN and infinities must return InvalidTime, including before the zero-duration shortcut"]
fn nonfinite_times_are_rejected_for_moving_and_zero_duration_profiles() {
    for end in [[4.0, 1.0], [0.0, 0.0], [0.0, -1.0]] {
        let motion = extreme(end, [4.0, 4.0], [4.0, 4.0]);
        for time in [f64::NAN, -f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert_eq!(motion.velocity_at(time), Err(PlannerError::InvalidTime));
        }
    }
}
