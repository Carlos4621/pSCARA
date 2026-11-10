use core::f64::consts::{FRAC_PI_2, TAU};

use kinematics::{ElbowConfiguration, JointAngles, Point, Scara};
use planner::{
    JointAccelerationLimits, JointMotionProfile, JointTrajectory, JointVelocityLimits,
    MotionProfile, PlannerError,
};
use trajectory::LineSegment;

// Ordinary test inputs, including calculated query times, are checked at
// the API boundary. Extreme regressions below deliberately bypass these helpers.
fn bounded(value: f64) -> f64 {
    assert!(
        (-10000.0..=10000.0).contains(&value),
        "input outside the ordinary test range: {value}"
    );
    value
}

fn angles(values: [f64; 2]) -> JointAngles {
    JointAngles::new(bounded(values[0]), bounded(values[1])).unwrap()
}

fn make(
    start: [f64; 2],
    end: [f64; 2],
    velocities: [f64; 2],
    accelerations: [f64; 2],
) -> JointMotionProfile {
    JointMotionProfile::new(
        angles(start),
        angles(end),
        &JointVelocityLimits::new(bounded(velocities[0]), bounded(velocities[1])).unwrap(),
        &JointAccelerationLimits::new(bounded(accelerations[0]), bounded(accelerations[1]))
            .unwrap(),
    )
    .unwrap()
}

fn sample(motion: &JointMotionProfile, time: f64) -> [f64; 2] {
    let result = motion.position_at(bounded(time)).unwrap();
    [result.first_theta(), result.second_theta()]
}

fn assert_close(actual: f64, expected: f64) {
    assert!(actual.is_finite(), "nonfinite result: {actual}");
    let tolerance = 1e-10 + 64.0 * f64::EPSILON * expected.abs();
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {expected:e}, got {actual:e}"
    );
}

fn assert_position(actual: [f64; 2], expected: [f64; 2]) {
    for joint in 0..2 {
        assert_close(actual[joint], expected[joint]);
    }
}

// Covers either slower joint, equal times, mixed profiles, negative angles,
// crossing zero, fractional values, the requested bounds, and stationary axes.
type MotionCase = ([f64; 2], [f64; 2], [f64; 2], [f64; 2]);

const CASES: [MotionCase; 9] = [
    ([0.0, 0.0], [4.0, 1.0], [4.0, 4.0], [4.0, 4.0]),
    ([0.0, 0.0], [1.0, 4.0], [4.0, 4.0], [4.0, 4.0]),
    ([-4.0, 1.0], [0.0, 0.0], [4.0, 1.0], [4.0, 4.0]),
    ([-10.25, 13.75], [13.75, -10.25], [7.5, 2.5], [2.5, 7.5]),
    (
        [-10000.0, 10000.0],
        [10000.0, -10000.0],
        [10.0, 16.0],
        [4.0, 8.0],
    ),
    ([0.0, -0.0], [1.0, -4.0], [1.0, 4.0], [1.0, 4.0]),
    ([12.5, -10.0], [12.5, -9.0], [10000.0, 1.0], [10000.0, 4.0]),
    ([-10.0, 12.5], [-9.0, 12.5], [1.0, 10000.0], [4.0, 10000.0]),
    (
        [-10000.0, 10000.0],
        [-10000.0, 10000.0],
        [0.25, 10000.0],
        [10000.0, 0.25],
    ),
];

#[test]
fn triangular_axes_arrive_together_at_the_slower_axes_duration() {
    let motion = make([0.0, 0.0], [4.0, 1.0], [4.0, 4.0], [4.0, 4.0]);
    assert_eq!(motion.total_time(), 2.0);
    for (time, expected) in [
        (0.0, [0.0, 0.0]),
        (0.5, [0.5, 0.125]),
        (1.0, [2.0, 0.5]),
        (1.5, [3.5, 0.875]),
        (2.0, [4.0, 1.0]),
    ] {
        assert_position(sample(&motion, time), expected);
    }
}

#[test]
fn faster_axis_is_stretched_instead_of_reaching_its_endpoint_early() {
    let motion = make([0.0, 0.0], [4.0, 1.0], [4.0, 4.0], [4.0, 4.0]);
    // The second axis alone would finish in one second. Synchronization
    // stretches its entire profile, leaving it halfway at that global time.
    assert_position(sample(&motion, 1.0), [2.0, 0.5]);
}

#[test]
fn triangle_and_trapezoid_keep_their_phases_after_synchronization() {
    let motion = make([0.0, 0.0], [4.0, 1.0], [4.0, 1.0], [4.0, 4.0]);
    assert_eq!(motion.total_time(), 2.0);
    // Axis 2 has a local duration of 1.25 and a local ramp of 0.25.
    // Its acceleration/cruise transition therefore occurs globally at 0.4.
    for (time, expected) in [
        (0.2, [0.08, 0.03125]),
        (0.4, [0.32, 0.125]),
        (1.0, [2.0, 0.5]),
        (1.6, [3.68, 0.875]),
        (1.8, [3.92, 0.96875]),
    ] {
        assert_position(sample(&motion, time), expected);
    }
}

#[test]
fn either_axis_can_control_duration_even_with_a_smaller_displacement() {
    let motion = make([0.0, 0.0], [4.0, 1.0], [4.0, 0.25], [4.0, 1.0]);
    assert_eq!(motion.total_time(), 4.25);
    assert_position(sample(&motion, 2.125), [2.0, 0.5]);
    let swapped = make([0.0, 0.0], [1.0, 4.0], [0.25, 4.0], [1.0, 4.0]);
    assert_eq!(swapped.total_time(), 4.25);
    assert_position(sample(&swapped, 2.125), [0.5, 2.0]);
}

#[test]
fn equal_independent_durations_do_not_change_either_profile() {
    let motion = make([-4.0, 1.0], [0.0, 0.0], [4.0, 1.0], [4.0, 1.0]);
    assert_eq!(motion.total_time(), 2.0);
    assert_position(sample(&motion, 0.5), [-3.5, 0.875]);
    assert_position(sample(&motion, 1.0), [-2.0, 0.5]);
    assert_position(sample(&motion, 1.5), [-0.5, 0.125]);
}

#[test]
fn stationary_axes_preserve_their_angles_at_every_global_time() {
    for (start, end, speeds, accelerations) in [
        ([-123.5, -10.0], [-123.5, -9.0], [10000.0, 1.0], [0.25, 4.0]),
        ([-10.0, -123.5], [-9.0, -123.5], [1.0, 10000.0], [4.0, 0.25]),
    ] {
        let motion = make(start, end, speeds, accelerations);
        assert_eq!(motion.total_time(), 1.25);
        for fraction in [0.0, 0.125, 0.5, 0.875, 1.0] {
            let result = sample(&motion, fraction * motion.total_time());
            for joint in 0..2 {
                if start[joint] == end[joint] {
                    assert_eq!(result[joint], start[joint]);
                }
            }
        }
    }
}

#[test]
fn two_stationary_axes_have_zero_duration_and_accept_only_zero_time() {
    for position in [[-10000.0, 10000.0], [-123.5, -0.25], [0.0, -0.0]] {
        let motion = make(position, position, [0.25, 10000.0], [10000.0, 0.25]);
        assert_eq!(motion.total_time(), 0.0);
        assert_eq!(sample(&motion, 0.0), position);
        assert_eq!(sample(&motion, -0.0), position);
        for time in [-10000.0, -0.125, 0.125, 10000.0] {
            assert_eq!(
                motion.position_at(time).unwrap_err(),
                PlannerError::InvalidTime
            );
        }
    }
}

#[test]
fn finite_times_outside_the_shared_duration_are_rejected() {
    let motion = make([0.0, 0.0], [4.0, 1.0], [4.0, 4.0], [4.0, 4.0]);
    for time in [-10000.0, -1.0, -0.125, 2.125, 10.0, 10000.0] {
        assert_eq!(
            motion.position_at(bounded(time)).unwrap_err(),
            PlannerError::InvalidTime
        );
    }
    assert_eq!(sample(&motion, -0.0), [0.0, 0.0]);
    assert_eq!(sample(&motion, 2.0), [4.0, 1.0]);
}

#[test]
fn queries_are_repeatable_after_out_of_order_and_invalid_times() {
    let motion = make([-4.0, 1.0], [0.0, 0.0], [4.0, 1.0], [4.0, 1.0]);
    for (time, expected) in [
        (1.5, [-0.5, 0.125]),
        (0.5, [-3.5, 0.875]),
        (2.0, [0.0, 0.0]),
        (0.0, [-4.0, 1.0]),
        (1.5, [-0.5, 0.125]),
    ] {
        assert_eq!(
            motion.position_at(-1.0).unwrap_err(),
            PlannerError::InvalidTime
        );
        assert_position(sample(&motion, time), expected);
    }
}

#[test]
fn start_and_final_samples_preserve_both_requested_positions() {
    for (start, end, speeds, accelerations) in CASES {
        let motion = make(start, end, speeds, accelerations);
        assert_eq!(sample(&motion, 0.0), start);
        assert_eq!(sample(&motion, motion.total_time()), end);
    }
}

#[test]
fn endpoint_is_exact_when_reconstructing_it_would_lose_precision() {
    let motion = make([-10000.0, 10000.0], [0.1, -0.1], [10.0, 20.0], [4.0, 8.0]);
    assert_eq!(sample(&motion, motion.total_time()), [0.1, -0.1]);
}

#[test]
fn endpoint_preserves_signed_zero_in_each_joint() {
    for end in [[-0.0_f64, 0.0], [0.0, -0.0]] {
        for start in [[-1.0, 1.0], [0.0, 0.0]] {
            let motion = make(start, end, [4.0, 4.0], [4.0, 4.0]);
            let result = sample(&motion, motion.total_time());
            assert_eq!(result[0].to_bits(), end[0].to_bits());
            assert_eq!(result[1].to_bits(), end[1].to_bits());
        }
    }
}

#[test]
fn duration_is_the_maximum_of_the_independent_profile_durations() {
    for (start, end, speeds, accelerations) in CASES {
        let first = MotionProfile::new(start[0], end[0], accelerations[0], speeds[0]).unwrap();
        let second = MotionProfile::new(start[1], end[1], accelerations[1], speeds[1]).unwrap();
        let motion = make(start, end, speeds, accelerations);
        assert_eq!(
            motion.total_time(),
            first.total_time().max(second.total_time())
        );
    }
}

#[test]
fn samples_match_independent_profiles_at_the_same_fraction_of_their_duration() {
    for (start, end, speeds, accelerations) in CASES {
        let first = MotionProfile::new(start[0], end[0], accelerations[0], speeds[0]).unwrap();
        let second = MotionProfile::new(start[1], end[1], accelerations[1], speeds[1]).unwrap();
        let motion = make(start, end, speeds, accelerations);
        for i in 0..=32 {
            let fraction = f64::from(i) / 32.0;
            let expected = [
                first
                    .position_at(bounded(fraction * first.total_time()))
                    .unwrap(),
                second
                    .position_at(bounded(fraction * second.total_time()))
                    .unwrap(),
            ];
            assert_position(sample(&motion, fraction * motion.total_time()), expected);
        }
    }
}

#[test]
fn swapping_joints_and_their_limits_swaps_every_sample() {
    let swap = |values: [f64; 2]| [values[1], values[0]];
    for (start, end, speeds, accelerations) in CASES {
        let original = make(start, end, speeds, accelerations);
        let swapped = make(swap(start), swap(end), swap(speeds), swap(accelerations));
        assert_eq!(original.total_time(), swapped.total_time());
        for i in 0..=32 {
            let time = original.total_time() * (f64::from(i) / 32.0);
            assert_position(sample(&swapped, time), swap(sample(&original, time)));
        }
    }
}

#[test]
fn reflecting_angles_reflects_every_sample_and_preserves_duration() {
    let reflect = |values: [f64; 2]| [-values[0], -values[1]];
    for (start, end, speeds, accelerations) in CASES {
        let original = make(start, end, speeds, accelerations);
        let reflected = make(reflect(start), reflect(end), speeds, accelerations);
        assert_eq!(original.total_time(), reflected.total_time());
        for i in 0..=32 {
            let time = original.total_time() * (f64::from(i) / 32.0);
            assert_position(sample(&reflected, time), reflect(sample(&original, time)));
        }
    }
}

#[test]
fn each_joint_can_be_translated_independently() {
    let original = make([-4.0, 1.0], [0.0, 0.0], [4.0, 1.0], [4.0, 4.0]);
    for offsets in [[-9000.0, 9000.0], [123.5, -456.25], [0.0, 0.0]] {
        let translated = make(
            [-4.0 + offsets[0], 1.0 + offsets[1]],
            offsets,
            [4.0, 1.0],
            [4.0, 4.0],
        );
        assert_eq!(original.total_time(), translated.total_time());
        for i in 0..=32 {
            let time = original.total_time() * (f64::from(i) / 32.0);
            let reference = sample(&original, time);
            assert_position(
                sample(&translated, time),
                [reference[0] + offsets[0], reference[1] + offsets[1]],
            );
        }
    }
}

#[test]
fn reversing_both_moves_reverses_the_shared_time_sequence() {
    for (start, end, speeds, accelerations) in CASES {
        let forward = make(start, end, speeds, accelerations);
        let reverse = make(end, start, speeds, accelerations);
        assert_eq!(forward.total_time(), reverse.total_time());
        for i in 0..=32 {
            let time = forward.total_time() * (f64::from(i) / 32.0);
            assert_position(
                sample(&forward, time),
                sample(&reverse, forward.total_time() - time),
            );
        }
    }
}

#[test]
fn scaling_speed_and_acceleration_rescales_shared_time_without_changing_positions() {
    let original = make([-4.0, 1.0], [0.0, 0.0], [4.0, 1.0], [4.0, 4.0]);
    for factor in [0.25, 0.5, 1.0, 2.0, 4.0] {
        let scaled = make(
            [-4.0, 1.0],
            [0.0, 0.0],
            [4.0 * factor, factor],
            [4.0 * factor * factor, 4.0 * factor * factor],
        );
        assert_close(scaled.total_time(), original.total_time() / factor);
        for i in 0..=32 {
            let time = original.total_time() * (f64::from(i) / 32.0);
            assert_position(sample(&scaled, time / factor), sample(&original, time));
        }
    }
}

#[test]
fn sampled_positions_remain_monotonic_and_inside_each_joints_endpoints() {
    for (start, end, speeds, accelerations) in CASES {
        let motion = make(start, end, speeds, accelerations);
        let mut previous = sample(&motion, 0.0);
        for i in 1..=128 {
            let current = sample(&motion, motion.total_time() * (f64::from(i) / 128.0));
            for joint in 0..2 {
                assert!(current[joint] >= start[joint].min(end[joint]) - 1e-9);
                assert!(current[joint] <= start[joint].max(end[joint]) + 1e-9);
                if end[joint] >= start[joint] {
                    assert!(current[joint] >= previous[joint] - 1e-9);
                } else {
                    assert!(current[joint] <= previous[joint] + 1e-9);
                }
            }
            previous = current;
        }
    }
}

#[test]
fn synchronized_samples_respect_each_joints_speed_and_acceleration_limits() {
    for (start, end, speeds, accelerations) in CASES {
        let motion = make(start, end, speeds, accelerations);
        if motion.total_time() == 0.0 {
            continue;
        }
        let step = motion.total_time() / 128.0;
        let samples: Vec<_> = (0..=128)
            .map(|i| sample(&motion, motion.total_time() * (f64::from(i) / 128.0)))
            .collect();
        for joint in 0..2 {
            for pair in samples.windows(2) {
                assert!((pair[1][joint] - pair[0][joint]).abs() <= speeds[joint] * step + 1e-8);
            }
            for triple in samples.windows(3) {
                let second_difference =
                    (triple[2][joint] - 2.0 * triple[1][joint] + triple[0][joint]).abs();
                assert!(second_difference <= accelerations[joint] * step * step + 1e-8);
            }
        }
    }
}

#[test]
fn full_positive_and_negative_turns_are_preserved() {
    let start = [-2.0 * TAU, 3.0 * TAU];
    let end = [3.0 * TAU, -TAU];
    let motion = make(start, end, [TAU, TAU], [TAU, TAU]);
    assert_close(motion.total_time(), 6.0);
    assert_position(sample(&motion, 3.0), [TAU / 2.0, TAU]);
    assert_eq!(sample(&motion, motion.total_time()), end);
}

#[test]
fn shared_duration_agrees_with_joint_acceleration_limits_on_a_signed_grid() {
    for first in [-10000.0, -17.25, 0.0, 17.25, 10000.0] {
        for second in [-10000.0, -17.25, 0.0, 17.25, 10000.0] {
            for speeds in [[4.0, 16.0], [16.0, 4.0], [10000.0, 10000.0]] {
                for accelerations in [[1.0, 4.0], [4.0, 1.0], [10000.0, 10000.0]] {
                    let start = [first, second];
                    let end = [-first, -second];
                    let motion = make(start, end, speeds, accelerations);
                    let expected = JointAccelerationLimits::new(accelerations[0], accelerations[1])
                        .unwrap()
                        .min_duration_between(
                            angles(start),
                            angles(end),
                            &JointVelocityLimits::new(speeds[0], speeds[1]).unwrap(),
                        )
                        .unwrap();
                    assert_close(motion.total_time(), expected);
                }
            }
        }
    }
}

#[test]
fn new_acceleration_accessors_preserve_joint_order_and_values() {
    for first in [0.125, 1.0, 37.5, 10000.0] {
        for second in [0.125, 1.0, 37.5, 10000.0] {
            let limits = JointAccelerationLimits::new(bounded(first), bounded(second)).unwrap();
            assert_eq!(limits.first_joint_max_acceleration(), first);
            assert_eq!(limits.second_joint_max_acceleration(), second);
        }
    }
}

#[test]
fn profiles_accept_validated_trajectory_samples_in_both_elbow_configurations() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = LineSegment {
        start: Point::new(160.0, 0.0).unwrap(),
        end: Point::new(100.0, 60.0).unwrap(),
    };
    for configuration in [ElbowConfiguration::Positive, ElbowConfiguration::Negative] {
        let samples: Vec<_> = JointTrajectory::new(&robot, &path, 10000.0, configuration)
            .unwrap()
            .into_validated()
            .unwrap()
            .map(Result::unwrap)
            .collect();
        assert_eq!(samples.len(), 2);
        let start = [samples[0].first_theta(), samples[0].second_theta()];
        let end = [samples[1].first_theta(), samples[1].second_theta()];
        let motion = make(start, end, [10.0, 1.0], [10.0, 1.0]);
        assert_close(motion.total_time(), FRAC_PI_2 + 1.0);
        assert_eq!(sample(&motion, 0.0), start);
        assert_eq!(sample(&motion, motion.total_time()), end);
    }
}

// Precision regressions are intentionally ignored. Every case asserts the
// desired mathematical behavior and documents why the current code fails.
// Reproduce: cargo test -p planner --test joint_motion_profile -- --ignored

fn extreme_profile(end: [f64; 2], speeds: [f64; 2], accelerations: [f64; 2]) -> JointMotionProfile {
    JointMotionProfile::new(
        JointAngles::new(0.0, 0.0).unwrap(),
        JointAngles::new(end[0], end[1]).unwrap(),
        &JointVelocityLimits::new(speeds[0], speeds[1]).unwrap(),
        &JointAccelerationLimits::new(accelerations[0], accelerations[1]).unwrap(),
    )
    .unwrap()
}

#[test]
#[ignore = "precision defect: time*axis_duration overflows before division, rejecting a valid interior time as InvalidTime"]
fn regression_large_duration_rescaling_keeps_a_valid_midpoint() {
    for swap in [false, true] {
        let motion = if swap {
            extreme_profile([0.0, 1.0], [1.0, 1e-200], [1.0, 1.0])
        } else {
            extreme_profile([1.0, 0.0], [1e-200, 1.0], [1.0, 1.0])
        };
        let result = motion.position_at(motion.total_time() / 2.0).unwrap();
        assert_close(
            if swap {
                result.second_theta()
            } else {
                result.first_theta()
            },
            0.5,
        );
    }
}

#[test]
#[ignore = "precision defect: time*axis_duration underflows to zero before division, freezing a moving joint at its start"]
fn regression_tiny_duration_rescaling_preserves_a_nonzero_midpoint() {
    let scale = 2.0_f64.powi(-600);
    let motion = extreme_profile([scale, 0.0], [1.0, 1.0], [2.0_f64.powi(600), 1.0]);
    let result = motion.position_at(motion.total_time() / 2.0).unwrap();
    // Relative comparison avoids a fixed absolute tolerance masking tiny motion.
    assert_close(result.first_theta() / scale, 0.5);
}

#[test]
#[ignore = "inherited precision defect: a*d underflows in MotionProfile, and a zero-duration axis jumps to its end at global time zero"]
fn regression_underflowed_axis_keeps_its_start_while_the_other_axis_moves() {
    let scale = 2.0_f64.powi(-600);
    let motion = extreme_profile([scale, 1.0], [1.0, 4.0], [scale, 4.0]);
    assert_eq!(motion.position_at(0.0).unwrap().first_theta(), 0.0);
}

#[test]
#[ignore = "inherited precision defect: v*v underflows in stationary scalar profiles, assigning two nonzero ramps to zero movement"]
fn regression_stationary_joints_have_zero_duration_with_subnormal_limits() {
    let smallest = f64::from_bits(1);
    let motion = extreme_profile([0.0, 0.0], [smallest, smallest], [smallest, smallest]);
    assert_eq!(motion.total_time(), 0.0);
}

#[test]
#[ignore = "inherited precision defect: a*d overflows before sqrt even though the synchronized two-second duration is representable"]
fn regression_finite_duration_is_not_rejected_when_scalar_products_overflow() {
    let scale = 2.0_f64.powi(600);
    let motion = extreme_profile([scale, scale], [scale, scale], [scale, scale]);
    assert_eq!(motion.total_time(), 2.0);
}
