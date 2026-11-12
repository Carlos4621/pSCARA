use kinematics::JointAngles;
use planner::{
    JointAccelerationLimits, JointMotionProfile, JointVelocityLimits, MotionProfile, PlannerError,
};

fn bounded(value: f64) -> f64 {
    assert!((-10000.0..=10000.0).contains(&value));
    value
}

fn make(start: [f64; 2], end: [f64; 2], speeds: [f64; 2], a: [f64; 2]) -> JointMotionProfile {
    JointMotionProfile::new(
        JointAngles::new(bounded(start[0]), bounded(start[1])).unwrap(),
        JointAngles::new(bounded(end[0]), bounded(end[1])).unwrap(),
        &JointVelocityLimits::new(bounded(speeds[0]), bounded(speeds[1])).unwrap(),
        &JointAccelerationLimits::new(bounded(a[0]), bounded(a[1])).unwrap(),
    )
    .unwrap()
}

fn acceleration(motion: &JointMotionProfile, time: f64) -> [f64; 2] {
    let value = motion.acceleration_at(bounded(time)).unwrap();
    [value.0, value.1]
}

fn velocity(motion: &JointMotionProfile, time: f64) -> [f64; 2] {
    let value = motion.velocity_at(bounded(time)).unwrap();
    [value.0, value.1]
}

fn assert_close(actual: [f64; 2], expected: [f64; 2]) {
    for joint in 0..2 {
        assert!(actual[joint].is_finite());
        assert!(
            (actual[joint] - expected[joint]).abs()
                <= 1e-10 + 128.0 * f64::EPSILON * expected[joint].abs(),
            "axis {joint}: expected {}, got {}",
            expected[joint],
            actual[joint]
        );
    }
}

type MotionCase = ([f64; 2], [f64; 2], [f64; 2], [f64; 2]);
const CASES: [MotionCase; 5] = [
    ([0.0, 0.0], [4.0, 1.0], [4.0, 4.0], [4.0, 4.0]),
    ([-2.0, 1.0], [2.0, 0.0], [4.0, 1.0], [4.0, 4.0]),
    ([0.0, 0.0], [4.0, 1.0], [4.0, 0.25], [4.0, 1.0]),
    (
        [-10000.0, 10000.0],
        [10000.0, -10000.0],
        [10000.0, 10000.0],
        [10000.0, 10000.0],
    ),
    ([12.5, -1.0], [12.5, 0.0], [10000.0, 4.0], [0.25, 4.0]),
];

#[test]
fn synchronized_triangles_scale_acceleration_by_the_squared_time_factor() {
    let motion = make([0.0, 0.0], [4.0, 1.0], [4.0, 4.0], [4.0, 4.0]);
    // Factor 1/2 for axis 2 gives global acceleration 4*(1/2)^2=1.
    // Exact peak follows the scalar API's left-phase convention.
    for (time, expected) in [
        (0.0, [4.0, 1.0]),
        (0.5, [4.0, 1.0]),
        (1.0, [4.0, 1.0]),
        (1.5, [-4.0, -1.0]),
        (2.0, [-4.0, -1.0]),
    ] {
        assert_close(acceleration(&motion, time), expected);
    }
}

#[test]
fn mixed_profiles_have_independent_ramp_cruise_and_deceleration_phases() {
    let motion = make([0.0, 0.0], [4.0, 1.0], [4.0, 1.0], [4.0, 4.0]);
    for (time, expected) in [
        (0.0, [4.0, 1.5625]),
        (0.2, [4.0, 1.5625]),
        (0.4, [4.0, 1.5625]),
        (0.6, [4.0, 0.0]),
        (1.2, [-4.0, 0.0]),
        (1.6, [-4.0, 0.0]),
        (1.8, [-4.0, -1.5625]),
        (2.0, [-4.0, -1.5625]),
    ] {
        assert_close(acceleration(&motion, time), expected);
    }
}

#[test]
fn equal_durations_preserve_signed_accelerations_for_both_axes() {
    let motion = make([-4.0, 1.0], [0.0, 0.0], [4.0, 1.0], [4.0, 1.0]);
    assert_eq!(motion.total_time(), 2.0);
    assert_close(acceleration(&motion, 0.5), [4.0, -1.0]);
    assert_close(acceleration(&motion, 1.5), [-4.0, 1.0]);
}

#[test]
fn either_axis_can_be_slower_without_swapping_acceleration_limits() {
    let motion = make([0.0, 0.0], [4.0, 1.0], [4.0, 0.25], [4.0, 1.0]);
    let swapped = make([0.0, 0.0], [1.0, 4.0], [0.25, 4.0], [1.0, 4.0]);
    let first = 4.0 * (2.0 / 4.25) * (2.0 / 4.25);
    assert_close(acceleration(&motion, 0.125), [first, 1.0]);
    assert_close(acceleration(&swapped, 0.125), [1.0, first]);
    assert_close(acceleration(&motion, 2.125), [first, 0.0]);
}

#[test]
fn a_stationary_axis_has_zero_global_acceleration_while_the_other_moves() {
    for (start, end, speeds, a, stationary) in [
        ([12.5, -1.0], [12.5, 0.0], [10000.0, 4.0], [0.25, 4.0], 0),
        ([-1.0, 12.5], [0.0, 12.5], [4.0, 10000.0], [4.0, 0.25], 1),
    ] {
        let motion = make(start, end, speeds, a);
        for i in 0..=64 {
            assert_eq!(
                acceleration(&motion, motion.total_time() * (f64::from(i) / 64.0))[stationary],
                0.0
            );
        }
    }
}

#[test]
fn two_stationary_axes_return_zero_acceleration_at_zero_time() {
    // Ordinary logical regression: total_time=0 must not lead to 0/0 and
    // InvalidTime. Keep active, like the scalar stationary regression.
    for at in [[-10000.0, 10000.0], [-12.5, -0.125], [-0.0, 0.0]] {
        let motion = make(at, at, [0.25, 10000.0], [10000.0, 0.25]);
        assert_eq!(motion.total_time(), 0.0);
        assert_eq!(motion.velocity_at(0.0), Ok((0.0, 0.0)));
        assert_eq!(motion.acceleration_at(0.0), Ok((0.0, 0.0)));
        assert_eq!(motion.acceleration_at(-0.0), Ok((0.0, 0.0)));
    }
}

#[test]
fn finite_invalid_times_are_rejected_including_for_stationary_profiles() {
    for (start, end) in [([0.0, 0.0], [4.0, 1.0]), ([-1.0, 1.0], [-1.0, 1.0])] {
        let motion = make(start, end, [4.0, 4.0], [4.0, 4.0]);
        for time in [-10000.0, -0.125, motion.total_time() + 0.125, 10000.0] {
            assert_eq!(
                motion.acceleration_at(bounded(time)),
                Err(PlannerError::InvalidTime)
            );
        }
    }
}

#[test]
fn global_acceleration_matches_the_derivative_of_global_velocity() {
    for (start, end, speeds, a) in [CASES[0], CASES[1], CASES[2]] {
        let motion = make(start, end, speeds, a);
        for fraction in [0.125, 0.375, 0.625, 0.875] {
            let time = motion.total_time() * fraction;
            let h = 0.0001;
            let before = velocity(&motion, time - h);
            let after = velocity(&motion, time + h);
            assert_close(
                acceleration(&motion, time),
                [
                    (after[0] - before[0]) / (2.0 * h),
                    (after[1] - before[1]) / (2.0 * h),
                ],
            );
        }
    }
}

#[test]
fn integrating_each_acceleration_recovers_both_global_velocities() {
    for (start, end, speeds, a) in [CASES[0], CASES[1], CASES[2], CASES[3]] {
        let motion = make(start, end, speeds, a);
        let mut integral = [0.0, 0.0];
        let mut previous_time = 0.0;
        // 1020 midpoint intervals align with all global phase transitions
        // in these profiles, including the slow trapezoid's 1/17 ramp.
        for i in 1..=1020 {
            let time = motion.total_time() * (f64::from(i) / 1020.0);
            let values = acceleration(&motion, (previous_time + time) / 2.0);
            let expected = velocity(&motion, time);
            for joint in 0..2 {
                integral[joint] += values[joint] * (time - previous_time);
                assert!((integral[joint] - expected[joint]).abs() <= 1e-8);
            }
            previous_time = time;
        }
    }
}

#[test]
fn sampled_accelerations_are_finite_and_obey_each_joints_limit() {
    for (start, end, speeds, a) in CASES {
        let motion = make(start, end, speeds, a);
        for i in 0..=128 {
            let values = acceleration(&motion, motion.total_time() * (f64::from(i) / 128.0));
            for joint in 0..2 {
                assert!(values[joint].is_finite());
                assert!(values[joint].abs() <= a[joint] + 1e-9);
            }
        }
    }
}

#[test]
fn independent_scalar_accelerations_use_the_squared_time_factors() {
    for (start, end, speeds, a) in CASES {
        let scalar = [
            MotionProfile::new(start[0], end[0], a[0], speeds[0]).unwrap(),
            MotionProfile::new(start[1], end[1], a[1], speeds[1]).unwrap(),
        ];
        let motion = make(start, end, speeds, a);
        for i in 0..=64 {
            let fraction = f64::from(i) / 64.0;
            let mut expected = [0.0, 0.0];
            for joint in 0..2 {
                let factor = scalar[joint].total_time() / motion.total_time();
                expected[joint] = scalar[joint]
                    .acceleration_at(bounded(fraction * scalar[joint].total_time()))
                    .unwrap()
                    * factor
                    * factor;
            }
            assert_close(
                acceleration(&motion, fraction * motion.total_time()),
                expected,
            );
        }
    }
}

#[test]
fn swapping_axes_and_reflecting_angles_transform_acceleration_correctly() {
    let swap = |x: [f64; 2]| [x[1], x[0]];
    let reflect = |x: [f64; 2]| [-x[0], -x[1]];
    for (start, end, speeds, a) in CASES {
        let original = make(start, end, speeds, a);
        let swapped = make(swap(start), swap(end), swap(speeds), swap(a));
        let reflected = make(reflect(start), reflect(end), speeds, a);
        for i in 0..=64 {
            let time = original.total_time() * (f64::from(i) / 64.0);
            let expected = acceleration(&original, time);
            assert_close(acceleration(&swapped, time), swap(expected));
            assert_close(acceleration(&reflected, time), reflect(expected));
        }
    }
}

#[test]
fn translating_angles_does_not_change_global_acceleration() {
    let original = make([-2.0, 1.0], [2.0, 0.0], [4.0, 1.0], [4.0, 4.0]);
    for offsets in [[-9000.0, 9000.0], [123.5, -456.25]] {
        let translated = make(
            [-2.0 + offsets[0], 1.0 + offsets[1]],
            [2.0 + offsets[0], offsets[1]],
            [4.0, 1.0],
            [4.0, 4.0],
        );
        for i in 0..=64 {
            let time = original.total_time() * (f64::from(i) / 64.0);
            assert_close(
                acceleration(&translated, time),
                acceleration(&original, time),
            );
        }
    }
}

#[test]
fn scaling_limits_rescales_global_acceleration_by_the_square_of_the_factor() {
    for (start, end, speeds, a) in [CASES[0], CASES[1], CASES[2]] {
        let original = make(start, end, speeds, a);
        for factor in [0.25, 0.5, 1.0, 2.0, 4.0] {
            let scaled = make(
                start,
                end,
                [speeds[0] * factor, speeds[1] * factor],
                [a[0] * factor * factor, a[1] * factor * factor],
            );
            for fraction in [0.125, 0.375, 0.625, 0.875] {
                let time = original.total_time() * fraction;
                let expected = acceleration(&original, time);
                assert_close(
                    acceleration(&scaled, time / factor),
                    [expected[0] * factor * factor, expected[1] * factor * factor],
                );
            }
        }
    }
}

#[test]
fn queries_are_repeatable_without_consuming_position_or_velocity() {
    let motion = make([0.0, 0.0], [4.0, 1.0], [4.0, 4.0], [4.0, 4.0]);
    for (time, expected) in [
        (1.5, [-4.0, -1.0]),
        (0.5, [4.0, 1.0]),
        (2.0, [-4.0, -1.0]),
        (0.0, [4.0, 1.0]),
        (1.5, [-4.0, -1.0]),
    ] {
        assert_eq!(motion.acceleration_at(-1.0), Err(PlannerError::InvalidTime));
        assert_close(acceleration(&motion, time), expected);
        assert_eq!(motion.velocity_at(2.0), Ok((0.0, 0.0)));
        let end = motion.position_at(2.0).unwrap();
        assert_eq!([end.first_theta(), end.second_theta()], [4.0, 1.0]);
    }
}

// Numerical regressions run normally; only the separately marked IEEE
// validation case requires --include-ignored.
fn extreme(end: [f64; 2], speeds: [f64; 2], a: [f64; 2]) -> JointMotionProfile {
    JointMotionProfile::new(
        JointAngles::new(0.0, 0.0).unwrap(),
        JointAngles::new(end[0], end[1]).unwrap(),
        &JointVelocityLimits::new(speeds[0], speeds[1]).unwrap(),
        &JointAccelerationLimits::new(a[0], a[1]).unwrap(),
    )
    .unwrap()
}

#[test]
fn regression_underflowed_progress_preserves_the_correct_acceleration_phase() {
    let speed = 2.0_f64.powi(-600);
    let motion = extreme([1.0, 0.0], [speed, 1.0], [1.0, 1.0]);
    // The ramp ends at speed/a = speed. At twice that time, the slower
    // scalar axis is already cruising and must have zero acceleration.
    assert_eq!(motion.acceleration_at(2.0 * speed).unwrap().0, 0.0);
}

#[test]
fn regression_underflowed_scalar_duration_preserves_initial_global_acceleration() {
    let scale = 2.0_f64.powi(-600);
    let motion = extreme([scale, 1.0], [1.0, 4.0], [scale, 4.0]);
    // The correct first-axis duration is 2, so it should control shared
    // time and keep its initial acceleration equal to scale.
    assert_eq!(motion.acceleration_at(0.0).unwrap().0 / scale, 1.0);
}

#[test]
#[ignore = "IEEE validation boundary: NaN and infinities must return InvalidTime before processing moving or zero-duration profiles"]
fn nonfinite_times_are_rejected_for_all_shared_durations() {
    for end in [[4.0, 1.0], [0.0, 0.0], [0.0, -1.0]] {
        let motion = extreme(end, [4.0, 4.0], [4.0, 4.0]);
        for time in [f64::NAN, -f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert_eq!(motion.acceleration_at(time), Err(PlannerError::InvalidTime));
        }
    }
}
