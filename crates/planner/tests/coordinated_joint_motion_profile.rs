use kinematics::JointAngles;
use planner::{
    CoordinatedJointMotionProfile, JointAccelerationLimits, JointVelocityLimits, PlannerError,
};

fn angles(values: [f64; 2]) -> JointAngles {
    JointAngles::new(values[0], values[1]).unwrap()
}

fn build(
    start: [f64; 2],
    end: [f64; 2],
    velocity: [f64; 2],
    acceleration: [f64; 2],
    initial: f64,
    final_speed: f64,
) -> Result<CoordinatedJointMotionProfile, PlannerError> {
    CoordinatedJointMotionProfile::new(
        angles(start),
        angles(end),
        &JointVelocityLimits::new(velocity[0], velocity[1]).unwrap(),
        &JointAccelerationLimits::new(acceleration[0], acceleration[1]).unwrap(),
        initial,
        final_speed,
    )
}

fn triangle() -> CoordinatedJointMotionProfile {
    // Progress a=1, v_max=2: a triangle lasting 2 seconds.
    build([1.0, 3.0], [5.0, 1.0], [8.0, 4.0], [4.0, 2.0], 0.0, 0.0).unwrap()
}

fn close(actual: f64, expected: f64) {
    assert!(actual.is_finite(), "nonfinite value: {actual:e}");
    assert!(
        (actual - expected).abs() <= 128.0 * f64::EPSILON * expected.abs().max(1.0),
        "expected {expected:e}, got {actual:e}"
    );
}

fn position(motion: &CoordinatedJointMotionProfile, time: f64) -> [f64; 2] {
    let result = motion.position_at(time).unwrap();
    [result.first_theta(), result.second_theta()]
}

#[test]
fn starts_at_the_supplied_angles_with_zero_velocity() {
    let motion = triangle();
    assert_eq!(position(&motion, 0.0), [1.0, 3.0]);
    assert_eq!(motion.velocity_at(0.0), Ok((0.0, 0.0)));
    assert_eq!(motion.velocity_at(2.0), Ok((0.0, 0.0)));
}

#[test]
fn triangular_positions_match_analytical_samples_in_both_directions() {
    for (start, end) in [([1.0, 3.0], [5.0, 1.0]), ([5.0, 1.0], [1.0, 3.0])] {
        let motion = build(start, end, [8.0, 4.0], [4.0, 2.0], 0.0, 0.0).unwrap();
        for (time, progress) in [
            (0.0, 0.0),
            (0.5, 0.125),
            (1.0, 0.5),
            (1.5, 0.875),
            (2.0, 1.0),
        ] {
            let actual = position(&motion, time);
            for joint in 0..2 {
                close(
                    actual[joint],
                    start[joint] + (end[joint] - start[joint]) * progress,
                );
            }
        }
    }
}

#[test]
fn triangular_velocities_follow_each_joint_displacement_sign() {
    let motion = triangle();
    for (time, progress_speed) in [(0.0, 0.0), (0.5, 0.5), (1.0, 1.0), (1.5, 0.5), (2.0, 0.0)] {
        let actual = motion.velocity_at(time).unwrap();
        close(actual.0, 4.0 * progress_speed);
        close(actual.1, -2.0 * progress_speed);
    }
}

#[test]
fn triangular_acceleration_changes_sign_after_the_peak() {
    let motion = triangle();
    for (time, progress_acceleration) in [
        (0.0, 1.0),
        (1.0, 1.0),
        (1.0_f64.next_up(), -1.0),
        (2.0, -1.0),
    ] {
        assert_eq!(
            motion.acceleration_at(time),
            Ok((4.0 * progress_acceleration, -2.0 * progress_acceleration))
        );
    }
}

#[test]
fn both_axes_arrive_at_the_exact_requested_endpoints() {
    let motion = triangle();
    assert_eq!(position(&motion, 2.0), [5.0, 1.0]);
}

#[test]
fn trapezoidal_motion_has_the_expected_cruise_and_ramps() {
    // Progress v_max=1/4, a=1: ramp=1/4, total=17/4.
    let motion = build([0.0, 0.0], [4.0, -2.0], [1.0, 0.5], [4.0, 2.0], 0.0, 0.0).unwrap();
    for (time, progress, speed, acceleration) in [
        (0.125, 0.0078125, 0.125, 1.0),
        (0.25, 0.03125, 0.25, 1.0),
        (2.125, 0.5, 0.25, 0.0),
        (4.0, 0.96875, 0.25, 0.0),
        (4.125, 0.9921875, 0.125, -1.0),
        (4.25, 1.0, 0.0, -1.0),
    ] {
        let q = position(&motion, time);
        let v = motion.velocity_at(time).unwrap();
        let a = motion.acceleration_at(time).unwrap();
        close(q[0], 4.0 * progress);
        close(q[1], -2.0 * progress);
        close(v.0, 4.0 * speed);
        close(v.1, -2.0 * speed);
        close(a.0, 4.0 * acceleration);
        close(a.1, -2.0 * acceleration);
    }
}

#[test]
fn asymmetric_boundary_speeds_are_progress_speeds() {
    // Progress a=.2, vi=.1, vf=.3, peak=.5: ramps last 2s and 1s.
    let motion = build([2.0, 6.0], [4.0, 2.0], [2.0, 4.0], [0.4, 0.8], 0.1, 0.3).unwrap();
    for (time, progress, speed) in [
        (0.0, 0.0, 0.1),
        (1.0, 0.2, 0.3),
        (2.0, 0.6, 0.5),
        (2.5, 0.825, 0.4),
        (3.0, 1.0, 0.3),
    ] {
        let q = position(&motion, time);
        let v = motion.velocity_at(time).unwrap();
        close(v.0, 2.0 * speed);
        close(v.1, -4.0 * speed);
        close(q[0], 2.0 + 2.0 * progress);
        close(q[1], 6.0 - 4.0 * progress);
    }
}

#[test]
fn boundary_speed_at_the_cap_gives_constant_joint_velocity() {
    let motion = build([0.0, 0.0], [2.0, -4.0], [1.0, 2.0], [2.0, 4.0], 0.5, 0.5).unwrap();
    for time in [0.0, 0.5, 1.0, 2.0] {
        assert_eq!(motion.acceleration_at(time), Ok((0.0, 0.0)));
        assert_eq!(motion.velocity_at(time), Ok((1.0, -2.0)));
        assert_eq!(position(&motion, time), [time, -2.0 * time]);
    }
}

#[test]
fn first_joint_can_remain_stationary() {
    let motion = build([7.0, 0.0], [7.0, 4.0], [1.0, 8.0], [1.0, 4.0], 0.0, 0.0).unwrap();
    for time in [0.0, 0.5, 1.0, 2.0] {
        assert_eq!(position(&motion, time)[0], 7.0);
        assert_eq!(motion.velocity_at(time).unwrap().0, 0.0);
        assert_eq!(motion.acceleration_at(time).unwrap().0, 0.0);
    }
    assert_eq!(position(&motion, 2.0), [7.0, 4.0]);
}

#[test]
fn second_joint_can_remain_stationary() {
    let motion = build([0.0, -7.0], [4.0, -7.0], [8.0, 1.0], [4.0, 1.0], 0.0, 0.0).unwrap();
    for time in [0.0, 0.5, 1.0, 2.0] {
        assert_eq!(position(&motion, time)[1], -7.0);
        assert_eq!(motion.velocity_at(time).unwrap().1, 0.0);
        assert_eq!(motion.acceleration_at(time).unwrap().1, 0.0);
    }
    assert_eq!(position(&motion, 2.0), [4.0, -7.0]);
}

#[test]
fn zero_displacement_is_rejected_including_signed_zeros() {
    for (start, end) in [
        ([0.0, -0.0], [-0.0, 0.0]),
        ([3.0, -2.0], [3.0, -2.0]),
        ([f64::MAX, -f64::MAX], [f64::MAX, -f64::MAX]),
    ] {
        assert_eq!(
            build(start, end, [1.0, 1.0], [1.0, 1.0], 0.0, 0.0).unwrap_err(),
            PlannerError::ZeroDisplacementMotion
        );
    }
}

#[test]
fn invalid_initial_progress_speeds_have_a_specific_error() {
    for invalid in [
        -1.0,
        -f64::from_bits(1),
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
    ] {
        assert_eq!(
            build([0.0, 0.0], [4.0, 2.0], [8.0, 4.0], [4.0, 2.0], invalid, 0.0).unwrap_err(),
            PlannerError::InvalidInitialSpeed
        );
    }
}

#[test]
fn invalid_final_progress_speeds_have_a_specific_error() {
    for invalid in [
        -1.0,
        -f64::from_bits(1),
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
    ] {
        assert_eq!(
            build([0.0, 0.0], [4.0, 2.0], [8.0, 4.0], [4.0, 2.0], 0.0, invalid).unwrap_err(),
            PlannerError::InvalidFinalSpeed
        );
    }
}

#[test]
fn boundary_speeds_cannot_exceed_the_most_restrictive_joint_limit() {
    for (initial, final_speed) in [(0.5_f64.next_up(), 0.0), (0.0, 0.5_f64.next_up())] {
        assert_eq!(
            build(
                [0.0, 0.0],
                [2.0, 4.0],
                [1.0, 20.0],
                [2.0, 4.0],
                initial,
                final_speed
            )
            .unwrap_err(),
            PlannerError::InvalidBoundarySpeed
        );
    }
}

#[test]
fn infeasible_progress_speed_changes_are_rejected() {
    for (initial, final_speed) in [(0.0, 2.0), (2.0, 0.0)] {
        assert_eq!(
            build(
                [0.0, 0.0],
                [2.0, 4.0],
                [8.0, 16.0],
                [2.0, 4.0],
                initial,
                final_speed
            )
            .unwrap_err(),
            PlannerError::InfeasibleMotion
        );
    }
}

#[test]
fn negative_zero_boundary_speeds_are_valid() {
    let motion = build([1.0, 3.0], [5.0, 1.0], [8.0, 4.0], [4.0, 2.0], -0.0, -0.0).unwrap();
    assert_eq!(position(&motion, -0.0), [1.0, 3.0]);
    assert_eq!(motion.velocity_at(-0.0), Ok((0.0, 0.0)));
}

#[test]
fn invalid_times_are_rejected_by_all_three_queries() {
    let motion = triangle();
    for time in [
        -1.0,
        -f64::from_bits(1),
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
        2.0_f64.next_up(),
        f64::MAX,
    ] {
        assert!(
            matches!(motion.position_at(time), Err(PlannerError::InvalidTime)),
            "time={time:?}"
        );
        assert_eq!(motion.velocity_at(time), Err(PlannerError::InvalidTime));
        assert_eq!(motion.acceleration_at(time), Err(PlannerError::InvalidTime));
    }
}

#[test]
fn overflowing_displacements_are_rejected_for_either_axis() {
    for sign in [-1.0, 1.0] {
        for (start, end) in [
            ([sign * f64::MAX, 0.0], [-sign * f64::MAX, 1.0]),
            ([0.0, sign * f64::MAX], [1.0, -sign * f64::MAX]),
        ] {
            assert_eq!(
                build(start, end, [1.0, 1.0], [1.0, 1.0], 0.0, 0.0).unwrap_err(),
                PlannerError::NumericalFailure
            );
        }
    }
}

#[test]
fn progress_limits_that_underflow_to_zero_are_rejected() {
    let tiny = f64::from_bits(1);
    for (velocity, acceleration) in [
        ([tiny, 1.0], [1.0, 1.0]),
        ([1.0, tiny], [1.0, 1.0]),
        ([1.0, 1.0], [tiny, 1.0]),
        ([1.0, 1.0], [1.0, tiny]),
    ] {
        assert_eq!(
            build([0.0, 0.0], [2.0, 2.0], velocity, acceleration, 0.0, 0.0).unwrap_err(),
            PlannerError::NumericalFailure
        );
    }
}

#[test]
fn unrepresentable_progress_limits_are_rejected() {
    let tiny = f64::from_bits(1);
    assert_eq!(
        build([0.0, 0.0], [tiny, tiny], [1.0, 1.0], [1.0, 1.0], 0.0, 0.0).unwrap_err(),
        PlannerError::NumericalFailure
    );
}

#[test]
fn finite_joint_outputs_remain_within_the_supplied_limits() {
    let motion = triangle();
    for index in 0..=128 {
        let time = f64::from(index) / 64.0;
        let velocity = motion.velocity_at(time).unwrap();
        let acceleration = motion.acceleration_at(time).unwrap();
        assert!(velocity.0.abs() <= 8.0 && velocity.1.abs() <= 4.0);
        assert!(acceleration.0.abs() <= 4.0 && acceleration.1.abs() <= 2.0);
    }
}

#[test]
fn returned_velocity_and_acceleration_match_position_derivatives() {
    // This checks internal consistency separately from endpoint/direction tests.
    let motion = triangle();
    let h = 1e-4;
    for time in [0.25, 0.75, 1.25, 1.75] {
        let before = position(&motion, time - h);
        let after = position(&motion, time + h);
        let velocity = motion.velocity_at(time).unwrap();
        let v_before = motion.velocity_at(time - h).unwrap();
        let v_after = motion.velocity_at(time + h).unwrap();
        let acceleration = motion.acceleration_at(time).unwrap();
        assert!(((after[0] - before[0]) / (2.0 * h) - velocity.0).abs() < 1e-9);
        assert!(((after[1] - before[1]) / (2.0 * h) - velocity.1).abs() < 1e-9);
        assert!(((v_after.0 - v_before.0) / (2.0 * h) - acceleration.0).abs() < 1e-9);
        assert!(((v_after.1 - v_before.1) / (2.0 * h) - acceleration.1).abs() < 1e-9);
    }
}

#[test]
fn extreme_finite_scales_preserve_the_expected_joint_positions() {
    for exponent in [-1000, -500, 0, 500, 1000] {
        let scale = 2.0_f64.powi(exponent);
        let motion = build(
            [0.0, 0.0],
            [scale, -scale],
            [2.0 * scale; 2],
            [scale; 2],
            0.0,
            0.0,
        )
        .unwrap();
        let q = position(&motion, 1.0);
        close(q[0] / scale, 0.5);
        close(q[1] / scale, -0.5);
    }
}

#[test]
fn large_finite_endpoints_are_sampleable_without_spurious_overflow() {
    let large = f64::MAX;
    let motion = build(
        [large, 1.0],
        [large / 2.0, 2.0],
        [large, 4.0],
        [large, 4.0],
        0.0,
        0.0,
    )
    .unwrap();
    // The first joint sets progress a=2, v_max=2. At t=.5 the
    // accelerating progress is .25, safely before the triangular peak.
    assert!(motion.position_at(0.5).is_ok());
    assert_eq!(position(&motion, 0.5), [large * 0.875, 1.25]);
}
