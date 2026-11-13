use kinematics::JointAngles;
use planner::{
    CoordinatedJointMotionProfile, JointAccelerationLimits, JointVelocityLimits, PlannerError,
};

fn angles(q: [f64; 2]) -> JointAngles {
    JointAngles::new(q[0], q[1]).unwrap()
}

fn next_speed(a: [f64; 2], b: [f64; 2], c: [f64; 2], speed: f64) -> Result<f64, PlannerError> {
    CoordinatedJointMotionProfile::calculate_next_progress_speed(
        angles(a),
        angles(b),
        angles(c),
        speed,
    )
}

#[test]
fn the_previous_two_to_one_displacement_example_requires_quarter_progress_speed() {
    assert_eq!(
        next_speed([0.0, 0.0], [2.0, -1.0], [6.0, -3.0], 0.5),
        Ok(0.25)
    );
}

#[test]
fn speed_scales_inversely_with_the_next_displacement() {
    for factor in [0.25, 0.5, 1.0, 2.0, 4.0] {
        assert_eq!(
            next_speed(
                [0.0, 0.0],
                [2.0, -1.0],
                [2.0 + 2.0 * factor, -1.0 - factor],
                0.5
            ),
            Ok(0.5 / factor)
        );
    }
}

#[test]
fn signs_and_translation_do_not_change_the_required_speed() {
    for sx in [-1.0, 1.0] {
        for sy in [-1.0, 1.0] {
            let a = [32.0, -64.0];
            let b = [a[0] + 2.0 * sx, a[1] + sy];
            let c = [a[0] + 6.0 * sx, a[1] + 3.0 * sy];
            assert_eq!(next_speed(a, b, c, 0.5), Ok(0.25));
        }
    }
}

#[test]
fn first_joint_may_remain_stationary_when_scaling_the_next_speed() {
    for sign in [-1.0, 1.0] {
        assert_eq!(
            next_speed([3.0, 0.0], [3.0, sign], [3.0, 3.0 * sign], 0.5),
            Ok(0.25)
        );
    }
}

#[test]
fn second_joint_may_remain_stationary_when_scaling_the_next_speed() {
    for sign in [-1.0, 1.0] {
        assert_eq!(
            next_speed([0.0, 3.0], [sign, 3.0], [3.0 * sign, 3.0], 0.5),
            Ok(0.25)
        );
    }
}

#[test]
fn swapping_joints_does_not_change_the_required_speed() {
    assert_eq!(
        next_speed([0.0, 3.0], [1.0, 3.0], [3.0, 3.0], 0.5),
        next_speed([3.0, 0.0], [3.0, 1.0], [3.0, 3.0], 0.5)
    );
}

#[test]
fn a_corner_requires_zero_next_progress_speed() {
    for c in [[6.0, -2.0], [4.0, 0.0], [2.0, 3.0]] {
        assert_eq!(next_speed([0.0, 0.0], [2.0, -1.0], c, 0.5), Ok(0.0));
    }
}

#[test]
fn reversing_direction_requires_zero_next_progress_speed() {
    for c in [[0.0, 0.0], [-2.0, 1.0]] {
        assert_eq!(next_speed([0.0, 0.0], [2.0, -1.0], c, 0.5), Ok(0.0));
    }
}

#[test]
fn a_degenerate_segment_returns_zero_next_progress_speed() {
    for (a, b, c) in [
        ([0.0; 2], [0.0; 2], [1.0; 2]),
        ([0.0; 2], [1.0; 2], [1.0; 2]),
        ([0.0, -0.0], [-0.0, 0.0], [0.0, -0.0]),
    ] {
        assert_eq!(next_speed(a, b, c, 0.5), Ok(0.0));
    }
}

#[test]
fn the_current_input_contract_requires_a_positive_finite_final_speed() {
    for invalid in [
        -1.0,
        -f64::from_bits(1),
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
    ] {
        // Input validation also precedes the stop shortcut at a corner.
        for c in [[6.0, -3.0], [4.0, 0.0]] {
            assert_eq!(
                next_speed([0.0, 0.0], [2.0, -1.0], c, invalid),
                Err(PlannerError::InvalidFinalSpeed)
            );
        }
    }
}

#[test]
fn displacement_overflow_propagates_from_either_segment_or_joint() {
    let large = f64::MAX;
    for (a, b, c) in [
        ([-large, 0.0], [large, 1.0], [large, 2.0]),
        ([0.0, -large], [1.0, large], [2.0, large]),
        ([large, 0.0], [large, 1.0], [-large, 2.0]),
        ([0.0, large], [1.0, large], [2.0, -large]),
    ] {
        assert_eq!(
            next_speed(a, b, c, 0.5),
            Err(PlannerError::NumericalFailure)
        );
    }
}

#[test]
fn unrepresentable_positive_speed_does_not_silently_turn_into_a_stop() {
    let tiny = f64::from_bits(1);
    // The first physical velocity is (tiny,tiny). Returning zero for the next
    // progress speed would make its physical velocity (0,0), breaking continuity.
    assert_eq!(
        next_speed([-tiny; 2], [0.0; 2], [2.0; 2], 1.0),
        Err(PlannerError::NumericalFailure)
    );
}

#[test]
fn overflowing_final_speed_is_rejected() {
    assert_eq!(
        next_speed([-2.0; 2], [0.0; 2], [1.0; 2], f64::MAX),
        Err(PlannerError::NumericalFailure)
    );
}

#[test]
fn smallest_positive_and_largest_finite_output_speeds_are_preserved() {
    let tiny = f64::from_bits(1);
    assert_eq!(next_speed([-1.0; 2], [0.0; 2], [1.0; 2], tiny), Ok(tiny));
    assert_eq!(next_speed([-tiny; 2], [0.0; 2], [1.0; 2], 1.0), Ok(tiny));
    assert_eq!(
        next_speed([-1.0; 2], [0.0; 2], [1.0; 2], f64::MAX),
        Ok(f64::MAX)
    );
}

#[test]
fn intermediate_multiplication_overflow_does_not_destroy_a_finite_result() {
    let speed = 2.0_f64.powi(900);
    let first_delta = 2.0_f64.powi(900);
    let second_delta = 2.0_f64.powi(1000);
    assert!((speed * first_delta).is_infinite());
    assert_eq!(
        next_speed([-first_delta; 2], [0.0; 2], [second_delta; 2], speed),
        Ok(2.0_f64.powi(800))
    );
}

#[test]
fn intermediate_multiplication_underflow_does_not_destroy_a_finite_result() {
    let speed = 2.0_f64.powi(-900);
    let first_delta = 2.0_f64.powi(-900);
    let second_delta = 2.0_f64.powi(-1000);
    assert_eq!(speed * first_delta, 0.0);
    assert_eq!(
        next_speed([-first_delta; 2], [0.0; 2], [second_delta; 2], speed),
        Ok(2.0_f64.powi(-800))
    );
}

#[test]
fn common_displacement_scales_preserve_the_required_speed() {
    for exponent in [-1074, -1000, -500, 0, 500, 1000, 1021] {
        let scale = libm::scalbn(1.0, exponent);
        assert!(scale.is_finite() && scale > 0.0);
        assert_eq!(
            next_speed([-scale; 2], [0.0; 2], [2.0 * scale; 2], 0.5),
            Ok(0.25)
        );
    }
}

#[test]
fn changing_the_incoming_speed_linearly_changes_the_outgoing_speed() {
    for speed in [2.0_f64.powi(-1000), 0.25, 1.0, 2.0_f64.powi(1000)] {
        assert_eq!(
            next_speed([-1.0; 2], [0.0; 2], [2.0; 2], speed),
            Ok(speed / 2.0)
        );
    }
}

#[test]
fn reversing_the_entire_chain_inverts_the_displacement_scaling() {
    let (a, b, c) = ([0.0, 0.0], [2.0, -1.0], [6.0, -3.0]);
    let outgoing = next_speed(a, b, c, 0.5).unwrap();
    assert_eq!(next_speed(c, b, a, outgoing), Ok(0.5));
}

#[test]
fn calculated_speed_preserves_joint_velocity_when_used_to_build_both_profiles() {
    let (a, b, c) = (angles([0.0, 0.0]), angles([2.0, -1.0]), angles([6.0, -3.0]));
    for (velocity, acceleration) in [([20.0; 2], [40.0; 2]), ([4.0, 3.0], [40.0, 30.0])] {
        let velocity_limits = JointVelocityLimits::new(velocity[0], velocity[1]).unwrap();
        let acceleration_limits =
            JointAccelerationLimits::new(acceleration[0], acceleration[1]).unwrap();
        let speed =
            CoordinatedJointMotionProfile::calculate_next_progress_speed(a, b, c, 0.5).unwrap();
        let first = CoordinatedJointMotionProfile::new(
            a,
            b,
            &velocity_limits,
            &acceleration_limits,
            0.0,
            0.5,
        )
        .unwrap();
        let second = CoordinatedJointMotionProfile::new(
            b,
            c,
            &velocity_limits,
            &acceleration_limits,
            speed,
            0.0,
        )
        .unwrap();
        assert_eq!(
            first.velocity_at(first.total_time()),
            second.velocity_at(0.0)
        );
        assert_eq!(second.velocity_at(0.0), Ok((1.0, -0.5)));
        let first_end = first.position_at(first.total_time()).unwrap();
        let second_start = second.position_at(0.0).unwrap();
        assert_eq!(
            [first_end.first_theta(), first_end.second_theta()],
            [2.0, -1.0]
        );
        assert_eq!(
            [second_start.first_theta(), second_start.second_theta()],
            [2.0, -1.0]
        );
        for profile in [&first, &second] {
            for step in 0..=1024 {
                let time = profile.total_time() * (f64::from(step) / 1024.0);
                let v = profile.velocity_at(time).unwrap();
                let a = profile.acceleration_at(time).unwrap();
                for (actual, limit) in [
                    (v.0, velocity[0]),
                    (v.1, velocity[1]),
                    (a.0, acceleration[0]),
                    (a.1, acceleration[1]),
                ] {
                    assert!(
                        actual.is_finite() && actual.abs() <= limit * (1.0 + 32.0 * f64::EPSILON)
                    );
                }
            }
        }
    }
}
