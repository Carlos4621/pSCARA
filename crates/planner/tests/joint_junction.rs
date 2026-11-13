use kinematics::JointAngles;
use planner::{
    CoordinatedJointMotionProfile, JointAccelerationLimits, JointVelocityLimits, PlannerError,
    calculate_max_junction_progress_speeds,
};

type Pair = [f64; 2];

fn angles(q: Pair) -> JointAngles {
    JointAngles::new(q[0], q[1]).unwrap()
}

fn junction(
    a: Pair,
    b: Pair,
    c: Pair,
    velocity: Pair,
    acceleration: Pair,
) -> Result<(f64, f64), PlannerError> {
    calculate_max_junction_progress_speeds(
        angles(a),
        angles(b),
        angles(c),
        &JointVelocityLimits::new(velocity[0], velocity[1]).unwrap(),
        JointAccelerationLimits::new(acceleration[0], acceleration[1]).unwrap(),
    )
}

fn close(actual: f64, expected: f64) {
    assert!(actual.is_finite());
    assert!(
        actual == expected || (actual - expected).abs() <= 128.0 * f64::EPSILON * expected.abs(),
        "expected {expected:e}, got {actual:e}"
    );
}

fn assert_profiles(points: [Pair; 3], velocity: Pair, acceleration: Pair, speeds: (f64, f64)) {
    let [a, b, c] = points.map(angles);
    let vl = JointVelocityLimits::new(velocity[0], velocity[1]).unwrap();
    let al = JointAccelerationLimits::new(acceleration[0], acceleration[1]).unwrap();
    let first = CoordinatedJointMotionProfile::new(a, b, &vl, &al, 0.0, speeds.0).unwrap();
    let second = CoordinatedJointMotionProfile::new(b, c, &vl, &al, speeds.1, 0.0).unwrap();
    let before = first.velocity_at(first.total_time()).unwrap();
    let after = second.velocity_at(0.0).unwrap();
    close(before.0, after.0);
    close(before.1, after.1);
    assert_eq!(first.velocity_at(0.0), Ok((0.0, 0.0)));
    assert_eq!(second.velocity_at(second.total_time()), Ok((0.0, 0.0)));
    let before_position = first.position_at(first.total_time()).unwrap();
    let after_position = second.position_at(0.0).unwrap();
    assert_eq!(
        [
            before_position.first_theta(),
            before_position.second_theta()
        ],
        points[1]
    );
    assert_eq!(
        [after_position.first_theta(), after_position.second_theta()],
        points[1]
    );
    for profile in [&first, &second] {
        for step in 0..=32 {
            let time = profile.total_time() * (f64::from(step) / 32.0);
            let v = profile.velocity_at(time).unwrap();
            let a = profile.acceleration_at(time).unwrap();
            for (actual, limit) in [
                (v.0, velocity[0]),
                (v.1, velocity[1]),
                (a.0, acceleration[0]),
                (a.1, acceleration[1]),
            ] {
                assert!(actual.is_finite() && actual.abs() <= limit * (1.0 + 32.0 * f64::EPSILON));
            }
        }
    }
}

#[test]
fn acceleration_distance_before_b_can_limit_the_crossing_speed() {
    // Joint displacement (2,-1), acceleration (4,2): reaching (4,-2)
    // from rest consumes exactly AB. The longer BC allows braking afterward.
    let points = [[0.0, 0.0], [2.0, -1.0], [6.0, -3.0]];
    let speeds = junction(points[0], points[1], points[2], [20.0, 10.0], [4.0, 2.0]).unwrap();
    assert_eq!(speeds, (2.0, 1.0));
    assert_profiles(points, [20.0, 10.0], [4.0, 2.0], speeds);
}

#[test]
fn braking_distance_after_b_can_limit_the_crossing_speed() {
    let points = [[0.0, 0.0], [4.0, -2.0], [6.0, -3.0]];
    let speeds = junction(points[0], points[1], points[2], [20.0, 10.0], [4.0, 2.0]).unwrap();
    assert_eq!(speeds, (1.0, 2.0));
    assert_profiles(points, [20.0, 10.0], [4.0, 2.0], speeds);
}

#[test]
fn joint_velocity_caps_limit_the_junction_even_when_acceleration_is_large() {
    let points = [[0.0, 0.0], [2.0, -1.0], [6.0, -3.0]];
    let speeds = junction(points[0], points[1], points[2], [2.0, 1.0], [40.0, 20.0]).unwrap();
    assert_eq!(speeds, (1.0, 0.5));
    assert_profiles(points, [2.0, 1.0], [40.0, 20.0], speeds);
}

#[test]
fn different_joints_can_supply_the_velocity_and_acceleration_constraints() {
    let points = [[0.0, 0.0], [2.0, -1.0], [6.0, -3.0]];
    let speeds = junction(points[0], points[1], points[2], [20.0, 1.0], [0.25, 20.0]).unwrap();
    assert_eq!(speeds, (0.5, 0.25));
    assert_profiles(points, [20.0, 1.0], [0.25, 20.0], speeds);
}

#[test]
fn equal_segments_share_the_same_maximum_progress_speed() {
    let points = [[0.0; 2], [1.0; 2], [2.0; 2]];
    let speeds = junction(points[0], points[1], points[2], [10.0; 2], [2.0; 2]).unwrap();
    assert_eq!(speeds, (2.0, 2.0));
    assert_profiles(points, [10.0; 2], [2.0; 2], speeds);
}

#[test]
fn either_joint_can_stay_stationary_without_changing_the_crossing_speed() {
    for points in [
        [[3.0, 0.0], [3.0, 2.0], [3.0, 6.0]],
        [[0.0, 3.0], [2.0, 3.0], [6.0, 3.0]],
    ] {
        let speeds = junction(points[0], points[1], points[2], [20.0; 2], [4.0; 2]).unwrap();
        assert_eq!(speeds, (2.0, 1.0));
        assert_profiles(points, [20.0; 2], [4.0; 2], speeds);
    }
}

#[test]
fn reflection_and_translation_preserve_maximum_speeds() {
    for sx in [-1.0, 1.0] {
        for sy in [-1.0, 1.0] {
            let a = [32.0, -64.0];
            let b = [a[0] + 2.0 * sx, a[1] + sy];
            let c = [a[0] + 6.0 * sx, a[1] + 3.0 * sy];
            let speeds = junction(a, b, c, [20.0, 10.0], [4.0, 2.0]).unwrap();
            assert_eq!(speeds, (2.0, 1.0));
            assert_profiles([a, b, c], [20.0, 10.0], [4.0, 2.0], speeds);
        }
    }
}

#[test]
fn reversing_the_entire_path_exchanges_incoming_and_outgoing_progress_speeds() {
    let (a, b, c) = ([0.0, 0.0], [2.0, -1.0], [6.0, -3.0]);
    let forward = junction(a, b, c, [20.0, 10.0], [4.0, 2.0]).unwrap();
    let backward = junction(c, b, a, [20.0, 10.0], [4.0, 2.0]).unwrap();
    close(forward.0, backward.1);
    close(forward.1, backward.0);
}

#[test]
fn corners_reversals_and_degenerate_segments_require_a_stop() {
    for (a, b, c) in [
        ([0.0, 0.0], [2.0, -1.0], [6.0, -2.0]),
        ([0.0, 0.0], [2.0, -1.0], [0.0, 0.0]),
        ([0.0, 0.0], [1.0, 0.0], [1.0, 1.0]),
        ([0.0; 2], [0.0; 2], [1.0; 2]),
        ([0.0; 2], [1.0; 2], [1.0; 2]),
        ([0.0, -0.0], [-0.0, 0.0], [0.0, -0.0]),
    ] {
        assert_eq!(junction(a, b, c, [20.0; 2], [40.0; 2]), Ok((0.0, 0.0)));
    }
}

#[test]
fn nonfinite_displacements_are_rejected_before_the_stop_shortcut() {
    let large = f64::MAX;
    for (a, b, c) in [
        ([-large, 0.0], [large, 1.0], [large, 2.0]),
        ([0.0, -large], [1.0, large], [2.0, large]),
        ([large, 0.0], [large, 1.0], [-large, 2.0]),
        ([-large; 2], [-large; 2], [large; 2]),
    ] {
        assert_eq!(
            junction(a, b, c, [1.0; 2], [1.0; 2]),
            Err(PlannerError::NumericalFailure)
        );
    }
}

#[test]
fn uniform_scaling_preserves_progress_speeds_at_extreme_coordinate_scales() {
    for exponent in [-1074, -1000, -500, 0, 500, 1000] {
        let s = libm::scalbn(1.0, exponent);
        let points = [[-2.0 * s, s], [0.0; 2], [4.0 * s, -2.0 * s]];
        let speeds = junction(
            points[0],
            points[1],
            points[2],
            [20.0 * s, 10.0 * s],
            [4.0 * s, 2.0 * s],
        )
        .unwrap();
        assert_eq!(speeds, (2.0, 1.0));
        assert_profiles(points, [20.0 * s, 10.0 * s], [4.0 * s, 2.0 * s], speeds);
    }
}

#[test]
fn square_root_rounding_does_not_make_the_returned_boundary_speeds_infeasible() {
    for a in [0.1, 0.5, 1.0, 3.0, 7.0, 10.0, f64::MAX] {
        let points = [[-1.0; 2], [0.0; 2], [1.0; 2]];
        let speeds = junction(points[0], points[1], points[2], [f64::MAX; 2], [a; 2]).unwrap();
        let expected = libm::sqrt(a) * libm::sqrt(2.0);
        close(speeds.0, expected);
        close(speeds.1, expected);
        assert_profiles(points, [f64::MAX; 2], [a; 2], speeds);
    }
}

#[test]
fn the_smallest_positive_acceleration_still_allows_a_nonzero_junction_speed() {
    let tiny = f64::from_bits(1);
    let points = [[-1.0; 2], [0.0; 2], [1.0; 2]];
    let speeds = junction(points[0], points[1], points[2], [1.0; 2], [tiny; 2]).unwrap();
    assert!(speeds.0 > 0.0 && speeds.1 > 0.0);
    close(speeds.0, libm::sqrt(2.0 * tiny));
    assert_profiles(points, [1.0; 2], [tiny; 2], speeds);
}

#[test]
fn unrepresentable_progress_limits_and_motion_durations_are_reported() {
    let tiny = f64::from_bits(1);
    for (a, c, velocity, acceleration) in [
        ([-1.0; 2], [1.0; 2], [tiny; 2], [1.0; 2]), // Duration overflows.
        ([-tiny; 2], [f64::MAX; 2], [tiny; 2], [tiny; 2]), // Progress underflows.
        ([-tiny; 2], [tiny; 2], [1.0; 2], [1.0; 2]), // Progress limits overflow.
    ] {
        assert_eq!(
            junction(a, [0.0; 2], c, velocity, acceleration),
            Err(PlannerError::NumericalFailure)
        );
    }
}

#[test]
fn maximal_crossing_speeds_match_an_independent_joint_space_oracle() {
    for x in [1.0_f64, 2.0, 3.0] {
        for y in [1.0_f64, 2.0, 4.0] {
            for factor in [0.25, 0.5, 1.0, 2.0, 4.0] {
                for velocity in [[0.25, 0.5], [1.0, 10.0], [20.0, 5.0]] {
                    for acceleration in [[0.125, 0.5], [1.0, 10.0], [40.0, 30.0]] {
                        let points = [[-x, -y], [0.0; 2], [factor * x, factor * y]];
                        let speeds =
                            junction(points[0], points[1], points[2], velocity, acceleration)
                                .unwrap();
                        // Work in physical first-joint speed, using v²=2*a*d
                        // on the shorter segment and both physical velocity caps.
                        let expected = velocity[0]
                            .min(velocity[1] * x / y)
                            .min(libm::sqrt(2.0 * acceleration[0] * x.min(factor * x)))
                            .min(libm::sqrt(2.0 * acceleration[1] * y.min(factor * y)) * x / y);
                        close(speeds.0 * x, expected);
                        close(speeds.1 * factor * x, expected);
                        assert_profiles(points, velocity, acceleration, speeds);
                    }
                }
            }
        }
    }
}
