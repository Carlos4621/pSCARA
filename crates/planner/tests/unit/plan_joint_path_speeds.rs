use crate::joint_path_velocity_planner::plan_joint_path_speeds;
use crate::{JointAccelerationLimits, JointVelocityLimits, MotionProfile, PlannerError};
use kinematics::JointAngles;

type Pair = [f64; 2];

fn angles(point: Pair) -> JointAngles {
    JointAngles::new(point[0], point[1]).unwrap()
}

fn plan<const N: usize>(
    points: [Pair; N],
    velocity: Pair,
    acceleration: Pair,
) -> Result<[f64; N], PlannerError> {
    let mut speeds = [f64::NAN; N];
    plan_joint_path_speeds(
        &points.map(angles),
        &JointVelocityLimits::new(velocity[0], velocity[1]).unwrap(),
        &JointAccelerationLimits::new(acceleration[0], acceleration[1]).unwrap(),
        &mut speeds,
    )?;
    Ok(speeds)
}

fn close(actual: f64, expected: f64) {
    assert!(actual.is_finite() && actual >= 0.0, "{actual:e}");
    assert!(
        actual == expected || (actual - expected).abs() <= 64.0 * f64::EPSILON * expected,
        "actual={actual:.17e}, expected={expected:.17e}"
    );
}

fn assert_error<const N: usize>(points: [Pair; N], buffer_len: usize, expected: PlannerError) {
    let sentinel = 12345.0;
    let mut storage = [sentinel; 10];
    let result = plan_joint_path_speeds(
        &points.map(angles),
        &JointVelocityLimits::new(10.0, 10.0).unwrap(),
        &JointAccelerationLimits::new(2.0, 2.0).unwrap(),
        &mut storage[1..1 + buffer_len],
    );
    assert_eq!(result, Err(expected));
    // An error makes no promise about contents inside the supplied slice.
    // Only storage outside that slice must remain intact.
    assert_eq!(storage[0], sentinel);
    assert!(
        storage[1 + buffer_len..]
            .iter()
            .all(|&value| value == sentinel)
    );
}

fn assert_profiles<const N: usize>(
    points: [Pair; N],
    velocity: Pair,
    acceleration: Pair,
    speeds: [f64; N],
) {
    assert_eq!(speeds[0], 0.0);
    assert_eq!(speeds[N - 1], 0.0);
    for i in 0..N - 1 {
        let delta = [
            points[i + 1][0] - points[i][0],
            points[i + 1][1] - points[i][1],
        ];
        let length = delta[0].abs().max(delta[1].abs());
        let fraction = [delta[0] / length, delta[1] / length];
        let max_velocity = (velocity[0] / fraction[0].abs()).min(velocity[1] / fraction[1].abs());
        let max_acceleration =
            (acceleration[0] / fraction[0].abs()).min(acceleration[1] / fraction[1].abs());
        for speed in [speeds[i], speeds[i + 1]] {
            assert!(speed.is_finite() && speed >= 0.0 && speed <= max_velocity);
        }
        let result = MotionProfile::with_boundary_speeds(
            0.0,
            length,
            max_acceleration,
            max_velocity,
            speeds[i],
            speeds[i + 1],
        );
        assert!(
            result.is_ok(),
            "segment {i}: from={:?}, to={:?}, vi={:.17e}, vf={:.17e}, profile={result:?}",
            points[i],
            points[i + 1],
            speeds[i],
            speeds[i + 1]
        );
        let profile = result.unwrap();
        for step in 0..=32 {
            let time = profile.total_time() * (f64::from(step) / 32.0);
            let speed = profile.velocity_at(time).unwrap();
            let accel = profile.acceleration_at(time).unwrap();
            for joint in 0..2 {
                assert!(
                    speed.abs() * fraction[joint].abs()
                        <= velocity[joint] * (1.0 + 32.0 * f64::EPSILON)
                );
                assert!(
                    accel.abs() * fraction[joint].abs()
                        <= acceleration[joint] * (1.0 + 32.0 * f64::EPSILON)
                );
            }
        }
        if i + 2 < N {
            let next_delta = [
                points[i + 2][0] - points[i + 1][0],
                points[i + 2][1] - points[i + 1][1],
            ];
            let next_length = next_delta[0].abs().max(next_delta[1].abs());
            for joint in 0..2 {
                let before = fraction[joint] * speeds[i + 1];
                let after = next_delta[joint] / next_length * speeds[i + 1];
                assert!(
                    before == after || (before - after).abs() <= 64.0 * f64::EPSILON * before.abs(),
                    "velocity discontinuity at point {}: {before:e} vs {after:e}",
                    i + 1
                );
            }
        }
    }
}

#[test]
fn empty_or_single_point_paths_return_insufficient_points() {
    for buffer_len in 0..=3 {
        assert_error([], buffer_len, PlannerError::InsufficientPoints);
        assert_error([[3.0, -2.0]], buffer_len, PlannerError::InsufficientPoints);
    }
}

#[test]
fn both_shorter_and_longer_buffers_are_rejected() {
    for buffer_len in [0, 1, 2, 4, 5] {
        assert_error(
            [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]],
            buffer_len,
            PlannerError::MorePointsThanSpeedsBuffer,
        );
    }
}

#[test]
fn buffer_length_validation_precedes_segment_validation() {
    assert_error([[0.0, 0.0]; 3], 2, PlannerError::MorePointsThanSpeedsBuffer);
}

#[test]
fn duplicate_consecutive_points_are_rejected_at_every_position() {
    for points in [
        [[0.0, 0.0], [0.0, 0.0], [1.0, 0.0], [2.0, 0.0]],
        [[0.0, 0.0], [1.0, 0.0], [1.0, 0.0], [2.0, 0.0]],
        [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [2.0, 0.0]],
    ] {
        assert_error(points, 4, PlannerError::ZeroDisplacementMotion);
    }
}

#[test]
fn overflowed_displacements_are_rejected_for_either_axis_and_direction() {
    for axis in 0..2 {
        for sign in [-1.0, 1.0] {
            let mut points = [[0.0; 2]; 3];
            points[0][axis] = sign * f64::MAX;
            points[1][axis] = -sign * f64::MAX;
            assert_error(points, 3, PlannerError::NumericalFailure);
            let mut points = [[0.0; 2]; 3];
            points[1][axis] = sign * f64::MAX;
            points[2][axis] = -sign * f64::MAX;
            assert_error(points, 3, PlannerError::NumericalFailure);
        }
    }
}

#[test]
fn a_single_segment_starts_and_ends_at_rest() {
    for end in [[1.0, 0.0], [0.0, -2.0], [4.0, -3.0]] {
        let points = [[0.0, 0.0], end];
        let speeds = plan(points, [10.0; 2], [2.0; 2]).unwrap();
        assert_eq!(speeds, [0.0; 2]);
        assert_profiles(points, [10.0; 2], [2.0; 2], speeds);
    }
}

#[test]
fn success_overwrites_arbitrary_buffer_contents() {
    let points = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]].map(angles);
    for initial in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -1.0, f64::MAX] {
        let mut speeds = [initial; 3];
        assert_eq!(
            plan_joint_path_speeds(
                &points,
                &JointVelocityLimits::new(10.0, 10.0).unwrap(),
                &JointAccelerationLimits::new(2.0, 2.0).unwrap(),
                &mut speeds,
            ),
            Ok(())
        );
        assert_eq!(speeds, [0.0, 2.0, 0.0]);
    }
}

#[test]
fn an_offset_output_slice_does_not_modify_its_neighbors() {
    let points = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]].map(angles);
    let mut storage = [12345.0; 5];
    plan_joint_path_speeds(
        &points,
        &JointVelocityLimits::new(10.0, 10.0).unwrap(),
        &JointAccelerationLimits::new(2.0, 2.0).unwrap(),
        &mut storage[1..4],
    )
    .unwrap();
    assert_eq!(storage, [12345.0, 0.0, 2.0, 0.0, 12345.0]);
}

#[test]
fn equal_collinear_segments_share_an_analytic_junction_speed() {
    assert_eq!(
        plan([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]], [10.0; 2], [2.0; 2]),
        Ok([0.0, 2.0, 0.0])
    );
}

#[test]
fn a_short_incoming_segment_limits_acceleration() {
    assert_eq!(
        plan([[0.0, 0.0], [1.0, 0.0], [5.0, 0.0]], [10.0; 2], [2.0; 2]),
        Ok([0.0, 2.0, 0.0])
    );
}

#[test]
fn a_short_outgoing_segment_limits_braking() {
    assert_eq!(
        plan([[0.0, 0.0], [4.0, 0.0], [5.0, 0.0]], [10.0; 2], [2.0; 2]),
        Ok([0.0, 2.0, 0.0])
    );
}

#[test]
fn forward_and_backward_limits_propagate_across_multiple_segments() {
    let points = [0.0, 1.0, 4.0, 9.0, 12.0, 13.0].map(|x| [x, 0.0]);
    let speeds = plan(points, [100.0; 2], [2.0; 2]).unwrap();
    for (actual, expected) in speeds.into_iter().zip([0.0, 2.0, 4.0, 4.0, 2.0, 0.0]) {
        close(actual, expected);
    }
    assert_profiles(points, [100.0; 2], [2.0; 2], speeds);
}

#[test]
fn joint_velocity_limits_cap_internal_speeds() {
    let points = [0.0, 1.0, 2.0, 3.0].map(|x| [x, 0.0]);
    let speeds = plan(points, [1.0, 10.0], [100.0; 2]).unwrap();
    assert_eq!(speeds, [0.0, 1.0, 1.0, 0.0]);
    assert_profiles(points, [1.0, 10.0], [100.0; 2], speeds);
}

#[test]
fn corners_and_reversals_force_a_stop() {
    for points in [
        [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]],
        [[0.0, 0.0], [1.0, 0.0], [0.0, 0.0]],
        [[0.0, 0.0], [1.0, -1.0], [0.0, 0.0]],
        [[0.0, 0.0], [1.0, 1.0], [2.0, 2.0_f64.next_up()]],
    ] {
        assert_eq!(plan(points, [10.0; 2], [2.0; 2]), Ok([0.0; 3]));
    }
}

#[test]
fn a_corner_splits_acceleration_and_braking_into_independent_runs() {
    let points = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [2.0, 1.0], [2.0, 2.0]];
    let speeds = plan(points, [10.0; 2], [2.0; 2]).unwrap();
    assert_eq!(speeds, [0.0, 2.0, 0.0, 2.0, 0.0]);
    assert_profiles(points, [10.0; 2], [2.0; 2], speeds);
}

#[test]
fn either_joint_may_be_stationary_including_signed_zero() {
    for zero in [0.0, -0.0] {
        for points in [
            [[0.0, zero], [1.0, -zero], [2.0, zero]],
            [[zero, 0.0], [-zero, 1.0], [zero, 2.0]],
        ] {
            assert_eq!(plan(points, [10.0; 2], [2.0; 2]), Ok([0.0, 2.0, 0.0]));
        }
    }
}

#[test]
fn unequal_collinear_segments_preserve_joint_velocity_continuity() {
    let points = [[0.0, 0.0], [2.0, -1.0], [6.0, -3.0], [12.0, -6.0]];
    let speeds = plan(points, [20.0, 10.0], [4.0, 2.0]).unwrap();
    assert_profiles(points, [20.0, 10.0], [4.0, 2.0], speeds);
}

#[test]
fn different_joints_can_limit_velocity_and_acceleration() {
    let points = [[0.0, 0.0], [2.0, -1.0], [4.0, -2.0], [6.0, -3.0]];
    let speeds = plan(points, [20.0, 1.0], [1.0, 20.0]).unwrap();
    assert_eq!(speeds, [0.0, 2.0, 2.0, 0.0]);
    assert_profiles(points, [20.0, 1.0], [1.0, 20.0], speeds);
}

#[test]
fn reflection_translation_and_joint_exchange_preserve_speeds() {
    let original = [[0.0, 0.0], [2.0, -1.0], [6.0, -3.0], [12.0, -6.0]];
    let expected = plan(original, [20.0, 10.0], [4.0, 2.0]).unwrap();
    for sx in [-1.0, 1.0] {
        for sy in [-1.0, 1.0] {
            let points = original.map(|q| [32.0 + sx * q[0], -64.0 + sy * q[1]]);
            assert_eq!(plan(points, [20.0, 10.0], [4.0, 2.0]), Ok(expected));
        }
    }
    assert_eq!(
        plan(original.map(|q| [q[1], q[0]]), [10.0, 20.0], [2.0, 4.0]),
        Ok(expected)
    );
}

#[test]
fn reversing_a_path_reverses_the_speed_sequence() {
    let points = [[0.0, 0.0], [2.0, -1.0], [6.0, -3.0], [12.0, -6.0]];
    let mut reversed = points;
    reversed.reverse();
    let forward = plan(points, [20.0, 10.0], [4.0, 2.0]).unwrap();
    let backward = plan(reversed, [20.0, 10.0], [4.0, 2.0]).unwrap();
    for i in 0..points.len() {
        close(forward[i], backward[points.len() - i - 1]);
    }
}

#[test]
fn irrational_junction_speed_is_adjusted_to_build_feasible_profiles() {
    let points = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]];
    let speeds = plan(points, [10.0; 2], [1.0; 2]).unwrap();
    assert_eq!(speeds, [0.0, core::f64::consts::SQRT_2.next_down(), 0.0]);
    assert_profiles(points, [10.0; 2], [1.0; 2], speeds);
}

#[test]
fn displacement_scaling_obeys_the_square_root_acceleration_law() {
    for exponent in [-1000, -500, 0, 500, 1000] {
        let scale = libm::scalbn(1.0, exponent);
        let points = [[0.0, 0.0], [scale, 0.0], [2.0 * scale, 0.0]];
        assert_eq!(
            plan(points, [f64::MAX; 2], [2.0; 2]),
            Ok([0.0, 2.0 * libm::scalbn(1.0, exponent / 2), 0.0])
        );
    }
}

#[test]
fn common_scaling_preserves_motion_at_subnormal_and_extreme_scales() {
    for scale in [
        f64::from_bits(1),
        f64::MIN_POSITIVE,
        1e-200,
        1.0,
        1e200,
        libm::scalbn(1.0, 1000),
    ] {
        let points = [[0.0, 0.0], [scale, 0.0], [2.0 * scale, 0.0]];
        assert_eq!(
            plan(points, [10.0 * scale; 2], [2.0 * scale; 2]),
            Ok([0.0, 2.0 * scale, 0.0])
        );
    }
}

#[test]
fn a_subnormal_speed_cap_is_preserved() {
    let tiny = f64::from_bits(1);
    assert_eq!(
        plan([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]], [tiny; 2], [2.0; 2]),
        Ok([0.0, tiny, 0.0])
    );
}

#[test]
fn overflowed_theoretical_speeds_are_capped_instead_of_rejected() {
    let points = [[-f64::MAX, 0.0], [0.0, 0.0], [f64::MAX, 0.0]];
    for cap in [1.0, 1e200, f64::MAX] {
        assert_eq!(plan(points, [cap; 2], [f64::MAX; 2]), Ok([0.0, cap, 0.0]));
    }
}

#[test]
fn three_point_oracle_covers_independent_acceleration_braking_and_speed_caps() {
    for incoming in 1..=5 {
        for outgoing in 1..=5 {
            for velocity in [0.25_f64, 1.0, 4.0, 20.0] {
                for acceleration in [0.125_f64, 0.5, 1.0, 2.0, 8.0] {
                    let x = f64::from(incoming);
                    let end = f64::from(incoming + outgoing);
                    let points = [[0.0, 0.0], [x, -x / 2.0], [end, -end / 2.0]];
                    let speeds = plan(
                        points,
                        [velocity, velocity / 2.0],
                        [acceleration, acceleration / 2.0],
                    )
                    .unwrap();
                    let expected = velocity.min(libm::sqrt(
                        2.0 * acceleration * f64::from(incoming.min(outgoing)),
                    ));
                    assert_eq!(speeds[0], 0.0);
                    assert_eq!(speeds[2], 0.0);
                    close(speeds[1], expected);
                }
            }
        }
    }
}

#[test]
fn four_point_oracle_uses_total_distance_to_the_start_and_end() {
    for first in 1..=3 {
        for second in 1..=3 {
            for third in 1..=3 {
                for velocity in [0.5_f64, 2.0, 100.0] {
                    for acceleration in [0.5_f64, 1.0, 2.0] {
                        let positions = [
                            0.0,
                            f64::from(first),
                            f64::from(first + second),
                            f64::from(first + second + third),
                        ];
                        let points = positions.map(|x| [x, 0.0]);
                        let speeds = plan(points, [velocity; 2], [acceleration; 2]).unwrap();
                        for i in 0..positions.len() {
                            let distance = positions[i].min(positions[3] - positions[i]);
                            close(
                                speeds[i],
                                velocity.min(libm::sqrt(2.0 * acceleration * distance)),
                            );
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn propagated_speeds_build_feasible_profiles_for_each_segment() {
    for first in 1..=3 {
        for second in 1..=3 {
            for third in 1..=3 {
                for acceleration in [0.5, 1.0, 2.0] {
                    let positions = [
                        0.0,
                        f64::from(first),
                        f64::from(first + second),
                        f64::from(first + second + third),
                    ];
                    let points = positions.map(|x| [x, 0.0]);
                    let speeds = plan(points, [100.0; 2], [acceleration; 2]).unwrap();
                    assert_profiles(points, [100.0; 2], [acceleration; 2], speeds);
                }
            }
        }
    }
}
