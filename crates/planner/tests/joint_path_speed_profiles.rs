use kinematics::JointAngles;
use planner::{
    CoordinatedJointMotionProfile, CoordinatedJointPathProfiles, JointAccelerationLimits,
    JointVelocityLimits, plan_joint_path_speeds,
};

type Pair = [f64; 2];
const RELATIVE_TOLERANCE: f64 = 64.0 * f64::EPSILON;

fn pair(value: (f64, f64)) -> Pair {
    [value.0, value.1]
}

fn close(actual: f64, expected: f64, context: &str) {
    assert!(actual.is_finite() && expected.is_finite(), "{context}");
    let scale = actual.abs().max(expected.abs());
    let tolerance = RELATIVE_TOLERANCE * scale + f64::from_bits(1);
    assert!(
        (actual - expected).abs() <= tolerance,
        "{context}: actual={actual:.17e}, expected={expected:.17e}, tolerance={tolerance:e}"
    );
}

fn check_limits(
    profile: &CoordinatedJointMotionProfile,
    time: f64,
    velocity: Pair,
    acceleration: Pair,
    segment: usize,
) {
    let actual_velocity = pair(profile.velocity_at(time).unwrap());
    let actual_acceleration = pair(profile.acceleration_at(time).unwrap());
    for joint in 0..2 {
        for (quantity, actual, limit) in [
            ("velocity", actual_velocity[joint], velocity[joint]),
            (
                "acceleration",
                actual_acceleration[joint],
                acceleration[joint],
            ),
        ] {
            assert!(
                actual.is_finite() && actual.abs() <= limit * (1.0 + RELATIVE_TOLERANCE),
                "segment={segment}, joint={joint}, time={time:e}, \
                 {quantity}={actual:.17e}, limit={limit:.17e}"
            );
        }
    }
}

fn check_chain<const N: usize>(
    coordinates: [Pair; N],
    velocity: Pair,
    acceleration: Pair,
) -> [f64; N] {
    assert!(N >= 2);
    let points = coordinates.map(|q| JointAngles::new(q[0], q[1]).unwrap());
    let velocity_limits = JointVelocityLimits::new(velocity[0], velocity[1]).unwrap();
    let acceleration_limits =
        JointAccelerationLimits::new(acceleration[0], acceleration[1]).unwrap();
    let mut speeds = [f64::NAN; N];
    plan_joint_path_speeds(&points, &velocity_limits, &acceleration_limits, &mut speeds).unwrap();
    assert_eq!(speeds[0], 0.0);
    assert_eq!(speeds[N - 1], 0.0);
    assert!(
        speeds
            .iter()
            .all(|speed| speed.is_finite() && *speed >= 0.0)
    );

    // Build every coordinated profile using the public API. Path speeds use
    // s in [0, length], while each profile uses progress in [0, 1]. Thus each
    // segment needs its own conversion, even at a shared junction.
    let mut profiles = Vec::with_capacity(N - 1);
    let mut chain =
        CoordinatedJointPathProfiles::new(&points, &speeds, &velocity_limits, &acceleration_limits)
            .unwrap();
    for i in 0..N - 1 {
        let delta = [
            coordinates[i + 1][0] - coordinates[i][0],
            coordinates[i + 1][1] - coordinates[i][1],
        ];
        let length = delta[0].abs().max(delta[1].abs());
        let initial_progress_speed = speeds[i] / length;
        let final_progress_speed = speeds[i + 1] / length;
        let result = CoordinatedJointMotionProfile::new(
            points[i],
            points[i + 1],
            &velocity_limits,
            &acceleration_limits,
            initial_progress_speed,
            final_progress_speed,
        );
        assert!(
            result.is_ok(),
            "segment={i}, from={:?}, to={:?}, speeds={speeds:?}, length={length:e}, \
             progress=({initial_progress_speed:.17e}, {final_progress_speed:.17e}), result={result:?}",
            coordinates[i],
            coordinates[i + 1]
        );
        let expected = result.unwrap();
        let profile = chain
            .next()
            .expect("missing segment")
            .expect("validated segment failed");
        assert_eq!(profile.total_time(), expected.total_time());
        let duration = profile.total_time();
        assert!(duration.is_finite() && duration > 0.0);

        // Dense sampling plus the velocity extrema and both sides of the
        // acceleration switches prevents a short ramp or cruise being missed.
        for step in 0..=1024 {
            check_limits(
                &profile,
                duration * (f64::from(step) / 1024.0),
                velocity,
                acceleration,
                i,
            );
        }
        let mut progress_velocity = f64::INFINITY;
        let mut progress_acceleration = f64::INFINITY;
        for joint in 0..2 {
            if delta[joint] != 0.0 {
                progress_velocity = progress_velocity.min(velocity[joint] / delta[joint].abs());
                progress_acceleration =
                    progress_acceleration.min(acceleration[joint] / delta[joint].abs());
            }
        }
        let peak = libm::sqrt(
            progress_acceleration
                + (initial_progress_speed * initial_progress_speed
                    + final_progress_speed * final_progress_speed)
                    / 2.0,
        )
        .min(progress_velocity);
        let acceleration_end = (peak - initial_progress_speed) / progress_acceleration;
        let deceleration_start = duration - (peak - final_progress_speed) / progress_acceleration;
        for boundary in [0.0, acceleration_end, deceleration_start, duration] {
            for time in [boundary.next_down(), boundary, boundary.next_up()] {
                if (0.0..=duration).contains(&time) {
                    check_limits(&profile, time, velocity, acceleration, i);
                }
            }
        }

        let start_position = profile.position_at(0.0).unwrap();
        let end_position = profile.position_at(duration).unwrap();
        assert_eq!(
            [start_position.first_theta(), start_position.second_theta()],
            coordinates[i]
        );
        assert_eq!(
            [end_position.first_theta(), end_position.second_theta()],
            coordinates[i + 1]
        );

        // Check that the constructor consumed the planned boundary speeds,
        // rather than silently replacing them with another feasible speed.
        let start_velocity = pair(profile.velocity_at(0.0).unwrap());
        let end_velocity = pair(profile.velocity_at(duration).unwrap());
        for joint in 0..2 {
            close(
                start_velocity[joint],
                (delta[joint] / length) * speeds[i],
                "planned start velocity",
            );
            close(
                end_velocity[joint],
                (delta[joint] / length) * speeds[i + 1],
                "planned end velocity",
            );
        }
        profiles.push(profile);
    }
    for _ in 0..3 {
        assert!(
            chain.next().is_none(),
            "unexpected segment after exhaustion"
        );
    }

    // The robot is at rest, in both joints, at the beginning and the end.
    assert_eq!(profiles[0].velocity_at(0.0).unwrap(), (0.0, 0.0));
    let last = profiles.last().unwrap();
    assert_eq!(last.velocity_at(last.total_time()).unwrap(), (0.0, 0.0));

    for (i, adjacent) in profiles.windows(2).enumerate() {
        let before = pair(adjacent[0].velocity_at(adjacent[0].total_time()).unwrap());
        let after = pair(adjacent[1].velocity_at(0.0).unwrap());
        for joint in 0..2 {
            close(
                before[joint],
                after[joint],
                &format!("junction {}, joint {joint}", i + 1),
            );
        }
    }
    speeds
}

#[test]
fn equal_collinear_segments_build_a_complete_coordinated_chain() {
    let speeds = check_chain(
        [[0.0, 0.0], [2.0, -1.0], [4.0, -2.0], [6.0, -3.0]],
        [20.0, 10.0],
        [4.0, 2.0],
    );
    assert!(speeds[1] > 0.0 && speeds[2] > 0.0);
}

#[test]
fn unequal_segments_convert_each_junction_speed_to_its_own_progress_scale() {
    let speeds = check_chain(
        [[0.0, 0.0], [2.0, -1.0], [6.0, -3.0], [12.0, -6.0]],
        [20.0, 10.0],
        [4.0, 2.0],
    );
    assert!(speeds[1] > 0.0 && speeds[2] > 0.0);
}

#[test]
fn corners_and_reversals_build_profiles_with_stopped_junctions() {
    let speeds = check_chain(
        [
            [0.0, 0.0],
            [2.0, -1.0],
            [4.0, -2.0],
            [4.0, 0.0],
            [4.0, 2.0],
            [4.0, 0.0],
            [2.0, 0.0],
        ],
        [20.0, 10.0],
        [4.0, 2.0],
    );
    for junction in [2, 4, 5] {
        assert_eq!(speeds[junction], 0.0);
    }
    assert!(speeds[1] > 0.0 && speeds[3] > 0.0);
}

#[test]
fn either_joint_can_remain_stationary_through_a_planned_chain() {
    for zero in [0.0, -0.0] {
        check_chain(
            [[zero, 0.0], [-zero, 1.0], [zero, 3.0], [-zero, 6.0]],
            [10.0; 2],
            [2.0; 2],
        );
        check_chain(
            [[0.0, zero], [1.0, -zero], [3.0, zero], [6.0, -zero]],
            [10.0; 2],
            [2.0; 2],
        );
    }
}

#[test]
fn both_triangular_and_velocity_capped_profiles_respect_joint_limits() {
    let points = [[0.0, 0.0], [2.0, -1.0], [6.0, -3.0], [12.0, -6.0]];
    for (velocity, acceleration) in [
        ([20.0, 10.0], [4.0, 2.0]),
        ([1.0, 0.5], [40.0, 20.0]),
        ([20.0, 1.0], [1.0, 20.0]),
        ([1.0, 20.0], [20.0, 1.0]),
    ] {
        check_chain(points, velocity, acceleration);
    }
}

#[test]
fn reflected_translated_reversed_and_exchanged_paths_build_continuous_chains() {
    let original = [[0.0, 0.0], [2.0, -1.0], [6.0, -3.0], [12.0, -6.0]];
    for sx in [-1.0, 1.0] {
        for sy in [-1.0, 1.0] {
            let mut points = original.map(|q| [32.0 + sx * q[0], -64.0 + sy * q[1]]);
            check_chain(points, [20.0, 10.0], [4.0, 2.0]);
            points.reverse();
            check_chain(points, [20.0, 10.0], [4.0, 2.0]);
        }
    }
    check_chain(original.map(|q| [q[1], q[0]]), [10.0, 20.0], [2.0, 4.0]);
}

#[test]
fn rounded_irrational_boundary_speeds_survive_progress_conversion() {
    for length in [0.5, 1.0, 2.0, 3.0, 5.0] {
        check_chain(
            [[0.0, 0.0], [length, 0.0], [2.0 * length, 0.0]],
            [10.0; 2],
            [1.0; 2],
        );
    }
}

#[test]
fn scaled_chains_remain_feasible_and_respect_scaled_joint_limits() {
    for scale in [
        libm::scalbn(1.0, -500),
        1e-100,
        1.0,
        1e100,
        libm::scalbn(1.0, 500),
    ] {
        check_chain(
            [
                [0.0, 0.0],
                [2.0 * scale, -scale],
                [6.0 * scale, -3.0 * scale],
                [12.0 * scale, -6.0 * scale],
            ],
            [20.0 * scale, 10.0 * scale],
            [4.0 * scale, 2.0 * scale],
        );
    }
}

#[test]
fn a_single_segment_builds_a_coordinated_profile_at_rest_at_both_ends() {
    check_chain([[3.0, -2.0], [-4.0, 5.0]], [3.0, 2.0], [4.0, 5.0]);
}
