use kinematics::JointAngles;
use planner::{CoordinatedJointMotionProfile, JointAccelerationLimits, JointVelocityLimits};

type Angles = [f64; 2];
type Chain = [Angles; 3];

const CHAINS: [Chain; 6] = [
    [[0.0, 0.0], [2.0, -1.0], [4.0, -2.0]], // Equal displacement vectors.
    [[4.0, -2.0], [2.0, -1.0], [0.0, 0.0]], // Reverse direction.
    [[0.0, 0.0], [2.0, -1.0], [6.0, -3.0]], // Longer second segment.
    [[0.0, 0.0], [2.0, -1.0], [4.0, 0.0]],  // Corner in joint space.
    [[3.0, 0.0], [3.0, 1.0], [3.0, 2.0]],   // Stationary first joint.
    [[0.0, 3.0], [1.0, 3.0], [2.0, 3.0]],   // Stationary second joint.
];

fn angles(values: Angles) -> JointAngles {
    JointAngles::new(values[0], values[1]).unwrap()
}

fn chain(
    [a, b, c]: Chain,
    velocity: Angles,
    acceleration: Angles,
) -> (CoordinatedJointMotionProfile, CoordinatedJointMotionProfile) {
    let velocity_limits = JointVelocityLimits::new(velocity[0], velocity[1]).unwrap();
    let acceleration_limits =
        JointAccelerationLimits::new(acceleration[0], acceleration[1]).unwrap();
    let (a, b, c) = (angles(a), angles(b), angles(c));

    let first =
        CoordinatedJointMotionProfile::new(a, b, &velocity_limits, &acceleration_limits, 0.0, 0.5)
            .unwrap();

    let second =
        CoordinatedJointMotionProfile::new(b, c, &velocity_limits, &acceleration_limits, 0.5, 0.0)
            .unwrap();

    assert!(first.total_time().is_finite() && first.total_time() > 0.0);
    assert!(second.total_time().is_finite() && second.total_time() > 0.0);
    (first, second)
}

fn position(profile: &CoordinatedJointMotionProfile, time: f64) -> Angles {
    let q = profile.position_at(time).unwrap();
    [q.first_theta(), q.second_theta()]
}

#[test]
fn chained_profiles_have_exact_position_continuity_at_b() {
    for [a, b, c] in CHAINS {
        let (first, second) = chain([a, b, c], [20.0; 2], [40.0; 2]);
        let first_end = position(&first, first.total_time());
        let second_start = position(&second, 0.0);
        assert_eq!(first_end, second_start);
        assert_eq!(first_end, b);
        assert_eq!(position(&first, 0.0), a);
        assert_eq!(position(&second, second.total_time()), c);
    }
}

#[test]
fn equal_displacement_vectors_have_exact_joint_velocity_continuity() {
    for points in [CHAINS[0], CHAINS[1], CHAINS[4], CHAINS[5]] {
        let (first, second) = chain(points, [20.0; 2], [40.0; 2]);
        let first_end = first.velocity_at(first.total_time()).unwrap();
        let second_start = second.velocity_at(0.0).unwrap();
        let expected = (
            (points[1][0] - points[0][0]) * 0.5,
            (points[1][1] - points[0][1]) * 0.5,
        );
        assert_eq!(first_end, second_start);
        assert_eq!(first_end, expected);
        assert_eq!(first.velocity_at(0.0), Ok((0.0, 0.0)));
        assert_eq!(second.velocity_at(second.total_time()), Ok((0.0, 0.0)));
    }
}

fn assert_physical_limits(
    profile: &CoordinatedJointMotionProfile,
    delta: Angles,
    velocity_limits: Angles,
    acceleration_limits: Angles,
    initial_speed: f64,
    final_speed: f64,
) {
    let check = |time| {
        let (v1, v2) = profile.velocity_at(time).unwrap();
        let (a1, a2) = profile.acceleration_at(time).unwrap();
        for (actual, limit) in [
            (v1, velocity_limits[0]),
            (v2, velocity_limits[1]),
            (a1, acceleration_limits[0]),
            (a2, acceleration_limits[1]),
        ] {
            assert!(actual.is_finite());
            assert!(
                actual.abs() <= limit * (1.0 + 32.0 * f64::EPSILON),
                "time={time:e}, value={actual:e}, limit={limit:e}"
            );
        }
    };

    for step in 0..=1024 {
        check(profile.total_time() * (f64::from(step) / 1024.0));
    }

    // Check the velocity extrema and both sides of the acceleration switches,
    // since a uniform sample grid can miss a narrow peak or phase boundary.
    let mut progress_velocity = f64::INFINITY;
    let mut progress_acceleration = f64::INFINITY;
    for joint in 0..2 {
        if delta[joint] != 0.0 {
            progress_velocity = progress_velocity.min(velocity_limits[joint] / delta[joint].abs());
            progress_acceleration =
                progress_acceleration.min(acceleration_limits[joint] / delta[joint].abs());
        }
    }
    // Unit progress distance: vp^2 = a + (vi^2 + vf^2)/2 before capping.
    let peak = (progress_acceleration
        + (initial_speed * initial_speed + final_speed * final_speed) / 2.0)
        .sqrt()
        .min(progress_velocity);
    let acceleration_end = (peak - initial_speed) / progress_acceleration;
    let deceleration_start = profile.total_time() - (peak - final_speed) / progress_acceleration;
    for boundary in [
        0.0,
        acceleration_end,
        deceleration_start,
        profile.total_time(),
    ] {
        for time in [boundary.next_down(), boundary, boundary.next_up()] {
            if (0.0..=profile.total_time()).contains(&time) {
                check(time);
            }
        }
    }
}

#[test]
fn both_profiles_respect_joint_velocity_and_acceleration_limits() {
    for points in CHAINS {
        // Large caps allow triangles; lower velocity caps also exercise cruises.
        for (velocity_limits, acceleration_limits) in
            [([20.0; 2], [40.0; 2]), ([4.0, 3.0], [40.0, 30.0])]
        {
            let (first, second) = chain(points, velocity_limits, acceleration_limits);
            for (profile, from, to, initial, final_speed) in [
                (&first, points[0], points[1], 0.0, 0.5),
                (&second, points[1], points[2], 0.5, 0.0),
            ] {
                assert_physical_limits(
                    profile,
                    [to[0] - from[0], to[1] - from[1]],
                    velocity_limits,
                    acceleration_limits,
                    initial,
                    final_speed,
                );
            }
        }
    }
}

#[test]
fn equal_progress_speeds_do_not_match_joint_velocities_for_unequal_lengths() {
    let (first, second) = chain(CHAINS[2], [20.0; 2], [40.0; 2]);
    let first_end = first.velocity_at(first.total_time()).unwrap();
    let second_start = second.velocity_at(0.0).unwrap();
    // q_dot = (end - start)*s_dot: a common s_dot=.5 does not compensate
    // for the second segment's doubled displacement vector.
    assert_eq!(first_end, (1.0, -0.5));
    assert_eq!(second_start, (2.0, -1.0));
    assert_ne!(first_end, second_start);
}

#[test]
fn equal_progress_speeds_do_not_match_joint_velocities_at_a_corner() {
    let (first, second) = chain(CHAINS[3], [20.0; 2], [40.0; 2]);
    let first_end = first.velocity_at(first.total_time()).unwrap();
    let second_start = second.velocity_at(0.0).unwrap();
    assert_eq!(first_end, (1.0, -0.5));
    assert_eq!(second_start, (1.0, 0.5));
    assert_ne!(first_end, second_start);
}
