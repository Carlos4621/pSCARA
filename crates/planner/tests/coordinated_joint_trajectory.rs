use kinematics::JointAngles;
use planner::{
    CoordinatedJointPathProfiles, CoordinatedJointTrajectory, JointAccelerationLimits,
    JointVelocityLimits, PlannerError, TimedCoordinatedJointPathProfiles, plan_joint_path_speeds,
};

type Pair = [f64; 2];

fn angles([first, second]: Pair) -> JointAngles {
    JointAngles::new(first, second).unwrap()
}

fn timed_path<'a>(
    points: &'a [JointAngles],
    speeds: &'a [f64],
    velocity: &'a JointVelocityLimits,
    acceleration: &'a JointAccelerationLimits,
) -> TimedCoordinatedJointPathProfiles<'a> {
    let profiles =
        CoordinatedJointPathProfiles::new(points, speeds, velocity, acceleration).unwrap();
    TimedCoordinatedJointPathProfiles::new(profiles).unwrap()
}

fn position(trajectory: &CoordinatedJointTrajectory<'_>, time: f64) -> Result<Pair, PlannerError> {
    trajectory
        .position_at(time)
        .map(|q| [q.first_theta(), q.second_theta()])
}

#[test]
fn one_segment_accepts_signed_zero_and_both_endpoints() {
    let points = [[2.0, -1.0], [6.0, -3.0]].map(angles);
    let velocity = JointVelocityLimits::new(20.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(4.0, 2.0).unwrap();
    let timed = timed_path(&points, &[0.0; 2], &velocity, &acceleration);
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();

    for time in [0.0, -0.0, f64::from_bits(1)] {
        assert_eq!(position(&trajectory, time), Ok([2.0, -1.0]));
    }
    assert_eq!(position(&trajectory, 1.0), Ok([4.0, -2.0]));
    assert_eq!(position(&trajectory, 2.0), Ok([6.0, -3.0]));
    assert_eq!(
        position(&trajectory, 2.0_f64.next_up()),
        Err(PlannerError::InvalidTime)
    );
}

#[test]
fn repeated_queries_can_move_across_the_path_in_either_direction() {
    let points = [[0.0, 0.0], [4.0, -2.0], [4.0, -14.0], [-5.0, -14.0]].map(angles);
    let velocity = JointVelocityLimits::new(4.0, 2.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let timed = timed_path(&points, &[0.0; 4], &velocity, &acceleration);
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();

    // The triangular, trapezoidal and triangular segments take 4, 8 and 6.
    for (time, expected) in [
        (18.0, [-5.0, -14.0]),
        (0.0, [0.0, 0.0]),
        (8.0, [4.0, -8.0]),
        (4.0, [4.0, -2.0]),
        (15.0, [-0.5, -14.0]),
        (2.0, [2.0, -1.0]),
        (12.0, [4.0, -14.0]),
        (5.0, [4.0, -2.5]),
        (11.0, [4.0, -13.5]),
        (8.0, [4.0, -8.0]),
    ] {
        assert_eq!(position(&trajectory, time), Ok(expected), "time={time}");
    }
}

#[test]
fn composing_with_the_speed_planner_preserves_moving_junctions_and_stops() {
    let points = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [2.0, 1.0], [2.0, 2.0]].map(angles);
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(2.0, 2.0).unwrap();
    let mut speeds = [f64::NAN; 5];
    plan_joint_path_speeds(&points, &velocity, &acceleration, &mut speeds).unwrap();
    assert_eq!(speeds, [0.0, 2.0, 0.0, 2.0, 0.0]);
    let timed = timed_path(&points, &speeds, &velocity, &acceleration);
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();

    for (time, expected) in [
        (0.5, [0.25, 0.0]),
        (1.0, [1.0, 0.0]),
        (1.5, [1.75, 0.0]),
        (2.0, [2.0, 0.0]),
        (2.5, [2.0, 0.25]),
        (3.0, [2.0, 1.0]),
        (3.5, [2.0, 1.75]),
        (4.0, [2.0, 2.0]),
    ] {
        assert_eq!(position(&trajectory, time), Ok(expected), "time={time}");
    }
}

#[test]
fn a_partly_consumed_timed_path_keeps_the_original_clock_and_has_no_prefix() {
    let points = [0.0, 1.0, 5.0, 14.0].map(|x| angles([x, 0.0]));
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let mut timed = timed_path(&points, &[0.0; 4], &velocity, &acceleration);
    assert_eq!(timed.next().unwrap().unwrap().end_time(), Ok(2.0));
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();

    for time in [0.0, 1.0, 2.0_f64.next_down()] {
        assert_eq!(
            position(&trajectory, time),
            Err(PlannerError::ProfileInstanceNotFound)
        );
    }
    for (time, expected) in [(2.0, 1.0), (4.0, 3.0), (6.0, 5.0), (9.0, 9.5), (12.0, 14.0)] {
        assert_eq!(position(&trajectory, time), Ok([expected, 0.0]));
    }
    assert_eq!(
        position(&trajectory, 12.0_f64.next_up()),
        Err(PlannerError::InvalidTime)
    );
}

#[test]
fn an_exhausted_timed_path_has_no_position_at_zero() {
    let points = [[0.0, 0.0], [1.0, 0.0]].map(angles);
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let mut timed = timed_path(&points, &[0.0; 2], &velocity, &acceleration);
    assert!(timed.next().unwrap().is_ok());
    assert!(timed.next().is_none());
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();

    for time in [0.0, -0.0] {
        assert_eq!(
            position(&trajectory, time),
            Err(PlannerError::ProfileInstanceNotFound)
        );
    }
    assert_eq!(
        position(&trajectory, f64::from_bits(1)),
        Err(PlannerError::InvalidTime)
    );
}

#[test]
fn very_large_representable_times_keep_their_segment_boundaries() {
    let points = [0.0, 1.0, 2.0, 3.0].map(|x| angles([x, 0.0]));
    let velocity = JointVelocityLimits::new(f64::MIN_POSITIVE, 1.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let timed = timed_path(&points, &[0.0; 4], &velocity, &acceleration);
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();

    let duration = libm::scalbn(1.0, 1022);
    for x in [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0] {
        assert_eq!(position(&trajectory, x * duration), Ok([x, 0.0]));
    }
    assert_eq!(
        position(&trajectory, (3.0 * duration).next_up()),
        Err(PlannerError::InvalidTime)
    );
}

#[test]
fn one_ulp_in_global_time_is_enough_to_reach_the_exact_final_position() {
    let distance = 0.25_f64.next_up().next_up();
    let points = [[0.0, 0.0], [distance, 0.0], [distance, 1.0]].map(angles);
    let velocity = JointVelocityLimits::new(10.0, libm::scalbn(1.0, 56)).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, libm::scalbn(1.0, 108)).unwrap();
    let timed = timed_path(&points, &[0.0; 3], &velocity, &acceleration);
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();

    // The final segment lasts half an ulp locally. Its global interval is one ulp.
    let junction = 1.0_f64.next_up();
    let end = junction.next_up();
    assert_eq!(position(&trajectory, junction), Ok([distance, 0.0]));
    assert_eq!(position(&trajectory, end), Ok([distance, 1.0]));
    assert_eq!(
        position(&trajectory, end.next_up()),
        Err(PlannerError::InvalidTime)
    );
}

#[test]
fn negative_finite_times_are_invalid() {
    let points = [[0.0, 0.0], [1.0, 0.0]].map(angles);
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let timed = timed_path(&points, &[0.0; 2], &velocity, &acceleration);
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();

    let times = [-f64::from_bits(1), -1.0, -f64::MAX];
    assert_eq!(
        times.map(|t| position(&trajectory, t)),
        [Err(PlannerError::InvalidTime); 3]
    );
}

#[test]
fn nonfinite_times_are_invalid() {
    let points = [[0.0, 0.0], [1.0, 0.0]].map(angles);
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let timed = timed_path(&points, &[0.0; 2], &velocity, &acceleration);
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();

    let times = [f64::NAN, f64::NEG_INFINITY, f64::INFINITY];
    assert_eq!(
        times.map(|t| position(&trajectory, t)),
        [Err(PlannerError::InvalidTime); 3]
    );
}
