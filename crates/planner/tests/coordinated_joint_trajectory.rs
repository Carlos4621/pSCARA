use kinematics::JointAngles;
use planner::{
    CoordinatedJointPathProfiles, CoordinatedJointSample, CoordinatedJointTrajectory,
    JointAccelerationLimits, JointVelocityLimits, PlannerError, TimedCoordinatedJointPathProfiles,
    plan_joint_path_speeds,
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

fn assert_query_error(trajectory: &CoordinatedJointTrajectory<'_>, time: f64, error: PlannerError) {
    assert_eq!(position(trajectory, time), Err(error), "position at {time}");
    assert_eq!(
        trajectory.velocity_at(time),
        Err(error),
        "velocity at {time}"
    );
    assert_eq!(
        trajectory.acceleration_at(time),
        Err(error),
        "acceleration at {time}"
    );
    assert_eq!(
        trajectory.sample_at(time).unwrap_err(),
        error,
        "sample at {time}"
    );
}

fn assert_sample(
    trajectory: &CoordinatedJointTrajectory<'_>,
    time: f64,
    expected_position: Pair,
    expected_velocity: (f64, f64),
    expected_acceleration: (f64, f64),
) {
    let CoordinatedJointSample {
        position,
        velocity,
        acceleration,
    } = trajectory.sample_at(time).unwrap();
    assert_eq!(
        [position.first_theta(), position.second_theta()],
        expected_position,
        "sample position at {time}"
    );
    assert_eq!(velocity, expected_velocity, "sample velocity at {time}");
    assert_eq!(
        acceleration, expected_acceleration,
        "sample acceleration at {time}"
    );
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
        assert_eq!(trajectory.velocity_at(time), Ok((4.0 * time, -2.0 * time)));
        assert_eq!(trajectory.acceleration_at(time), Ok((4.0, -2.0)));
        assert_sample(
            &trajectory,
            time,
            [2.0, -1.0],
            (4.0 * time, -2.0 * time),
            (4.0, -2.0),
        );
    }
    assert_eq!(position(&trajectory, 1.0), Ok([4.0, -2.0]));
    assert_eq!(position(&trajectory, 2.0), Ok([6.0, -3.0]));
    assert_eq!(trajectory.velocity_at(2.0), Ok((0.0, 0.0)));
    assert_eq!(trajectory.acceleration_at(2.0), Ok((-4.0, 2.0)));
    for time in [2.0_f64.next_up(), f64::MAX] {
        assert_query_error(&trajectory, time, PlannerError::InvalidTime);
    }
}

#[test]
fn clamped_position_uses_the_final_position_for_later_times() {
    let points = [[0.0, 0.0], [9.0 / 16.0, 0.0]].map(angles);
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let timed = timed_path(&points, &[0.0; 2], &velocity, &acceleration);
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();
    let clamped_position = |time| {
        trajectory
            .position_at_clamped(time)
            .map(|q| [q.first_theta(), q.second_theta()])
    };

    let end = 1.5_f64;
    let final_position = position(&trajectory, end).unwrap();
    assert_eq!(final_position, [9.0 / 16.0, 0.0]);
    assert_eq!(clamped_position(0.0), Ok([0.0, 0.0]));
    assert_eq!(clamped_position(1.0), Ok([7.0 / 16.0, 0.0]));
    for time in [-0.0, f64::from_bits(1), 0.75, 1.0, end.next_down(), end] {
        assert_eq!(clamped_position(time), position(&trajectory, time));
    }
    for time in [end.next_up(), 2.0, 100.0, f64::MAX] {
        assert_eq!(position(&trajectory, time), Err(PlannerError::InvalidTime));
        assert_eq!(clamped_position(time), Ok(final_position));
    }
    for time in [
        -f64::from_bits(1),
        -1.0,
        f64::NAN,
        -f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
    ] {
        assert_eq!(clamped_position(time), Err(PlannerError::InvalidTime));
    }
}

#[test]
fn repeated_queries_can_move_across_the_path_in_either_direction() {
    let points = [[0.0, 0.0], [4.0, -2.0], [4.0, -14.0], [-5.0, -14.0]].map(angles);
    let velocity = JointVelocityLimits::new(4.0, 2.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let timed = timed_path(&points, &[0.0; 4], &velocity, &acceleration);
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();

    // The triangular, trapezoidal and triangular segments take 4, 8 and 6.
    for (time, expected, velocity, acceleration) in [
        (18.0, [-5.0, -14.0], (0.0, 0.0), (1.0, 0.0)),
        (0.0, [0.0, 0.0], (0.0, 0.0), (1.0, -0.5)),
        (8.0, [4.0, -8.0], (0.0, -2.0), (0.0, 0.0)),
        (4.0, [4.0, -2.0], (0.0, 0.0), (-1.0, 0.5)),
        (15.0, [-0.5, -14.0], (-3.0, 0.0), (-1.0, 0.0)),
        (2.0, [2.0, -1.0], (2.0, -1.0), (1.0, -0.5)),
        (12.0, [4.0, -14.0], (0.0, 0.0), (0.0, 1.0)),
        (5.0, [4.0, -2.5], (0.0, -1.0), (0.0, -1.0)),
        (11.0, [4.0, -13.5], (0.0, -1.0), (0.0, 1.0)),
        (8.0, [4.0, -8.0], (0.0, -2.0), (0.0, 0.0)),
    ] {
        assert_eq!(position(&trajectory, time), Ok(expected), "time={time}");
        assert_eq!(trajectory.velocity_at(time), Ok(velocity), "time={time}");
        assert_eq!(
            trajectory.acceleration_at(time),
            Ok(acceleration),
            "time={time}"
        );
        assert_sample(&trajectory, time, expected, velocity, acceleration);
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
    for (time, velocity, left_acceleration, right_acceleration) in [
        (1.0_f64, (2.0, 0.0), (2.0, 0.0), (-2.0, 0.0)),
        (2.0, (0.0, 0.0), (-2.0, 0.0), (0.0, 2.0)),
        (3.0, (0.0, 2.0), (0.0, 2.0), (0.0, -2.0)),
    ] {
        assert_eq!(trajectory.velocity_at(time), Ok(velocity));
        assert_eq!(
            trajectory.acceleration_at(time.next_down()),
            Ok(left_acceleration)
        );
        assert_eq!(trajectory.acceleration_at(time), Ok(left_acceleration));
        assert_eq!(
            trajectory.acceleration_at(time.next_up()),
            Ok(right_acceleration)
        );
        let sample = trajectory.sample_at(time).unwrap();
        assert_eq!(sample.velocity, velocity);
        assert_eq!(sample.acceleration, left_acceleration);
        assert_eq!(
            trajectory.sample_at(time.next_down()).unwrap().acceleration,
            left_acceleration
        );
        assert_eq!(
            trajectory.sample_at(time.next_up()).unwrap().acceleration,
            right_acceleration
        );
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
        assert_query_error(&trajectory, time, PlannerError::ProfileInstanceNotFound);
    }
    for (time, expected, velocity, acceleration) in [
        (2.0, 1.0, 0.0, 1.0),
        (4.0, 3.0, 2.0, 1.0),
        (6.0, 5.0, 0.0, -1.0),
        (9.0, 9.5, 3.0, 1.0),
        (12.0, 14.0, 0.0, -1.0),
    ] {
        assert_eq!(position(&trajectory, time), Ok([expected, 0.0]));
        assert_eq!(trajectory.velocity_at(time), Ok((velocity, 0.0)));
        assert_eq!(trajectory.acceleration_at(time), Ok((acceleration, 0.0)));
        assert_sample(
            &trajectory,
            time,
            [expected, 0.0],
            (velocity, 0.0),
            (acceleration, 0.0),
        );
    }
    assert_query_error(&trajectory, 12.0_f64.next_up(), PlannerError::InvalidTime);
}

#[test]
fn an_exhausted_timed_path_has_no_profile_at_zero() {
    let points = [[0.0, 0.0], [1.0, 0.0]].map(angles);
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let mut timed = timed_path(&points, &[0.0; 2], &velocity, &acceleration);
    assert!(timed.next().unwrap().is_ok());
    assert!(timed.next().is_none());
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();

    for time in [0.0, -0.0] {
        assert_query_error(&trajectory, time, PlannerError::ProfileInstanceNotFound);
    }
    assert_query_error(&trajectory, f64::from_bits(1), PlannerError::InvalidTime);
    assert_query_error(&trajectory, f64::NAN, PlannerError::InvalidTime);
}

#[test]
fn very_large_representable_times_keep_their_segment_boundaries() {
    let points = [0.0, 1.0, 2.0, 3.0].map(|x| angles([x, 0.0]));
    let velocity = JointVelocityLimits::new(f64::MIN_POSITIVE, 1.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let timed = timed_path(&points, &[0.0; 4], &velocity, &acceleration);
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();

    let duration = libm::scalbn(1.0, 1022);
    for (x, velocity, acceleration) in [
        (0.0, 0.0, 1.0),
        (0.5, f64::MIN_POSITIVE, 0.0),
        (1.0, 0.0, -1.0),
        (1.5, f64::MIN_POSITIVE, 0.0),
        (2.0, 0.0, -1.0),
        (2.5, f64::MIN_POSITIVE, 0.0),
        (3.0, 0.0, -1.0),
    ] {
        let time = x * duration;
        assert_eq!(position(&trajectory, time), Ok([x, 0.0]));
        assert_eq!(trajectory.velocity_at(time), Ok((velocity, 0.0)));
        assert_eq!(trajectory.acceleration_at(time), Ok((acceleration, 0.0)));
        assert_sample(
            &trajectory,
            time,
            [x, 0.0],
            (velocity, 0.0),
            (acceleration, 0.0),
        );
    }
    assert_query_error(
        &trajectory,
        (3.0 * duration).next_up(),
        PlannerError::InvalidTime,
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
    assert_eq!(trajectory.velocity_at(junction), Ok((0.0, 0.0)));
    assert_eq!(trajectory.acceleration_at(junction), Ok((-1.0, 0.0)));
    assert_eq!(trajectory.velocity_at(end), Ok((0.0, 0.0)));
    assert_eq!(
        trajectory.acceleration_at(end),
        Ok((0.0, -libm::scalbn(1.0, 108)))
    );
    assert_sample(
        &trajectory,
        junction,
        [distance, 0.0],
        (0.0, 0.0),
        (-1.0, 0.0),
    );
    assert_sample(
        &trajectory,
        end,
        [distance, 1.0],
        (0.0, 0.0),
        (0.0, -libm::scalbn(1.0, 108)),
    );
    assert_query_error(&trajectory, end.next_up(), PlannerError::InvalidTime);
}

#[test]
fn negative_finite_times_are_invalid() {
    let points = [[0.0, 0.0], [1.0, 0.0]].map(angles);
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let timed = timed_path(&points, &[0.0; 2], &velocity, &acceleration);
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();

    for time in [-f64::from_bits(1), -1.0, -f64::MAX] {
        assert_query_error(&trajectory, time, PlannerError::InvalidTime);
    }
}

#[test]
fn nonfinite_times_are_invalid() {
    let points = [[0.0, 0.0], [1.0, 0.0]].map(angles);
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let timed = timed_path(&points, &[0.0; 2], &velocity, &acceleration);
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();

    for time in [f64::NAN, f64::NEG_INFINITY, f64::INFINITY] {
        assert_query_error(&trajectory, time, PlannerError::InvalidTime);
    }
}

#[test]
fn a_moving_suffix_uses_its_first_segment_at_the_original_junction() {
    let points = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]].map(angles);
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(2.0, 2.0).unwrap();
    let mut timed = timed_path(&points, &[0.0, 2.0, 0.0], &velocity, &acceleration);
    let full = CoordinatedJointTrajectory::new(timed).unwrap();
    let junction = timed.next().unwrap().unwrap().end_time().unwrap();
    let suffix = CoordinatedJointTrajectory::new(timed).unwrap();

    assert_eq!(junction, 1.0);
    assert_query_error(
        &suffix,
        junction.next_down(),
        PlannerError::ProfileInstanceNotFound,
    );
    for trajectory in [&full, &suffix] {
        assert_eq!(position(trajectory, junction), Ok([1.0, 0.0]));
        assert_eq!(trajectory.velocity_at(junction), Ok((2.0, 0.0)));
    }
    assert_eq!(full.acceleration_at(junction), Ok((2.0, 0.0)));
    assert_eq!(suffix.acceleration_at(junction), Ok((-2.0, 0.0)));
    assert_eq!(suffix.velocity_at(2.0), Ok((0.0, 0.0)));
    assert_sample(&full, junction, [1.0, 0.0], (2.0, 0.0), (2.0, 0.0));
    assert_sample(&suffix, junction, [1.0, 0.0], (2.0, 0.0), (-2.0, 0.0));
}

#[test]
fn derivatives_match_centered_differences_inside_later_segments() {
    let points = [[0.0, 0.0], [4.0, -2.0], [4.0, -14.0], [-5.0, -14.0]].map(angles);
    let velocity = JointVelocityLimits::new(4.0, 2.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let timed = timed_path(&points, &[0.0; 4], &velocity, &acceleration);
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();

    // These samples are inside the ramps and cruise, away from any transition.
    let h = 1.0 / 1024.0;
    for time in [5.0, 8.0, 11.0, 13.0, 17.0] {
        let before = position(&trajectory, time - h).unwrap();
        let after = position(&trajectory, time + h).unwrap();
        let (v_before_1, v_before_2) = trajectory.velocity_at(time - h).unwrap();
        let (v_after_1, v_after_2) = trajectory.velocity_at(time + h).unwrap();
        let (v1, v2) = trajectory.velocity_at(time).unwrap();
        let (a1, a2) = trajectory.acceleration_at(time).unwrap();
        for (expected, finite_difference) in [
            (v1, (after[0] - before[0]) / (2.0 * h)),
            (v2, (after[1] - before[1]) / (2.0 * h)),
            (a1, (v_after_1 - v_before_1) / (2.0 * h)),
            (a2, (v_after_2 - v_before_2) / (2.0 * h)),
        ] {
            assert!(
                (expected - finite_difference).abs() <= 1e-10,
                "t={time}: {expected} vs {finite_difference}"
            );
        }
    }
}

#[test]
fn sample_agrees_bitwise_with_individual_queries_at_fractional_times() {
    let points = [[0.0, -0.0], [1.0, -0.0], [1.0, -0.3], [1.7, -0.6]].map(angles);
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 2.0).unwrap();
    let timed = timed_path(&points, &[0.0; 4], &velocity, &acceleration);
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();

    assert_eq!(
        trajectory
            .sample_at(0.0)
            .unwrap()
            .position
            .second_theta()
            .to_bits(),
        (-0.0_f64).to_bits()
    );
    for segment in timed {
        let segment = segment.unwrap();
        let start = segment.start_time();
        let end = segment.end_time().unwrap();
        for step in (0..=32).rev() {
            let time = if step == 32 {
                end
            } else {
                start + (end - start) * (f64::from(step) / 32.0)
            };
            let sample = trajectory.sample_at(time).unwrap();
            let [first, second] = position(&trajectory, time).unwrap();
            let (v1, v2) = trajectory.velocity_at(time).unwrap();
            let (a1, a2) = trajectory.acceleration_at(time).unwrap();
            let actual = [
                sample.position.first_theta(),
                sample.position.second_theta(),
                sample.velocity.0,
                sample.velocity.1,
                sample.acceleration.0,
                sample.acceleration.1,
            ];
            let expected = [first, second, v1, v2, a1, a2];
            assert_eq!(
                actual.map(f64::to_bits),
                expected.map(f64::to_bits),
                "t={time}"
            );
        }
    }
}

#[test]
fn sample_propagates_a_numerical_error_from_one_component() {
    let points = [[0.0, 0.0], [1.04, 0.0]].map(angles);
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(f64::MAX, 1.0).unwrap();
    let timed = timed_path(&points, &[0.0; 2], &velocity, &acceleration);
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();

    // Converting the rounded progress acceleration back to the first joint
    // overflows, even though position and velocity at the start are finite.
    assert_eq!(position(&trajectory, 0.0), Ok([0.0, 0.0]));
    assert_eq!(trajectory.velocity_at(0.0), Ok((0.0, 0.0)));
    assert_eq!(
        trajectory.acceleration_at(0.0),
        Err(PlannerError::NumericalFailure)
    );
    assert_eq!(
        trajectory.sample_at(0.0).unwrap_err(),
        PlannerError::NumericalFailure
    );

    let cruise = trajectory.sample_at(0.05).unwrap();
    assert_eq!(cruise.acceleration, (0.0, 0.0));
    assert!(cruise.position.first_theta() > 0.0 && cruise.velocity.0 > 0.0);
    assert_eq!(
        trajectory.sample_at(0.0).unwrap_err(),
        PlannerError::NumericalFailure
    );
}
