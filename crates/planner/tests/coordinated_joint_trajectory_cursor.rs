use kinematics::JointAngles;
use planner::{
    CoordinatedJointPathProfiles, CoordinatedJointSample, CoordinatedJointTrajectory,
    CoordinatedJointTrajectoryCursor, JointAccelerationLimits, JointVelocityLimits,
    PlannerError, TimedCoordinatedJointPathProfiles, plan_joint_path_speeds,
};

fn angles([first, second]: [f64; 2]) -> JointAngles {
    JointAngles::new(first, second).unwrap()
}

fn trajectory<'a>(
    points: &'a [JointAngles],
    speeds: &'a [f64],
    velocity: &'a JointVelocityLimits,
    acceleration: &'a JointAccelerationLimits,
) -> CoordinatedJointTrajectory<'a> {
    let profiles = CoordinatedJointPathProfiles::new(points, speeds, velocity, acceleration).unwrap();
    let timed = TimedCoordinatedJointPathProfiles::new(profiles).unwrap();
    CoordinatedJointTrajectory::new(timed).unwrap()
}

fn bits(sample: CoordinatedJointSample) -> [u64; 6] {
    [
        sample.position.first_theta().to_bits(),
        sample.position.second_theta().to_bits(),
        sample.velocity.0.to_bits(),
        sample.velocity.1.to_bits(),
        sample.acceleration.0.to_bits(),
        sample.acceleration.1.to_bits(),
    ]
}

fn assert_matches(
    reference: &CoordinatedJointTrajectory<'_>,
    cursor: &mut CoordinatedJointTrajectoryCursor<'_>,
    time: f64,
) {
    assert_eq!(
        bits(cursor.sample_at(time).unwrap()),
        bits(reference.sample_at(time).unwrap()),
        "sample differs at t={time}"
    );
}

#[test]
fn increasing_and_repeated_queries_match_the_stateless_trajectory() {
    let points = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [2.0, 1.0], [2.0, 2.0]].map(angles);
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(2.0, 2.0).unwrap();
    let mut speeds = [f64::NAN; 5];
    plan_joint_path_speeds(&points, &velocity, &acceleration, &mut speeds).unwrap();
    let reference = trajectory(&points, &speeds, &velocity, &acceleration);
    let mut cursor = CoordinatedJointTrajectoryCursor::new(reference);

    assert_eq!(cursor.end_time(), 4.0);
    for time in [
        -0.0,
        0.0,
        0.5,
        1.0,
        1.0,
        1.0_f64.next_up(),
        1.25,
        2.0,
        2.0_f64.next_up(),
        2.75,
        3.0,
        3.0_f64.next_up(),
        3.5,
        4.0,
        4.0,
    ] {
        assert_matches(&reference, &mut cursor, time);
    }

    assert_eq!(
        cursor.sample_at(3.5).unwrap_err(),
        PlannerError::TimeWentBackwards
    );
    assert_matches(&reference, &mut cursor, 4.0);
}

#[test]
fn skipping_many_segments_still_returns_the_same_sample() {
    let points = [0.0, 1.0, 5.0, 14.0, 30.0].map(|x| angles([x, 0.0]));
    let speeds = [0.0; 5];
    let velocity = JointVelocityLimits::new(100.0, 100.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let reference = trajectory(&points, &speeds, &velocity, &acceleration);
    let mut cursor = CoordinatedJointTrajectoryCursor::new(reference);

    for time in [0.0, 11.0, 11.5, 20.0] {
        assert_matches(&reference, &mut cursor, time);
    }
    assert_eq!(
        cursor.sample_at(19.0).unwrap_err(),
        PlannerError::TimeWentBackwards
    );
    assert_eq!(
        cursor.sample_at(20.0_f64.next_up()).unwrap_err(),
        PlannerError::InvalidTime
    );
}

#[test]
fn invalid_queries_do_not_change_the_cursor_clock() {
    let points = [[0.0, 0.0], [1.0, 0.0]].map(angles);
    let speeds = [0.0; 2];
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let reference = trajectory(&points, &speeds, &velocity, &acceleration);
    let mut cursor = CoordinatedJointTrajectoryCursor::new(reference);

    for time in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -1.0, 2.0_f64.next_up()] {
        assert_eq!(cursor.sample_at(time).unwrap_err(), PlannerError::InvalidTime);
    }
    assert_matches(&reference, &mut cursor, 1.0);

    assert_eq!(
        cursor.sample_at(0.5).unwrap_err(),
        PlannerError::TimeWentBackwards
    );
    assert_matches(&reference, &mut cursor, 1.0);
    assert_matches(&reference, &mut cursor, 2.0);
}

#[test]
fn partially_consumed_source_keeps_original_global_boundaries() {
    let points = [0.0, 1.0, 5.0, 14.0].map(|x| angles([x, 0.0]));
    let speeds = [0.0; 4];
    let velocity = JointVelocityLimits::new(100.0, 100.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();

    let source = CoordinatedJointPathProfiles::new(&points, &speeds, &velocity, &acceleration).unwrap();
    let mut timed = TimedCoordinatedJointPathProfiles::new(source).unwrap();
    assert_eq!(timed.next().unwrap().unwrap().end_time(), Ok(2.0));
    let reference = CoordinatedJointTrajectory::new(timed).unwrap();
    let mut cursor = CoordinatedJointTrajectoryCursor::new(reference);
    assert_eq!(cursor.end_time(), 12.0);

    for time in [0.0, 1.0, 2.0_f64.next_down()] {
        assert_eq!(
            cursor.sample_at(time).unwrap_err(),
            PlannerError::ProfileInstanceNotFound
        );
    }
    for time in [2.0, 4.0, 6.0, 9.0, 12.0] {
        assert_matches(&reference, &mut cursor, time);
    }
}

#[test]
fn an_empty_suffix_has_no_samples() {
    let points = [[0.0, 0.0], [1.0, 0.0]].map(angles);
    let speeds = [0.0; 2];
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();

    let source = CoordinatedJointPathProfiles::new(&points, &speeds, &velocity, &acceleration).unwrap();
    let mut timed = TimedCoordinatedJointPathProfiles::new(source).unwrap();
    assert!(timed.next().unwrap().is_ok());
    assert!(timed.next().is_none());

    let reference = CoordinatedJointTrajectory::new(timed).unwrap();
    let mut cursor = CoordinatedJointTrajectoryCursor::new(reference);
    assert_eq!(cursor.end_time(), 0.0);
    assert_eq!(
        cursor.sample_at(0.0).unwrap_err(),
        PlannerError::ProfileInstanceNotFound
    );
    assert_eq!(
        cursor.sample_at(f64::from_bits(1)).unwrap_err(),
        PlannerError::InvalidTime
    );
}

#[test]
fn one_ulp_global_interval_preserves_exact_junction_and_final_samples() {
    let distance = 0.25_f64.next_up().next_up();
    let points = [[0.0, 0.0], [distance, 0.0], [distance, 1.0]].map(angles);
    let speeds = [0.0; 3];
    let velocity = JointVelocityLimits::new(10.0, libm::scalbn(1.0, 56)).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, libm::scalbn(1.0, 108)).unwrap();
    let reference = trajectory(&points, &speeds, &velocity, &acceleration);
    let mut cursor = CoordinatedJointTrajectoryCursor::new(reference);
    let junction = 1.0_f64.next_up();

    assert_matches(&reference, &mut cursor, 0.0);
    assert_matches(&reference, &mut cursor, junction);
    assert_matches(&reference, &mut cursor, junction.next_up());
    assert_eq!(
        cursor.sample_at(junction.next_up().next_up()).unwrap_err(),
        PlannerError::InvalidTime
    );
}

#[test]
fn numerical_sample_errors_can_be_retried_without_advancing_the_cursor() {
    let points = [[0.0, 0.0], [1.04, 0.0]].map(angles);
    let speeds = [0.0; 2];
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(f64::MAX, 1.0).unwrap();
    let reference = trajectory(&points, &speeds, &velocity, &acceleration);
    let mut cursor = CoordinatedJointTrajectoryCursor::new(reference);

    assert_eq!(
        cursor.sample_at(0.0).unwrap_err(),
        PlannerError::NumericalFailure
    );
    assert_matches(&reference, &mut cursor, 0.05);
    assert_eq!(
        cursor.sample_at(0.0).unwrap_err(),
        PlannerError::TimeWentBackwards
    );
}
