use kinematics::JointAngles;

use super::TimedCoordinatedJointPathProfiles;
use crate::{
    CoordinatedJointPathProfiles, JointAccelerationLimits, JointVelocityLimits, PlannerError,
    TimedCoordinatedJointProfile, plan_joint_path_speeds,
};

fn angles([first, second]: [f64; 2]) -> JointAngles {
    JointAngles::new(first, second).unwrap()
}

#[test]
fn one_segment_is_retained_and_elapsed_time_starts_at_positive_zero() {
    for scale in [1e-200, 1.0, 1e200] {
        let points = [[0.0, 0.0], [scale, 0.0]].map(angles);
        let speeds = [0.0, -0.0];
        let velocity = JointVelocityLimits::new(10.0 * scale, scale).unwrap();
        let acceleration = JointAccelerationLimits::new(scale, scale).unwrap();
        let profiles =
            CoordinatedJointPathProfiles::new(&points, &speeds, &velocity, &acceleration).unwrap();

        let mut timed = TimedCoordinatedJointPathProfiles::new(profiles).unwrap();
        assert_eq!(timed.elapsed_time.to_bits(), 0.0_f64.to_bits());
        assert_eq!(timed.profiles.len(), 1);
        let segment = timed.profiles.next().unwrap().unwrap();
        assert_eq!(segment.total_time(), 2.0);
        assert_eq!(segment.position_at(0.0).unwrap().first_theta(), 0.0);
        assert_eq!(segment.position_at(2.0).unwrap().first_theta(), scale);
        assert!(timed.profiles.next().is_none());
    }
}

#[test]
fn planned_nonzero_junction_is_retained_and_can_be_scheduled() {
    let points = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]].map(angles);
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(2.0, 2.0).unwrap();
    let mut speeds = [f64::NAN; 3];
    plan_joint_path_speeds(&points, &velocity, &acceleration, &mut speeds).unwrap();
    assert!(speeds[1] > 0.0);
    let profiles =
        CoordinatedJointPathProfiles::new(&points, &speeds, &velocity, &acceleration).unwrap();

    let mut timed = TimedCoordinatedJointPathProfiles::new(profiles).unwrap();
    assert_eq!(timed.profiles.len(), 2);
    let first = timed.profiles.next().unwrap().unwrap();
    let second = timed.profiles.next().unwrap().unwrap();
    assert_eq!(first.velocity_at(first.total_time()), Ok((speeds[1], 0.0)));
    assert_eq!(second.velocity_at(0.0), Ok((speeds[1], 0.0)));

    let first = TimedCoordinatedJointProfile::new(timed.elapsed_time, first).unwrap();
    let junction = first.end_time().unwrap();
    let second = TimedCoordinatedJointProfile::new(junction, second).unwrap();
    assert_eq!(junction, 1.0);
    assert_eq!(first.position_at(junction).unwrap().first_theta(), 1.0);
    assert_eq!(second.position_at(junction).unwrap().first_theta(), 1.0);
    assert_eq!(second.end_time(), Ok(2.0));
}

#[test]
fn a_partially_consumed_source_keeps_its_cursor_and_starts_a_fresh_clock() {
    let points = [0.0, 1.0, 5.0, 14.0].map(|x| angles([x, 0.0]));
    let speeds = [0.0; 4];
    let velocity = JointVelocityLimits::new(100.0, 100.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let mut profiles =
        CoordinatedJointPathProfiles::new(&points, &speeds, &velocity, &acceleration).unwrap();
    assert_eq!(profiles.next().unwrap().unwrap().total_time(), 2.0);

    let mut timed = TimedCoordinatedJointPathProfiles::new(profiles).unwrap();
    assert_eq!(timed.elapsed_time, 0.0);
    assert_eq!(timed.profiles.len(), 2);
    for (from, to, duration) in [(1.0, 5.0, 4.0), (5.0, 14.0, 6.0)] {
        let segment = timed.profiles.next().unwrap().unwrap();
        assert_eq!(segment.total_time(), duration);
        assert_eq!(segment.position_at(0.0).unwrap().first_theta(), from);
        assert_eq!(segment.position_at(duration).unwrap().first_theta(), to);
    }
    assert!(timed.profiles.next().is_none());
}

#[test]
fn an_exhausted_source_is_a_valid_empty_schedule() {
    let points = [[0.0, 0.0], [1.0, 0.0]].map(angles);
    let speeds = [0.0; 2];
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let mut profiles =
        CoordinatedJointPathProfiles::new(&points, &speeds, &velocity, &acceleration).unwrap();
    assert!(profiles.next().unwrap().is_ok());
    assert!(profiles.next().is_none());

    let mut timed = TimedCoordinatedJointPathProfiles::new(profiles).unwrap();
    assert_eq!(timed.elapsed_time.to_bits(), 0.0_f64.to_bits());
    assert_eq!(timed.profiles.len(), 0);
    assert!(timed.profiles.next().is_none());
}

#[test]
fn an_overflowing_path_is_rejected_but_its_finite_suffix_is_accepted() {
    let points = [0.0, 1.0, 2.0, 3.0, 4.0].map(|x| angles([x, 0.0]));
    let speeds = [0.0; 5];
    let velocity = JointVelocityLimits::new(f64::MIN_POSITIVE, 1.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let mut profiles =
        CoordinatedJointPathProfiles::new(&points, &speeds, &velocity, &acceleration).unwrap();
    // Each of the four segments takes 2^1022. Their total is 2^1024.
    let duration = libm::scalbn(1.0, 1022);
    assert!((4.0 * duration).is_infinite());
    assert_eq!(
        TimedCoordinatedJointPathProfiles::new(profiles).unwrap_err(),
        PlannerError::TimeElapsedOverflow
    );

    assert_eq!(profiles.next().unwrap().unwrap().total_time(), duration);
    let timed = TimedCoordinatedJointPathProfiles::new(profiles).unwrap();
    assert_eq!(timed.elapsed_time, 0.0);
    assert_eq!(timed.profiles.len(), 3);
    assert_eq!(timed.profiles.total_path_time(), Ok(3.0 * duration));
}

#[test]
fn complete_absorption_of_either_positive_duration_is_rejected() {
    let velocity = JointVelocityLimits::new(f64::MIN_POSITIVE, 1.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let speeds = [0.0; 3];
    let forward = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]].map(angles);
    let backward = [forward[2], forward[1], forward[0]];
    for points in [forward, backward] {
        let mut profiles =
            CoordinatedJointPathProfiles::new(&points, &speeds, &velocity, &acceleration).unwrap();
        assert_eq!(
            TimedCoordinatedJointPathProfiles::new(profiles).unwrap_err(),
            PlannerError::TimeElapsedPrecisionLost
        );
        profiles.next().unwrap().unwrap();
        let timed = TimedCoordinatedJointPathProfiles::new(profiles).unwrap();
        assert_eq!(timed.profiles.len(), 1);
    }
}

#[test]
fn half_an_ulp_is_accepted_only_when_the_elapsed_time_advances() {
    let velocity = JointVelocityLimits::new(10.0, libm::scalbn(1.0, 56)).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, libm::scalbn(1.0, 108)).unwrap();
    let speeds = [0.0; 3];
    for (distance, elapsed, accepted) in [
        (0.25_f64, 1.0_f64, false),
        (0.25_f64.next_up().next_up(), 1.0_f64.next_up(), true),
    ] {
        let points = [[0.0, 0.0], [distance, 0.0], [distance, 1.0]].map(angles);
        let mut profiles =
            CoordinatedJointPathProfiles::new(&points, &speeds, &velocity, &acceleration).unwrap();
        let result = TimedCoordinatedJointPathProfiles::new(profiles);
        assert_eq!(profiles.next().unwrap().unwrap().total_time(), elapsed);
        assert_eq!(
            profiles.next().unwrap().unwrap().total_time(),
            f64::EPSILON / 2.0
        );
        if accepted {
            let timed = result.unwrap();
            assert_eq!(timed.profiles.total_path_time(), Ok(elapsed.next_up()));
        } else {
            assert_eq!(result.unwrap_err(), PlannerError::TimeElapsedPrecisionLost);
        }
    }
}

#[test]
fn the_first_timing_error_is_returned() {
    let speeds = [0.0; 6];
    let velocity = JointVelocityLimits::new(f64::MIN_POSITIVE, 1.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    for (coordinates, expected) in [
        (
            [
                [0.0, 0.0],
                [1.0, 0.0],
                [1.0, 1.0],
                [2.0, 1.0],
                [3.0, 1.0],
                [4.0, 1.0],
            ],
            PlannerError::TimeElapsedPrecisionLost,
        ),
        (
            [
                [0.0, 0.0],
                [1.0, 0.0],
                [2.0, 0.0],
                [3.0, 0.0],
                [4.0, 0.0],
                [4.0, 1.0],
            ],
            PlannerError::TimeElapsedOverflow,
        ),
    ] {
        let points = coordinates.map(angles);
        let profiles =
            CoordinatedJointPathProfiles::new(&points, &speeds, &velocity, &acceleration).unwrap();
        assert_eq!(
            TimedCoordinatedJointPathProfiles::new(profiles).unwrap_err(),
            expected
        );
    }
}
