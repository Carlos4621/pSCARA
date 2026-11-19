use core::iter::FusedIterator;

use kinematics::JointAngles;

use super::TimedCoordinatedJointPathProfiles;
use crate::{
    CoordinatedJointPathProfiles, JointAccelerationLimits, JointVelocityLimits, PlannerError,
    plan_joint_path_speeds,
};

fn angles([first, second]: [f64; 2]) -> JointAngles {
    JointAngles::new(first, second).unwrap()
}

fn assert_exhausted<I: FusedIterator>(iter: &mut I) {
    assert!(iter.next().is_none());
    assert!(iter.next().is_none());
}

impl<'a> TimedCoordinatedJointPathProfiles<'a> {
    // The public constructor catches malformed sources before iteration.
    pub(crate) fn from_unvalidated_source_for_test(
        profiles: CoordinatedJointPathProfiles<'a>,
    ) -> Self {
        Self {
            profiles,
            elapsed_time: 0.0,
            is_finished: false,
        }
    }
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
        let segment = timed.next().unwrap().unwrap();
        assert_eq!(segment.start_time().to_bits(), 0.0_f64.to_bits());
        assert_eq!(segment.end_time(), Ok(2.0));
        assert_eq!(segment.position_at(0.0).unwrap().first_theta(), 0.0);
        assert_eq!(segment.position_at(2.0).unwrap().first_theta(), scale);
        assert_exhausted(&mut timed);
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
    let mut raw = timed.profiles;
    let before = raw.next().unwrap().unwrap();
    let after = raw.next().unwrap().unwrap();
    assert_eq!(
        before.velocity_at(before.total_time()),
        Ok((speeds[1], 0.0))
    );
    assert_eq!(after.velocity_at(0.0), Ok((speeds[1], 0.0)));

    let first = timed.next().unwrap().unwrap();
    let second = timed.next().unwrap().unwrap();
    let junction = first.end_time().unwrap();
    assert_eq!(first.start_time(), 0.0);
    assert_eq!(junction, 1.0);
    assert_eq!(second.start_time(), junction);
    assert_eq!(first.position_at(junction).unwrap().first_theta(), 1.0);
    assert_eq!(second.position_at(junction).unwrap().first_theta(), 1.0);
    assert_eq!(second.end_time(), Ok(2.0));
    assert_exhausted(&mut timed);
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
    for (from, to, start, end) in [(1.0, 5.0, 0.0, 4.0), (5.0, 14.0, 4.0, 10.0)] {
        let segment = timed.next().unwrap().unwrap();
        assert_eq!(segment.start_time(), start);
        assert_eq!(segment.end_time(), Ok(end));
        assert_eq!(segment.position_at(start).unwrap().first_theta(), from);
        assert_eq!(segment.position_at(end).unwrap().first_theta(), to);
    }
    assert_exhausted(&mut timed);
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
    assert_exhausted(&mut timed);
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
    assert_eq!(
        timed.last().unwrap().unwrap().end_time(),
        Ok(3.0 * duration)
    );
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
            let mut timed = result.unwrap();
            let first = timed.next().unwrap().unwrap();
            let second = timed.next().unwrap().unwrap();
            assert_eq!(first.end_time(), Ok(elapsed));
            assert_eq!(second.start_time(), elapsed);
            assert_eq!(second.end_time(), Ok(elapsed.next_up()));
            assert_exhausted(&mut timed);
        } else {
            assert_eq!(result.unwrap_err(), PlannerError::TimeElapsedPrecisionLost);
        }
    }
}

#[test]
fn next_reports_a_timing_error_once_and_preserves_the_last_time() {
    let points = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]].map(angles);
    let speeds = [0.0; 3];
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    for (speed, elapsed, expected) in [
        (
            10.0,
            libm::scalbn(1.0, 54),
            PlannerError::TimeElapsedPrecisionLost,
        ),
        (
            10.0,
            f64::from_bits(1),
            PlannerError::TimeElapsedPrecisionLost,
        ),
        (
            f64::MIN_POSITIVE,
            f64::MAX,
            PlannerError::TimeElapsedOverflow,
        ),
    ] {
        let velocity = JointVelocityLimits::new(speed, speed).unwrap();
        let profiles =
            CoordinatedJointPathProfiles::new(&points, &speeds, &velocity, &acceleration).unwrap();
        let mut timed = TimedCoordinatedJointPathProfiles::new(profiles).unwrap();
        // Construction normally detects these errors. Seed the clock to
        // exercise the defensive iterator path with another segment pending.
        timed.elapsed_time = elapsed;
        assert_eq!(timed.next().unwrap().unwrap_err(), expected);
        assert_eq!(timed.elapsed_time, elapsed);
        assert_eq!(timed.profiles.len(), 1);
        assert_exhausted(&mut timed);
        assert_eq!(timed.profiles.len(), 1);
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
