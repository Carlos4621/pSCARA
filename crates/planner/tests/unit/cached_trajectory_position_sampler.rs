use super::*;
use crate::{CoordinatedJointPathProfiles, JointAccelerationLimits, JointVelocityLimits};

fn timed_path<'a>(
    points: &'a [JointAngles],
    speeds: &'a [f64],
    velocity: &'a JointVelocityLimits,
    acceleration: &'a JointAccelerationLimits,
) -> TimedCoordinatedJointPathProfiles<'a> {
    let path = CoordinatedJointPathProfiles::new(points, speeds, velocity, acceleration).unwrap();
    TimedCoordinatedJointPathProfiles::new(path).unwrap()
}

fn assert_same_position(
    trajectory: CoordinatedJointTrajectory<'_>,
    sampler: &mut CachedTrajectoryPositionSampler<'_>,
    time: f64,
) {
    let bits = |angles: JointAngles| {
        [
            angles.first_theta().to_bits(),
            angles.second_theta().to_bits(),
        ]
    };
    let expected = trajectory.position_at_clamped(time).map(bits);
    let actual = sampler.position_at_clamped(time).map(bits);
    assert_eq!(actual, expected, "time={time:?}");
}

#[test]
fn cached_positions_match_original_bits_for_out_of_order_queries() {
    let points = [[-0.0, 0.0], [4.0, -2.0], [4.0, -14.0], [-5.0, -14.0]]
        .map(|[first, second]| JointAngles::new(first, second).unwrap());
    let velocity = JointVelocityLimits::new(4.0, 2.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let timed = timed_path(&points, &[0.0; 4], &velocity, &acceleration);
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();
    let mut sampler = CachedTrajectoryPositionSampler::new(trajectory);

    for time in [
        9.0,
        5.0,
        5.5,
        5.25,
        5.25,
        14.0,
        12.0_f64.next_down(),
        4.0_f64.next_up(),
        4.0_f64.next_down(),
        0.0,
        -0.0,
        f64::from_bits(1),
        18.0_f64.next_down(),
        18.0,
        18.0_f64.next_up(),
        f64::MAX,
        2.0,
    ] {
        assert_same_position(trajectory, &mut sampler, time);
    }
}

#[test]
fn a_shared_boundary_replaces_the_later_cached_profile_with_the_earlier_one() {
    let points = [0.0, 1.0, 2.0, 3.0].map(|first| JointAngles::new(first, -0.0).unwrap());
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let timed = timed_path(&points, &[0.0; 4], &velocity, &acceleration);
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();
    let mut sampler = CachedTrajectoryPositionSampler::new(trajectory);

    for (earlier_start, boundary) in [(0.0, 2.0_f64), (2.0, 4.0_f64)] {
        assert_same_position(trajectory, &mut sampler, boundary.next_up());
        assert_eq!(
            sampler.cached_profile.as_ref().unwrap().start_time(),
            boundary
        );
        // Es el mismo punto articular en ambos perfiles. También comprobamos
        // cuál queda en caché para detectar un inicio inclusivo por error.
        assert_same_position(trajectory, &mut sampler, boundary);
        assert_eq!(
            sampler.cached_profile.as_ref().unwrap().start_time(),
            earlier_start
        );
        assert_eq!(sampler.cached_end_time, boundary);
    }
}

#[test]
fn invalid_times_preserve_the_profile_and_match_the_original_error() {
    let points = [0.0, 1.0, 2.0].map(|first| JointAngles::new(first, 0.0).unwrap());
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let timed = timed_path(&points, &[0.0; 3], &velocity, &acceleration);
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();
    let mut sampler = CachedTrajectoryPositionSampler::new(trajectory);

    assert_same_position(trajectory, &mut sampler, 3.0);
    for time in [
        -f64::from_bits(1),
        -1.0,
        f64::NAN,
        -f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
    ] {
        assert_eq!(
            sampler.position_at_clamped(time).unwrap_err(),
            PlannerError::InvalidTime
        );
        assert_same_position(trajectory, &mut sampler, time);
        assert_eq!(sampler.cached_profile.as_ref().unwrap().start_time(), 2.0);
        assert_eq!(sampler.cached_end_time, 4.0);
    }
    assert_same_position(trajectory, &mut sampler, 3.5);
    assert_same_position(trajectory, &mut sampler, 1.5);
}

#[test]
fn a_suffix_or_exhausted_source_matches_stateless_position_queries() {
    let points = [0.0, 1.0, 5.0].map(|first| JointAngles::new(first, 0.0).unwrap());
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let mut timed = timed_path(&points, &[0.0; 3], &velocity, &acceleration);
    assert_eq!(timed.next().unwrap().unwrap().end_time(), Ok(2.0));
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();
    let mut sampler = CachedTrajectoryPositionSampler::new(trajectory);

    for time in [
        4.0,
        2.0_f64.next_down(),
        2.0,
        2.0_f64.next_up(),
        0.0,
        6.0,
        f64::MAX,
    ] {
        assert_same_position(trajectory, &mut sampler, time);
    }

    timed.next().unwrap().unwrap();
    assert!(timed.next().is_none());
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();
    let mut sampler = CachedTrajectoryPositionSampler::new(trajectory);

    for time in [0.0, 2.0, f64::MAX, f64::NAN] {
        assert_same_position(trajectory, &mut sampler, time);
        assert!(sampler.cached_profile.is_none());
    }
}

#[cfg(feature = "transition-hinted")]
#[test]
fn transition_estimates_require_the_cached_profile_to_cover_the_interval() {
    let points = [0.0, 1.0, 2.0].map(|angle| JointAngles::new(angle, 0.0).unwrap());
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let timed = timed_path(&points, &[0.0; 3], &velocity, &acceleration);
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();
    let mut sampler = CachedTrajectoryPositionSampler::new(trajectory);
    let converter = JointToStepsConverter::new(8.0, 8.0).unwrap();
    let timebase = StepTimeBase::new(10).unwrap();
    let initial = MotorSteps {
        first: 0,
        second: 0,
    };

    assert_eq!(
        sampler.estimate_transition_tick(0, 20, initial, &converter, timebase),
        None
    );
    sampler.position_at_clamped(0.0).unwrap();
    assert!(
        sampler
            .estimate_transition_tick(0, 20, initial, &converter, timebase)
            .is_some()
    );
    assert_eq!(
        sampler.estimate_transition_tick(19, 21, initial, &converter, timebase),
        None
    );
    assert_eq!(sampler.cached_end_time, 2.0);

    let position = converter
        .convert(sampler.position_at_clamped(3.0).unwrap())
        .unwrap();
    assert_eq!(
        sampler.estimate_transition_tick(19, 30, position, &converter, timebase),
        None
    );
    assert!(
        sampler
            .estimate_transition_tick(30, 41, position, &converter, timebase)
            .is_some()
    );
    assert_eq!(sampler.cached_end_time, 4.0);
}
