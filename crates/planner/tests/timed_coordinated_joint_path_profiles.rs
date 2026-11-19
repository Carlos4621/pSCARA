use core::iter::FusedIterator;

use kinematics::JointAngles;
use planner::{
    CoordinatedJointPathProfiles, JointAccelerationLimits, JointVelocityLimits, PlannerError,
    TimedCoordinatedJointPathProfiles, TimedCoordinatedJointProfile, plan_joint_path_speeds,
};

type Pair = [f64; 2];

fn angles([first, second]: Pair) -> JointAngles {
    JointAngles::new(first, second).unwrap()
}

fn position(profile: &TimedCoordinatedJointProfile, time: f64) -> Result<Pair, PlannerError> {
    profile
        .position_at(time)
        .map(|q| [q.first_theta(), q.second_theta()])
}

fn assert_interval(profile: &TimedCoordinatedJointProfile, start: f64, end: f64) {
    assert_eq!(profile.start_time(), start);
    assert_eq!(profile.end_time(), Ok(end));
}

fn assert_exhausted<I: FusedIterator>(iter: &mut I) {
    assert!(iter.next().is_none());
    assert!(iter.next().is_none());
}

#[test]
fn turns_and_reversals_keep_global_intervals_and_sample_positions() {
    let coordinates = [[0.0, 0.0], [4.0, -2.0], [4.0, -14.0], [-5.0, -14.0]];
    let points = coordinates.map(angles);
    let speeds = [0.0; 4];
    let velocity = JointVelocityLimits::new(4.0, 2.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let source =
        CoordinatedJointPathProfiles::new(&points, &speeds, &velocity, &acceleration).unwrap();
    let timed = TimedCoordinatedJointPathProfiles::new(source).unwrap();
    let segments = timed.collect::<Result<Vec<_>, _>>().unwrap();
    assert_eq!(segments.len(), 3);

    // The outer segments are triangular (4 and 6 s). The middle one has
    // two ramps of 2 s and a cruise of 4 s.
    let boundaries = [0.0_f64, 4.0, 12.0, 18.0];
    let midpoints = [[2.0, -1.0], [4.0, -8.0], [-0.5, -14.0]];
    for (i, segment) in segments.iter().enumerate() {
        let (start, end) = (boundaries[i], boundaries[i + 1]);
        assert_interval(segment, start, end);
        assert_eq!(position(segment, start), Ok(coordinates[i]));
        assert_eq!(position(segment, end), Ok(coordinates[i + 1]));
        assert_eq!(
            position(segment, f64::midpoint(start, end)),
            Ok(midpoints[i])
        );
        assert_eq!(
            position(segment, start.next_down()),
            Err(PlannerError::InvalidTime)
        );
        assert_eq!(
            position(segment, end.next_up()),
            Err(PlannerError::InvalidTime)
        );
    }
    assert_eq!(position(&segments[1], 5.0), Ok([4.0, -2.5]));
    assert_eq!(position(&segments[1], 11.0), Ok([4.0, -13.5]));
}

#[test]
fn timed_skipping_preserves_the_time_of_skipped_segments() {
    let points = [0.0, 1.0, 5.0, 14.0, 30.0].map(|x| angles([x, 0.0]));
    let speeds = [0.0; 5];
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let source =
        CoordinatedJointPathProfiles::new(&points, &speeds, &velocity, &acceleration).unwrap();
    let mut timed = TimedCoordinatedJointPathProfiles::new(source).unwrap();

    // Distances 1, 4, 9 and 16 give durations 2, 4, 6 and 8.
    let second = timed.nth(1).unwrap().unwrap();
    assert_interval(&second, 2.0, 6.0);
    assert_eq!(position(&second, 2.0), Ok([1.0, 0.0]));
    let mut prefix = timed.by_ref().take(1);
    assert_interval(&prefix.next().unwrap().unwrap(), 6.0, 12.0);
    assert!(prefix.next().is_none());
    assert_interval(&timed.next().unwrap().unwrap(), 12.0, 20.0);
    assert_exhausted(&mut timed);

    let mut raw = source;
    raw.nth(1).unwrap().unwrap();
    let mut suffix = TimedCoordinatedJointPathProfiles::new(raw).unwrap();
    let next = suffix.next().unwrap().unwrap();
    assert_interval(&next, 0.0, 6.0);
    assert_eq!(position(&next, 0.0), Ok([5.0, 0.0]));
    assert!(suffix.nth(10).is_none());
    assert_exhausted(&mut suffix);
}

#[test]
fn copying_a_started_iterator_keeps_an_independent_cursor_and_clock() {
    let points = [0.0, 1.0, 5.0, 14.0].map(|x| angles([x, 0.0]));
    let speeds = [0.0; 4];
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let source =
        CoordinatedJointPathProfiles::new(&points, &speeds, &velocity, &acceleration).unwrap();
    let mut original = TimedCoordinatedJointPathProfiles::new(source).unwrap();
    assert_interval(&original.next().unwrap().unwrap(), 0.0, 2.0);
    let mut copy = original;

    let iter: &mut dyn FusedIterator<Item = Result<TimedCoordinatedJointProfile, PlannerError>> =
        &mut copy;
    let last = iter.nth(1).unwrap().unwrap();
    assert_interval(&last, 6.0, 12.0);
    assert_eq!(position(&last, 12.0), Ok([14.0, 0.0]));
    assert_exhausted(&mut copy);

    let second = original.next().unwrap().unwrap();
    assert_interval(&second, 2.0, 6.0);
    assert_eq!(position(&second, 6.0), Ok([5.0, 0.0]));
    assert_interval(&original.next().unwrap().unwrap(), 6.0, 12.0);
    assert_exhausted(&mut original);
}

#[test]
fn public_speed_planner_feeds_a_path_with_moving_junctions_and_a_stop() {
    let coordinates = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [2.0, 1.0], [2.0, 2.0]];
    let points = coordinates.map(angles);
    let mut speeds = [f64::NAN; 5];
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(2.0, 2.0).unwrap();
    plan_joint_path_speeds(&points, &velocity, &acceleration, &mut speeds).unwrap();
    assert_eq!(speeds, [0.0, 2.0, 0.0, 2.0, 0.0]);
    let source =
        CoordinatedJointPathProfiles::new(&points, &speeds, &velocity, &acceleration).unwrap();
    let mut timed = TimedCoordinatedJointPathProfiles::new(source).unwrap();

    let samples = [[0.25, 0.0], [1.75, 0.0], [2.0, 0.25], [2.0, 1.75]];
    for (i, (start, sample)) in [0.0, 1.0, 2.0, 3.0].into_iter().zip(samples).enumerate() {
        let segment = timed.next().unwrap().unwrap();
        assert_interval(&segment, start, start + 1.0);
        assert_eq!(position(&segment, start), Ok(coordinates[i]));
        assert_eq!(position(&segment, start + 0.5), Ok(sample));
        assert_eq!(position(&segment, start + 1.0), Ok(coordinates[i + 1]));
    }
    assert_exhausted(&mut timed);
}

#[test]
fn a_segment_one_ulp_wide_in_global_time_reaches_both_exact_positions() {
    let distance = 0.25_f64.next_up().next_up();
    let points = [[0.0, 0.0], [distance, 0.0], [distance, 1.0]].map(angles);
    let speeds = [0.0; 3];
    let velocity = JointVelocityLimits::new(10.0, libm::scalbn(1.0, 56)).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, libm::scalbn(1.0, 108)).unwrap();
    let source =
        CoordinatedJointPathProfiles::new(&points, &speeds, &velocity, &acceleration).unwrap();
    let mut timed = TimedCoordinatedJointPathProfiles::new(source).unwrap();
    let first = timed.next().unwrap().unwrap();
    let second = timed.next().unwrap().unwrap();

    // The second profile's local duration is half an ulp. Its global time
    // interval advances by one ulp because the previous boundary is odd.
    let junction = 1.0_f64.next_up();
    let end = junction.next_up();
    assert_interval(&first, 0.0, junction);
    assert_interval(&second, junction, end);
    assert_eq!(position(&first, junction), Ok([distance, 0.0]));
    assert_eq!(position(&second, junction), Ok([distance, 0.0]));
    assert_eq!(position(&second, end), Ok([distance, 1.0]));
    assert_eq!(
        position(&second, end.next_up()),
        Err(PlannerError::InvalidTime)
    );
    assert_exhausted(&mut timed);
}
