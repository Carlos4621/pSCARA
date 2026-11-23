use kinematics::JointAngles;
use planner::{
    CoordinatedJointPathProfiles, JointAccelerationLimits, JointVelocityLimits, PlannerError,
    StepTimeBase, TimedCoordinatedJointPathProfiles, generate_segment_boundaries,
};

const UNWRITTEN: u64 = 12_345;

fn points<const N: usize>(first_joint: [f64; N]) -> [JointAngles; N] {
    first_joint.map(|angle| JointAngles::new(angle, 0.0).unwrap())
}

fn limits() -> (JointVelocityLimits, JointAccelerationLimits) {
    (
        JointVelocityLimits::new(10.0, 10.0).unwrap(),
        JointAccelerationLimits::new(1.0, 1.0).unwrap(),
    )
}

fn timed_path<'a>(
    points: &'a [JointAngles],
    speeds: &'a [f64],
    (velocity, acceleration): &'a (JointVelocityLimits, JointAccelerationLimits),
) -> TimedCoordinatedJointPathProfiles<'a> {
    let path = CoordinatedJointPathProfiles::new(points, speeds, velocity, acceleration).unwrap();
    TimedCoordinatedJointPathProfiles::new(path).unwrap()
}

fn end_times(profiles: TimedCoordinatedJointPathProfiles<'_>) -> Vec<f64> {
    profiles
        .map(|profile| profile.unwrap().end_time().unwrap())
        .collect()
}

#[test]
fn writes_global_tick_boundaries_and_preserves_unused_storage() {
    let points = points([0.0, 1.0, 5.0, 14.0]);
    let limits = limits();
    let timed = timed_path(&points, &[0.0; 4], &limits);
    let base = StepTimeBase::new(8).unwrap();

    // The three segments take 2, 4 and 6 seconds with unit acceleration.
    for capacity in [4, 6] {
        let mut storage = [UNWRITTEN; 8];
        assert_eq!(
            generate_segment_boundaries(timed, base, &mut storage[1..=capacity]),
            Ok(4)
        );
        assert_eq!(
            storage,
            [UNWRITTEN, 0, 16, 48, 96, UNWRITTEN, UNWRITTEN, UNWRITTEN]
        );
    }
}

#[test]
fn insufficient_capacity_preserves_the_buffer_and_allows_a_retry() {
    let points = points([0.0, 1.0, 5.0, 14.0]);
    let limits = limits();
    let timed = timed_path(&points, &[0.0; 4], &limits);
    let base = StepTimeBase::new(8).unwrap();
    let mut buffer = [UNWRITTEN; 4];

    for capacity in 0..buffer.len() {
        assert_eq!(
            generate_segment_boundaries(timed, base, &mut buffer[..capacity]),
            Err(PlannerError::InsufficientBoundaryBuffer),
            "capacity {capacity}"
        );
        assert_eq!(buffer, [UNWRITTEN; 4]);
    }

    assert_eq!(generate_segment_boundaries(timed, base, &mut buffer), Ok(4));
    assert_eq!(buffer, [0, 16, 48, 96]);
}

#[test]
fn rejects_a_timed_iterator_that_has_already_started() {
    let points = points([0.0, 1.0, 5.0]);
    let limits = limits();
    let mut timed = timed_path(&points, &[0.0; 3], &limits);
    let base = StepTimeBase::new(8).unwrap();
    let mut buffer = [UNWRITTEN; 2];

    assert_eq!(timed.next().unwrap().unwrap().end_time(), Ok(2.0));
    assert_eq!(
        generate_segment_boundaries(timed, base, &mut buffer),
        Err(PlannerError::InvalidTime)
    );
    assert_eq!(buffer, [UNWRITTEN; 2]);
}

#[test]
fn accepts_a_suffix_timed_from_zero() {
    let points = points([0.0, 1.0, 5.0]);
    let (velocity, acceleration) = limits();
    let mut path =
        CoordinatedJointPathProfiles::new(&points, &[0.0; 3], &velocity, &acceleration).unwrap();
    assert_eq!(path.next().unwrap().unwrap().total_time(), 2.0);
    let timed = TimedCoordinatedJointPathProfiles::new(path).unwrap();
    let base = StepTimeBase::new(8).unwrap();
    let mut buffer = [UNWRITTEN; 2];

    assert_eq!(generate_segment_boundaries(timed, base, &mut buffer), Ok(2));
    assert_eq!(buffer, [0, 32]);
}

#[test]
fn rejects_a_timed_iterator_with_no_profiles_left() {
    let points = points([0.0, 1.0]);
    let limits = limits();
    let mut timed = timed_path(&points, &[0.0; 2], &limits);
    let base = StepTimeBase::new(8).unwrap();
    let mut buffer = [UNWRITTEN; 2];

    timed.next().unwrap().unwrap();
    assert_eq!(
        generate_segment_boundaries(timed, base, &mut buffer),
        Err(PlannerError::InsufficientPoints)
    );
    assert!(timed.next().is_none());
    assert_eq!(
        generate_segment_boundaries(timed, base, &mut buffer),
        Err(PlannerError::InsufficientPoints)
    );
    assert_eq!(buffer, [UNWRITTEN; 2]);
}

#[test]
fn rejects_a_boundary_that_rounds_to_the_previous_tick() {
    const SHORT_DISTANCE: f64 = 1.0 / 4096.0;
    const SHORT_DURATION: f64 = 1.0 / 32.0;
    let limits = limits();
    let base = StepTimeBase::new(8).unwrap();

    for (coordinates, ends, rounded_ticks) in [
        (
            [0.0, SHORT_DISTANCE, 1.0 + SHORT_DISTANCE],
            [SHORT_DURATION, 2.0 + SHORT_DURATION],
            [0, 16],
        ),
        (
            [0.0, 1.0, 1.0 + SHORT_DISTANCE],
            [2.0, 2.0 + SHORT_DURATION],
            [16, 16],
        ),
    ] {
        let points = points(coordinates);
        let timed = timed_path(&points, &[0.0; 3], &limits);
        let mut buffer = [UNWRITTEN; 3];

        assert_eq!(end_times(timed), ends);
        assert_eq!(
            ends.map(|time| base.seconds_to_ticks(time)),
            rounded_ticks.map(Ok)
        );
        assert_eq!(
            generate_segment_boundaries(timed, base, &mut buffer),
            Err(PlannerError::InvalidTicks)
        );
        assert_eq!(buffer, [UNWRITTEN; 3]);
    }
}

#[test]
fn rejects_boundaries_between_ticks_even_when_rounded_ticks_increase() {
    let limits = limits();
    let base = StepTimeBase::new(1).unwrap();

    for (coordinates, ends, rounded_ticks) in [
        ([0.0, 1.0, 1.0 + 25.0 / 64.0], [2.0, 3.25], [2, 3]),
        ([0.0, 1.0, 1.0 + 9.0 / 16.0], [2.0, 3.5], [2, 4]),
        ([0.0, 9.0 / 16.0, 18.0 / 16.0], [1.5, 3.0], [2, 3]),
    ] {
        let points = points(coordinates);
        let timed = timed_path(&points, &[0.0; 3], &limits);
        let mut buffer = [UNWRITTEN; 3];

        assert_eq!(end_times(timed), ends);
        assert_eq!(
            ends.map(|time| base.seconds_to_ticks(time)),
            rounded_ticks.map(Ok)
        );
        assert_eq!(
            generate_segment_boundaries(timed, base, &mut buffer),
            Err(PlannerError::InvalidTicks)
        );
        assert_eq!(buffer, [UNWRITTEN; 3]);
    }
}

#[test]
fn accepts_one_tick_and_the_largest_representable_tick() {
    let limits = limits();

    for (distance, frequency, expected_tick) in [
        (1.0 / 256.0, 8, 1),
        (1.0 / 36.0, 3, 1),
        (0.25, u64::MAX, u64::MAX),
    ] {
        let points = points([0.0, distance]);
        let timed = timed_path(&points, &[0.0; 2], &limits);
        let base = StepTimeBase::new(frequency).unwrap();
        let mut buffer = [UNWRITTEN; 2];

        assert_eq!(end_times(timed), [base.ticks_to_seconds(expected_tick)]);
        assert_eq!(generate_segment_boundaries(timed, base, &mut buffer), Ok(2));
        assert_eq!(buffer, [0, expected_tick]);
    }
}

#[test]
fn propagates_tick_overflow_without_writing_an_earlier_valid_boundary() {
    let points = points([0.0, 0.25, 0.5]);
    let limits = limits();
    let timed = timed_path(&points, &[0.0; 3], &limits);
    let base = StepTimeBase::new(u64::MAX).unwrap();
    let mut buffer = [UNWRITTEN; 3];

    assert_eq!(end_times(timed), [1.0, 2.0]);
    assert_eq!(base.seconds_to_ticks(1.0), Ok(u64::MAX));
    assert_eq!(
        generate_segment_boundaries(timed, base, &mut buffer),
        Err(PlannerError::NumericalFailure)
    );
    assert_eq!(buffer, [UNWRITTEN; 3]);
}
