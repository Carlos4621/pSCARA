use core::{
    cell::{Cell, RefCell},
    iter::FusedIterator,
};

use kinematics::JointAngles;
use planner::{
    CoordinatedJointPathProfiles, CoordinatedJointTrajectory, JointAccelerationLimits,
    JointToStepsConverter, JointVelocityLimits, MotorStepMove, MotorSteps, PlannerError,
    SegmentedTrajectoryStepEventIterator, StepDirection, StepTimeBase,
    TimedCoordinatedJointPathProfiles, build_segmented_step_events,
};

fn steps(first: i64, second: i64) -> MotorSteps {
    MotorSteps { first, second }
}

fn assert_exhausted(
    iterator: &mut impl FusedIterator<Item = Result<(u64, MotorStepMove), PlannerError>>,
) {
    for _ in 0..3 {
        assert_eq!(iterator.next(), None);
    }
}

#[test]
fn a_stationary_trajectory_returns_none_and_stops_sampling() {
    let position = steps(3, -5);
    let queries = RefCell::new(Vec::new());
    let mut iterator =
        SegmentedTrajectoryStepEventIterator::new(&[7, 8, 32, u64::MAX], position, |tick| {
            queries.borrow_mut().push(tick);
            Ok(position)
        })
        .unwrap();

    assert_exhausted(&mut iterator);
    let queries = queries.borrow();
    assert_eq!(queries.first(), Some(&8));
    assert_eq!(queries.last(), Some(&u64::MAX));
    assert!(queries.contains(&32));
    assert!(queries.windows(2).all(|pair| pair[0] < pair[1]));
}

#[test]
fn an_event_on_the_last_boundary_is_followed_by_none() {
    let initial = steps(i64::MAX - 1, i64::MIN + 1);
    let target = steps(i64::MAX, i64::MIN);
    let calls = Cell::new(0);
    let mut iterator =
        SegmentedTrajectoryStepEventIterator::new(&[u64::MAX - 1, u64::MAX], initial, |tick| {
            assert_eq!(tick, u64::MAX);
            calls.set(calls.get() + 1);
            Ok(target)
        })
        .unwrap();

    assert_eq!(
        iterator.next(),
        Some(Ok((u64::MAX, MotorStepMove::new(initial, target))))
    );
    assert_exhausted(&mut iterator);
    assert_eq!(calls.get(), 1);
}

#[test]
fn a_last_event_before_the_final_boundary_is_followed_by_none() {
    let initial = steps(0, 0);
    let target = steps(1, 0);
    let mut iterator = SegmentedTrajectoryStepEventIterator::new(&[10, 14], initial, |tick| {
        assert!((11..=14).contains(&tick));
        Ok(target)
    })
    .unwrap();

    assert_eq!(
        iterator.next(),
        Some(Ok((11, MotorStepMove::new(initial, target))))
    );
    assert_exhausted(&mut iterator);
}

#[test]
fn emits_each_shared_boundary_event_once_across_reversals() {
    let positions = [
        steps(0, 0),
        steps(1, 0),
        steps(1, -1),
        steps(0, -1),
        steps(0, 0),
        steps(1, 1),
    ];
    let mut iterator =
        SegmentedTrajectoryStepEventIterator::new(&[10, 12, 14, 15], positions[0], |tick| {
            assert!((11..=15).contains(&tick));
            Ok(positions[usize::try_from(tick - 10).unwrap()])
        })
        .unwrap();

    for (tick, pair) in (11..=15).zip(positions.windows(2)) {
        assert_eq!(
            iterator.next(),
            Some(Ok((tick, MotorStepMove::new(pair[0], pair[1]))))
        );
    }
}

#[test]
fn skips_stationary_segments_before_and_between_events() {
    let initial = steps(3, -5);
    let first = steps(4, -6);
    let mut iterator =
        SegmentedTrajectoryStepEventIterator::new(&[5, 10, 15, 20, 25], initial, |tick| {
            assert!((6..=25).contains(&tick));
            Ok(if (14..24).contains(&tick) {
                first
            } else {
                initial
            })
        })
        .unwrap();

    for (tick, from, to) in [(14, initial, first), (24, first, initial)] {
        assert_eq!(
            iterator.next(),
            Some(Ok((tick, MotorStepMove::new(from, to))))
        );
    }
}

#[test]
fn searches_wide_segments_up_to_the_maximum_tick() {
    let initial = steps(i64::MIN, i64::MAX);
    let first = steps(i64::MIN + 1, i64::MAX - 1);
    let mut iterator =
        SegmentedTrajectoryStepEventIterator::new(&[0, u64::MAX - 1, u64::MAX], initial, |tick| {
            assert_ne!(tick, 0);
            Ok(if tick == u64::MAX - 1 { first } else { initial })
        })
        .unwrap();

    assert_eq!(
        iterator.next(),
        Some(Ok((u64::MAX - 1, MotorStepMove::new(initial, first))))
    );
    assert_eq!(
        iterator.next(),
        Some(Ok((u64::MAX, MotorStepMove::new(first, initial))))
    );
}

#[test]
fn propagates_endpoint_and_midpoint_errors_in_later_segments_once() {
    let initial = steps(0, 0);
    let first = steps(1, -1);

    for (fail_at, error) in [
        (15, PlannerError::InvalidTime),
        (14, PlannerError::NumericalFailure),
    ] {
        let calls = Cell::new(0);
        let mut iterator =
            SegmentedTrajectoryStepEventIterator::new(&[10, 11, 15, 20], initial, |tick| {
                assert!((11..=15).contains(&tick));
                calls.set(calls.get() + 1);
                match tick {
                    tick if tick == fail_at => Err(error),
                    11..15 => Ok(first),
                    _ => Ok(steps(2, -1)),
                }
            })
            .unwrap();

        assert_eq!(
            iterator.next(),
            Some(Ok((11, MotorStepMove::new(initial, first))))
        );
        assert_eq!(iterator.next(), Some(Err(error)));
        let calls_at_failure = calls.get();
        assert_exhausted(&mut iterator);
        assert_eq!(calls.get(), calls_at_failure);
    }
}

#[test]
fn rejects_multiple_steps_on_either_axis_in_a_later_segment() {
    let initial = steps(0, 0);
    let first = steps(1, -1);

    for invalid in [steps(3, -1), steps(1, -3), steps(i64::MIN, i64::MAX)] {
        let mut iterator =
            SegmentedTrajectoryStepEventIterator::new(&[0, 1, 2, 3], initial, |tick| match tick {
                1 => Ok(first),
                2 => Ok(invalid),
                _ => panic!("unexpected sample at tick {tick}"),
            })
            .unwrap();

        assert_eq!(
            iterator.next(),
            Some(Ok((1, MotorStepMove::new(initial, first))))
        );
        assert_eq!(
            iterator.next(),
            Some(Err(PlannerError::MultipleStepsOnTick))
        );
        assert_exhausted(&mut iterator);
    }
}

#[test]
fn a_real_round_trip_emits_the_same_events_as_tick_by_tick_sampling() -> Result<(), PlannerError> {
    let points = [[0.0, 0.0], [1.0, 0.0], [0.0, 0.0]].map(|[a, b]| JointAngles::new(a, b).unwrap());
    let velocity = JointVelocityLimits::new(10.0, 10.0)?;
    let acceleration = JointAccelerationLimits::new(1.0, 1.0)?;
    let profiles = CoordinatedJointPathProfiles::new(&points, &[0.0; 3], &velocity, &acceleration)?;
    let timed = TimedCoordinatedJointPathProfiles::new(profiles)?;
    let timebase = StepTimeBase::new(8)?;

    let trajectory = CoordinatedJointTrajectory::new(timed)?;
    let converter = JointToStepsConverter::new(4.0, 4.0)?;
    let sample = |tick| converter.convert(trajectory.position_at(timebase.ticks_to_seconds(tick))?);
    let initial = sample(0)?;
    let mut expected = Vec::new();
    let mut previous = initial;
    for tick in 1..=32 {
        let position = sample(tick)?;
        if previous != position {
            expected.push((tick, MotorStepMove::new(previous, position)));
        }
        previous = position;
    }
    assert_eq!(previous, initial);
    assert_eq!(
        expected.iter().map(|&(tick, _)| tick).collect::<Vec<_>>(),
        [4, 7, 10, 12, 21, 23, 26, 29]
    );

    let mut boundaries = [19, 23, 29, 48, 64];
    let mut iterator =
        build_segmented_step_events(timed, timebase, &mut boundaries, initial, sample)?;
    for event in expected {
        assert_eq!(iterator.next(), Some(Ok(event)));
    }
    assert_exhausted(&mut iterator);
    assert_eq!(boundaries, [0, 16, 32, 48, 64]);
    Ok(())
}

#[test]
fn a_two_motor_round_trip_matches_tick_by_tick_sampling() -> Result<(), PlannerError> {
    let points =
        [[0.0, 0.0], [1.0, -0.5], [0.0, 0.0]].map(|[a, b]| JointAngles::new(a, b).unwrap());
    let velocity = JointVelocityLimits::new(10.0, 10.0)?;
    let acceleration = JointAccelerationLimits::new(1.0, 1.0)?;
    let profiles = CoordinatedJointPathProfiles::new(&points, &[0.0; 3], &velocity, &acceleration)?;
    let timed = TimedCoordinatedJointPathProfiles::new(profiles)?;
    let trajectory = CoordinatedJointTrajectory::new(timed)?;
    let converter = JointToStepsConverter::new(4.0, 4.0)?;
    let timebase = StepTimeBase::new(8)?;

    let duration = timed
        .last()
        .expect("round trip has two profiles")?
        .end_time()?;
    assert_eq!(duration, 4.0);
    let end_tick = timebase.seconds_to_ticks(duration)?;

    let sample = |tick| {
        let time = timebase.ticks_to_seconds(tick);
        let angles = trajectory.position_at(time)?;
        converter.convert(angles)
    };
    let initial_position = sample(0)?;
    assert_eq!(initial_position, steps(0, 0));
    assert_eq!(sample(16)?, steps(4, -2));
    assert_eq!(sample(end_tick)?, initial_position);

    let mut expected = Vec::new();
    let mut previous = initial_position;
    for tick in 1..=end_tick {
        let position = sample(tick)?;
        if position != previous {
            expected.push((tick, MotorStepMove::new(previous, position)));
        }
        previous = position;
    }

    let mut boundaries = [0_u64; 3];
    let events =
        build_segmented_step_events(timed, timebase, &mut boundaries, initial_position, sample)?;
    let result = events.collect::<Result<Vec<_>, _>>()?;
    assert_eq!(boundaries, [0, 16, 32]);
    assert_eq!(result, expected);
    assert!(result.windows(2).all(|pair| pair[0].0 < pair[1].0));

    let mut position = initial_position;
    for (tick, movement) in result {
        for (axis, movement) in [
            (&mut position.first, movement.first()),
            (&mut position.second, movement.second()),
        ] {
            assert!(movement.pulses <= 1, "tick {tick}");
            let pulses = i64::try_from(movement.pulses).unwrap();
            match movement.direction {
                StepDirection::Positive => *axis += pulses,
                StepDirection::Negative => *axis -= pulses,
            }
        }
        assert_eq!(position, sample(tick)?, "tick {tick}");
    }
    assert_eq!(position, steps(0, 0));
    Ok(())
}
