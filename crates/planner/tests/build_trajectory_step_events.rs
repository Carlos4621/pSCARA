use kinematics::JointAngles;
use planner::{
    CoordinatedJointPathProfiles, CoordinatedJointTrajectory, JointAccelerationLimits,
    JointToStepsConverter, JointVelocityLimits, MotorStepMove, MotorSteps, PlannerError,
    StepDirection, StepTimeBase, TimedCoordinatedJointPathProfiles, build_trajectory_step_events,
};

fn steps(first: i64, second: i64) -> MotorSteps {
    MotorSteps { first, second }
}

fn points<const N: usize>(coordinates: [[f64; 2]; N]) -> [JointAngles; N] {
    coordinates.map(|[first, second]| JointAngles::new(first, second).unwrap())
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

#[test]
fn builds_a_two_motor_round_trip_from_a_nonzero_initial_position() -> Result<(), PlannerError> {
    let points = points([[1.0, -0.5], [2.0, -1.0], [1.0, -0.5]]);
    let limits = limits();
    let timed = timed_path(&points, &[0.0; 3], &limits);
    let base = StepTimeBase::new(8)?;
    let trajectory = CoordinatedJointTrajectory::new(timed)?;
    let reference_converter = JointToStepsConverter::new(4.0, 8.0)?;
    let sample =
        |tick| reference_converter.convert(trajectory.position_at(base.ticks_to_seconds(tick))?);

    let initial = sample(0)?;
    assert_eq!(initial, steps(4, -4));
    assert_eq!(sample(16)?, steps(8, -8));
    assert_eq!(sample(32)?, initial);

    let mut expected = Vec::new();
    let mut previous = initial;
    for tick in 1..=32 {
        let position = sample(tick)?;
        if position != previous {
            expected.push((tick, MotorStepMove::new(previous, position)));
        }
        previous = position;
    }

    let mut boundaries = [19, 23, 29, 48, 64];
    let events = build_trajectory_step_events(
        timed,
        JointToStepsConverter::new(4.0, 8.0)?,
        base,
        &mut boundaries,
    )?
    .collect::<Result<Vec<_>, _>>()?;

    assert_eq!(boundaries, [0, 16, 32, 48, 64]);
    assert_eq!(events, expected);
    assert_eq!(
        events.iter().map(|&(tick, _)| tick).collect::<Vec<_>>(),
        [4, 7, 10, 12, 21, 23, 26, 29]
    );
    for (tick, movement) in events {
        let first = movement.first();
        let second = movement.second();
        assert_eq!([first.pulses, second.pulses], [1, 1]);
        let directions = if tick <= 16 {
            [StepDirection::Positive, StepDirection::Negative]
        } else {
            [StepDirection::Negative, StepDirection::Positive]
        };
        assert_eq!([first.direction, second.direction], directions);
    }
    Ok(())
}

#[test]
fn a_two_motor_round_trip_replays_unequal_pulse_counts() -> Result<(), PlannerError> {
    let points = points([[0.0, 0.0], [1.0, -0.5], [0.0, 0.0]]);
    let limits = limits();
    let timed = timed_path(&points, &[0.0; 3], &limits);
    let base = StepTimeBase::new(8)?;
    let trajectory = CoordinatedJointTrajectory::new(timed)?;
    let converter = JointToStepsConverter::new(4.0, 4.0)?;
    let sample = |tick| converter.convert(trajectory.position_at(base.ticks_to_seconds(tick))?);

    let initial = sample(0)?;
    assert_eq!(initial, steps(0, 0));
    let mut expected = Vec::new();
    let mut previous = initial;
    for tick in 1..=32 {
        let position = sample(tick)?;
        if position != previous {
            expected.push((tick, MotorStepMove::new(previous, position)));
        }
        previous = position;
    }

    let mut boundaries = [0_u64; 3];
    let events = build_trajectory_step_events(timed, converter, base, &mut boundaries)?
        .collect::<Result<Vec<_>, _>>()?;
    assert_eq!(boundaries, [0, 16, 32]);
    assert_eq!(events, expected);

    let mut position = initial;
    let mut outbound_pulses = [0, 0];
    for (tick, movement) in events {
        let first = movement.first();
        let second = movement.second();
        assert!(first.pulses <= 1 && second.pulses <= 1, "tick {tick}");

        if tick <= boundaries[1] {
            assert!(
                first.pulses == 0 || first.direction == StepDirection::Positive,
                "tick {tick}"
            );
            assert!(
                second.pulses == 0 || second.direction == StepDirection::Negative,
                "tick {tick}"
            );
            outbound_pulses[0] += first.pulses;
            outbound_pulses[1] += second.pulses;
        }

        for (axis, movement) in [(&mut position.first, first), (&mut position.second, second)] {
            let pulses = i64::try_from(movement.pulses).unwrap();
            match movement.direction {
                StepDirection::Positive => *axis += pulses,
                StepDirection::Negative => *axis -= pulses,
            }
        }
    }
    assert_eq!(outbound_pulses, [4, 2]);
    assert_eq!(position, initial);
    Ok(())
}

#[test]
fn builds_step_events_for_a_reversal_between_ticks() -> Result<(), PlannerError> {
    let points = points([[0.0, 0.0], [25.0 / 16.0, 0.0], [1.0, 0.0]]);
    let limits = limits();
    let timed = timed_path(&points, &[0.0; 3], &limits);
    let base = StepTimeBase::new(3)?;
    let trajectory = CoordinatedJointTrajectory::new(timed)?;
    let converter = JointToStepsConverter::new(2.0, 2.0)?;
    let sample = |tick| converter.convert(trajectory.position_at(base.ticks_to_seconds(tick))?);

    let mut expected = Vec::new();
    let mut previous = sample(0)?;
    for tick in 1..=12 {
        let position = sample(tick)?;
        if position != previous {
            expected.push((tick, MotorStepMove::new(previous, position)));
        }
        previous = position;
    }

    let mut boundaries = [u64::MAX; 4];
    let events = build_trajectory_step_events(timed, converter, base, &mut boundaries)?
        .collect::<Result<Vec<_>, _>>()?;
    assert_eq!(boundaries, [0, 7, 8, 12]);
    assert_eq!(events, expected);
    assert_eq!(
        events
            .iter()
            .map(|&(tick, movement)| {
                let first = movement.first();
                (
                    tick,
                    first.direction,
                    first.pulses,
                    movement.second().pulses,
                )
            })
            .collect::<Vec<_>>(),
        [
            (3, StepDirection::Positive, 1, 0),
            (4, StepDirection::Positive, 1, 0),
            (6, StepDirection::Positive, 1, 0),
            (10, StepDirection::Negative, 1, 0),
        ]
    );
    Ok(())
}

#[test]
fn rejects_a_round_trip_inside_one_tick() -> Result<(), PlannerError> {
    let points = points([[0.0, 0.0], [1.0 / 16.0, 0.0], [0.0, 0.0]]);
    let limits = limits();
    let timed = timed_path(&points, &[0.0; 3], &limits);
    let timebase = StepTimeBase::new(1)?;
    let trajectory = CoordinatedJointTrajectory::new(timed)?;
    let converter = JointToStepsConverter::new(16.0, 16.0)?;
    let position_at = |seconds| converter.convert(trajectory.position_at(seconds)?);

    assert_eq!(position_at(0.0)?, steps(0, 0));
    assert_eq!(position_at(0.5)?, steps(1, 0));
    assert_eq!(position_at(1.0)?, steps(0, 0));

    let mut boundaries = [u64::MAX; 2];
    let result = build_trajectory_step_events(timed, converter, timebase, &mut boundaries);
    assert_eq!(
        result.err(),
        Some(PlannerError::SubtickMotionNotRepresentable)
    );
    assert_eq!(boundaries, [u64::MAX; 2]);
    Ok(())
}

#[test]
fn rejects_five_steps_inside_one_tick() -> Result<(), PlannerError> {
    let points = points([
        [0.0, 0.0],
        [1.0 / 256.0, 0.0],
        [0.0, 0.0],
        [1.0 / 256.0, 0.0],
        [0.0, 0.0],
        [1.0 / 256.0, 1.0 / 16.0],
    ]);
    let limits = limits();
    let timed = timed_path(&points, &[0.0; 6], &limits);
    let timebase = StepTimeBase::new(1)?;
    let trajectory = CoordinatedJointTrajectory::new(timed)?;
    let converter = JointToStepsConverter::new(256.0, 1.0)?;
    let position_at = |seconds| converter.convert(trajectory.position_at(seconds)?);

    for (time, first) in [
        (0.0, 0),
        (0.125, 1),
        (0.25, 0),
        (0.375, 1),
        (0.5, 0),
        (1.0, 1),
    ] {
        assert_eq!(position_at(time)?, steps(first, 0), "at {time} s");
    }

    let mut boundaries = [u64::MAX; 2];
    let result = build_trajectory_step_events(timed, converter, timebase, &mut boundaries);
    assert_eq!(
        result.err(),
        Some(PlannerError::SubtickMotionNotRepresentable)
    );
    assert_eq!(boundaries, [u64::MAX; 2]);
    Ok(())
}

#[test]
fn a_trajectory_that_quantizes_to_no_steps_exhausts_at_the_clock_limits() {
    let points = points([[0.0, 0.0], [0.25, -0.25]]);
    let limits = limits();
    let timed = timed_path(&points, &[0.0; 2], &limits);

    for end_tick in [1, u64::MAX] {
        let mut boundaries = [19, 23];
        {
            let mut events = build_trajectory_step_events(
                timed,
                JointToStepsConverter::new(1.0, 1.0).unwrap(),
                StepTimeBase::new(end_tick).unwrap(),
                &mut boundaries,
            )
            .unwrap();

            assert_eq!(events.next(), None);
            assert_eq!(events.next(), None);
        }
        assert_eq!(boundaries, [0, end_tick]);
    }
}

#[test]
fn an_insufficient_boundary_buffer_is_untouched_and_can_be_retried() {
    let points = points([[0.0, 0.0], [0.25, 0.0], [0.5, 0.0]]);
    let limits = limits();
    let timed = timed_path(&points, &[0.0; 3], &limits);
    let base = StepTimeBase::new(4).unwrap();
    let mut boundaries = [19, 23, 29];

    for capacity in 0..boundaries.len() {
        let result = build_trajectory_step_events(
            timed,
            JointToStepsConverter::new(4.0, 4.0).unwrap(),
            base,
            &mut boundaries[..capacity],
        );
        assert_eq!(result.err(), Some(PlannerError::InsufficientBoundaryBuffer));
        assert_eq!(boundaries, [19, 23, 29]);
    }

    let events = build_trajectory_step_events(
        timed,
        JointToStepsConverter::new(4.0, 4.0).unwrap(),
        base,
        &mut boundaries,
    )
    .unwrap()
    .collect::<Result<Vec<_>, _>>()
    .unwrap();
    assert_eq!(boundaries, [0, 4, 8]);
    assert_eq!(
        events,
        [
            (2, MotorStepMove::new(steps(0, 0), steps(1, 0))),
            (6, MotorStepMove::new(steps(1, 0), steps(2, 0))),
        ]
    );
}

#[test]
fn emits_the_final_step_after_an_off_grid_end() -> Result<(), PlannerError> {
    let points = points([[0.0, 0.0], [9.0 / 16.0, 0.0]]);
    let limits = limits();
    let timed = timed_path(&points, &[0.0; 2], &limits);
    let timebase = StepTimeBase::new(1)?;
    let converter = JointToStepsConverter::new(1.0, 1.0)?;

    let mut profiles = timed;
    let profile = profiles.next().expect("one profile")?;
    let end_time = profile.end_time()?;
    assert_eq!(end_time, 1.5);
    assert_eq!(
        converter.peak_step_frequencies(profile.peak_joint_velocities()?)?,
        (0.75, 0.0)
    );
    assert_eq!(timebase.bracketing_ticks(end_time)?, (1, 2));

    let mut boundaries = [19, 23];
    let events = build_trajectory_step_events(timed, converter, timebase, &mut boundaries)?
        .collect::<Result<Vec<_>, _>>()?;
    assert_eq!(boundaries, [0, 2]);
    assert_eq!(events, [(2, MotorStepMove::new(steps(0, 0), steps(1, 0)))]);
    Ok(())
}

#[test]
fn an_internal_boundary_and_the_end_can_share_the_final_tick_interval() -> Result<(), PlannerError>
{
    let points = points([[1.0 / 8.0, 0.0], [33.0 / 64.0, 0.0], [34.0 / 64.0, 0.0]]);
    let limits = limits();
    let timed = timed_path(&points, &[0.0; 3], &limits);
    let timebase = StepTimeBase::new(1)?;
    let converter = JointToStepsConverter::new(1.0, 1.0)?;

    let end_times = timed
        .map(|profile| profile?.end_time())
        .collect::<Result<Vec<_>, _>>()?;
    assert_eq!(end_times, [1.25, 1.5]);

    let mut boundaries = [19, 23, 29];
    let events = build_trajectory_step_events(timed, converter, timebase, &mut boundaries)?
        .collect::<Result<Vec<_>, _>>()?;
    assert_eq!(boundaries, [0, 1, 2]);
    assert_eq!(events, [(2, MotorStepMove::new(steps(0, 0), steps(1, 0)))]);
    Ok(())
}

#[test]
fn preserves_earlier_steps_when_the_end_after_an_aligned_boundary_is_off_grid()
-> Result<(), PlannerError> {
    let limits = limits();
    let timebase = StepTimeBase::new(1)?;
    let one_step = [(2, MotorStepMove::new(steps(0, 0), steps(1, 0)))];
    for (last_point, last_tick, expected) in [
        (0.25 + 1.0 / 256.0, 2, &[][..]),
        (0.25 + 9.0 / 16.0, 3, &one_step[..]),
    ] {
        let points = points([[0.0, 0.0], [0.25, 0.0], [last_point, 0.0]]);
        let timed = timed_path(&points, &[0.0; 3], &limits);
        let mut boundaries = [19, 23, 29];
        let events = build_trajectory_step_events(
            timed,
            JointToStepsConverter::new(1.0, 1.0)?,
            timebase,
            &mut boundaries,
        )?
        .collect::<Result<Vec<_>, _>>()?;
        assert_eq!(events.as_slice(), expected);
        assert_eq!(boundaries, [0, 1, last_tick]);
    }
    Ok(())
}

#[test]
fn tick_overflow_is_a_construction_error() -> Result<(), PlannerError> {
    let points = points([[0.0, 0.0], [0.25, 0.0], [0.5, 0.0]]);
    let limits = limits();
    let timed = timed_path(&points, &[0.0; 3], &limits);
    let mut boundaries = [19, 23, 29];
    let result = build_trajectory_step_events(
        timed,
        JointToStepsConverter::new(1.0, 1.0)?,
        StepTimeBase::new(u64::MAX)?,
        &mut boundaries,
    );
    assert_eq!(result.err(), Some(PlannerError::NumericalFailure));
    assert_eq!(boundaries, [19, 23, 29]);
    Ok(())
}

#[test]
fn a_started_or_exhausted_timed_path_has_no_profile_at_tick_zero() {
    let points = points([[0.0, 0.0], [0.25, 0.0], [0.5, 0.0]]);
    let limits = limits();
    let mut timed = timed_path(&points, &[0.0; 3], &limits);
    let base = StepTimeBase::new(1).unwrap();
    let mut boundaries = [19, 23, 29];

    for end_time in [1.0, 2.0] {
        assert_eq!(timed.next().unwrap().unwrap().end_time(), Ok(end_time));
        let result = build_trajectory_step_events(
            timed,
            JointToStepsConverter::new(1.0, 1.0).unwrap(),
            base,
            &mut boundaries,
        );
        assert_eq!(result.err(), Some(PlannerError::ProfileInstanceNotFound));
        assert_eq!(boundaries, [19, 23, 29]);
    }
}

#[test]
fn an_unrepresentable_initial_motor_position_fails_before_writing_boundaries() {
    let limits = limits();
    let base = StepTimeBase::new(1_u64 << 63).unwrap();
    let overflowing_scale = i64::MAX as f64;

    for (coordinates, scales) in [
        ([[1.0, 0.0], [1.25, 0.0]], [overflowing_scale, 1.0]),
        ([[0.0, 1.0], [0.0, 1.25]], [1.0, overflowing_scale]),
    ] {
        let points = points(coordinates);
        let timed = timed_path(&points, &[0.0; 2], &limits);
        let mut boundaries = [19, 23];
        let result = build_trajectory_step_events(
            timed,
            JointToStepsConverter::new(scales[0], scales[1]).unwrap(),
            base,
            &mut boundaries,
        );
        assert_eq!(result.err(), Some(PlannerError::NumericalFailure));
        assert_eq!(boundaries, [19, 23]);
    }
}

#[test]
fn rejects_later_motor_position_overflow_before_iteration() -> Result<(), PlannerError> {
    // Start the second motor at its last representable positive position.
    let initial_second = 1.0_f64.next_down();
    let points = points([[0.0, initial_second], [0.25, initial_second], [0.25, 1.0]]);
    let (velocity, _) = limits();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0 - initial_second)?;
    let limits = (velocity, acceleration);
    let timed = timed_path(&points, &[0.0; 3], &limits);
    let timebase = StepTimeBase::new(2048)?;
    let mut boundaries = [19, 23, 29];
    let result = build_trajectory_step_events(
        timed,
        JointToStepsConverter::new(4.0, i64::MAX as f64)?,
        timebase,
        &mut boundaries,
    );

    assert_eq!(result.err(), Some(PlannerError::NumericalFailure));
    assert_eq!(boundaries, [19, 23, 29]);
    Ok(())
}

#[test]
fn more_than_one_step_on_either_motor_is_rejected_before_iteration() {
    let points = points([[0.0, 0.0], [0.25, -0.25]]);
    let limits = limits();
    let timed = timed_path(&points, &[0.0; 2], &limits);
    let base = StepTimeBase::new(1).unwrap();

    for (first, second) in [(8.0, 4.0), (4.0, 8.0)] {
        let mut boundaries = [19, 23];
        let result = build_trajectory_step_events(
            timed,
            JointToStepsConverter::new(first, second).unwrap(),
            base,
            &mut boundaries,
        );
        assert_eq!(result.err(), Some(PlannerError::InsufficientTicksForSteps));
        assert_eq!(boundaries, [19, 23]);
    }
}

#[test]
fn an_insufficient_clock_leaves_the_buffer_intact_for_a_successful_retry()
-> Result<(), PlannerError> {
    let points = points([[0.0, 0.0], [0.25, -0.25]]);
    let limits = limits();
    let timed = timed_path(&points, &[0.0; 2], &limits);
    let mut boundaries = [19, 23];

    for clock in [1, 4] {
        let result = build_trajectory_step_events(
            timed,
            JointToStepsConverter::new(8.0, 4.0)?,
            StepTimeBase::new(clock)?,
            &mut boundaries,
        );
        assert_eq!(result.err(), Some(PlannerError::InsufficientTicksForSteps));
        assert_eq!(boundaries, [19, 23]);
    }

    let events = build_trajectory_step_events(
        timed,
        JointToStepsConverter::new(8.0, 4.0)?,
        StepTimeBase::new(8)?,
        &mut boundaries,
    )?
    .collect::<Result<Vec<_>, _>>()?;
    assert_eq!(boundaries, [0, 8]);
    assert_eq!(
        events,
        [
            (3, MotorStepMove::new(steps(0, 0), steps(1, 0))),
            (4, MotorStepMove::new(steps(1, 0), steps(1, -1))),
            (6, MotorStepMove::new(steps(1, -1), steps(2, -1))),
        ]
    );
    Ok(())
}
