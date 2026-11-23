use core::{cell::Cell, iter::FusedIterator};

use kinematics::JointAngles;
use planner::{
    CoordinatedJointPathProfiles, CoordinatedJointTrajectory, CoordinatedJointTrajectoryCursor,
    JointAccelerationLimits, JointToStepsConverter, JointVelocityLimits, MotorPositionSampler,
    MotorStepMove, MotorSteps, PlannerError, StepDirection, StepTimeBase,
    TimedCoordinatedJointPathProfiles, TrajectoryStepEventIterator,
};

fn steps(first: i64, second: i64) -> MotorSteps {
    MotorSteps { first, second }
}

fn assert_exhausted(
    iterator: &mut dyn FusedIterator<Item = Result<(u64, MotorStepMove), PlannerError>>,
) {
    for _ in 0..4 {
        assert_eq!(iterator.next(), None);
    }
}

fn replay(position: &mut MotorSteps, movement: MotorStepMove) {
    let first = movement.first();
    let second = movement.second();
    assert!(first.pulses <= 1 && second.pulses <= 1);
    assert!(first.pulses != 0 || second.pulses != 0);
    for (axis, delta) in [(&mut position.first, first), (&mut position.second, second)] {
        for _ in 0..delta.pulses {
            *axis = match delta.direction {
                StepDirection::Positive => axis.checked_add(1).unwrap(),
                StepDirection::Negative => axis.checked_sub(1).unwrap(),
            };
        }
    }
}

#[test]
fn rejects_reversed_intervals_without_calling_the_sampler() {
    for (start, end) in [(1, 0), (42, 41), (u64::MAX, 0)] {
        let iterator = TrajectoryStepEventIterator::new(
            start,
            end,
            steps(0, 0),
            |_: u64| -> Result<MotorSteps, PlannerError> { panic!("invalid interval sampled") },
        );
        assert_eq!(iterator.err(), Some(PlannerError::EndBeforeStart));
    }
}

#[test]
fn empty_intervals_are_fused_without_sampling() {
    for tick in [0, 42, u64::MAX] {
        let mut iterator = TrajectoryStepEventIterator::new(tick, tick, steps(3, -5), |_| {
            panic!("empty interval sampled")
        })
        .unwrap();
        assert_exhausted(&mut iterator);
    }
}

#[test]
fn stationary_intervals_stop_sampling_after_the_first_none() {
    let calls = Cell::new(0);
    let mut iterator = TrajectoryStepEventIterator::new(42, u64::MAX, steps(-7, 9), |tick| {
        assert_eq!(tick, u64::MAX);
        calls.set(calls.get() + 1);
        Ok(steps(-7, 9))
    })
    .unwrap();

    assert_eq!(calls.get(), 0);
    assert_exhausted(&mut iterator);
    assert_eq!(calls.get(), 1);
}

#[test]
fn emits_the_first_change_and_every_subsequent_change_in_order() {
    let positions = [
        steps(5, -3),
        steps(5, -3),
        steps(6, -3),
        steps(6, -4),
        steps(6, -4),
        steps(7, -5),
        steps(7, -5),
        steps(7, -6),
    ];
    let expected = [
        (12, MotorStepMove::new(positions[1], positions[2])),
        (13, MotorStepMove::new(positions[2], positions[3])),
        (15, MotorStepMove::new(positions[4], positions[5])),
        (17, MotorStepMove::new(positions[6], positions[7])),
    ];
    let mut iterator = TrajectoryStepEventIterator::new(10, 17, positions[0], |tick| {
        assert!((11..=17).contains(&tick));
        Ok(positions[usize::try_from(tick - 10).unwrap()])
    })
    .unwrap();

    for event in expected {
        assert_eq!(iterator.next(), Some(Ok(event)));
    }
    assert_exhausted(&mut iterator);
}

#[test]
fn every_short_monotone_schedule_matches_a_linear_scan() {
    // Exhaust all six-tick schedules of idle / first / second / both axes,
    // in each direction combination and at three absolute tick origins.
    // Each position differs permanently after a transition, as required by
    // the binary search. Expected events come from adjacent samples only.
    const WIDTH: usize = 6;
    for first_direction in [-1, 1] {
        for second_direction in [-1, 1] {
            for code in 0..4_usize.pow(WIDTH as u32) {
                let mut positions = [steps(-3, 4); WIDTH + 1];
                let mut pattern = code;
                for i in 1..=WIDTH {
                    let mask = pattern % 4;
                    pattern /= 4;
                    positions[i] = steps(
                        positions[i - 1].first + first_direction * (mask & 1) as i64,
                        positions[i - 1].second + second_direction * ((mask >> 1) & 1) as i64,
                    );
                }

                for start in [0, 41, u64::MAX - WIDTH as u64] {
                    let end = start + WIDTH as u64;
                    let expected: Vec<_> = positions
                        .windows(2)
                        .enumerate()
                        .filter(|(_, pair)| pair[0] != pair[1])
                        .map(|(i, pair)| {
                            (start + i as u64 + 1, MotorStepMove::new(pair[0], pair[1]))
                        })
                        .collect();
                    let mut iterator =
                        TrajectoryStepEventIterator::new(start, end, positions[0], |tick| {
                            assert!((start + 1..=end).contains(&tick));
                            Ok(positions[usize::try_from(tick - start).unwrap()])
                        })
                        .unwrap();
                    let actual = iterator.by_ref().collect::<Result<Vec<_>, _>>().unwrap();
                    assert_eq!(actual, expected, "code={code}, start={start}");
                    let mut reached = positions[0];
                    for (_, movement) in actual {
                        replay(&mut reached, movement);
                    }
                    assert_eq!(reached, positions[WIDTH]);
                    assert_exhausted(&mut iterator);
                }
            }
        }
    }
}

#[test]
fn one_tick_intervals_accept_each_axis_direction_at_signed_limits() {
    for initial in [
        steps(i64::MIN, i64::MIN),
        steps(i64::MIN, i64::MAX),
        steps(i64::MAX, i64::MIN),
        steps(i64::MAX, i64::MAX),
    ] {
        for (move_first, move_second) in [(true, false), (false, true), (true, true)] {
            let toward_zero = |position: i64| {
                if position == i64::MIN {
                    position + 1
                } else {
                    position - 1
                }
            };
            let target = steps(
                if move_first {
                    toward_zero(initial.first)
                } else {
                    initial.first
                },
                if move_second {
                    toward_zero(initial.second)
                } else {
                    initial.second
                },
            );
            for start in [0, 42, u64::MAX - 1] {
                let calls = Cell::new(0);
                let mut iterator =
                    TrajectoryStepEventIterator::new(start, start + 1, initial, |tick| {
                        assert_eq!(tick, start + 1);
                        calls.set(calls.get() + 1);
                        Ok(target)
                    })
                    .unwrap();
                assert_eq!(
                    iterator.next(),
                    Some(Ok((start + 1, MotorStepMove::new(initial, target))))
                );
                assert_exhausted(&mut iterator);
                assert_eq!(calls.get(), 1);
            }
        }
    }
}

#[test]
fn reaches_both_signed_limits_after_multiple_events_without_overflow() {
    for direction in [-1, 1] {
        let initial = if direction == 1 {
            steps(i64::MAX - 3, i64::MIN + 3)
        } else {
            steps(i64::MIN + 3, i64::MAX - 3)
        };
        let mut iterator = TrajectoryStepEventIterator::new(0, 3, initial, |tick| {
            let delta = i64::try_from(tick).unwrap() * direction;
            Ok(steps(initial.first + delta, initial.second - delta))
        })
        .unwrap();
        let mut reached = initial;
        for tick in 1..=3 {
            let event = iterator.next().unwrap().unwrap();
            assert_eq!(event.0, tick);
            replay(&mut reached, event.1);
        }
        assert_eq!(
            reached,
            if direction == 1 {
                steps(i64::MAX, i64::MIN)
            } else {
                steps(i64::MIN, i64::MAX)
            }
        );
        assert_exhausted(&mut iterator);
    }
}

#[test]
fn searches_the_full_unsigned_tick_range_without_overflow() {
    let initial = steps(-8, 5);
    let target = steps(-7, 4);
    for threshold in [1, u64::MAX / 2, u64::MAX / 2 + 1, u64::MAX - 1, u64::MAX] {
        let calls = Cell::new(0);
        let mut iterator = TrajectoryStepEventIterator::new(0, u64::MAX, initial, |tick| {
            assert_ne!(tick, 0);
            calls.set(calls.get() + 1);
            assert!(calls.get() <= 66, "search did not stay logarithmic");
            Ok(if tick < threshold { initial } else { target })
        })
        .unwrap();
        assert_eq!(
            iterator.next(),
            Some(Ok((threshold, MotorStepMove::new(initial, target))))
        );
        assert_exhausted(&mut iterator);
    }
}

#[test]
fn supports_direction_changes_when_no_position_is_revisited() {
    // The first motor reverses, but the second prevents a return to any
    // previous position pair; the binary-search precondition still holds.
    let positions = [steps(0, 0), steps(1, 1), steps(0, 2), steps(-1, 3)];
    let mut iterator = TrajectoryStepEventIterator::new(0, 3, positions[0], |tick| {
        Ok(positions[usize::try_from(tick).unwrap()])
    })
    .unwrap();
    for (i, pair) in positions.windows(2).enumerate() {
        assert_eq!(
            iterator.next(),
            Some(Ok((i as u64 + 1, MotorStepMove::new(pair[0], pair[1]))))
        );
    }
    assert_exhausted(&mut iterator);
}

#[test]
fn propagates_endpoint_and_midpoint_errors_once_then_stops_sampling() {
    for error in [
        PlannerError::InvalidTime,
        PlannerError::NumericalFailure,
        PlannerError::TimeWentBackwards,
    ] {
        for fail_at in [16, 8, 4, 2, 1] {
            let calls = Cell::new(0);
            let mut iterator = TrajectoryStepEventIterator::new(0, 16, steps(0, 0), |tick| {
                calls.set(calls.get() + 1);
                if tick == fail_at {
                    Err(error)
                } else {
                    Ok(steps(1, 0))
                }
            })
            .unwrap();
            assert_eq!(iterator.next(), Some(Err(error)));
            let calls_at_failure = calls.get();
            assert_exhausted(&mut iterator);
            assert_eq!(calls.get(), calls_at_failure);
        }
    }
}

#[test]
fn errors_after_a_successful_event_do_not_allow_a_retry() {
    let fail = Cell::new(false);
    let calls = Cell::new(0);
    let mut iterator = TrajectoryStepEventIterator::new(0, 4, steps(0, 0), |tick| {
        calls.set(calls.get() + 1);
        if fail.get() {
            Err(PlannerError::InvalidTime)
        } else {
            Ok(steps(i64::try_from(tick).unwrap(), 0))
        }
    })
    .unwrap();
    assert_eq!(
        iterator.next(),
        Some(Ok((1, MotorStepMove::new(steps(0, 0), steps(1, 0)))))
    );
    fail.set(true);
    assert_eq!(iterator.next(), Some(Err(PlannerError::InvalidTime)));
    let calls_at_failure = calls.get();
    fail.set(false);
    assert_exhausted(&mut iterator);
    assert_eq!(calls.get(), calls_at_failure);
}

#[test]
fn rejects_multiple_pulses_on_either_axis_and_becomes_fused() {
    let initial = steps(3, -5);
    for target in [
        steps(5, -5),
        steps(1, -5),
        steps(3, -3),
        steps(3, -7),
        steps(5, -7),
        steps(i64::MIN, i64::MAX),
    ] {
        let mut iterator = TrajectoryStepEventIterator::new(0, 4, initial, |_| Ok(target)).unwrap();
        assert_eq!(
            iterator.next(),
            Some(Err(PlannerError::MultipleStepsOnTick))
        );
        assert_exhausted(&mut iterator);
    }
}

#[test]
fn rejects_a_multiple_step_tick_after_emitting_the_valid_prefix() {
    let initial = steps(0, 0);
    let first = steps(1, -1);
    let mut iterator = TrajectoryStepEventIterator::new(0, 4, initial, |tick| {
        Ok(if tick == 1 { first } else { steps(3, -2) })
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

#[test]
fn rejects_extreme_signed_jumps_without_arithmetic_panics() {
    for (initial, target) in [
        (steps(i64::MIN, 0), steps(i64::MAX, 0)),
        (steps(0, i64::MAX), steps(0, i64::MIN)),
        (steps(i64::MAX, i64::MIN), steps(i64::MIN, i64::MAX)),
    ] {
        let mut iterator =
            TrajectoryStepEventIterator::new(u64::MAX - 1, u64::MAX, initial, |_| Ok(target))
                .unwrap();
        assert_eq!(
            iterator.next(),
            Some(Err(PlannerError::MultipleStepsOnTick))
        );
        assert_exhausted(&mut iterator);
    }
}

#[test]
fn nth_and_count_consume_only_the_remaining_events() {
    let mut iterator = TrajectoryStepEventIterator::new(0, 5, steps(0, 0), |tick| {
        Ok(steps(i64::try_from(tick).unwrap(), 0))
    })
    .unwrap();
    assert_eq!(
        iterator.nth(1),
        Some(Ok((2, MotorStepMove::new(steps(1, 0), steps(2, 0)))))
    );
    assert_eq!(iterator.by_ref().count(), 3);
    assert_exhausted(&mut iterator);
}

#[test]
fn a_non_copy_fn_mut_sampler_works_through_generic_and_dynamic_dispatch() {
    fn next_generic<I: FusedIterator<Item = Result<(u64, MotorStepMove), PlannerError>>>(
        iterator: &mut I,
    ) -> I::Item {
        iterator.next().unwrap()
    }
    let mut queries = Vec::new();
    let mut iterator = TrajectoryStepEventIterator::new(0, 2, steps(0, 0), move |tick| {
        queries.push(tick);
        assert!(queries.len() <= 3);
        Ok(steps(i64::try_from(tick).unwrap(), 0))
    })
    .unwrap();
    assert_eq!(
        next_generic(&mut iterator),
        Ok((1, MotorStepMove::new(steps(0, 0), steps(1, 0))))
    );
    let dynamic: &mut dyn FusedIterator<Item = Result<(u64, MotorStepMove), PlannerError>> =
        &mut iterator;
    assert_eq!(
        dynamic.next(),
        Some(Ok((2, MotorStepMove::new(steps(1, 0), steps(2, 0)))))
    );
    assert_exhausted(dynamic);
}

#[test]
fn returning_to_the_initial_position_is_outside_the_binary_search_contract() {
    // An endpoint equal to the initial position terminates the search. This
    // deliberately violates the search precondition to document why a
    // returning trajectory must be split into compatible intervals first.
    let mut queries = Vec::new();
    let mut iterator = TrajectoryStepEventIterator::new(0, 2, steps(0, 0), |tick| {
        queries.push(tick);
        Ok(if tick == 1 { steps(1, 0) } else { steps(0, 0) })
    })
    .unwrap();
    assert_exhausted(&mut iterator);
    assert_eq!(queries, [2]);
}

fn trajectory<'a>(
    points: &'a [JointAngles],
    speeds: &'a [f64],
    velocity: &'a JointVelocityLimits,
    acceleration: &'a JointAccelerationLimits,
) -> CoordinatedJointTrajectory<'a> {
    let profiles =
        CoordinatedJointPathProfiles::new(points, speeds, velocity, acceleration).unwrap();
    CoordinatedJointTrajectory::new(TimedCoordinatedJointPathProfiles::new(profiles).unwrap())
        .unwrap()
}

#[test]
fn real_accelerated_profiles_match_tick_by_tick_sampling_across_segments() {
    let points =
        [[0.0, 0.0], [1.0, -0.5], [1.0, -1.5]].map(|[a, b]| JointAngles::new(a, b).unwrap());
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let trajectory = trajectory(&points, &[0.0; 3], &velocity, &acceleration);
    let converter = JointToStepsConverter::new(4.0, 4.0).unwrap();
    let timebase = StepTimeBase::new(16).unwrap();
    let position_at_tick =
        |tick| converter.convert(trajectory.position_at(timebase.ticks_to_seconds(tick))?);

    for (start, end) in [(0, 64), (8, 56), (32, 64)] {
        let initial = position_at_tick(start).unwrap();
        let mut expected = Vec::new();
        let mut previous = initial;
        for tick in start + 1..=end {
            let current = position_at_tick(tick).unwrap();
            if current != previous {
                expected.push((tick, MotorStepMove::new(previous, current)));
            }
            previous = current;
        }
        assert!(!expected.is_empty());
        let mut iterator =
            TrajectoryStepEventIterator::new(start, end, initial, position_at_tick).unwrap();
        let actual = iterator.by_ref().collect::<Result<Vec<_>, _>>().unwrap();
        assert_eq!(actual, expected);
        let mut reached = initial;
        for (_, movement) in actual {
            replay(&mut reached, movement);
        }
        assert_eq!(reached, position_at_tick(end).unwrap());
        assert_exhausted(&mut iterator);
    }
}

#[test]
fn the_accelerated_single_axis_example_emits_at_4_7_10_and_12() {
    let points = [[0.0, 0.0], [1.0, 0.0]].map(|[a, b]| JointAngles::new(a, b).unwrap());
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let trajectory = trajectory(&points, &[0.0; 2], &velocity, &acceleration);
    let converter = JointToStepsConverter::new(4.0, 4.0).unwrap();
    let timebase = StepTimeBase::new(8).unwrap();
    let iterator = TrajectoryStepEventIterator::new(0, 16, steps(0, 0), |tick| {
        converter.convert(trajectory.position_at(timebase.ticks_to_seconds(tick))?)
    })
    .unwrap();
    let actual = iterator.collect::<Result<Vec<_>, _>>().unwrap();
    let expected: Vec<_> = [4, 7, 10, 12]
        .into_iter()
        .enumerate()
        .map(|(i, tick)| {
            (
                tick,
                MotorStepMove::new(steps(i as i64, 0), steps(i as i64 + 1, 0)),
            )
        })
        .collect();
    assert_eq!(actual, expected);
}

#[test]
fn a_round_trip_uses_one_iterator_per_direction_and_returns_to_zero() {
    let points = [[0.0, 0.0], [1.0, 0.0], [0.0, 0.0]].map(|[a, b]| JointAngles::new(a, b).unwrap());
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let profiles =
        CoordinatedJointPathProfiles::new(&points, &[0.0; 3], &velocity, &acceleration).unwrap();
    let timed = TimedCoordinatedJointPathProfiles::new(profiles).unwrap();
    let end_times: Vec<_> = timed
        .map(|profile| profile.unwrap().end_time().unwrap())
        .collect();
    assert_eq!(end_times, [2.0, 4.0]);

    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();
    let converter = JointToStepsConverter::new(4.0, 4.0).unwrap();
    let timebase = StepTimeBase::new(8).unwrap();
    assert_eq!(timebase.seconds_to_ticks(4.0), Ok(32));
    let position_at_tick =
        |tick| converter.convert(trajectory.position_at(timebase.ticks_to_seconds(tick))?);
    assert_eq!(position_at_tick(0), Ok(steps(0, 0)));
    assert_eq!(position_at_tick(16), Ok(steps(4, 0)));
    assert_eq!(position_at_tick(32), Ok(steps(0, 0)));

    // round(3.5) = 4 and round(0.5) = 1: on the way back the
    // transitions occur after these ties, at ticks 21 and 29.
    assert_eq!(position_at_tick(20), Ok(steps(4, 0)));
    assert_eq!(position_at_tick(28), Ok(steps(1, 0)));

    let mut reached = steps(0, 0);
    let mut all_events = Vec::new();
    for (start, end, direction, expected_ticks, target) in [
        (0, 16, StepDirection::Positive, [4, 7, 10, 12], steps(4, 0)),
        (
            16,
            32,
            StepDirection::Negative,
            [21, 23, 26, 29],
            steps(0, 0),
        ),
    ] {
        assert_eq!(position_at_tick(start), Ok(reached));
        let mut iterator = TrajectoryStepEventIterator::new(start, end, reached, |tick| {
            assert!((start + 1..=end).contains(&tick));
            position_at_tick(tick)
        })
        .unwrap();
        let events = iterator.by_ref().collect::<Result<Vec<_>, _>>().unwrap();
        assert_exhausted(&mut iterator);
        assert_eq!(
            events.iter().map(|event| event.0).collect::<Vec<_>>(),
            expected_ticks
        );

        let mut linear_events = Vec::new();
        let mut previous = reached;
        for tick in start + 1..=end {
            let position = position_at_tick(tick).unwrap();
            if position != previous {
                linear_events.push((tick, MotorStepMove::new(previous, position)));
            }
            previous = position;
        }
        assert_eq!(events, linear_events);

        for &(tick, movement) in &events {
            assert_eq!(movement.first().pulses, 1);
            assert_eq!(movement.first().direction, direction);
            assert_eq!(movement.second().pulses, 0);
            replay(&mut reached, movement);
            assert_eq!(position_at_tick(tick), Ok(reached));
        }
        assert_eq!(reached, target);
        all_events.extend(events);
    }
    assert_eq!(all_events.len(), 8);
    assert!(all_events.windows(2).all(|pair| pair[0].0 < pair[1].0));
    assert_eq!(reached, steps(0, 0));
}

#[test]
fn the_forward_only_motor_sampler_is_incompatible_with_binary_search() {
    let points = [[0.0, 0.0], [1.0, 0.0]].map(|[a, b]| JointAngles::new(a, b).unwrap());
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let trajectory = trajectory(&points, &[0.0; 2], &velocity, &acceleration);
    let mut sampler = MotorPositionSampler::new(
        CoordinatedJointTrajectoryCursor::new(trajectory),
        JointToStepsConverter::new(4.0, 4.0).unwrap(),
        StepTimeBase::new(8).unwrap(),
    );
    let initial = sampler.sample_at_tick(0).unwrap();
    let mut iterator =
        TrajectoryStepEventIterator::new(0, 16, initial, |tick| sampler.sample_at_tick(tick))
            .unwrap();
    assert_eq!(iterator.next(), Some(Err(PlannerError::TimeWentBackwards)));
    assert_exhausted(&mut iterator);
}
