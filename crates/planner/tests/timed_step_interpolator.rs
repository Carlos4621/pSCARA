use core::iter::FusedIterator;

use kinematics::JointAngles;
use planner::{
    CoordinatedJointPathProfiles, CoordinatedJointTrajectory, CoordinatedJointTrajectoryCursor,
    JointAccelerationLimits, JointToStepsConverter, JointVelocityLimits, MotorPositionSampler,
    MotorStepInterval, MotorSteps, PlannerError, StepDirection, StepEvent, StepTimeBase,
    TimedCoordinatedJointPathProfiles, TimedStepEvent, TimedStepInterpolator,
};

fn steps(first: i64, second: i64) -> MotorSteps {
    MotorSteps { first, second }
}

fn event(tick: u64, first_step: bool, second_step: bool) -> TimedStepEvent {
    TimedStepEvent {
        tick,
        event: StepEvent {
            first_step,
            second_step,
        },
    }
}

fn interval(start: u64, end: u64, first: u64, second: u64) -> MotorStepInterval {
    let target_for = |count| i64::try_from(i128::from(i64::MIN) + i128::from(count)).unwrap();
    MotorStepInterval::new(
        start,
        end,
        steps(i64::MIN, i64::MIN),
        steps(target_for(first), target_for(second)),
    )
    .unwrap()
}

fn assert_exhausted(iter: &mut impl FusedIterator<Item = TimedStepEvent>) {
    assert!(iter.next().is_none());
    assert!(iter.next().is_none());
}

#[test]
fn stationary_movement_is_empty_even_at_the_last_tick() {
    let at_rest = steps(-34, 81);
    let interval = MotorStepInterval::new(u64::MAX - 1, u64::MAX, at_rest, at_rest).unwrap();
    let mut iter = TimedStepInterpolator::new(interval).unwrap();
    assert_exhausted(&mut iter);
}

#[test]
fn duration_must_be_strictly_greater_than_the_event_count() {
    for (duration, first, second) in [
        (1, 1, 0),
        (1, 0, 1),
        (4, 4, 4),
        (4, 5, 2),
        (4, 2, 5),
        (u64::MAX, u64::MAX, 1),
    ] {
        assert_eq!(
            TimedStepInterpolator::new(interval(0, duration, first, second)).unwrap_err(),
            PlannerError::InsufficientTicksForSteps,
            "duration={duration}, pulses=({first}, {second})"
        );
    }

    let mut iter = TimedStepInterpolator::new(interval(u64::MAX - 6, u64::MAX, 5, 0)).unwrap();
    for offset in 2..=6 {
        assert_eq!(
            iter.next(),
            Some(event((u64::MAX - 6) + offset, true, false))
        );
    }
    assert_exhausted(&mut iter);
}

#[test]
fn a_single_event_is_scheduled_at_the_interval_end() {
    for (first, second, expected) in [
        (1, 0, event(13, true, false)),
        (0, 1, event(13, false, true)),
        (1, 1, event(13, true, true)),
    ] {
        let mut iter = TimedStepInterpolator::new(interval(10, 13, first, second)).unwrap();
        assert_eq!(iter.next(), Some(expected));
        assert_exhausted(&mut iter);
    }
}

#[test]
fn a_five_event_move_preserves_pulse_flags_and_rounds_ticks_up() {
    for (first, second) in [(5, 2), (2, 5)] {
        let mut iter = TimedStepInterpolator::new(interval(100, 112, first, second)).unwrap();
        for (tick, both) in [
            (103, false),
            (105, false),
            (108, true),
            (110, false),
            (112, true),
        ] {
            assert_eq!(
                iter.next(),
                Some(event(tick, first == 5 || both, second == 5 || both))
            );
        }
        assert_exhausted(&mut iter);
    }
}

#[test]
fn a_grid_of_schedules_has_ordered_ticks_and_matches_independent_integer_bounds() {
    let start = (1_u64 << 53) + 1;
    for duration in 1..=24 {
        for first in 0..=12_u64 {
            for second in 0..=12_u64 {
                let count = first.max(second);
                let outcome =
                    TimedStepInterpolator::new(interval(start, start + duration, first, second));
                if count >= duration {
                    assert_eq!(
                        outcome.unwrap_err(),
                        PlannerError::InsufficientTicksForSteps
                    );
                    continue;
                }

                let mut iter = outcome.unwrap();
                let mut previous_tick = start;
                let mut observed = [0_u64; 2];
                for index in 1..=count {
                    let sample = iter.next().expect("one event per primary pulse");
                    assert!(sample.tick > previous_tick && sample.tick <= start + duration);
                    let offset = sample.tick - start;
                    // The chosen tick is the first one at or after index*duration/count.
                    assert!((offset - 1) * count < index * duration);
                    assert!(index * duration <= offset * count);
                    observed[0] += u64::from(sample.event.first_step);
                    observed[1] += u64::from(sample.event.second_step);
                    assert_eq!(observed, [index * first / count, index * second / count]);
                    previous_tick = sample.tick;
                }
                assert_eq!(observed, [first, second]);
                if count > 0 {
                    assert_eq!(previous_tick, start + duration);
                }
                assert_exhausted(&mut iter);
            }
        }
    }
}

#[test]
fn taking_and_skipping_events_keeps_both_the_schedule_and_pulse_phase() {
    let mut iter = TimedStepInterpolator::new(interval(10, 23, 5, 2)).unwrap();
    assert_eq!(
        iter.by_ref().take(1).collect::<Vec<_>>(),
        [event(13, true, false)]
    );
    let erased: &mut dyn FusedIterator<Item = TimedStepEvent> = &mut iter;
    assert_eq!(erased.nth(1), Some(event(18, true, true)));
    assert_eq!(erased.next(), Some(event(21, true, false)));
    assert_eq!(erased.next(), Some(event(23, true, true)));
    assert_exhausted(&mut iter);
}

#[test]
fn a_maximum_duration_uses_wide_products_and_can_end_at_u64_max() {
    let mut iter = TimedStepInterpolator::new(interval(0, u64::MAX, 4, 3)).unwrap();
    let quarter = 1_u64 << 62;
    for expected in [
        event(quarter, true, false),
        event(2 * quarter, true, true),
        event(3 * quarter, true, true),
        event(u64::MAX, true, true),
    ] {
        assert_eq!(iter.next(), Some(expected));
    }
    assert_exhausted(&mut iter);
}

#[test]
fn almost_u64_max_events_can_be_sampled_lazily() {
    let count = u64::MAX - 1;
    let mut iter = TimedStepInterpolator::new(interval(0, u64::MAX, count - 1, count)).unwrap();
    for index in 1..=64 {
        assert_eq!(iter.next(), Some(event(index + 1, index > 1, true)));
    }
}

#[test]
fn consecutive_intervals_from_a_trajectory_reach_their_targets_without_duplicate_ticks() {
    let points = [[0.0, 0.0], [1.0, -0.5], [1.0, -1.5]]
        .map(|[first, second]| JointAngles::new(first, second).unwrap());
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let profiles =
        CoordinatedJointPathProfiles::new(&points, &[0.0; 3], &velocity, &acceleration).unwrap();
    let timed = TimedCoordinatedJointPathProfiles::new(profiles).unwrap();
    let cursor =
        CoordinatedJointTrajectoryCursor::new(CoordinatedJointTrajectory::new(timed).unwrap());
    let converter = JointToStepsConverter::new(8.0, 16.0).unwrap();
    let mut sampler = MotorPositionSampler::new(cursor, converter, StepTimeBase::new(16).unwrap());

    let mut previous_tick = 0;
    let mut reached = sampler.sample_at_tick(0).unwrap();
    for (end, expected) in [
        (16, steps(4, -4)),
        (32, steps(8, -8)),
        (48, steps(8, -16)),
        (64, steps(8, -24)),
    ] {
        let target = sampler.sample_at_tick(end).unwrap();
        assert_eq!(target, expected);
        let interval = MotorStepInterval::new(previous_tick, end, reached, target).unwrap();
        let movement = interval.movement();
        let signed_pulse = |direction| {
            if direction == StepDirection::Positive {
                1
            } else {
                -1
            }
        };
        for sample in TimedStepInterpolator::new(interval).unwrap() {
            assert!(sample.tick > previous_tick && sample.tick <= end);
            if sample.event.first_step {
                reached.first += signed_pulse(movement.first().direction);
            }
            if sample.event.second_step {
                reached.second += signed_pulse(movement.second().direction);
            }
            previous_tick = sample.tick;
        }
        assert_eq!(previous_tick, end);
        assert_eq!(reached, expected);
    }
}
