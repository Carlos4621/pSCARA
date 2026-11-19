use kinematics::JointAngles;
use planner::{
    AxisStepMove, CoordinatedJointPathProfiles, CoordinatedJointTrajectory,
    CoordinatedJointTrajectoryCursor, CoordinatedStepInterpolator, JointAccelerationLimits,
    JointToStepsConverter, JointVelocityLimits, MotorPositionSampler, MotorStepInterval,
    MotorSteps, PlannerError, StepDirection, StepEvent, StepTimeBase,
    TimedCoordinatedJointPathProfiles,
};

use StepDirection::{Negative, Positive};

fn steps(first: i64, second: i64) -> MotorSteps {
    MotorSteps { first, second }
}

fn axis(direction: StepDirection, pulses: u64) -> AxisStepMove {
    AxisStepMove { direction, pulses }
}

#[test]
fn keeps_both_timestamps_and_each_axis_movement() {
    let interval = MotorStepInterval::new(120, 170, steps(-12, 40), steps(5, -3)).unwrap();

    assert_eq!(interval.start_tick(), 120);
    assert_eq!(interval.end_tick(), 170);
    assert_eq!(interval.duration_ticks(), 50);
    assert_eq!(interval.movement().first(), axis(Positive, 17));
    assert_eq!(interval.movement().second(), axis(Negative, 43));
}

#[test]
fn rejects_equal_ticks_with_or_without_movement() {
    let origin = steps(-4, 7);
    for tick in [0, 1, 1_u64 << 53, 1_u64 << 63, u64::MAX] {
        for target in [origin, steps(3, -9)] {
            assert_eq!(
                MotorStepInterval::new(tick, tick, origin, target).unwrap_err(),
                PlannerError::EndBeforeStart,
                "tick={tick}, target={target:?}"
            );
        }
    }
}

#[test]
fn rejects_a_tick_before_the_start_across_the_unsigned_range() {
    for (start, end) in [
        (1, 0),
        (20, 19),
        (1_u64 << 63, (1_u64 << 63) - 1),
        (u64::MAX, u64::MAX - 1),
        (u64::MAX, 0),
    ] {
        assert_eq!(
            MotorStepInterval::new(start, end, steps(5, -7), steps(6, -7)).unwrap_err(),
            PlannerError::EndBeforeStart,
            "start={start}, end={end}"
        );
    }
}

#[test]
fn duration_is_exact_from_one_tick_to_the_full_u64_range() {
    for (start, end, duration) in [
        (0, 1, 1),
        (1_u64 << 53, (1_u64 << 53) + 1, 1),
        ((1_u64 << 63) - 1, 1_u64 << 63, 1),
        (u64::MAX - 1, u64::MAX, 1),
        (u64::MAX - 17, u64::MAX, 17),
        (0, u64::MAX, u64::MAX),
    ] {
        let interval = MotorStepInterval::new(start, end, steps(0, 0), steps(1, -1)).unwrap();
        assert_eq!(interval.start_tick(), start);
        assert_eq!(interval.end_tick(), end);
        assert_eq!(interval.duration_ticks(), duration);
    }
}

#[test]
fn a_stationary_interval_keeps_its_duration_without_any_pulses() {
    let at_rest = steps(-42, 100);
    let interval = MotorStepInterval::new(16, 64, at_rest, at_rest).unwrap();
    assert_eq!(interval.duration_ticks(), 48);
    let movement = interval.movement();
    assert_eq!(movement.first(), axis(Positive, 0));
    assert_eq!(movement.second(), axis(Positive, 0));
    let mut events = CoordinatedStepInterpolator::new(movement);
    assert!(events.next().is_none());
    assert!(events.next().is_none());
}

#[test]
fn full_signed_step_displacements_keep_their_directions_and_magnitudes() {
    let interval = MotorStepInterval::new(
        0,
        u64::MAX,
        steps(i64::MIN, i64::MAX),
        steps(i64::MAX, i64::MIN),
    )
    .unwrap();
    assert_eq!(interval.duration_ticks(), u64::MAX);
    let movement = interval.movement();
    assert_eq!(movement.first(), axis(Positive, u64::MAX));
    assert_eq!(movement.second(), axis(Negative, u64::MAX));
    let mut events = CoordinatedStepInterpolator::new(movement);
    for _ in 0..2 {
        assert_eq!(
            events.next(),
            Some(StepEvent {
                first_step: true,
                second_step: true
            })
        );
    }
}

#[test]
fn consecutive_motor_samples_create_intervals_whose_pulses_reach_each_target() {
    let points = [[0.0, 0.0], [1.0, -0.5], [1.0, -1.5]]
        .map(|[first, second]| JointAngles::new(first, second).unwrap());
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let profiles =
        CoordinatedJointPathProfiles::new(&points, &[0.0; 3], &velocity, &acceleration).unwrap();
    let timed = TimedCoordinatedJointPathProfiles::new(profiles).unwrap();
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();
    let cursor = CoordinatedJointTrajectoryCursor::new(trajectory);
    let converter = JointToStepsConverter::new(8.0, 16.0).unwrap();
    let mut sampler = MotorPositionSampler::new(cursor, converter, StepTimeBase::new(8).unwrap());

    let mut start = 0;
    let mut reached = sampler.sample_at_tick(start).unwrap();
    assert_eq!(reached, steps(0, 0));
    for (end, expected, event_count) in [
        (8, steps(4, -4), 4),
        (16, steps(8, -8), 4),
        (24, steps(8, -16), 8),
        (32, steps(8, -24), 8),
    ] {
        let target = sampler.sample_at_tick(end).unwrap();
        assert_eq!(target, expected);
        let interval = MotorStepInterval::new(start, end, reached, target).unwrap();
        assert_eq!(interval.duration_ticks(), 8);

        let movement = interval.movement();
        let signed_pulse = |direction| if direction == Positive { 1 } else { -1 };
        let mut observed_events = 0;
        for event in CoordinatedStepInterpolator::new(movement) {
            observed_events += 1;
            if event.first_step {
                reached.first += signed_pulse(movement.first().direction);
            }
            if event.second_step {
                reached.second += signed_pulse(movement.second().direction);
            }
        }
        assert_eq!(observed_events, event_count);
        assert_eq!(reached, expected);
        start = end;
    }
}
