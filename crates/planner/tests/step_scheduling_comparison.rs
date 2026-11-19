use kinematics::JointAngles;
use planner::{
    CoordinatedJointPathProfiles, CoordinatedJointTrajectory, CoordinatedJointTrajectoryCursor,
    JointAccelerationLimits, JointToStepsConverter, JointVelocityLimits, MotorPositionSampler,
    MotorStepInterval, MotorSteps, PlannerError, StepDirection, StepTimeBase,
    StepTransitionDetector, TimedCoordinatedJointPathProfiles, TimedStepInterpolator,
};

const END_TICK: u64 = 16;
const START: MotorSteps = MotorSteps {
    first: 0,
    second: 0,
};
const TARGET: MotorSteps = MotorSteps {
    first: 4,
    second: 0,
};

// Tick and signed pulse counts for the two motors.
type ScheduledPulses = (u64, [i64; 2]);

fn signed_pulses(pulses: u64, direction: StepDirection) -> i64 {
    let pulses = i64::try_from(pulses).unwrap();
    match direction {
        StepDirection::Positive => pulses,
        StepDirection::Negative => -pulses,
    }
}

fn uniform_schedule() -> Result<Vec<ScheduledPulses>, PlannerError> {
    let interval = MotorStepInterval::new(0, END_TICK, START, TARGET)?;
    let movement = interval.movement();
    let events = TimedStepInterpolator::new(interval)?;
    Ok(events
        .map(|event| {
            let first = signed_pulses(
                u64::from(event.event.first_step),
                movement.first().direction,
            );
            let second = signed_pulses(
                u64::from(event.event.second_step),
                movement.second().direction,
            );
            (event.tick, [first, second])
        })
        .collect())
}

fn accelerated_schedule() -> Result<(Vec<ScheduledPulses>, MotorSteps), PlannerError> {
    let points = [
        JointAngles::new(0.0, 0.0).map_err(PlannerError::Kinematics)?,
        JointAngles::new(1.0, 0.0).map_err(PlannerError::Kinematics)?,
    ];
    let velocity = JointVelocityLimits::new(10.0, 10.0)?;
    let acceleration = JointAccelerationLimits::new(1.0, 1.0)?;
    let profiles = CoordinatedJointPathProfiles::new(&points, &[0.0; 2], &velocity, &acceleration)?;
    let timed = TimedCoordinatedJointPathProfiles::new(profiles)?;
    let trajectory = CoordinatedJointTrajectory::new(timed)?;
    let cursor = CoordinatedJointTrajectoryCursor::new(trajectory);
    let timebase = StepTimeBase::new(8)?;
    assert_eq!(cursor.end_time(), 2.0);
    assert_eq!(timebase.ticks_to_seconds(END_TICK), 2.0);

    let converter = JointToStepsConverter::new(4.0, 4.0)?;
    let mut sampler = MotorPositionSampler::new(cursor, converter, timebase);
    let initial_position = sampler.sample_at_tick(0)?;
    assert_eq!(initial_position, START);
    let mut detector = StepTransitionDetector::new(0, initial_position);
    let mut position = initial_position;
    let mut schedule = Vec::new();
    for tick in 1..=END_TICK {
        position = sampler.sample_at_tick(tick)?;
        let movement = detector.update(tick, position)?;
        if let Some(movement) = movement {
            let first = movement.first();
            let second = movement.second();
            schedule.push((
                tick,
                [
                    signed_pulses(first.pulses, first.direction),
                    signed_pulses(second.pulses, second.direction),
                ],
            ));
        }
    }
    Ok((schedule, position))
}

fn scheduled_ticks(schedule: &[ScheduledPulses]) -> Vec<u64> {
    schedule.iter().map(|&(tick, _)| tick).collect()
}

#[test]
fn uniform_interpolation_emits_at_equal_tick_intervals() -> Result<(), PlannerError> {
    assert_eq!(scheduled_ticks(&uniform_schedule()?), [4, 8, 12, 16]);
    Ok(())
}

#[test]
fn sampling_the_accelerated_trajectory_emits_at_position_transitions() -> Result<(), PlannerError> {
    let (schedule, _) = accelerated_schedule()?;
    assert_eq!(scheduled_ticks(&schedule), [4, 7, 10, 12]);
    Ok(())
}

#[test]
fn both_finish_at_the_same_position_with_distinct_schedules() -> Result<(), PlannerError> {
    let uniform = uniform_schedule()?;
    let (accelerated, sampled_final_position) = accelerated_schedule()?;
    assert_ne!(scheduled_ticks(&uniform), scheduled_ticks(&accelerated));
    assert_eq!(sampled_final_position, TARGET);

    for schedule in [&uniform, &accelerated] {
        assert!(schedule.windows(2).all(|pair| pair[0].0 < pair[1].0));

        let mut pulse_counts = [0, 0];
        let mut reached = START;
        for &(tick, [first, second]) in schedule {
            assert!((1..=END_TICK).contains(&tick));
            assert!(first.unsigned_abs() <= 1, "first motor at tick {tick}");
            assert!(second.unsigned_abs() <= 1, "second motor at tick {tick}");
            pulse_counts[0] += first.unsigned_abs();
            pulse_counts[1] += second.unsigned_abs();
            reached.first += first;
            reached.second += second;
        }
        assert_eq!(pulse_counts, [4, 0]);
        assert_eq!(reached, sampled_final_position);
    }
    Ok(())
}
