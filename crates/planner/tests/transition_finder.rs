use kinematics::JointAngles;
use planner::{
    AxisStepMove, CoordinatedJointPathProfiles, CoordinatedJointTrajectory,
    JointAccelerationLimits, JointToStepsConverter, JointVelocityLimits, MotorStepMove, MotorSteps,
    PlannerError, StepDirection, StepTimeBase, TimedCoordinatedJointPathProfiles,
    find_first_transition_tick,
};

fn steps(first: i64, second: i64) -> MotorSteps {
    MotorSteps { first, second }
}

#[test]
fn rejects_a_reversed_interval_without_sampling() {
    assert_eq!(
        find_first_transition_tick(u64::MAX, 0, steps(0, 0), |_| unreachable!()),
        Err(PlannerError::EndBeforeStart)
    );
}

#[test]
fn an_empty_interval_needs_no_samples() {
    for tick in [0, 42, u64::MAX] {
        assert_eq!(
            find_first_transition_tick(tick, tick, steps(8, -4), |_| unreachable!()),
            Ok(None)
        );
    }
}

#[test]
fn a_stationary_interval_only_samples_its_end() {
    let initial = steps(-2, 7);
    let mut sampled = Vec::new();
    let actual = find_first_transition_tick(10, u64::MAX, initial, |tick| {
        sampled.push(tick);
        Ok(initial)
    });

    assert_eq!(actual, Ok(None));
    assert_eq!(sampled, [u64::MAX]);
}

#[test]
fn finds_every_possible_first_change_in_short_intervals() {
    let start = 41;
    let initial = steps(7, -5);
    let first_change = steps(7, -4);
    let later_position = steps(6, -5);

    for width in 1..=40 {
        let end = start + width;
        for transition in start + 1..=end {
            let actual = find_first_transition_tick(start, end, initial, |tick| {
                assert!((start + 1..=end).contains(&tick));
                Ok(match tick.cmp(&transition) {
                    core::cmp::Ordering::Less => initial,
                    core::cmp::Ordering::Equal => first_change,
                    core::cmp::Ordering::Greater => later_position,
                })
            });
            assert_eq!(
                actual,
                Ok(Some((
                    transition,
                    MotorStepMove::new(initial, first_change)
                ))),
                "start={start}, end={end}, transition={transition}"
            );
        }
    }
}

#[test]
fn reports_signed_displacements_on_both_axes_at_the_first_change() {
    let initial = steps(i64::MAX, i64::MIN);
    let target = steps(i64::MIN, i64::MAX);
    let (tick, movement) = find_first_transition_tick(u64::MAX - 1, u64::MAX, initial, |tick| {
        assert_eq!(tick, u64::MAX);
        Ok(target)
    })
    .unwrap()
    .unwrap();

    assert_eq!(tick, u64::MAX);
    assert_eq!(
        movement.first(),
        AxisStepMove {
            direction: StepDirection::Negative,
            pulses: u64::MAX,
        }
    );
    assert_eq!(
        movement.second(),
        AxisStepMove {
            direction: StepDirection::Positive,
            pulses: u64::MAX,
        }
    );
}

#[test]
fn searches_the_full_tick_range_with_at_most_65_distinct_samples() {
    let initial = steps(3, 0);
    let target = steps(4, 0);
    for threshold in [1, u64::MAX / 2, u64::MAX / 2 + 1, u64::MAX - 1, u64::MAX] {
        let mut sampled = Vec::new();
        let actual = find_first_transition_tick(0, u64::MAX, initial, |tick| {
            assert_ne!(tick, 0);
            assert!(!sampled.contains(&tick), "sampled tick {tick} twice");
            assert!(sampled.len() < 65);
            sampled.push(tick);
            Ok(if tick < threshold { initial } else { target })
        });
        assert_eq!(
            actual,
            Ok(Some((threshold, MotorStepMove::new(initial, target))))
        );
    }
}

#[test]
fn propagates_sampling_errors_at_the_end_and_at_a_midpoint() {
    let initial = steps(0, 0);
    let mut sampled = Vec::new();
    let at_end = find_first_transition_tick(4, 12, initial, |tick| {
        sampled.push(tick);
        Err(PlannerError::InvalidTime)
    });
    assert_eq!(at_end, Err(PlannerError::InvalidTime));
    assert_eq!(sampled, [12]);

    sampled.clear();
    let at_midpoint = find_first_transition_tick(4, 12, initial, |tick| {
        sampled.push(tick);
        if tick == 12 {
            Ok(steps(1, 0))
        } else {
            Err(PlannerError::NumericalFailure)
        }
    });
    assert_eq!(at_midpoint, Err(PlannerError::NumericalFailure));
    assert_eq!(sampled, [12, 8]);
}

#[test]
fn repeated_searches_match_the_accelerated_trajectory_schedule() -> Result<(), PlannerError> {
    let points = [
        JointAngles::new(0.0, 0.0).map_err(PlannerError::Kinematics)?,
        JointAngles::new(1.0, 0.0).map_err(PlannerError::Kinematics)?,
    ];
    let velocity = JointVelocityLimits::new(10.0, 10.0)?;
    let acceleration = JointAccelerationLimits::new(1.0, 1.0)?;
    let profiles = CoordinatedJointPathProfiles::new(&points, &[0.0; 2], &velocity, &acceleration)?;
    let timed = TimedCoordinatedJointPathProfiles::new(profiles)?;
    let trajectory = CoordinatedJointTrajectory::new(timed)?;
    let timebase = StepTimeBase::new(8)?;
    let converter = JointToStepsConverter::new(4.0, 4.0)?;
    let position_at_tick = |tick| {
        let angles = trajectory.position_at(timebase.ticks_to_seconds(tick))?;
        converter.convert(angles)
    };

    let mut start = 0;
    let mut position = position_at_tick(start)?;
    assert_eq!(position, steps(0, 0));
    for (expected_tick, next_position) in [(4, 1), (7, 2), (10, 3), (12, 4)] {
        let (tick, movement) =
            find_first_transition_tick(start, 16, position, position_at_tick)?.unwrap();
        let expected_position = steps(next_position, 0);
        assert_eq!(tick, expected_tick);
        assert_eq!(movement, MotorStepMove::new(position, expected_position));
        start = tick;
        position = expected_position;
    }
    assert_eq!(position_at_tick(16)?, position);
    assert_eq!(
        find_first_transition_tick(start, 16, position, position_at_tick)?,
        None
    );
    Ok(())
}
