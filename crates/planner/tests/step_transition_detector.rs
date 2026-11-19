use kinematics::JointAngles;
use planner::{
    AxisStepMove, CoordinatedJointPathProfiles, CoordinatedJointTrajectory,
    CoordinatedJointTrajectoryCursor, JointAccelerationLimits, JointToStepsConverter,
    JointVelocityLimits, MotorPositionSampler, MotorSteps, PlannerError, StepDirection,
    StepTimeBase, StepTransitionDetector, TimedCoordinatedJointPathProfiles,
};

fn steps(first: i64, second: i64) -> MotorSteps {
    MotorSteps { first, second }
}

#[test]
fn stationary_positions_emit_no_move() {
    let initial = steps(-12, 23);
    let mut detector = StepTransitionDetector::new(5, initial);

    assert_eq!(detector.update(6, initial), Ok(None));
    assert_eq!(detector.update(8, initial), Ok(None));
    assert_eq!(detector.update(13, initial), Ok(None));
}

#[test]
fn reports_each_combination_of_single_steps_from_the_initial_position() {
    let initial = steps(-12, 23);
    let deltas = [
        (-1, StepDirection::Negative),
        (0, StepDirection::Positive),
        (1, StepDirection::Positive),
    ];

    for (first, first_direction) in deltas {
        for (second, second_direction) in deltas {
            let mut detector = StepTransitionDetector::new(5, initial);
            let movement = detector
                .update(6, steps(initial.first + first, initial.second + second))
                .unwrap();

            if first == 0 && second == 0 {
                assert_eq!(movement, None);
                continue;
            }

            let movement = movement.unwrap();
            assert_eq!(
                movement.first(),
                AxisStepMove {
                    direction: first_direction,
                    pulses: first.unsigned_abs(),
                }
            );
            assert_eq!(
                movement.second(),
                AxisStepMove {
                    direction: second_direction,
                    pulses: second.unsigned_abs(),
                }
            );
        }
    }
}

#[test]
fn consecutive_moves_use_the_last_accepted_position() {
    let mut detector = StepTransitionDetector::new(11, steps(7, -9));

    let first = detector.update(12, steps(8, -10)).unwrap().unwrap();
    assert_eq!(first.first().direction, StepDirection::Positive);
    assert_eq!(first.second().direction, StepDirection::Negative);
    assert_eq!(detector.update(13, steps(8, -10)), Ok(None));

    let second = detector.update(16, steps(9, -10)).unwrap().unwrap();
    assert_eq!(second.first().pulses, 1);
    assert_eq!(second.second().pulses, 0);

    let third = detector.update(17, steps(8, -9)).unwrap().unwrap();
    assert_eq!(third.first().direction, StepDirection::Negative);
    assert_eq!(third.first().pulses, 1);
    assert_eq!(third.second().direction, StepDirection::Positive);
    assert_eq!(third.second().pulses, 1);
}

#[test]
fn rejects_ticks_at_or_before_the_initial_tick_even_without_movement() {
    let initial = steps(4, -5);
    let mut detector = StepTransitionDetector::new(10, initial);
    for tick in [0, 9, 10] {
        assert_eq!(
            detector.update(tick, initial),
            Err(PlannerError::InvalidTicks)
        );
        assert_eq!(
            detector.update(tick, steps(40, -50)),
            Err(PlannerError::InvalidTicks)
        );
    }
    assert!(detector.update(11, steps(5, -5)).unwrap().is_some());
}

#[test]
fn rejects_repeated_and_earlier_ticks_after_a_move() {
    let mut detector = StepTransitionDetector::new(10, steps(4, -5));

    assert!(detector.update(15, steps(5, -5)).unwrap().is_some());
    assert_eq!(
        detector.update(14, steps(5, -4)),
        Err(PlannerError::InvalidTicks)
    );
    assert_eq!(
        detector.update(15, steps(5, -5)),
        Err(PlannerError::InvalidTicks)
    );
    let next = detector.update(16, steps(5, -4)).unwrap().unwrap();
    assert_eq!(next.first().pulses, 0);
    assert_eq!(next.second().pulses, 1);
}

#[test]
fn a_stationary_sample_rejects_a_later_update_at_an_earlier_tick() {
    let initial = steps(4, -5);
    let mut detector = StepTransitionDetector::new(10, initial);

    assert_eq!(detector.update(15, initial), Ok(None));
    assert_eq!(
        detector.update(14, steps(5, -5)),
        Err(PlannerError::InvalidTicks)
    );
}

#[test]
fn a_stationary_sample_rejects_a_later_update_at_the_same_tick() {
    let initial = steps(4, -5);
    let mut detector = StepTransitionDetector::new(10, initial);

    assert_eq!(detector.update(15, initial), Ok(None));
    assert_eq!(
        detector.update(15, initial),
        Err(PlannerError::InvalidTicks)
    );
}

#[test]
fn reports_multiple_steps_on_either_axis() {
    let origin = steps(-8, 10);
    for (tick, target) in [
        (1, steps(-6, 10)),
        (1, steps(-10, 11)),
        (1, steps(-8, 12)),
        (1, steps(-7, 8)),
        (1, steps(-10, 12)),
        (100, steps(-8, 8)),
    ] {
        let mut detector = StepTransitionDetector::new(0, origin);
        assert_eq!(
            detector.update(tick, target),
            Err(PlannerError::MultipleStepsOnTick),
            "tick={tick}, target={target:?}"
        );
    }
}

#[test]
fn a_rejected_jump_preserves_the_tick_and_both_axis_positions_for_retry() {
    let mut detector = StepTransitionDetector::new(0, steps(4, -5));

    assert!(detector.update(1, steps(5, -5)).unwrap().is_some());
    assert!(detector.update(2, steps(6, -7)).is_err());
    assert_eq!(
        detector.update(1, steps(5, -5)),
        Err(PlannerError::InvalidTicks)
    );

    let actual = detector.update(2, steps(6, -6)).unwrap().unwrap();
    assert_eq!(actual.first().pulses, 1);
    assert_eq!(actual.first().direction, StepDirection::Positive);
    assert_eq!(actual.second().pulses, 1);
    assert_eq!(actual.second().direction, StepDirection::Negative);
}

#[test]
fn supports_adjacent_extreme_positions_at_the_last_representable_tick() {
    let origin = steps(i64::MIN, i64::MAX);
    let adjacent = steps(i64::MIN + 1, i64::MAX - 1);
    let mut detector = StepTransitionDetector::new(u64::MAX - 2, origin);

    let outward = detector.update(u64::MAX - 1, adjacent).unwrap().unwrap();
    assert_eq!(outward.first().direction, StepDirection::Positive);
    assert_eq!(outward.first().pulses, 1);
    assert_eq!(outward.second().direction, StepDirection::Negative);
    assert_eq!(outward.second().pulses, 1);

    let inward = detector.update(u64::MAX, origin).unwrap().unwrap();
    assert_eq!(inward.first().direction, StepDirection::Negative);
    assert_eq!(inward.first().pulses, 1);
    assert_eq!(inward.second().direction, StepDirection::Positive);
    assert_eq!(inward.second().pulses, 1);
    assert_eq!(
        detector.update(u64::MAX, adjacent),
        Err(PlannerError::InvalidTicks)
    );
}

#[test]
fn handles_absolute_differences_that_do_not_fit_in_i64() {
    let origin = steps(i64::MIN, i64::MAX);
    let mut detector = StepTransitionDetector::new(0, origin);

    assert!(detector.update(1, steps(i64::MAX, i64::MIN)).is_err());
    assert_eq!(detector.update(1, origin), Ok(None));
}

#[test]
fn detects_steps_from_sampled_motor_positions() {
    let points = [
        JointAngles::new(0.0, 0.0).unwrap(),
        JointAngles::new(1.0, -0.5).unwrap(),
    ];
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let profiles =
        CoordinatedJointPathProfiles::new(&points, &[0.0; 2], &velocity, &acceleration).unwrap();
    let timed = TimedCoordinatedJointPathProfiles::new(profiles).unwrap();
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();
    let converter = JointToStepsConverter::new(4.0, 8.0).unwrap();
    let mut sampler = MotorPositionSampler::new(
        CoordinatedJointTrajectoryCursor::new(trajectory),
        converter,
        StepTimeBase::new(8).unwrap(),
    );
    let initial = sampler.sample_at_tick(0).unwrap();
    let mut detector = StepTransitionDetector::new(0, initial);

    // With unit progress acceleration, the step magnitude is tick²/32 in
    // the first half and 4 - (16 - tick)²/32 in the second half, before rounding.
    let mut emitted_at = Vec::new();
    for tick in 1..=16 {
        let position = sampler.sample_at_tick(tick).unwrap();
        if let Some(movement) = detector.update(tick, position).unwrap() {
            emitted_at.push(tick);
            assert_eq!(movement.first().direction, StepDirection::Positive);
            assert_eq!(movement.first().pulses, 1);
            assert_eq!(movement.second().direction, StepDirection::Negative);
            assert_eq!(movement.second().pulses, 1);
        }
    }
    assert_eq!(emitted_at, [4, 7, 10, 12]);
}
