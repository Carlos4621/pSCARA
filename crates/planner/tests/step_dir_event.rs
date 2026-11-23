use planner::{
    MotorStepMove, MotorSteps, PlannerError, StepDirEvent,
    StepDirection::{Negative, Positive},
};

fn steps(first: i64, second: i64) -> MotorSteps {
    MotorSteps { first, second }
}

#[test]
fn only_the_moving_motor_has_a_direction_for_a_single_step() {
    let origin = steps(10, 5);
    for (target, first_direction, second_direction) in [
        (steps(11, 5), Some(Positive), None),
        (steps(9, 5), Some(Negative), None),
        (steps(10, 6), None, Some(Positive)),
        (steps(10, 4), None, Some(Negative)),
    ] {
        assert_eq!(
            StepDirEvent::new(u64::MAX, MotorStepMove::new(origin, target)),
            Ok(StepDirEvent {
                tick: u64::MAX,
                first_direction,
                second_direction,
            })
        );
    }
}

#[test]
fn both_motors_can_step_in_any_direction_on_the_same_tick() {
    let origin = steps(10, 5);
    for (target, first_direction, second_direction) in [
        (steps(11, 4), Positive, Negative),
        (steps(9, 6), Negative, Positive),
        (steps(11, 6), Positive, Positive),
        (steps(9, 4), Negative, Negative),
    ] {
        assert_eq!(
            StepDirEvent::new(150, MotorStepMove::new(origin, target)),
            Ok(StepDirEvent {
                tick: 150,
                first_direction: Some(first_direction),
                second_direction: Some(second_direction),
            })
        );
    }
}

#[test]
fn a_stationary_movement_has_no_direction_on_either_motor() {
    let position = steps(i64::MIN, i64::MAX);
    assert_eq!(
        StepDirEvent::new(0, MotorStepMove::new(position, position)),
        Ok(StepDirEvent {
            tick: 0,
            first_direction: None,
            second_direction: None,
        })
    );
}

#[test]
fn more_than_one_pulse_on_either_motor_rejects_the_event() {
    for (origin, target) in [
        (steps(10, 5), steps(12, 5)),
        (steps(10, 5), steps(8, 6)),
        (steps(10, 5), steps(10, 7)),
        (steps(10, 5), steps(11, 3)),
        (steps(10, 5), steps(12, 3)),
        (steps(i64::MIN, 0), steps(i64::MAX, 0)),
        (steps(0, i64::MAX), steps(0, i64::MIN)),
    ] {
        assert_eq!(
            StepDirEvent::new(150, MotorStepMove::new(origin, target)),
            Err(PlannerError::MultipleStepsOnTick)
        );
    }
}
