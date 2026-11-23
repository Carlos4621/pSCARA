use planner::{
    MotorAxis::{self, First, Second},
    PinOperation::{self, SetDirection, StepHigh, StepLow},
    PlannerError, ScheduledDirChange, ScheduledStepDirEvent,
    StepDirection::{Negative, Positive},
    TimedPinOperation, expand_step_dir_event,
};

fn operation(tick: u64, motor: MotorAxis, operation: PinOperation) -> Option<TimedPinOperation> {
    Some(TimedPinOperation {
        tick,
        motor,
        operation,
    })
}

fn step_event(first_step: bool, second_step: bool) -> ScheduledStepDirEvent {
    ScheduledStepDirEvent {
        step_tick: 100,
        first_dir_change: None,
        second_dir_change: None,
        first_step,
        second_step,
    }
}

#[test]
fn puts_the_two_edges_of_each_pulse_in_its_motors_slots() {
    let first_high = operation(100, First, StepHigh);
    let first_low = operation(102, First, StepLow);
    let second_high = operation(100, Second, StepHigh);
    let second_low = operation(102, Second, StepLow);

    for (event, expected) in [
        (
            step_event(true, false),
            [None, None, first_high, None, first_low, None],
        ),
        (
            step_event(false, true),
            [None, None, None, second_high, None, second_low],
        ),
        (
            step_event(true, true),
            [None, None, first_high, second_high, first_low, second_low],
        ),
    ] {
        assert_eq!(expand_step_dir_event(event, 2), Ok(expected));
    }
}

#[test]
fn adds_direction_operations_at_their_own_ticks() {
    let first_dir_change = Some(ScheduledDirChange {
        tick: 97,
        direction: Positive,
    });
    let second_dir_change = Some(ScheduledDirChange {
        tick: 98,
        direction: Negative,
    });

    let first_only = ScheduledStepDirEvent {
        first_dir_change,
        ..step_event(true, false)
    };
    assert_eq!(
        expand_step_dir_event(first_only, 2),
        Ok([
            operation(97, First, SetDirection(Positive)),
            None,
            operation(100, First, StepHigh),
            None,
            operation(102, First, StepLow),
            None,
        ])
    );

    let both = ScheduledStepDirEvent {
        first_dir_change,
        second_dir_change,
        ..step_event(true, true)
    };
    assert_eq!(
        expand_step_dir_event(both, 2),
        Ok([
            operation(97, First, SetDirection(Positive)),
            operation(98, Second, SetDirection(Negative)),
            operation(100, First, StepHigh),
            operation(100, Second, StepHigh),
            operation(102, First, StepLow),
            operation(102, Second, StepLow),
        ])
    );
}

#[test]
fn an_idle_event_at_the_last_tick_has_no_operations() {
    let event = ScheduledStepDirEvent {
        step_tick: u64::MAX,
        ..step_event(false, false)
    };

    assert_eq!(expand_step_dir_event(event, 2), Ok([None; 6]));
}

#[test]
fn rejects_zero_high_duration() {
    for event in [step_event(true, false), step_event(false, false)] {
        assert_eq!(
            expand_step_dir_event(event, 0),
            Err(PlannerError::InvalidStepDirTiming)
        );
    }
}

#[test]
fn accepts_a_falling_edge_at_u64_max_and_rejects_overflow() {
    let max = u64::MAX;
    let event = ScheduledStepDirEvent {
        step_tick: max - 2,
        ..step_event(false, true)
    };

    assert_eq!(
        expand_step_dir_event(event, 2),
        Ok([
            None,
            None,
            None,
            operation(max - 2, Second, StepHigh),
            None,
            operation(max, Second, StepLow),
        ])
    );
    assert_eq!(
        expand_step_dir_event(event, 3),
        Err(PlannerError::NumericalFailure)
    );
}
