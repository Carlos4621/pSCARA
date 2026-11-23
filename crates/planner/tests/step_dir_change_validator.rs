use planner::{
    PlannerError, StepDirEvent, StepDirTiming, StepDirection,
    StepDirection::{Negative, Positive},
    StepTimeBase, validate_step_dir_changes,
};

fn event(
    tick: u64,
    first_direction: Option<StepDirection>,
    second_direction: Option<StepDirection>,
) -> StepDirEvent {
    StepDirEvent {
        tick,
        first_direction,
        second_direction,
    }
}

fn validate_changes(
    events: impl IntoIterator<Item = Result<StepDirEvent, PlannerError>>,
    initial_first_direction: Option<StepDirection>,
    initial_second_direction: Option<StepDirection>,
) -> Result<(), PlannerError> {
    let timing = StepDirTiming::new(1_000, 1_000, 3_000, 2_000).unwrap();
    let timebase = StepTimeBase::new(1_000_000).unwrap();
    validate_step_dir_changes(
        events,
        timing,
        timebase,
        initial_first_direction,
        initial_second_direction,
    )
}

#[test]
fn checks_each_reversal_against_the_motors_latest_step() {
    let unschedulable = Err(PlannerError::DirChangeNotSchedulable);
    for (reverse_tick, return_tick, expected) in [
        (105, 115, Ok(())),
        (104, 115, unschedulable),
        (105, 114, unschedulable),
    ] {
        let events = [
            event(100, Some(Positive), None),
            event(reverse_tick, Some(Negative), None),
            event(110, Some(Negative), None),
            event(return_tick, Some(Positive), None),
        ];

        assert_eq!(validate_changes(events.map(Ok), None, None), expected);
    }
}

#[test]
fn simultaneous_reversals_use_each_motors_previous_step() {
    for (reverse_tick, expected) in [
        (105, Err(PlannerError::DirChangeNotSchedulable)),
        (106, Ok(())),
    ] {
        let events = [
            event(100, Some(Positive), None),
            event(101, None, Some(Negative)),
            event(reverse_tick, Some(Negative), Some(Positive)),
        ];

        assert_eq!(validate_changes(events.map(Ok), None, None), expected);
    }
}

#[test]
fn one_motor_can_reverse_while_the_other_keeps_stepping_in_the_same_direction() {
    let events = [
        event(100, Some(Positive), None),
        event(103, None, Some(Negative)),
        event(105, Some(Negative), Some(Negative)),
    ];

    assert_eq!(validate_changes(events.map(Ok), None, None), Ok(()));
}

#[test]
fn an_unknown_direction_needs_setup_time_before_its_first_step() {
    for (tick, expected) in [(2, Err(PlannerError::DirChangeNotSchedulable)), (3, Ok(()))] {
        let events = [event(tick, Some(Positive), Some(Negative))];

        assert_eq!(validate_changes(events.map(Ok), None, None), expected);
        assert_eq!(
            validate_changes(events.map(Ok), Some(Positive), None),
            expected
        );
    }
}

#[test]
fn known_matching_directions_allow_steps_from_tick_zero() {
    let events = [
        event(0, Some(Positive), Some(Negative)),
        event(2, Some(Positive), Some(Negative)),
    ];

    assert_eq!(
        validate_changes(events.map(Ok), Some(Positive), Some(Negative)),
        Ok(())
    );
    assert_eq!(
        validate_changes(events.map(Ok), Some(Negative), Some(Negative)),
        Err(PlannerError::DirChangeNotSchedulable)
    );
}

#[test]
fn rejects_repeated_and_decreasing_ticks_before_validating_changes() {
    for tick in [2, 1] {
        let events = [
            event(2, Some(Positive), None),
            event(tick, None, Some(Negative)),
        ];

        assert_eq!(
            validate_changes(events.map(Ok), Some(Positive), None),
            Err(PlannerError::InvalidTicks)
        );
    }
}

#[test]
fn propagates_an_error_from_the_source_iterator() {
    let events = [
        Ok(event(100, Some(Positive), None)),
        Err(PlannerError::InvalidTime),
    ]
    .into_iter()
    .chain(core::iter::from_fn(|| {
        panic!("se consumió un evento posterior al error")
    }));

    assert_eq!(
        validate_changes(events, None, None),
        Err(PlannerError::InvalidTime)
    );
}

#[test]
fn propagates_conversion_errors_only_when_a_direction_change_is_needed() {
    let timing = StepDirTiming::new(1, 1, u64::MAX, 1).unwrap();
    let timebase = StepTimeBase::new(2_000_000_000).unwrap();
    let events = [event(100, Some(Positive), None)];

    assert_eq!(
        validate_step_dir_changes(events.map(Ok), timing, timebase, Some(Positive), None),
        Ok(())
    );
    assert_eq!(
        validate_step_dir_changes(events.map(Ok), timing, timebase, None, None),
        Err(PlannerError::NumericalFailure)
    );
}
