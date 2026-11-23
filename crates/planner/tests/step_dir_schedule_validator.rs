use planner::{
    PlannerError, StepDirEvent, StepDirScheduleCursor, StepDirTiming, StepDirection,
    StepDirection::{Negative, Positive},
    StepTimeBase, validate_step_dir_changes, validate_step_dir_intervals,
    validate_step_dir_schedule,
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

fn validate_schedule(
    events: impl IntoIterator<Item = Result<StepDirEvent, PlannerError>>,
    initial_first_direction: Option<StepDirection>,
    initial_second_direction: Option<StepDirection>,
) -> Result<(), PlannerError> {
    let timing = StepDirTiming::new(1_000, 1_000, 3_000, 2_000).unwrap();
    let timebase = StepTimeBase::new(1_000_000).unwrap();
    validate_step_dir_schedule(
        events,
        timing,
        timebase,
        initial_first_direction,
        initial_second_direction,
    )
}

#[test]
fn cursor_and_validator_agree_at_the_discrete_interval_boundary() {
    let timing = StepDirTiming::new(1_100, 1_100, 1_000, 1_000).unwrap();
    let timebase = StepTimeBase::new(1_000_000).unwrap();
    let too_short = Err(PlannerError::StepIntervalTooShort);

    for (next_tick, expected_discrete, expected_total) in [
        (102, too_short, too_short),
        (103, too_short, Ok(())),
        (104, Ok(()), Ok(())),
        (105, Ok(()), Ok(())),
    ] {
        let events = [
            event(100, Some(Positive), None),
            event(next_tick, Some(Positive), None),
        ];
        let mut cursor = StepDirScheduleCursor::new(timing, timebase, None, None).unwrap();
        let scheduled = events
            .into_iter()
            .try_for_each(|event| cursor.schedule(event).map(|_| ()));
        let validated = validate_step_dir_schedule(events.map(Ok), timing, timebase, None, None);

        assert_eq!(scheduled, expected_discrete, "next_tick={next_tick}");
        assert_eq!(validated, scheduled, "next_tick={next_tick}");
        assert_eq!(
            validate_step_dir_intervals(events.map(Ok), timing, timebase),
            expected_total,
            "next_tick={next_tick}"
        );
    }
}

#[test]
fn checks_the_step_interval_before_the_dir_change_window() {
    for (tick, direction, expected) in [
        (101, Positive, Err(PlannerError::StepIntervalTooShort)),
        (101, Negative, Err(PlannerError::StepIntervalTooShort)),
        (104, Negative, Err(PlannerError::DirChangeNotSchedulable)),
        (105, Negative, Ok(())),
    ] {
        let events = [
            event(100, Some(Positive), None),
            event(tick, Some(direction), None),
        ];

        assert_eq!(
            validate_schedule(events.map(Ok), None, None),
            expected,
            "tick={tick}, direction={direction:?}"
        );
    }
}

#[test]
fn a_step_error_on_the_second_motor_takes_priority_over_a_dir_error_on_the_first() {
    let events = [
        event(100, Some(Positive), None),
        event(103, None, Some(Negative)),
        event(104, Some(Negative), Some(Negative)),
    ];

    assert_eq!(
        validate_schedule(events.map(Ok), None, None),
        Err(PlannerError::StepIntervalTooShort)
    );
}

#[test]
fn valid_interleaved_steps_agree_with_the_separate_validators() {
    let timing = StepDirTiming::new(1_000, 1_000, 3_000, 2_000).unwrap();
    let timebase = StepTimeBase::new(1_000_000).unwrap();
    let events = [
        event(0, Some(Positive), None),
        event(4, None, Some(Negative)),
        event(5, Some(Negative), None),
        event(6, None, Some(Negative)),
        event(11, Some(Negative), Some(Positive)),
    ]
    .map(Ok);

    let combined = validate_step_dir_schedule(events, timing, timebase, Some(Positive), None);

    assert_eq!(combined, Ok(()));
    assert_eq!(
        combined,
        validate_step_dir_intervals(events, timing, timebase)
    );
    assert_eq!(
        combined,
        validate_step_dir_changes(events, timing, timebase, Some(Positive), None)
    );
}

#[test]
fn setup_before_the_first_step_depends_on_the_initial_direction() {
    for (tick, initial_direction, expected) in [
        (2, None, Err(PlannerError::DirChangeNotSchedulable)),
        (3, None, Ok(())),
        (0, Some(Positive), Ok(())),
        (
            2,
            Some(Negative),
            Err(PlannerError::DirChangeNotSchedulable),
        ),
    ] {
        let events = [event(tick, Some(Positive), None)];

        assert_eq!(
            validate_schedule(events.map(Ok), initial_direction, None),
            expected
        );
    }
}

#[test]
fn rejects_repeated_or_decreasing_ticks_before_step_and_dir_checks() {
    for (tick, first, second) in [(100, None, Some(Negative)), (99, Some(Negative), None)] {
        let events = [event(100, Some(Positive), None), event(tick, first, second)];

        assert_eq!(
            validate_schedule(events.map(Ok), None, None),
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
        validate_schedule(events, None, None),
        Err(PlannerError::InvalidTime)
    );
}

#[test]
fn checks_that_the_discrete_step_interval_fits_before_reading_events() {
    let timing = StepDirTiming::new(u64::MAX, u64::MAX, 1, 1).unwrap();
    let timebase = StepTimeBase::new(500_000_000).unwrap();
    let events = core::iter::from_fn(|| panic!("se consumió un evento con un intervalo inválido"));

    assert_eq!(
        validate_step_dir_schedule(events, timing, timebase, None, None),
        Err(PlannerError::NumericalFailure)
    );
}
