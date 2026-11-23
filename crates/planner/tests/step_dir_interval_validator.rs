use planner::{
    PlannerError, StepDirEvent, StepDirTiming, StepDirection,
    StepDirection::{Negative, Positive},
    StepTimeBase, validate_step_dir_intervals,
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

fn validate_at_four_ticks(
    events: impl IntoIterator<Item = Result<StepDirEvent, PlannerError>>,
) -> Result<(), PlannerError> {
    let timing = StepDirTiming::new(2_000, 2_000, 200, 200).unwrap();
    let timebase = StepTimeBase::new(1_000_000).unwrap();
    validate_step_dir_intervals(events, timing, timebase)
}

#[test]
fn simultaneous_steps_can_be_separated_by_the_minimum_or_more() {
    let events = [
        event(100, Some(Positive), Some(Negative)),
        event(104, Some(Negative), Some(Positive)),
        event(109, Some(Negative), Some(Positive)),
    ];

    assert_eq!(validate_at_four_ticks(events.map(Ok)), Ok(()));
}

#[test]
fn rejects_steps_too_close_on_either_motor() {
    for (first, second) in [(Some(Positive), None), (None, Some(Negative))] {
        let events = [event(100, first, second), event(102, first, second)];

        assert_eq!(
            validate_at_four_ticks(events.map(Ok)),
            Err(PlannerError::StepIntervalTooShort)
        );
    }
}

#[test]
fn checks_interleaved_pulses_against_each_motors_own_history() {
    for (last_tick, expected) in [
        (105, Err(PlannerError::StepIntervalTooShort)),
        (106, Ok(())),
    ] {
        let events = [
            event(100, Some(Positive), None),
            event(102, None, Some(Negative)),
            event(103, None, None),
            event(104, Some(Positive), None),
            event(last_tick, None, Some(Negative)),
        ];

        assert_eq!(validate_at_four_ticks(events.map(Ok)), expected);
    }
}

#[test]
fn rejects_repeated_or_decreasing_ticks_across_motors_and_idle_events() {
    for (tick, second) in [(100, Some(Negative)), (99, None)] {
        let events = [event(100, Some(Positive), None), event(tick, None, second)];

        assert_eq!(
            validate_at_four_ticks(events.map(Ok)),
            Err(PlannerError::InvalidTicks)
        );
    }
}

#[test]
fn propagates_a_source_error_after_a_valid_prefix() {
    let events = [
        Ok(event(100, Some(Positive), None)),
        Err(PlannerError::InvalidTime),
    ]
    .into_iter()
    .chain(core::iter::from_fn(|| {
        panic!("se consumió un evento posterior al error")
    }));

    assert_eq!(
        validate_at_four_ticks(events),
        Err(PlannerError::InvalidTime)
    );
}

#[test]
fn rejects_an_unrepresentable_interval_before_reading_events() {
    let timing = StepDirTiming::new(u64::MAX, u64::MAX, 200, 200).unwrap();
    let timebase = StepTimeBase::new(u64::MAX).unwrap();
    let events = core::iter::from_fn(|| panic!("se consumió un evento con un intervalo inválido"));

    assert_eq!(
        validate_step_dir_intervals(events, timing, timebase),
        Err(PlannerError::NumericalFailure)
    );
}

#[test]
fn compares_ticks_near_u64_max_without_overflow() {
    let max = u64::MAX;
    for (first_tick, second_tick, expected) in [
        (max - 4, max, Ok(())),
        (max - 3, max, Err(PlannerError::StepIntervalTooShort)),
        (max, max - 1, Err(PlannerError::InvalidTicks)),
    ] {
        let events = [
            event(first_tick, None, Some(Positive)),
            event(second_tick, None, Some(Positive)),
        ];

        assert_eq!(validate_at_four_ticks(events.map(Ok)), expected);
    }
}
