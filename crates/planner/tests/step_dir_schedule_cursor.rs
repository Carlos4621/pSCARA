use planner::{
    PlannerError, ScheduledDirChange, ScheduledStepDirEvent, StepDirEvent, StepDirScheduleCursor,
    StepDirTiming, StepDirection,
    StepDirection::{Negative, Positive},
    StepTimeBase,
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

fn cursor(
    initial_first_direction: Option<StepDirection>,
    initial_second_direction: Option<StepDirection>,
) -> StepDirScheduleCursor {
    let timing = StepDirTiming::new(1_000, 1_000, 3_000, 2_000).unwrap();
    let timebase = StepTimeBase::new(1_000_000).unwrap();
    StepDirScheduleCursor::new(
        timing,
        timebase,
        initial_first_direction,
        initial_second_direction,
    )
    .unwrap()
}

#[test]
fn schedules_interleaved_steps_using_independent_direction_and_pulse_histories() {
    let mut cursor = cursor(None, None);
    let dir_change = |tick, direction| Some(ScheduledDirChange { tick, direction });

    for (tick, first_direction, second_direction, first_dir_change, second_dir_change) in [
        (100, Some(Positive), None, dir_change(97, Positive), None),
        (103, None, Some(Negative), None, dir_change(100, Negative)),
        (105, Some(Negative), None, dir_change(102, Negative), None),
        (110, Some(Negative), None, None, None),
    ] {
        assert_eq!(
            cursor.schedule(event(tick, first_direction, second_direction)),
            Ok(ScheduledStepDirEvent {
                step_tick: tick,
                first_dir_change,
                second_dir_change,
                first_step: first_direction.is_some(),
                second_step: second_direction.is_some(),
            }),
            "tick={tick}"
        );
    }
}

#[test]
fn a_failed_reversal_can_be_retried_at_the_first_valid_tick() {
    let mut cursor = cursor(Some(Positive), None);
    cursor.schedule(event(100, Some(Positive), None)).unwrap();

    assert_eq!(
        cursor.schedule(event(103, Some(Negative), None)),
        Err(PlannerError::DirChangeNotSchedulable)
    );
    assert_eq!(
        cursor.schedule(event(105, Some(Negative), None)),
        Ok(ScheduledStepDirEvent {
            step_tick: 105,
            first_dir_change: Some(ScheduledDirChange {
                tick: 102,
                direction: Negative,
            }),
            second_dir_change: None,
            first_step: true,
            second_step: false,
        })
    );
}

#[test]
fn an_error_on_one_motor_preserves_both_histories_and_the_last_event_tick() {
    let mut cursor = cursor(None, None);
    cursor.schedule(event(100, Some(Positive), None)).unwrap();
    cursor.schedule(event(103, None, Some(Negative))).unwrap();

    assert_eq!(
        cursor.schedule(event(105, Some(Negative), Some(Positive))),
        Err(PlannerError::DirChangeNotSchedulable)
    );

    let first = cursor.schedule(event(104, Some(Positive), None)).unwrap();
    assert!(first.first_step);
    assert_eq!(first.first_dir_change, None);

    let second = cursor.schedule(event(105, None, Some(Negative))).unwrap();
    assert!(second.second_step);
    assert_eq!(second.second_dir_change, None);
}

#[test]
fn rejects_close_steps_before_checking_a_reversal_and_allows_a_retry() {
    let mut cursor = cursor(None, None);
    cursor.schedule(event(100, Some(Positive), None)).unwrap();

    for direction in [Positive, Negative] {
        assert_eq!(
            cursor.schedule(event(101, Some(direction), None)),
            Err(PlannerError::StepIntervalTooShort)
        );
    }

    let scheduled = cursor.schedule(event(102, Some(Positive), None)).unwrap();
    assert!(scheduled.first_step);
    assert_eq!(scheduled.first_dir_change, None);
}

#[test]
fn rejects_repeated_or_decreasing_event_ticks_across_motors() {
    let mut cursor = cursor(None, None);
    cursor.schedule(event(100, Some(Positive), None)).unwrap();

    assert_eq!(
        cursor.schedule(event(100, None, Some(Negative))),
        Err(PlannerError::InvalidTicks)
    );
    assert_eq!(
        cursor.schedule(event(99, Some(Negative), None)),
        Err(PlannerError::InvalidTicks)
    );
}

#[test]
fn accepts_matching_initial_directions_on_both_motors_at_tick_zero() {
    let mut cursor = cursor(Some(Positive), Some(Negative));

    assert_eq!(
        cursor.schedule(event(0, Some(Positive), Some(Negative))),
        Ok(ScheduledStepDirEvent {
            step_tick: 0,
            first_dir_change: None,
            second_dir_change: None,
            first_step: true,
            second_step: true,
        })
    );
}

#[test]
fn rejects_an_unrepresentable_discrete_step_interval_when_constructing_the_cursor() {
    let timing = StepDirTiming::new(u64::MAX, u64::MAX, 1, 1).unwrap();
    let timebase = StepTimeBase::new(500_000_000).unwrap();

    assert_eq!(timing.min_step_interval_ticks(timebase), Ok(u64::MAX));
    assert_eq!(
        StepDirScheduleCursor::new(timing, timebase, None, None).unwrap_err(),
        PlannerError::NumericalFailure
    );
}
