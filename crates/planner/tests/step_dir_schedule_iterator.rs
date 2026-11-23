use core::{
    cell::Cell,
    iter::{FusedIterator, empty, from_fn},
};

use planner::{
    PlannerError, ScheduledDirChange, ScheduledStepDirEvent, StepDirEvent, StepDirScheduleCursor,
    StepDirScheduleIterator, StepDirTiming, StepDirection,
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

fn cursor() -> StepDirScheduleCursor {
    let timing = StepDirTiming::new(1_000, 1_000, 3_000, 2_000).unwrap();
    let timebase = StepTimeBase::new(1_000_000).unwrap();
    StepDirScheduleCursor::new(timing, timebase, None, None).unwrap()
}

#[test]
fn an_empty_source_returns_none() {
    let mut scheduled = StepDirScheduleIterator::new(empty(), cursor());

    assert_eq!(scheduled.next(), None);
    assert_eq!(scheduled.next(), None);
}

#[test]
fn schedules_each_event_on_demand_with_independent_motor_directions() {
    let mut events = [
        event(100, Some(Positive), Some(Negative)),
        event(102, Some(Positive), None),
        event(107, Some(Negative), Some(Negative)),
        event(112, None, Some(Positive)),
    ]
    .map(Ok)
    .into_iter();
    let polls = Cell::new(0);
    let source = from_fn(|| {
        polls.set(polls.get() + 1);
        events.next()
    });
    let mut scheduled = StepDirScheduleIterator::new(source, cursor());
    let change = |tick, direction| Some(ScheduledDirChange { tick, direction });

    assert_eq!(polls.get(), 0);
    for (index, (step_tick, first_dir_change, second_dir_change, first_step, second_step)) in [
        (100, change(97, Positive), change(97, Negative), true, true),
        (102, None, None, true, false),
        (107, change(104, Negative), None, true, true),
        (112, None, change(109, Positive), false, true),
    ]
    .into_iter()
    .enumerate()
    {
        assert_eq!(
            scheduled.next(),
            Some(Ok(ScheduledStepDirEvent {
                step_tick,
                first_dir_change,
                second_dir_change,
                first_step,
                second_step,
            }))
        );
        assert_eq!(polls.get(), index + 1);
    }
    assert_eq!(scheduled.next(), None);
    assert_eq!(polls.get(), 5);
}

#[test]
fn does_not_poll_the_source_after_the_first_none() {
    let mut polls = [Some(Ok(event(100, Some(Positive), None))), None].into_iter();
    let source = from_fn(|| polls.next().expect("se consultó la fuente agotada"));
    let mut scheduled = StepDirScheduleIterator::new(source, cursor());
    let fused: &mut dyn FusedIterator<Item = Result<ScheduledStepDirEvent, PlannerError>> =
        &mut scheduled;

    assert_eq!(fused.next().unwrap().unwrap().step_tick, 100);
    assert_eq!(fused.next(), None);
    assert_eq!(fused.next(), None);
    assert_eq!(fused.next(), None);
}

#[test]
fn returns_the_source_error_and_never_polls_again() {
    let source = [
        Ok(event(100, Some(Positive), None)),
        Err(PlannerError::InvalidTime),
    ]
    .into_iter()
    .chain(from_fn(|| {
        panic!("se consultó la fuente después de su error")
    }));
    let mut scheduled = StepDirScheduleIterator::new(source, cursor());

    assert_eq!(scheduled.next().unwrap().unwrap().step_tick, 100);
    assert_eq!(scheduled.next(), Some(Err(PlannerError::InvalidTime)));
    assert_eq!(scheduled.next(), None);
    assert_eq!(scheduled.next(), None);
}

#[test]
fn returns_the_cursor_error_and_never_polls_again() {
    let source = [
        Ok(event(100, Some(Positive), None)),
        Ok(event(104, Some(Negative), None)),
    ]
    .into_iter()
    .chain(from_fn(|| {
        panic!("se consultó la fuente después del error del cursor")
    }));
    let mut scheduled = StepDirScheduleIterator::new(source, cursor());

    assert_eq!(scheduled.next().unwrap().unwrap().step_tick, 100);
    assert_eq!(
        scheduled.next(),
        Some(Err(PlannerError::DirChangeNotSchedulable))
    );
    assert_eq!(scheduled.next(), None);
    assert_eq!(scheduled.next(), None);
}
