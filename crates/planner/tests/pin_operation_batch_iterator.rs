use core::{
    cell::Cell,
    iter::{FusedIterator, empty, from_fn},
};

use planner::{
    PinOperationBatch, PinOperationBatchIterator, PlannerError, ScheduledDirChange,
    ScheduledStepDirEvent,
    StepDirection::{Negative, Positive},
    StepPinLevel::{High, Low},
};

fn step_event(step_tick: u64, first_step: bool, second_step: bool) -> ScheduledStepDirEvent {
    ScheduledStepDirEvent {
        step_tick,
        first_dir_change: None,
        second_dir_change: None,
        first_step,
        second_step,
    }
}

#[test]
fn rejects_zero_durations_before_reading_the_source() {
    for (step_high, dir_setup) in [(0, 3), (2, 0)] {
        let source = from_fn(|| panic!("se consultó la fuente durante la construcción"));
        let mut buffer = [PinOperationBatch::new(0); 3];

        assert_eq!(
            PinOperationBatchIterator::new(source, step_high, dir_setup, &mut buffer).err(),
            Some(PlannerError::InvalidStepDirTiming)
        );
    }
}

#[test]
fn an_empty_source_yields_none_with_zero_capacity() {
    let mut batches = PinOperationBatchIterator::new(empty(), 2, 3, &mut []).unwrap();

    assert_eq!(batches.next(), None);
    assert_eq!(batches.next(), None);
}

#[test]
fn drains_both_edges_of_a_pulse_after_the_first_end_of_source() {
    let mut reads = [Some(Ok(step_event(0, true, false))), None].into_iter();
    let source = from_fn(|| reads.next().expect("se consultó la fuente agotada"));
    let mut buffer = [PinOperationBatch::new(0); 2];
    let mut batches = PinOperationBatchIterator::new(source, 2, 3, &mut buffer).unwrap();
    let fused: &mut dyn FusedIterator<Item = Result<PinOperationBatch, PlannerError>> =
        &mut batches;

    for (tick, level) in [(0, High), (2, Low)] {
        assert_eq!(
            fused.next(),
            Some(Ok(PinOperationBatch {
                first_step: Some(level),
                ..PinOperationBatch::new(tick)
            }))
        );
    }
    assert_eq!(fused.next(), None);
    assert_eq!(fused.next(), None);
}

#[test]
fn interleaves_operations_and_merges_batches_at_the_same_tick() {
    let dir_change = |tick, direction| Some(ScheduledDirChange { tick, direction });
    let source = [
        ScheduledStepDirEvent {
            first_dir_change: dir_change(97, Positive),
            ..step_event(100, true, false)
        },
        ScheduledStepDirEvent {
            second_dir_change: dir_change(98, Negative),
            ..step_event(101, false, true)
        },
        ScheduledStepDirEvent {
            first_dir_change: dir_change(102, Negative),
            ..step_event(105, true, false)
        },
    ]
    .map(Ok)
    .into_iter();
    let mut buffer = [PinOperationBatch::new(0); 6];
    let mut batches = PinOperationBatchIterator::new(source, 2, 3, &mut buffer).unwrap();

    for expected in [
        PinOperationBatch {
            first_direction: Some(Positive),
            ..PinOperationBatch::new(97)
        },
        PinOperationBatch {
            second_direction: Some(Negative),
            ..PinOperationBatch::new(98)
        },
        PinOperationBatch {
            first_step: Some(High),
            ..PinOperationBatch::new(100)
        },
        PinOperationBatch {
            second_step: Some(High),
            ..PinOperationBatch::new(101)
        },
        PinOperationBatch {
            first_direction: Some(Negative),
            first_step: Some(Low),
            ..PinOperationBatch::new(102)
        },
        PinOperationBatch {
            second_step: Some(Low),
            ..PinOperationBatch::new(103)
        },
        PinOperationBatch {
            first_step: Some(High),
            ..PinOperationBatch::new(105)
        },
        PinOperationBatch {
            first_step: Some(Low),
            ..PinOperationBatch::new(107)
        },
    ] {
        assert_eq!(batches.next(), Some(Ok(expected)));
    }
    assert_eq!(batches.next(), None);
}

#[test]
fn an_idle_event_advances_the_release_limit_without_creating_a_batch() {
    let mut events = [step_event(100, true, false), step_event(105, false, false)]
        .map(Ok)
        .into_iter();
    let polls = Cell::new(0);
    let source = from_fn(|| {
        polls.set(polls.get() + 1);
        events.next()
    });
    let mut buffer = [PinOperationBatch::new(0); 2];
    let mut batches = PinOperationBatchIterator::new(source, 2, 3, &mut buffer).unwrap();

    assert_eq!(polls.get(), 0);
    for (tick, level) in [(100, High), (102, Low)] {
        assert_eq!(
            batches.next(),
            Some(Ok(PinOperationBatch {
                first_step: Some(level),
                ..PinOperationBatch::new(tick)
            }))
        );
        assert_eq!(polls.get(), 2);
    }
    assert_eq!(batches.next(), None);
    assert_eq!(polls.get(), 3);
}

#[test]
fn insufficient_capacity_yields_an_error_before_releasing_any_part_of_the_event() {
    let event = ScheduledStepDirEvent {
        first_dir_change: Some(ScheduledDirChange {
            tick: 97,
            direction: Positive,
        }),
        ..step_event(100, true, false)
    };
    let source = [Ok(event)].into_iter().chain(from_fn(|| {
        panic!("se consultó la fuente tras fallar la inserción")
    }));
    let mut buffer = [PinOperationBatch::new(0); 2];
    let mut batches = PinOperationBatchIterator::new(source, 2, 3, &mut buffer).unwrap();

    assert_eq!(
        batches.next(),
        Some(Err(PlannerError::InsufficientPinOperationBuffer))
    );
    assert_eq!(batches.next(), None);
    assert_eq!(batches.next(), None);
}

#[test]
fn rejects_direction_changes_at_the_wrong_tick_or_without_a_step() {
    let first_change = |tick| {
        Some(ScheduledDirChange {
            tick,
            direction: Positive,
        })
    };
    let second_change = |tick| {
        Some(ScheduledDirChange {
            tick,
            direction: Negative,
        })
    };
    for event in [
        ScheduledStepDirEvent {
            first_dir_change: first_change(96),
            ..step_event(100, true, false)
        },
        ScheduledStepDirEvent {
            second_dir_change: second_change(98),
            ..step_event(100, false, true)
        },
        ScheduledStepDirEvent {
            first_dir_change: first_change(97),
            ..step_event(100, false, true)
        },
        ScheduledStepDirEvent {
            second_dir_change: second_change(0),
            ..step_event(2, false, true)
        },
    ] {
        let mut buffer = [PinOperationBatch::new(0); 3];
        let mut batches =
            PinOperationBatchIterator::new([Ok(event)].into_iter(), 2, 3, &mut buffer).unwrap();

        assert_eq!(
            batches.next(),
            Some(Err(PlannerError::DirChangeNotSchedulable)),
            "event={event:?}"
        );
        assert_eq!(batches.next(), None);
    }
}

#[test]
fn rejects_repeated_and_decreasing_ticks_before_inserting_the_event() {
    for tick in [100, 98] {
        let source = [step_event(100, true, false), step_event(tick, true, false)]
            .map(Ok)
            .into_iter();
        let mut buffer = [PinOperationBatch::new(0); 4];
        let mut batches = PinOperationBatchIterator::new(source, 2, 3, &mut buffer).unwrap();

        assert_eq!(batches.next(), Some(Err(PlannerError::InvalidTicks)));
        assert_eq!(batches.next(), None);
    }
}

#[test]
fn conflicting_edges_terminate_without_draining_pending_batches() {
    let source = [step_event(100, true, false), step_event(102, true, false)]
        .map(Ok)
        .into_iter();
    let mut buffer = [PinOperationBatch::new(0); 4];
    let mut batches = PinOperationBatchIterator::new(source, 2, 3, &mut buffer).unwrap();

    assert_eq!(
        batches.next(),
        Some(Err(PlannerError::ConflictingPinOperations))
    );
    assert_eq!(batches.next(), None);
}

#[test]
fn a_source_error_terminates_without_draining_pending_batches() {
    let event = ScheduledStepDirEvent {
        first_dir_change: Some(ScheduledDirChange {
            tick: 97,
            direction: Positive,
        }),
        ..step_event(100, true, false)
    };
    let source = [Ok(event), Err(PlannerError::InvalidTime)]
        .into_iter()
        .chain(from_fn(|| {
            panic!("se consultó la fuente después de su error")
        }));
    let mut buffer = [PinOperationBatch::new(0); 3];
    let mut batches = PinOperationBatchIterator::new(source, 2, 3, &mut buffer).unwrap();

    assert_eq!(
        batches.next(),
        Some(Ok(PinOperationBatch {
            first_direction: Some(Positive),
            ..PinOperationBatch::new(97)
        }))
    );
    assert_eq!(batches.next(), Some(Err(PlannerError::InvalidTime)));
    assert_eq!(batches.next(), None);
    assert_eq!(batches.next(), None);
}

#[test]
fn accepts_a_falling_edge_at_u64_max() {
    let max = u64::MAX;
    let source = [Ok(step_event(max - 2, false, true))].into_iter();
    let mut buffer = [PinOperationBatch::new(0); 2];
    let mut batches = PinOperationBatchIterator::new(source, 2, 3, &mut buffer).unwrap();

    for (tick, level) in [(max - 2, High), (max, Low)] {
        assert_eq!(
            batches.next(),
            Some(Ok(PinOperationBatch {
                second_step: Some(level),
                ..PinOperationBatch::new(tick)
            }))
        );
    }
    assert_eq!(batches.next(), None);
}

#[test]
fn propagates_falling_edge_overflow_without_draining_pending_batches() {
    let max = u64::MAX;
    let source = [
        step_event(max - 4, true, false),
        step_event(max - 1, false, true),
    ]
    .map(Ok)
    .into_iter();
    let mut buffer = [PinOperationBatch::new(0); 4];
    let mut batches = PinOperationBatchIterator::new(source, 2, 3, &mut buffer).unwrap();

    assert_eq!(batches.next(), Some(Err(PlannerError::NumericalFailure)));
    assert_eq!(batches.next(), None);
}
