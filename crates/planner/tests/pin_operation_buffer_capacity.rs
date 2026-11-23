use planner::{
    PinOperationBatch, PinOperationBatchIterator, PlannerError, ScheduledDirChange,
    ScheduledStepDirEvent, StepDirTiming,
    StepDirection::{Negative, Positive},
    StepPinLevel::{High, Low},
    StepTimeBase, pin_operation_buffer_capacity_bound,
};

#[test]
fn bounds_the_number_of_pending_groups_using_rounded_ticks() {
    let timebase = StepTimeBase::new(1_000_000).unwrap();
    for (setup, high, expected) in [(1, 1, 5), (3, 2, 8), (10, 2, 15), (100, 50, 153)] {
        let timing = StepDirTiming::new(high * 1_000 - 1, 1, setup * 1_000 - 1, 1).unwrap();

        assert_eq!(
            pin_operation_buffer_capacity_bound(timing, timebase),
            Ok(expected)
        );
    }
}

#[test]
fn rejects_overflow_in_either_addition() {
    let timebase = StepTimeBase::new(1_000_000_000).unwrap();
    for setup in [u64::MAX, u64::MAX - 3] {
        let timing = StepDirTiming::new(1, 1, setup, 1).unwrap();

        assert_eq!(
            pin_operation_buffer_capacity_bound(timing, timebase),
            Err(PlannerError::NumericalFailure)
        );
    }
}

#[test]
fn propagates_overflow_from_either_timing_conversion() {
    let timebase = StepTimeBase::new(2_000_000_000).unwrap();
    for (high, setup) in [(u64::MAX, 1), (1, u64::MAX)] {
        let timing = StepDirTiming::new(high, 1, setup, 1).unwrap();

        assert_eq!(
            pin_operation_buffer_capacity_bound(timing, timebase),
            Err(PlannerError::NumericalFailure)
        );
    }
}

#[test]
fn checks_the_capacity_at_the_usize_limit() {
    let timebase = StepTimeBase::new(1_000_000_000).unwrap();
    let max = usize::MAX as u64;
    let timing = StepDirTiming::new(1, 1, max - 4, 1).unwrap();

    assert_eq!(
        pin_operation_buffer_capacity_bound(timing, timebase),
        Ok(usize::MAX)
    );

    #[cfg(target_pointer_width = "32")]
    {
        let timing = StepDirTiming::new(1, 1, max - 3, 1).unwrap();
        assert_eq!(
            pin_operation_buffer_capacity_bound(timing, timebase),
            Err(PlannerError::NumericalFailure)
        );
    }
}

#[test]
fn a_reversal_can_need_all_five_groups_before_releasing_the_next_batch() {
    let timing = StepDirTiming::new(1, 1, 1, 1).unwrap();
    let timebase = StepTimeBase::new(1_000_000_000).unwrap();
    let step_high_ticks = timing.min_step_high_ticks(timebase).unwrap();
    let dir_setup_ticks = timing.min_dir_setup_ticks(timebase).unwrap();
    let capacity = pin_operation_buffer_capacity_bound(timing, timebase).unwrap();
    let mut buffer = [PinOperationBatch::new(0); 5];
    let events =
        [(99, 100, Positive), (103, 104, Negative)].map(|(dir_tick, step_tick, direction)| {
            Ok(ScheduledStepDirEvent {
                step_tick,
                first_dir_change: Some(ScheduledDirChange {
                    tick: dir_tick,
                    direction,
                }),
                second_dir_change: None,
                first_step: true,
                second_step: false,
            })
        });
    let first = PinOperationBatch {
        first_direction: Some(Positive),
        ..PinOperationBatch::new(99)
    };

    assert_eq!(capacity, 5);
    {
        let mut batches = PinOperationBatchIterator::new(
            events.into_iter(),
            step_high_ticks,
            dir_setup_ticks,
            &mut buffer[..capacity],
        )
        .unwrap();
        for expected in [
            first,
            PinOperationBatch {
                first_step: Some(High),
                ..PinOperationBatch::new(100)
            },
            PinOperationBatch {
                first_step: Some(Low),
                ..PinOperationBatch::new(101)
            },
            PinOperationBatch {
                first_direction: Some(Negative),
                ..PinOperationBatch::new(103)
            },
            PinOperationBatch {
                first_step: Some(High),
                ..PinOperationBatch::new(104)
            },
            PinOperationBatch {
                first_step: Some(Low),
                ..PinOperationBatch::new(105)
            },
        ] {
            assert_eq!(batches.next(), Some(Ok(expected)));
        }
        assert_eq!(batches.next(), None);
    }

    let mut batches = PinOperationBatchIterator::new(
        events.into_iter(),
        step_high_ticks,
        dir_setup_ticks,
        &mut buffer[..capacity - 1],
    )
    .unwrap();
    assert_eq!(batches.next(), Some(Ok(first)));
    assert_eq!(
        batches.next(),
        Some(Err(PlannerError::InsufficientPinOperationBuffer))
    );
    assert_eq!(batches.next(), None);
}
