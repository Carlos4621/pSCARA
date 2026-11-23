use planner::{
    MotorAxis::{self, First, Second},
    PendingPinOperationBuffer,
    PinOperation::{self, SetDirection, StepHigh, StepLow},
    PinOperationBatch, PlannerError,
    StepDirection::Negative,
    StepPinLevel::{High, Low},
    TimedPinOperation,
};

fn operation(tick: u64, motor: MotorAxis, operation: PinOperation) -> TimedPinOperation {
    TimedPinOperation {
        tick,
        motor,
        operation,
    }
}

fn high_batch(tick: u64) -> PinOperationBatch {
    PinOperationBatch {
        first_step: Some(High),
        ..PinOperationBatch::new(tick)
    }
}

#[test]
fn starts_empty_with_the_supplied_capacity() {
    let initial = [high_batch(102), high_batch(90)];
    let mut storage = initial;
    {
        let mut pending = PendingPinOperationBuffer::new(&mut storage);

        assert_eq!(pending.capacity(), 2);
        assert_eq!(pending.len(), 0);
        assert!(pending.is_empty());
        assert_eq!(pending.peek_first(), None);
        assert_eq!(pending.pop_first(), None);
    }

    assert_eq!(storage, initial);
}

#[test]
fn inserts_in_any_order_and_pops_in_increasing_tick_order() {
    for ticks in [[90, 95, 100, 102], [102, 100, 95, 90], [90, 100, 102, 95]] {
        let mut storage = [PinOperationBatch::new(0); 4];
        let mut pending = PendingPinOperationBuffer::new(&mut storage);

        for (index, tick) in ticks.into_iter().enumerate() {
            pending.insert(operation(tick, First, StepHigh)).unwrap();
            assert_eq!(pending.len(), index + 1);
        }
        assert!(!pending.is_empty());
        assert_eq!(pending.peek_first(), Some(high_batch(90)));
        assert_eq!(pending.peek_first(), Some(high_batch(90)));
        assert_eq!(pending.len(), 4);

        for (index, tick) in [90, 95, 100, 102].into_iter().enumerate() {
            assert_eq!(pending.peek_first(), Some(high_batch(tick)));
            assert_eq!(pending.pop_first(), Some(high_batch(tick)));
            assert_eq!(pending.len(), 3 - index);
        }
        assert!(pending.is_empty());
        assert_eq!(pending.peek_first(), None);
        assert_eq!(pending.pop_first(), None);

        pending.insert(operation(0, First, StepHigh)).unwrap();
        assert_eq!(pending.pop_first(), Some(high_batch(0)));
    }
}

#[test]
fn merges_at_the_same_tick_even_when_capacity_is_full() {
    let mut storage = [PinOperationBatch::new(0); 2];
    let mut pending = PendingPinOperationBuffer::new(&mut storage);
    pending.insert(operation(100, First, StepHigh)).unwrap();
    pending.insert(operation(100, Second, StepLow)).unwrap();
    assert_eq!(pending.len(), 1);

    pending.insert(operation(90, First, StepHigh)).unwrap();
    assert_eq!(pending.len(), pending.capacity());

    let direction = operation(100, Second, SetDirection(Negative));
    pending.insert(direction).unwrap();
    pending.insert(direction).unwrap();
    assert_eq!(pending.len(), 2);

    assert_eq!(pending.pop_first(), Some(high_batch(90)));
    assert_eq!(
        pending.pop_first(),
        Some(PinOperationBatch {
            second_direction: Some(Negative),
            second_step: Some(Low),
            ..high_batch(100)
        })
    );
    assert!(pending.is_empty());
}

#[test]
fn a_conflict_preserves_the_existing_groups_and_unused_storage() {
    let sentinel = high_batch(u64::MAX);
    let mut storage = [sentinel; 3];
    {
        let mut pending = PendingPinOperationBuffer::new(&mut storage);
        pending.insert(operation(100, First, StepHigh)).unwrap();
        pending.insert(operation(90, First, StepHigh)).unwrap();

        assert_eq!(
            pending.insert(operation(100, First, StepLow)),
            Err(PlannerError::ConflictingPinOperations)
        );
        assert_eq!(pending.len(), 2);
        assert_eq!(pending.peek_first(), Some(high_batch(90)));
    }

    assert_eq!(storage, [high_batch(90), high_batch(100), sentinel]);
}

#[test]
fn a_full_buffer_rejects_new_ticks_and_still_reports_existing_conflicts() {
    let mut storage = [PinOperationBatch::new(0); 2];
    {
        let mut pending = PendingPinOperationBuffer::new(&mut storage);
        pending.insert(operation(90, First, StepHigh)).unwrap();
        pending.insert(operation(100, First, StepHigh)).unwrap();

        assert_eq!(
            pending.insert(operation(95, First, StepHigh)),
            Err(PlannerError::InsufficientPinOperationBuffer)
        );
        assert_eq!(pending.len(), 2);
        assert_eq!(pending.peek_first(), Some(high_batch(90)));

        assert_eq!(
            pending.insert(operation(100, First, StepLow)),
            Err(PlannerError::ConflictingPinOperations)
        );
        assert_eq!(pending.len(), 2);
        assert_eq!(pending.peek_first(), Some(high_batch(90)));
    }

    assert_eq!(storage, [high_batch(90), high_batch(100)]);
}

#[test]
fn zero_capacity_rejects_insertion_and_stays_empty() {
    let mut storage = [];
    let mut pending = PendingPinOperationBuffer::new(&mut storage);

    assert_eq!(pending.capacity(), 0);
    assert_eq!(
        pending.insert(operation(0, First, StepHigh)),
        Err(PlannerError::InsufficientPinOperationBuffer)
    );
    assert_eq!(pending.len(), 0);
    assert!(pending.is_empty());
    assert_eq!(pending.peek_first(), None);
    assert_eq!(pending.pop_first(), None);
}
