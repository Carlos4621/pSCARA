use planner::{
    MotorAxis::{self, First, Second},
    PinOperation::{self, SetDirection, StepHigh, StepLow},
    PinOperationBatch, PlannerError,
    StepDirection::{Negative, Positive},
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

#[test]
fn a_new_batch_preserves_all_pins_at_its_tick() {
    let tick = u64::MAX;
    let batch = PinOperationBatch::new(tick);

    assert!(batch.is_empty());
    assert_eq!(
        batch,
        PinOperationBatch {
            tick,
            first_direction: None,
            second_direction: None,
            first_step: None,
            second_step: None,
        }
    );
}

#[test]
fn inserts_each_pin_operation_and_accepts_an_identical_repeat() {
    let tick = 100;
    let empty = PinOperationBatch::new(tick);
    for (motor, pin_operation, expected) in [
        (
            First,
            StepHigh,
            PinOperationBatch {
                first_step: Some(High),
                ..empty
            },
        ),
        (
            First,
            StepLow,
            PinOperationBatch {
                first_step: Some(Low),
                ..empty
            },
        ),
        (
            Second,
            StepHigh,
            PinOperationBatch {
                second_step: Some(High),
                ..empty
            },
        ),
        (
            Second,
            StepLow,
            PinOperationBatch {
                second_step: Some(Low),
                ..empty
            },
        ),
        (
            First,
            SetDirection(Positive),
            PinOperationBatch {
                first_direction: Some(Positive),
                ..empty
            },
        ),
        (
            Second,
            SetDirection(Negative),
            PinOperationBatch {
                second_direction: Some(Negative),
                ..empty
            },
        ),
    ] {
        let mut batch = empty;
        let operation = operation(tick, motor, pin_operation);

        batch.insert(operation).unwrap();
        assert_eq!(batch, expected);
        assert!(!batch.is_empty());

        batch.insert(operation).unwrap();
        assert_eq!(batch, expected);
    }
}

#[test]
fn combines_simultaneous_operations_on_different_motors() {
    let mut batch = PinOperationBatch::new(100);
    batch.insert(operation(100, First, StepHigh)).unwrap();
    batch
        .insert(operation(100, Second, SetDirection(Negative)))
        .unwrap();
    batch.insert(operation(100, Second, StepLow)).unwrap();

    assert_eq!(
        batch,
        PinOperationBatch {
            tick: 100,
            first_direction: None,
            second_direction: Some(Negative),
            first_step: Some(High),
            second_step: Some(Low),
        }
    );
}

#[test]
fn conflicts_on_either_motor_leave_the_batch_unchanged() {
    let tick = 100;
    for (step_motor, dir_motor, step, opposite_step, dir, opposite_dir) in [
        (First, Second, StepHigh, StepLow, Positive, Negative),
        (Second, First, StepLow, StepHigh, Negative, Positive),
    ] {
        let mut batch = PinOperationBatch::new(tick);
        batch.insert(operation(tick, step_motor, step)).unwrap();
        batch
            .insert(operation(tick, dir_motor, SetDirection(dir)))
            .unwrap();
        let before = batch;

        assert_eq!(
            batch.insert(operation(tick, step_motor, opposite_step)),
            Err(PlannerError::ConflictingPinOperations)
        );
        assert_eq!(batch, before);

        assert_eq!(
            batch.insert(operation(tick, dir_motor, SetDirection(opposite_dir))),
            Err(PlannerError::ConflictingPinOperations)
        );
        assert_eq!(batch, before);
    }
}

#[test]
fn a_wrong_tick_leaves_the_batch_unchanged_even_if_the_pin_would_conflict() {
    let mut batch = PinOperationBatch::new(100);
    let empty = batch;
    assert_eq!(
        batch.insert(operation(101, First, StepHigh)),
        Err(PlannerError::InvalidTicks)
    );
    assert_eq!(batch, empty);

    batch.insert(operation(100, First, StepHigh)).unwrap();
    let before = batch;
    assert_eq!(
        batch.insert(operation(99, First, StepLow)),
        Err(PlannerError::InvalidTicks)
    );
    assert_eq!(batch, before);
}
