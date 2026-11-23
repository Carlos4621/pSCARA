use core::{
    cell::Cell,
    iter::{FusedIterator, empty, from_fn},
};

use executor::{PinFrame, PinFrameEncoder, PinFrameIterator};
use kinematics::JointAngles;
use planner::{
    CoordinatedJointPathProfiles, JointAccelerationLimits, JointToStepsConverter,
    JointVelocityLimits, PinOperationBatch, PlannerError, StepDirTiming,
    StepDirection::{Negative, Positive},
    StepPinLevel::{High, Low},
    StepTimeBase, TimedCoordinatedJointPathProfiles, build_verified_pin_operation_batches,
};

fn encoder() -> PinFrameEncoder {
    PinFrameEncoder::new(Negative, Positive, true, true)
}

#[test]
fn an_empty_source_yields_none() {
    let mut frames = PinFrameIterator::new(empty(), encoder());

    assert_eq!(frames.next(), None);
    assert_eq!(frames.next(), None);
}

#[test]
fn encodes_one_batch_per_call_and_stops_polling_when_the_source_ends() {
    let mut batches = [
        PinOperationBatch {
            first_step: Some(High),
            ..PinOperationBatch::new(0)
        },
        PinOperationBatch {
            first_step: Some(Low),
            ..PinOperationBatch::new(3)
        },
    ]
    .map(Ok)
    .into_iter();
    let polls = Cell::new(0);
    let source = from_fn(|| {
        polls.set(polls.get() + 1);
        batches.next()
    });
    let mut frames = PinFrameIterator::new(source, encoder());
    let fused: &mut dyn FusedIterator<Item = Result<PinFrame, PlannerError>> = &mut frames;

    assert_eq!(polls.get(), 0);
    assert_eq!(
        fused.next(),
        Some(Ok(PinFrame {
            delay_ticks: 0,
            levels: 0b1001
        }))
    );
    assert_eq!(polls.get(), 1);
    assert_eq!(
        fused.next(),
        Some(Ok(PinFrame {
            delay_ticks: 3,
            levels: 0b1000
        }))
    );
    assert_eq!(polls.get(), 2);
    assert_eq!(fused.next(), None);
    assert_eq!(polls.get(), 3);
    assert_eq!(fused.next(), None);
    assert_eq!(polls.get(), 3);
}

#[test]
fn propagates_a_source_error_and_never_polls_again() {
    let source = [
        Ok(PinOperationBatch::new(2)),
        Err(PlannerError::InsufficientPinOperationBuffer),
    ]
    .into_iter()
    .chain(from_fn(|| {
        panic!("se consultó la fuente después del error")
    }));
    let mut frames = PinFrameIterator::new(source, encoder());

    assert_eq!(
        frames.next(),
        Some(Ok(PinFrame {
            delay_ticks: 2,
            levels: 0b1000
        }))
    );
    assert_eq!(
        frames.next(),
        Some(Err(PlannerError::InsufficientPinOperationBuffer))
    );
    assert_eq!(frames.next(), None);
    assert_eq!(frames.next(), None);
}

#[test]
fn propagates_an_encoder_error_and_never_polls_again() {
    let source = [Ok(PinOperationBatch::new(5)); 2]
        .into_iter()
        .chain(from_fn(|| {
            panic!("se consultó la fuente tras el error del encoder")
        }));
    let mut frames = PinFrameIterator::new(source, encoder());

    assert_eq!(
        frames.next(),
        Some(Ok(PinFrame {
            delay_ticks: 5,
            levels: 0b1000
        }))
    );
    assert_eq!(frames.next(), Some(Err(PlannerError::InvalidTicks)));
    assert_eq!(frames.next(), None);
    assert_eq!(frames.next(), None);
}

#[test]
fn encodes_the_verified_two_motor_trajectory() -> Result<(), PlannerError> {
    let points = [
        JointAngles::new(0.0, 0.0).map_err(PlannerError::Kinematics)?,
        JointAngles::new(0.25, -0.25).map_err(PlannerError::Kinematics)?,
    ];
    let velocity = JointVelocityLimits::new(10.0, 10.0)?;
    let acceleration = JointAccelerationLimits::new(1.0, 1.0)?;
    let speeds = [0.0; 2];
    let profiles = CoordinatedJointPathProfiles::new(&points, &speeds, &velocity, &acceleration)?;
    let timed = TimedCoordinatedJointPathProfiles::new(profiles)?;
    let converter = JointToStepsConverter::new(8.0, 4.0)?;
    let timebase = StepTimeBase::new(8)?;
    let timing = StepDirTiming::new(125_000_000, 125_000_000, 1, 1)?;
    let (first_direction, second_direction) = (Negative, Positive);
    let mut boundaries = [0; 2];
    let mut operations = [PinOperationBatch::new(0); 4];

    {
        let batches = build_verified_pin_operation_batches(
            timed,
            converter,
            timebase,
            timing,
            Some(first_direction),
            Some(second_direction),
            &mut boundaries,
            &mut operations,
        )?;
        let encoder = PinFrameEncoder::new(first_direction, second_direction, true, true);
        assert_eq!(encoder.levels(), 0b1000);
        let mut frames = PinFrameIterator::new(batches, encoder);
        let mut elapsed_ticks = 0;

        for (tick, delay_ticks, levels) in [
            (2, 2, 0b1010),
            (3, 1, 0b0011),
            (4, 1, 0b0110),
            (5, 1, 0b0010),
            (6, 1, 0b0011),
            (7, 1, 0b0010),
        ] {
            let frame = frames.next().expect("una trama por grupo")?;
            assert_eq!(
                frame,
                PinFrame {
                    delay_ticks,
                    levels
                }
            );
            elapsed_ticks += frame.delay_ticks;
            assert_eq!(elapsed_ticks, tick);
        }
        assert_eq!(frames.next(), None);
    }

    assert_eq!(boundaries, [0, 8]);
    Ok(())
}
