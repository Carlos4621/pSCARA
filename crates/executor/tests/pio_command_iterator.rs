use core::{
    cell::Cell,
    iter::{FusedIterator, empty, from_fn},
};

use executor::{
    ExecutionCommand::{self, End, Frame},
    ExecutionCommandIterator, ExecutorError, PIO_FRAME_OVERHEAD_CYCLES, PackedPinFrame,
    PackedPinFrameIterator, PinFrame, PioCommandIterator, PioCommandWord, pio_frame_cycles,
};
use planner::{PlannerError, StepTimeBase};

fn unit_timebase() -> StepTimeBase {
    StepTimeBase::new(1).unwrap()
}

fn frame(delay_ticks: u64, levels: u8) -> ExecutionCommand {
    Frame(
        PackedPinFrame::try_from(PinFrame {
            delay_ticks,
            levels,
        })
        .unwrap(),
    )
}

fn word(wait_counter: u32, levels: u8) -> PioCommandWord {
    PioCommandWord::frame(wait_counter, levels).unwrap()
}

fn guarded_source(
    commands: impl IntoIterator<Item = Result<ExecutionCommand, ExecutorError>>,
) -> impl Iterator<Item = Result<ExecutionCommand, ExecutorError>> {
    commands.into_iter().chain(from_fn(|| {
        panic!("se consultó la fuente después de una condición terminal")
    }))
}

#[test]
fn emits_end_once_with_any_initial_direction() {
    for initial_levels in [0b0000, 0b0010, 0b1000, 0b1010] {
        let source = guarded_source([Ok(End)]);
        let mut commands =
            PioCommandIterator::new(source, initial_levels, unit_timebase(), 10).unwrap();

        assert_eq!(commands.next(), Some(Ok(PioCommandWord::END)));
        assert_eq!(commands.next(), None);
        assert_eq!(commands.next(), None);
    }
}

#[test]
fn reports_missing_end_for_an_empty_source() {
    let mut commands = PioCommandIterator::new(empty(), 0, unit_timebase(), 10).unwrap();

    assert_eq!(commands.next(), Some(Err(ExecutorError::MissingEndCommand)));
    assert_eq!(commands.next(), None);
    assert_eq!(commands.next(), None);
}

#[test]
fn reports_missing_end_after_movement_and_does_not_poll_a_resumable_source() {
    let mut items = [Some(Ok(frame(0, 0b0001))), None, Some(Ok(End))].into_iter();
    let source = from_fn(|| items.next().flatten());
    let mut commands = PioCommandIterator::new(source, 0, unit_timebase(), 10).unwrap();

    assert_eq!(commands.next(), Some(Ok(word(0, 0b0001))));
    assert_eq!(commands.next(), Some(Err(ExecutorError::MissingEndCommand)));
    assert_eq!(commands.next(), None);
    assert_eq!(commands.next(), None);
    assert_eq!(items.next(), Some(Some(Ok(End))));
}

#[test]
fn rejects_invalid_initial_levels_without_polling_the_source() {
    for initial_levels in 16..=u8::MAX {
        assert_eq!(
            PioCommandIterator::new(guarded_source([]), initial_levels, unit_timebase(), 10).err(),
            Some(ExecutorError::InvalidPinLevels)
        );
    }
}

#[test]
fn rejects_either_initial_step_in_high_without_polling_the_source() {
    for directions in [0b0000, 0b0010, 0b1000, 0b1010] {
        for steps in [0b0001, 0b0100, 0b0101] {
            assert_eq!(
                PioCommandIterator::new(
                    guarded_source([]),
                    directions | steps,
                    unit_timebase(),
                    10
                )
                .err(),
                Some(ExecutorError::StepPinsNotLow)
            );
        }
    }
}

#[test]
fn rejects_invalid_clock_ratios_without_polling_the_source() {
    let timebase = StepTimeBase::new(2).unwrap();
    for pio_frequency_hz in [0, 10, 13] {
        assert_eq!(
            PioCommandIterator::new(guarded_source([]), 0, timebase, pio_frequency_hz).err(),
            Some(ExecutorError::InvalidPioClockRatio)
        );
    }
}

#[test]
fn compensates_a_delayed_first_event_at_the_minimum_clock_ratio() {
    let source = guarded_source([Ok(frame(1, 0b1010)), Ok(frame(1, 0b1000)), Ok(End)]);
    let mut commands = PioCommandIterator::new(source, 0b1000, unit_timebase(), 6).unwrap();

    assert_eq!(commands.next(), Some(Ok(word(6, 0b1010))));
    assert_eq!(commands.next(), Some(Ok(word(0, 0b1000))));
    assert_eq!(commands.next(), Some(Ok(PioCommandWord::END)));
    assert_eq!(commands.next(), None);
}

#[test]
fn splits_consecutive_long_waits_and_polls_only_when_the_active_splitter_is_exhausted() {
    let max_counter = PioCommandWord::MAX_WAIT_COUNTER;
    // Con diez ciclos por tick, las esperas son W + 3 y 2 * MAX + 4, respectivamente.
    let mut source = guarded_source([
        Ok(frame(13_421_773, 0b0011)),
        Ok(frame(26_843_547, 0b0010)),
        Ok(End),
    ]);
    let polls = Cell::new(0);
    let source = from_fn(|| {
        polls.set(polls.get() + 1);
        source.next()
    });
    let mut commands = PioCommandIterator::new(source, 0b1010, unit_timebase(), 10).unwrap();
    let fused: &mut dyn FusedIterator<Item = Result<PioCommandWord, ExecutorError>> = &mut commands;

    assert_eq!(polls.get(), 0);
    for (counter, levels, expected_polls) in [
        (max_counter - 3, 0b1010, 1),
        (0, 0b0011, 1),
        (max_counter, 0b0011, 2),
        (max_counter - 2, 0b0011, 2),
        (0, 0b0010, 2),
    ] {
        assert_eq!(fused.next(), Some(Ok(word(counter, levels))));
        assert_eq!(polls.get(), expected_polls);
    }
    assert_eq!(fused.next(), Some(Ok(PioCommandWord::END)));
    assert_eq!(polls.get(), 3);
    assert_eq!(fused.next(), None);
    assert_eq!(fused.next(), None);
    assert_eq!(polls.get(), 3);
}

#[test]
fn emits_a_prefix_when_the_cycle_product_is_exactly_u64_max() {
    let source = guarded_source([Ok(frame(1, 0b0011))]);
    let mut commands = PioCommandIterator::new(source, 0b1010, unit_timebase(), u64::MAX).unwrap();

    for _ in 0..3 {
        assert_eq!(
            commands.next(),
            Some(Ok(word(PioCommandWord::MAX_WAIT_COUNTER, 0b1010)))
        );
    }
}

#[test]
fn reports_cycle_count_overflow_and_terminates() {
    let source = guarded_source([Ok(frame(0, 0b0010)), Ok(frame(2, 0b0011))]);
    let mut commands = PioCommandIterator::new(source, 0, unit_timebase(), u64::MAX).unwrap();

    assert_eq!(commands.next(), Some(Ok(word(0, 0b0010))));
    assert_eq!(
        commands.next(),
        Some(Err(ExecutorError::CycleCountOverflow))
    );
    assert_eq!(commands.next(), None);
    assert_eq!(commands.next(), None);
}

#[test]
fn rejects_a_zero_delay_after_the_first_frame() {
    let source = guarded_source([Ok(frame(0, 0b0010)), Ok(frame(0, 0b0011))]);
    let mut commands = PioCommandIterator::new(source, 0, unit_timebase(), 10).unwrap();

    assert_eq!(commands.next(), Some(Ok(word(0, 0b0010))));
    assert_eq!(commands.next(), Some(Err(ExecutorError::DelayTooShort)));
    assert_eq!(commands.next(), None);
    assert_eq!(commands.next(), None);
}

#[test]
fn rejects_end_when_either_step_remains_high() {
    for levels in [0b0011, 0b0100, 0b1111] {
        let source = guarded_source([Ok(frame(0, levels)), Ok(End)]);
        let mut commands = PioCommandIterator::new(source, 0, unit_timebase(), 10).unwrap();

        assert_eq!(commands.next(), Some(Ok(word(0, levels))));
        assert_eq!(commands.next(), Some(Err(ExecutorError::StepPinsNotLow)));
        assert_eq!(commands.next(), None);
        assert_eq!(commands.next(), None);
    }
}

#[test]
fn propagates_a_source_error_and_terminates_without_end() {
    let error = ExecutorError::Planner(PlannerError::InvalidTicks);
    let source = guarded_source([Ok(frame(0, 0b0010)), Err(error)]);
    let mut commands = PioCommandIterator::new(source, 0, unit_timebase(), 10).unwrap();

    assert_eq!(commands.next(), Some(Ok(word(0, 0b0010))));
    assert_eq!(commands.next(), Some(Err(error)));
    assert_eq!(commands.next(), None);
    assert_eq!(commands.next(), None);
}

#[test]
fn preserves_the_expected_words_and_logical_timing_through_execution_command_iterator() {
    let frames = [(0, 0b0010), (3, 0b0011), (2, 0b0010)].map(|(delay_ticks, levels)| {
        Ok(PinFrame {
            delay_ticks,
            levels,
        })
    });
    let packed = PackedPinFrameIterator::new(frames.into_iter(), 0).unwrap();
    let source = ExecutionCommandIterator::new(packed, 0).unwrap();
    let timebase = StepTimeBase::new(1_000_000).unwrap();
    let words = PioCommandIterator::new(source, 0, timebase, 10_000_000)
        .unwrap()
        .collect::<Result<Vec<_>, _>>()
        .unwrap();

    let bits: Vec<_> = words.iter().map(PioCommandWord::bits).collect();
    assert_eq!(bits, [0x2000_0000, 0x3000_0030, 0x2000_001C, 0x0000_0001]);

    let mut elapsed = 0;
    for (command, logical_cycle) in words[..3].iter().zip([0, 30, 50]) {
        elapsed += pio_frame_cycles(command.wait_counter().unwrap()).unwrap();
        assert_eq!(elapsed - PIO_FRAME_OVERHEAD_CYCLES, logical_cycle);
    }
}
