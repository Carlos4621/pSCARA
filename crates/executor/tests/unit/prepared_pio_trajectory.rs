use planner::{StepDirTiming, StepDirection, StepTimeBase};

use super::{
    ExecutorError, PioCommandWord, PioExecutionConfig, accumulate_frame_cycles, collect_commands,
};
use crate::PIO_MAX_FRAME_CYCLES;

fn config() -> PioExecutionConfig {
    PioExecutionConfig {
        timebase: StepTimeBase::new(10).unwrap(),
        timing: StepDirTiming::new(1, 1, 1, 1).unwrap(),
        pio_frequency_hz: 100,
        initial_first_direction: StepDirection::Negative,
        initial_second_direction: StepDirection::Positive,
        first_positive_is_high: true,
        second_positive_is_high: true,
    }
}

fn frame(wait_counter: u32, levels: u8) -> PioCommandWord {
    PioCommandWord::frame(wait_counter, levels).unwrap()
}

#[test]
fn stores_only_the_verified_prefix_and_reuses_the_same_storage() {
    let words = [frame(0, 0b1001), frame(4, 0b0010), PioCommandWord::END];
    let unused = frame(42, 0);
    let mut storage = [unused; 4];
    let buffer_address = storage.as_ptr();

    {
        let prepared = collect_commands(words.into_iter().map(Ok), config(), &mut storage).unwrap();
        assert_eq!(prepared.commands(), &words);
        assert_eq!(prepared.commands().as_ptr(), buffer_address);
        assert_eq!(prepared.len(), 3);
        assert!(!prepared.is_empty());
        assert_eq!(prepared.initial_levels(), 0b1000);
        assert_eq!(prepared.final_levels(), 0b0010);
        assert_eq!(prepared.frame_cycles(), 16);
        assert_eq!(prepared.pio_frequency_hz(), 100);
        assert_eq!(prepared.duration_seconds(), 0.16);
        assert!(prepared.iter().eq(words.into_iter().map(Ok)));
    }
    assert_eq!(storage[3], unused);

    let prepared = collect_commands(
        core::iter::once(Ok(PioCommandWord::END)),
        config(),
        &mut storage,
    )
    .unwrap();
    assert_eq!(prepared.commands(), &[PioCommandWord::END]);
    assert_eq!(prepared.initial_levels(), prepared.final_levels());
    assert_eq!(prepared.frame_cycles(), 0);
    assert_eq!(prepared.duration_seconds(), 0.0);
}

#[test]
fn requires_end_even_if_the_source_has_no_frames() {
    let mut storage = [PioCommandWord::END; 1];
    for words in [&[][..], &[Ok(frame(0, 0))][..]] {
        assert_eq!(
            collect_commands(words.iter().copied(), config(), &mut storage).unwrap_err(),
            ExecutorError::MissingEndCommand
        );
    }
}

#[test]
fn rejects_any_word_after_end() {
    let mut storage = [PioCommandWord::END; 1];
    for trailing in [PioCommandWord::END, frame(0, 0)] {
        let source = [Ok(PioCommandWord::END), Ok(trailing)];
        assert_eq!(
            collect_commands(source.into_iter(), config(), &mut storage).unwrap_err(),
            ExecutorError::UnexpectedCommandAfterEnd
        );
    }
}

#[test]
fn propagates_a_source_error_during_collection_or_after_end() {
    let error = ExecutorError::DelayTooShort;
    let mut storage = [PioCommandWord::END; 2];
    for first in [frame(0, 0), PioCommandWord::END] {
        let source = [Ok(first), Err(error)];
        assert_eq!(
            collect_commands(source.into_iter(), config(), &mut storage).unwrap_err(),
            error
        );
    }
}

#[test]
fn requires_both_step_pins_low_at_end() {
    let mut storage = [PioCommandWord::END; 2];
    for levels in [0b0001, 0b0100, 0b0101] {
        let source = [Ok(frame(0, levels)), Ok(PioCommandWord::END)];
        assert_eq!(
            collect_commands(source.into_iter(), config(), &mut storage).unwrap_err(),
            ExecutorError::StepPinsNotLow
        );
    }
}

#[test]
fn rejects_overflow_at_the_accumulated_frame_cycle_limit() {
    // La misma operación que usa el recolector, sin necesitar miles de millones de tramas.
    let counter = PioCommandWord::MAX_WAIT_COUNTER;
    let total = u64::MAX - PIO_MAX_FRAME_CYCLES;
    assert_eq!(accumulate_frame_cycles(total, counter), Ok(u64::MAX));
    assert_eq!(
        accumulate_frame_cycles(total + 1, counter),
        Err(ExecutorError::CycleCountOverflow)
    );
}
