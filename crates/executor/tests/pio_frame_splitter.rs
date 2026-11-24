use core::iter::FusedIterator;

use executor::{
    ExecutorError, PIO_FRAME_OVERHEAD_CYCLES, PIO_MAX_FRAME_CYCLES, PioCommandWord,
    PioFrameSplitter, pio_frame_cycles,
};

const MAX: u64 = PIO_MAX_FRAME_CYCLES;
const MAX_COUNTER: u32 = PioCommandWord::MAX_WAIT_COUNTER;
const FIRST_MAX: u64 = MAX_COUNTER as u64;

fn assert_split(delay_cycles: u64, first_frame: bool, expected_counters: &[u32]) {
    let previous_levels = 0b1010;
    let target_levels = 0b0101;
    let mut splitter =
        PioFrameSplitter::new(previous_levels, target_levels, delay_cycles, first_frame).unwrap();
    let mut actual_cycles = 0_u128;

    for (index, &expected_counter) in expected_counters.iter().enumerate() {
        let word = splitter.next().unwrap();
        let levels = if index + 1 == expected_counters.len() {
            target_levels
        } else {
            previous_levels
        };
        let counter = word.wait_counter().unwrap();
        assert_eq!(counter, expected_counter, "espera de {delay_cycles} ciclos");
        assert_eq!(word.levels(), Some(levels));
        actual_cycles += u128::from(pio_frame_cycles(counter).unwrap());
    }

    if first_frame {
        actual_cycles -= u128::from(PIO_FRAME_OVERHEAD_CYCLES);
    }
    assert_eq!(actual_cycles, u128::from(delay_cycles));
    assert_eq!(splitter.next(), None);
}

#[test]
fn splits_regular_delays_without_leaving_a_short_tail() {
    let cases: [(u64, &[u32]); 8] = [
        (6, &[0]),
        (100, &[94]),
        (MAX, &[MAX_COUNTER]),
        (MAX + 1, &[MAX_COUNTER - 5, 0]),
        (MAX + 5, &[MAX_COUNTER - 1, 0]),
        (MAX + 6, &[MAX_COUNTER, 0]),
        (2 * MAX, &[MAX_COUNTER, MAX_COUNTER]),
        (2 * MAX + 1, &[MAX_COUNTER, MAX_COUNTER - 5, 0]),
    ];
    for (delay_cycles, expected_counters) in cases {
        assert_split(delay_cycles, false, expected_counters);
    }
}

#[test]
fn applies_the_startup_compensation_only_to_the_first_word() {
    let cases: [(u64, &[u32]); 8] = [
        (0, &[0]),
        (1, &[1]),
        (100, &[100]),
        (FIRST_MAX, &[MAX_COUNTER]),
        (FIRST_MAX + 1, &[MAX_COUNTER - 5, 0]),
        (FIRST_MAX + 5, &[MAX_COUNTER - 1, 0]),
        (FIRST_MAX + 6, &[MAX_COUNTER, 0]),
        (FIRST_MAX + MAX + 1, &[MAX_COUNTER, MAX_COUNTER - 5, 0]),
    ];
    for (delay_cycles, expected_counters) in cases {
        assert_split(delay_cycles, true, expected_counters);
    }
}

#[test]
fn rejects_regular_delays_below_six_cycles() {
    for delay_cycles in 0..6 {
        assert_eq!(
            PioFrameSplitter::new(0, 15, delay_cycles, false).err(),
            Some(ExecutorError::DelayTooShort)
        );
    }
}

#[test]
fn rejects_invalid_previous_or_target_levels_in_both_modes() {
    for first_frame in [false, true] {
        for (previous_levels, target_levels) in [(16, 0), (0, 16), (u8::MAX, 0), (0, u8::MAX)] {
            assert_eq!(
                PioFrameSplitter::new(previous_levels, target_levels, 6, first_frame).err(),
                Some(ExecutorError::InvalidPinLevels)
            );
        }
    }
}

#[test]
fn remains_exhausted_after_the_last_word_in_both_modes() {
    for (delay_cycles, first_frame) in [(6, false), (0, true)] {
        let mut splitter = PioFrameSplitter::new(0, 15, delay_cycles, first_frame).unwrap();
        let fused: &mut dyn FusedIterator<Item = PioCommandWord> = &mut splitter;

        assert_eq!(fused.next(), Some(PioCommandWord::frame(0, 15).unwrap()));
        assert_eq!(fused.next(), None);
        assert_eq!(fused.next(), None);
    }
}

#[test]
fn generates_only_the_requested_prefix_of_a_u64_max_delay() {
    for first_frame in [false, true] {
        let mut splitter = PioFrameSplitter::new(0b1010, 0b0101, u64::MAX, first_frame).unwrap();
        let expected = PioCommandWord::frame(MAX_COUNTER, 0b1010).unwrap();

        for _ in 0..3 {
            assert_eq!(splitter.next(), Some(expected));
        }
    }
}
