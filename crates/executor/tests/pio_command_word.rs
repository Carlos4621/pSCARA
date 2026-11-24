use core::mem::{align_of, size_of};

use executor::{ExecutorError, PioCommandWord};

#[test]
fn packs_the_expected_words_and_recovers_the_original_fields() {
    let max = PioCommandWord::MAX_WAIT_COUNTER;
    assert_eq!(max, 134_217_727);

    for (wait_counter, levels, expected_bits) in [
        (0, 0b0000, 0x0000_0000),
        (0, 0b1111, 0xF000_0000),
        (1, 0b0000, 0x0000_0002),
        (10, 0b1010, 0xA000_0014),
        (max, 0b0000, 0x0FFF_FFFE),
        (max, 0b1111, 0xFFFF_FFFE),
    ] {
        let command = PioCommandWord::frame(wait_counter, levels).unwrap();

        assert_eq!(command.bits(), expected_bits);
        assert_eq!(PioCommandWord::from_bits(expected_bits), Ok(command));
        assert!(!command.is_end());
        assert_eq!(command.levels(), Some(levels));
        assert_eq!(command.wait_counter(), Some(wait_counter));
    }
}

#[test]
fn decodes_each_counter_bit_with_every_pin_level_mask() {
    for levels in 0_u8..=0b1111 {
        for bit in 1..=27 {
            let bits = (u32::from(levels) << 28) | (1_u32 << bit);
            let command = PioCommandWord::from_bits(bits).unwrap();
            let wait_counter = 1_u32 << (bit - 1);

            assert_eq!(command.bits(), bits);
            assert_eq!(command.levels(), Some(levels));
            assert_eq!(command.wait_counter(), Some(wait_counter));
            assert_eq!(PioCommandWord::frame(wait_counter, levels), Ok(command));
        }
    }
}

#[test]
fn rejects_wait_counters_that_do_not_fit() {
    for wait_counter in [PioCommandWord::MAX_WAIT_COUNTER + 1, 1 << 31, u32::MAX] {
        assert_eq!(
            PioCommandWord::frame(wait_counter, 0),
            Err(ExecutorError::WaitCounterOutOfRange)
        );
    }
}

#[test]
fn rejects_levels_that_do_not_fit() {
    for levels in 16..=u8::MAX {
        assert_eq!(
            PioCommandWord::frame(0, levels),
            Err(ExecutorError::InvalidPinLevels)
        );
    }
}

#[test]
fn rejects_reserved_odd_words() {
    let single_extra_bits = (1..u32::BITS).map(|bit| 1 | (1_u32 << bit));
    for bits in single_extra_bits.chain([u32::MAX]) {
        assert_eq!(
            PioCommandWord::from_bits(bits),
            Err(ExecutorError::InvalidPioCommandWord),
            "bits = {bits:#010x}"
        );
    }
}

#[test]
fn queries_end_in_const_contexts() {
    const END: PioCommandWord = PioCommandWord::END;

    const {
        assert!(END.bits() == 0x0000_0001);
        assert!(END.is_end());
        assert!(END.levels().is_none());
        assert!(END.wait_counter().is_none());
    }

    assert_eq!(PioCommandWord::from_bits(0x0000_0001), Ok(END));
}

#[test]
fn has_the_size_and_alignment_of_a_u32() {
    assert_eq!(size_of::<PioCommandWord>(), size_of::<u32>());
    assert_eq!(align_of::<PioCommandWord>(), align_of::<u32>());
}
