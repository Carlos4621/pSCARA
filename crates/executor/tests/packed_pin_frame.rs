use core::mem::{align_of, size_of};

use executor::{ExecutorError, PackedPinFrame, PinFrame};
use planner::PlannerError;

#[test]
fn packs_the_expected_words_and_round_trips_the_original_frames() {
    let max = PackedPinFrame::MAX_DELAY_TICKS;
    assert_eq!(max, 268_435_455);

    for (delay_ticks, levels, expected_bits) in [
        (0, 0, 0x0000_0000),
        (1, 15, 0x0000_001F),
        (10, 10, 0x0000_00AA),
        (max, 0, 0xFFFF_FFF0),
        (max, 15, 0xFFFF_FFFF),
    ] {
        let frame = PinFrame {
            delay_ticks,
            levels,
        };
        let packed = PackedPinFrame::try_from(frame).unwrap();

        assert_eq!(packed.bits(), expected_bits);
        assert_eq!(packed, PackedPinFrame::from_bits(expected_bits));
        assert_eq!(PinFrame::from(packed), frame);
    }
}

#[test]
fn extracts_delay_and_pin_levels_in_const_contexts() {
    const PACKED: PackedPinFrame = PackedPinFrame::from_bits(0x0123_456A);
    const BITS: u32 = PACKED.bits();
    const DELAY_TICKS: u64 = PACKED.delay_ticks();
    const LEVELS: u8 = PACKED.levels();

    assert_eq!(BITS, 0x0123_456A);
    assert_eq!(DELAY_TICKS, 0x0012_3456);
    assert_eq!(LEVELS, 0b1010);
}

#[test]
fn rejects_values_outside_each_field() {
    for delay_ticks in [PackedPinFrame::MAX_DELAY_TICKS + 1, u64::MAX] {
        assert_eq!(
            PackedPinFrame::try_from(PinFrame {
                delay_ticks,
                levels: 0
            }),
            Err(ExecutorError::DelayOutOfRange)
        );
    }

    for levels in [16, u8::MAX] {
        assert_eq!(
            PackedPinFrame::try_from(PinFrame {
                delay_ticks: 0,
                levels
            }),
            Err(ExecutorError::InvalidPinLevels)
        );
    }
}

#[test]
fn has_the_size_and_alignment_of_a_u32() {
    assert_eq!(size_of::<PackedPinFrame>(), size_of::<u32>());
    assert_eq!(align_of::<PackedPinFrame>(), align_of::<u32>());
}

#[test]
fn wraps_planner_errors_without_changing_the_original_error() {
    let error = PlannerError::InvalidTicks;

    assert_eq!(ExecutorError::from(error), ExecutorError::Planner(error));
}
