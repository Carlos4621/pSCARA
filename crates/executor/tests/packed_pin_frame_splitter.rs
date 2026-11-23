use core::iter::FusedIterator;

use executor::{ExecutorError, PackedPinFrame, PackedPinFrameSplitter, PinFrame};

const MAX: u64 = PackedPinFrame::MAX_DELAY_TICKS;

#[test]
fn splits_at_the_boundaries_and_changes_levels_only_in_the_final_frame() {
    let cases: [(u64, &[u64]); 8] = [
        (0, &[0]),
        (1, &[1]),
        (MAX - 1, &[MAX - 1]),
        (MAX, &[MAX]),
        (MAX + 1, &[MAX, 1]),
        (MAX + 100, &[MAX, 100]),
        (2 * MAX, &[MAX, MAX]),
        (2 * MAX + 1, &[MAX, MAX, 1]),
    ];
    let previous_levels = 0b0010;
    let target_levels = 0b0011;

    for (delay_ticks, expected_ticks) in cases {
        let frame = PinFrame {
            delay_ticks,
            levels: target_levels,
        };
        let mut splitter = PackedPinFrameSplitter::new(previous_levels, frame).unwrap();
        let mut total_ticks = 0;

        for (index, &expected) in expected_ticks.iter().enumerate() {
            let levels = if index + 1 == expected_ticks.len() {
                target_levels
            } else {
                previous_levels
            };
            let packed = splitter.next().unwrap();
            assert_eq!(
                PinFrame::from(packed),
                PinFrame {
                    delay_ticks: expected,
                    levels
                }
            );
            total_ticks += packed.delay_ticks();
        }

        assert_eq!(total_ticks, delay_ticks);
        assert_eq!(splitter.next(), None);
    }
}

#[test]
fn rejects_invalid_levels_in_either_mask() {
    for (previous_levels, levels) in [(16, 0), (0, 16), (u8::MAX, 0), (0, u8::MAX)] {
        let frame = PinFrame {
            delay_ticks: MAX + 1,
            levels,
        };

        assert_eq!(
            PackedPinFrameSplitter::new(previous_levels, frame).err(),
            Some(ExecutorError::InvalidPinLevels)
        );
    }
}

#[test]
fn remains_exhausted_after_the_final_frame() {
    let frame = PinFrame {
        delay_ticks: 0,
        levels: 0b1111,
    };
    let mut splitter = PackedPinFrameSplitter::new(0, frame).unwrap();
    let fused: &mut dyn FusedIterator<Item = PackedPinFrame> = &mut splitter;

    assert_eq!(fused.next(), Some(PackedPinFrame::try_from(frame).unwrap()));
    assert_eq!(fused.next(), None);
    assert_eq!(fused.next(), None);
}

#[test]
fn generates_only_the_requested_prefix_of_a_u64_max_delay() {
    let frame = PinFrame {
        delay_ticks: u64::MAX,
        levels: 0b0011,
    };
    let mut prefix = PackedPinFrameSplitter::new(0b0010, frame).unwrap().take(3);
    let expected = PackedPinFrame::try_from(PinFrame {
        delay_ticks: MAX,
        levels: 0b0010,
    })
    .unwrap();

    for _ in 0..3 {
        assert_eq!(prefix.next(), Some(expected));
    }
    assert_eq!(prefix.next(), None);
}
