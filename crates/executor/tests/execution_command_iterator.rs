use core::{
    cell::Cell,
    iter::{FusedIterator, empty, from_fn},
};

use executor::{
    ExecutionCommand::{self, End, Frame},
    ExecutionCommandIterator, ExecutorError, PackedPinFrame, PackedPinFrameIterator, PinFrame,
};

fn packed(delay_ticks: u64, levels: u8) -> PackedPinFrame {
    PackedPinFrame::try_from(PinFrame {
        delay_ticks,
        levels,
    })
    .unwrap()
}

#[test]
fn an_empty_source_yields_end_once_with_any_initial_direction() {
    for levels in [0b0000, 0b0010, 0b1000, 0b1010] {
        let mut commands = ExecutionCommandIterator::new(empty(), levels).unwrap();

        assert_eq!(commands.next(), Some(Ok(End)));
        assert_eq!(commands.next(), None);
        assert_eq!(commands.next(), None);
    }
}

#[test]
fn forwards_all_frames_before_end() {
    let frames = [packed(10, 0b0010), packed(3, 0b0011), packed(2, 0b0010)];
    let mut commands = ExecutionCommandIterator::new(frames.map(Ok).into_iter(), 0).unwrap();

    for frame in frames {
        assert_eq!(commands.next(), Some(Ok(Frame(frame))));
    }
    assert_eq!(commands.next(), Some(Ok(End)));
    assert_eq!(commands.next(), None);
}

#[test]
fn treats_zero_and_all_one_words_as_frames() {
    let zero = PackedPinFrame::from_bits(0);
    let all_ones = PackedPinFrame::from_bits(u32::MAX);
    let source = [Ok(zero), Ok(all_ones), Ok(zero)].into_iter();

    let commands = ExecutionCommandIterator::new(source, 0)
        .unwrap()
        .collect::<Result<Vec<_>, _>>()
        .unwrap();

    assert_eq!(commands, [Frame(zero), Frame(all_ones), Frame(zero), End]);
}

#[test]
fn rejects_invalid_initial_levels_without_polling_the_source() {
    for levels in 16..=u8::MAX {
        let source = from_fn(|| panic!("se consultó la fuente durante la construcción"));

        assert_eq!(
            ExecutionCommandIterator::new(source, levels).err(),
            Some(ExecutorError::InvalidPinLevels)
        );
    }
}

#[test]
fn rejects_either_initial_step_in_high() {
    for directions in [0b0000, 0b0010, 0b1000, 0b1010] {
        for steps in [0b0001, 0b0100, 0b0101] {
            let source = from_fn(|| panic!("se consultó la fuente durante la construcción"));

            assert_eq!(
                ExecutionCommandIterator::new(source, directions | steps).err(),
                Some(ExecutorError::StepPinsNotLow)
            );
        }
    }
}

#[test]
fn reports_an_error_if_either_step_remains_high_at_exhaustion() {
    for levels in [0b0011, 0b0100, 0b1111] {
        let frame = packed(3, levels);
        let mut items = [Some(Ok(frame)), None].into_iter();
        let source = from_fn(|| {
            items
                .next()
                .expect("se consultó la fuente después de agotarla")
        });
        let mut commands = ExecutionCommandIterator::new(source, 0).unwrap();

        assert_eq!(commands.next(), Some(Ok(Frame(frame))));
        assert_eq!(commands.next(), Some(Err(ExecutorError::StepPinsNotLow)));
        assert_eq!(commands.next(), None);
        assert_eq!(commands.next(), None);
    }
}

#[test]
fn allows_both_direction_pins_in_high_at_exhaustion() {
    let frame = packed(3, 0b1010);
    let mut commands = ExecutionCommandIterator::new([Ok(frame)].into_iter(), 0).unwrap();

    assert_eq!(commands.next(), Some(Ok(Frame(frame))));
    assert_eq!(commands.next(), Some(Ok(End)));
    assert_eq!(commands.next(), None);
}

#[test]
fn propagates_source_errors_and_stops_permanently() {
    for levels in [0b0010, 0b0011] {
        let frame = packed(10, levels);
        let error = ExecutorError::DelayOutOfRange;
        let source = [Ok(frame), Err(error)].into_iter().chain(from_fn(|| {
            panic!("se consultó la fuente después del error")
        }));
        let mut commands = ExecutionCommandIterator::new(source, 0).unwrap();

        assert_eq!(commands.next(), Some(Ok(Frame(frame))));
        assert_eq!(commands.next(), Some(Err(error)));
        assert_eq!(commands.next(), None);
        assert_eq!(commands.next(), None);
    }
}

#[test]
fn polls_on_demand_and_stays_finished_even_if_the_source_could_resume() {
    let first = packed(10, 0b1011);
    let second = packed(2, 0b1010);
    let polls = Cell::new(0);
    let source = from_fn(|| {
        let poll = polls.get() + 1;
        polls.set(poll);
        match poll {
            1 => Some(Ok(first)),
            2 => Some(Ok(second)),
            3 => None,
            _ => Some(Ok(first)),
        }
    });
    let mut commands = ExecutionCommandIterator::new(source, 0b1010).unwrap();
    let fused: &mut dyn FusedIterator<Item = Result<ExecutionCommand, ExecutorError>> =
        &mut commands;

    assert_eq!(polls.get(), 0);
    assert_eq!(fused.next(), Some(Ok(Frame(first))));
    assert_eq!(polls.get(), 1);
    assert_eq!(fused.next(), Some(Ok(Frame(second))));
    assert_eq!(polls.get(), 2);
    assert_eq!(fused.next(), Some(Ok(End)));
    assert_eq!(polls.get(), 3);
    assert_eq!(fused.next(), None);
    assert_eq!(fused.next(), None);
    assert_eq!(polls.get(), 3);
}

#[test]
fn emits_end_after_the_last_word_from_packed_pin_frame_iterator() {
    let max = PackedPinFrame::MAX_DELAY_TICKS;
    let initial_levels = 0b0010;
    let frames = [
        PinFrame {
            delay_ticks: 0,
            levels: 0b0011,
        },
        PinFrame {
            delay_ticks: max + 2,
            levels: 0b0010,
        },
    ];
    let source = PackedPinFrameIterator::new(frames.map(Ok).into_iter(), initial_levels).unwrap();
    let mut commands = ExecutionCommandIterator::new(source, initial_levels).unwrap();

    for frame in [packed(0, 0b0011), packed(max, 0b0011), packed(2, 0b0010)] {
        assert_eq!(commands.next(), Some(Ok(Frame(frame))));
    }
    assert_eq!(commands.next(), Some(Ok(End)));
    assert_eq!(commands.next(), None);
}
