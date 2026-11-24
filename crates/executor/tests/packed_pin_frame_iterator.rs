use core::{
    cell::Cell,
    iter::{FusedIterator, empty, from_fn},
};

use executor::{
    ExecutorError, PackedPinFrame, PackedPinFrameIterator, PinFrame, PinFrameEncoder,
    PinFrameIterator,
};
use planner::{
    PinOperationBatch, PlannerError,
    StepDirection::{Negative, Positive},
    StepPinLevel::{High, Low},
};

const MAX: u64 = PackedPinFrame::MAX_DELAY_TICKS;

fn frame(delay_ticks: u64, levels: u8) -> PinFrame {
    PinFrame {
        delay_ticks,
        levels,
    }
}

fn packed(delay_ticks: u64, levels: u8) -> PackedPinFrame {
    PackedPinFrame::try_from(frame(delay_ticks, levels)).unwrap()
}

#[test]
fn an_empty_source_yields_none() {
    let mut frames = PackedPinFrameIterator::new(empty(), 0).unwrap();

    assert_eq!(frames.next(), None);
    assert_eq!(frames.next(), None);
}

#[test]
fn rejects_invalid_initial_levels_without_polling_the_source() {
    for initial_levels in 16..=u8::MAX {
        let source = from_fn(|| panic!("se consultó la fuente durante la construcción"));

        assert_eq!(
            PackedPinFrameIterator::new(source, initial_levels).err(),
            Some(ExecutorError::InvalidPinLevels)
        );
    }
}

#[test]
fn packs_frames_that_fit_in_one_word() {
    let frames = [frame(1, 0), frame(MAX - 1, 0b1101), frame(MAX, 0b1111)];
    let source = frames.map(Ok).into_iter();
    let expected = frames.map(|frame| PackedPinFrame::try_from(frame).unwrap());

    let actual = PackedPinFrameIterator::new(source, 0b1111)
        .unwrap()
        .collect::<Result<Vec<_>, _>>()
        .unwrap();

    assert_eq!(actual, expected);
}

#[test]
fn splits_a_long_delay_and_applies_the_step_at_its_end() {
    let source = [
        frame(10, 0b0010),
        frame(MAX + 100, 0b0011),
        frame(20, 0b0010),
    ]
    .map(Ok)
    .into_iter();

    let actual = PackedPinFrameIterator::new(source, 0)
        .unwrap()
        .collect::<Result<Vec<_>, _>>()
        .unwrap();

    assert_eq!(
        actual,
        [
            packed(10, 0b0010),
            packed(MAX, 0b0010),
            packed(100, 0b0011),
            packed(20, 0b0010)
        ]
    );
}

#[test]
fn keeps_the_previous_levels_across_consecutive_split_frames() {
    let source = [
        frame(2 * MAX, 0b0010),
        frame(2 * MAX + 1, 0b1011),
        frame(MAX + 1, 0b1001),
    ]
    .map(Ok)
    .into_iter();

    let actual = PackedPinFrameIterator::new(source, 0b1100)
        .unwrap()
        .collect::<Result<Vec<_>, _>>()
        .unwrap();

    assert_eq!(
        actual,
        [
            packed(MAX, 0b1100),
            packed(MAX, 0b0010),
            packed(MAX, 0b0010),
            packed(MAX, 0b0010),
            packed(1, 0b1011),
            packed(MAX, 0b1011),
            packed(1, 0b1001),
        ]
    );
}

#[test]
fn polls_the_source_only_after_delivering_every_word_of_the_active_frame() {
    let mut source = [frame(2 * MAX + 1, 0b0011), frame(9, 0b1010)]
        .map(Ok)
        .into_iter();
    let polls = Cell::new(0);
    let source = from_fn(|| {
        polls.set(polls.get() + 1);
        source.next()
    });
    let mut frames = PackedPinFrameIterator::new(source, 0b0010).unwrap();

    assert_eq!(polls.get(), 0);
    for expected in [packed(MAX, 0b0010), packed(MAX, 0b0010), packed(1, 0b0011)] {
        assert_eq!(frames.next(), Some(Ok(expected)));
        assert_eq!(polls.get(), 1);
    }
    assert_eq!(frames.next(), Some(Ok(packed(9, 0b1010))));
    assert_eq!(polls.get(), 2);
    assert_eq!(frames.next(), None);
    assert_eq!(polls.get(), 3);
    assert_eq!(frames.next(), None);
    assert_eq!(polls.get(), 3);
}

#[test]
fn rejects_invalid_frame_levels_and_never_polls_again() {
    let source = [frame(10, 0b0010), frame(MAX + 1, 0b1_0011)]
        .map(Ok)
        .into_iter()
        .chain(from_fn(|| {
            panic!("se consultó la fuente después del error")
        }));
    let mut frames = PackedPinFrameIterator::new(source, 0).unwrap();

    assert_eq!(frames.next(), Some(Ok(packed(10, 0b0010))));
    assert_eq!(frames.next(), Some(Err(ExecutorError::InvalidPinLevels)));
    assert_eq!(frames.next(), None);
    assert_eq!(frames.next(), None);
}

#[test]
fn wraps_the_planner_error_and_never_polls_again() {
    let error = PlannerError::InsufficientPinOperationBuffer;
    let source = [Ok(frame(MAX + 1, 0b0011)), Err(error)]
        .into_iter()
        .chain(from_fn(|| {
            panic!("se consultó la fuente después del error")
        }));
    let mut frames = PackedPinFrameIterator::new(source, 0b0010).unwrap();

    assert_eq!(frames.next(), Some(Ok(packed(MAX, 0b0010))));
    assert_eq!(frames.next(), Some(Ok(packed(1, 0b0011))));
    assert_eq!(frames.next(), Some(Err(ExecutorError::Planner(error))));
    assert_eq!(frames.next(), None);
    assert_eq!(frames.next(), None);
}

#[test]
fn preserves_an_initial_zero_delay_frame() {
    let source = [Ok(frame(0, 0b0010)), Ok(frame(1, 0b0011))].into_iter();
    let mut frames = PackedPinFrameIterator::new(source, 0).unwrap();

    assert_eq!(frames.next(), Some(Ok(packed(0, 0b0010))));
    assert_eq!(frames.next(), Some(Ok(packed(1, 0b0011))));
    assert_eq!(frames.next(), None);
}

#[test]
fn remains_exhausted_even_if_the_source_could_resume() {
    let mut items = [
        Some(Ok(frame(10, 0b0010))),
        None,
        Some(Ok(frame(1, 0b0011))),
    ]
    .into_iter();
    let mut frames = PackedPinFrameIterator::new(from_fn(|| items.next().flatten()), 0).unwrap();
    let fused: &mut dyn FusedIterator<Item = Result<PackedPinFrame, ExecutorError>> = &mut frames;

    assert_eq!(fused.next(), Some(Ok(packed(10, 0b0010))));
    assert_eq!(fused.next(), None);
    assert_eq!(fused.next(), None);
    assert_eq!(items.next(), Some(Some(Ok(frame(1, 0b0011)))));
}

#[test]
fn packs_pin_operation_batches_through_pin_frame_iterator() {
    let batches = [
        PinOperationBatch {
            first_direction: Some(Positive),
            ..PinOperationBatch::new(0)
        },
        PinOperationBatch {
            first_step: Some(High),
            ..PinOperationBatch::new(MAX + 100)
        },
        PinOperationBatch {
            first_step: Some(Low),
            ..PinOperationBatch::new(MAX + 120)
        },
    ];
    let encoder = PinFrameEncoder::new(Negative, Positive, true, true);
    let initial_levels = encoder.levels();
    let source = PinFrameIterator::new(batches.map(Ok).into_iter(), encoder);

    let actual = PackedPinFrameIterator::new(source, initial_levels)
        .unwrap()
        .collect::<Result<Vec<_>, _>>()
        .unwrap();

    assert_eq!(
        actual,
        [
            packed(0, 0b1010),
            packed(MAX, 0b1010),
            packed(100, 0b1011),
            packed(20, 0b1010)
        ]
    );
}
