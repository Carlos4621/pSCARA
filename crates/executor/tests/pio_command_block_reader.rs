use core::{cell::Cell, iter};

use executor::{
    ExecutorError, PioCommandBlock, PioCommandBlockReader, PioCommandWord, PioExecutionConfig,
    build_verified_pio_commands,
};
use kinematics::JointAngles;
use planner::{
    CoordinatedJointPathProfiles, JointAccelerationLimits, JointToStepsConverter,
    JointVelocityLimits, PinOperationBatch, StepDirTiming, StepDirection, StepTimeBase,
    TimedCoordinatedJointPathProfiles,
};

const UNWRITTEN: u32 = u32::MAX;

fn frame(wait: u32, levels: u8) -> PioCommandWord {
    PioCommandWord::frame(wait, levels).unwrap()
}

#[test]
fn splits_commands_into_multiple_blocks() {
    let commands = [
        frame(0, 0b0010),
        frame(10, 0b0011),
        frame(10, 0b0010),
        frame(20, 0b0011),
        frame(10, 0b0010),
        PioCommandWord::END,
    ];
    let words = commands.map(|word| word.bits());
    let mut reader = PioCommandBlockReader::new(commands.into_iter().map(Ok));
    let mut buffer = [UNWRITTEN; 4];

    assert!(!reader.is_finished());
    assert_eq!(
        reader.next_block(&mut buffer),
        Ok(Some(PioCommandBlock {
            len: 4,
            is_final: false,
        }))
    );
    assert_eq!(buffer, words[..4]);
    assert!(!reader.is_finished());

    buffer.fill(UNWRITTEN);
    assert_eq!(
        reader.next_block(&mut buffer),
        Ok(Some(PioCommandBlock {
            len: 2,
            is_final: true,
        }))
    );
    assert_eq!(buffer[..2], words[4..]);
    assert_eq!(buffer[2..], [UNWRITTEN; 2]);
    assert!(reader.is_finished());
    assert_eq!(reader.next_block(&mut buffer), Ok(None));
    assert_eq!(reader.next_block(&mut []), Ok(None));
}

#[test]
fn accepts_a_sequence_containing_only_end() {
    let mut reader = PioCommandBlockReader::new(iter::once(Ok(PioCommandWord::END)));
    let mut buffer = [UNWRITTEN; 3];

    assert_eq!(
        reader.next_block(&mut buffer),
        Ok(Some(PioCommandBlock {
            len: 1,
            is_final: true,
        }))
    );
    assert_eq!(buffer, [PioCommandWord::END.bits(), UNWRITTEN, UNWRITTEN]);
    assert!(reader.is_finished());
}

#[test]
fn marks_a_full_block_as_final_if_end_is_in_its_last_slot() {
    let commands = [frame(8, 0b1000), PioCommandWord::END];
    let mut reader = PioCommandBlockReader::new(commands.into_iter().map(Ok));
    let mut buffer = [UNWRITTEN; 2];

    assert_eq!(
        reader.next_block(&mut buffer),
        Ok(Some(PioCommandBlock {
            len: buffer.len(),
            is_final: true,
        }))
    );
    assert_eq!(buffer, commands.map(|word| word.bits()));
    assert!(reader.is_finished());
    assert_eq!(reader.next_block(&mut buffer), Ok(None));
}

#[test]
fn fills_one_word_at_a_time_and_retries_after_an_empty_buffer() {
    let commands = [frame(0, 0b0010), frame(10, 0b1000), PioCommandWord::END];
    let mut source = commands.into_iter().map(Ok);
    let reads = Cell::new(0);
    let mut reader = PioCommandBlockReader::new(iter::from_fn(|| {
        reads.set(reads.get() + 1);
        source.next()
    }));

    assert_eq!(reads.get(), 0);
    for (index, word) in commands.into_iter().enumerate() {
        assert_eq!(
            reader.next_block(&mut []),
            Err(ExecutorError::EmptyCommandBuffer)
        );
        assert!(!reader.is_finished());
        assert_eq!(reads.get(), index);

        let mut buffer = [UNWRITTEN];
        assert_eq!(
            reader.next_block(&mut buffer),
            Ok(Some(PioCommandBlock {
                len: 1,
                is_final: word.is_end(),
            }))
        );
        assert_eq!(buffer, [word.bits()]);
        assert_eq!(reads.get(), index + 1 + usize::from(word.is_end()));
    }
    assert!(reader.is_finished());
    assert_eq!(reader.next_block(&mut []), Ok(None));
    assert_eq!(reads.get(), commands.len() + 1);
}

#[test]
fn reports_missing_end_for_an_empty_source() {
    let mut reader = PioCommandBlockReader::new(iter::empty());
    let mut buffer = [UNWRITTEN; 2];

    assert_eq!(
        reader.next_block(&mut buffer),
        Err(ExecutorError::MissingEndCommand)
    );
    assert_eq!(buffer, [UNWRITTEN; 2]);
    assert!(reader.is_finished());
    assert_eq!(reader.next_block(&mut buffer), Ok(None));
}

#[test]
fn reports_missing_end_when_the_last_block_is_partial() {
    let word = frame(10, 0b1000);
    let mut reader = PioCommandBlockReader::new(iter::once(Ok(word)));
    let mut buffer = [UNWRITTEN; 2];

    assert_eq!(
        reader.next_block(&mut buffer),
        Err(ExecutorError::MissingEndCommand)
    );
    assert_eq!(buffer, [word.bits(), UNWRITTEN]);
    assert!(reader.is_finished());

    buffer.fill(UNWRITTEN);
    assert_eq!(reader.next_block(&mut buffer), Ok(None));
    assert_eq!(buffer, [UNWRITTEN; 2]);
}

#[test]
fn reports_missing_end_on_the_call_after_an_intermediate_block() {
    let commands = [frame(0, 0b0010), frame(12, 0b1000)];
    let mut reader = PioCommandBlockReader::new(commands.into_iter().map(Ok));
    let mut buffer = [UNWRITTEN; 2];

    assert_eq!(
        reader.next_block(&mut buffer),
        Ok(Some(PioCommandBlock {
            len: 2,
            is_final: false,
        }))
    );
    assert_eq!(buffer, commands.map(|word| word.bits()));
    assert!(!reader.is_finished());

    buffer.fill(UNWRITTEN);
    assert_eq!(
        reader.next_block(&mut buffer),
        Err(ExecutorError::MissingEndCommand)
    );
    assert_eq!(buffer, [UNWRITTEN; 2]);
    assert!(reader.is_finished());
    assert_eq!(reader.next_block(&mut []), Ok(None));
}

#[test]
fn rejects_a_second_end_or_a_frame_after_end() {
    for extra in [PioCommandWord::END, frame(10, 0b0010)] {
        let commands = [PioCommandWord::END, extra, PioCommandWord::END];
        let reads = Cell::new(0);
        let mut reader = PioCommandBlockReader::new(commands.into_iter().map(Ok).inspect(|_| {
            reads.set(reads.get() + 1);
        }));
        let mut buffer = [UNWRITTEN; 2];

        assert_eq!(
            reader.next_block(&mut buffer),
            Err(ExecutorError::UnexpectedCommandAfterEnd)
        );
        assert_eq!(buffer, [PioCommandWord::END.bits(), UNWRITTEN]);
        assert!(reader.is_finished());
        assert_eq!(reads.get(), 2);
        assert_eq!(reader.next_block(&mut buffer), Ok(None));
        assert_eq!(reads.get(), 2);
    }
}

#[test]
fn propagates_source_errors_before_a_frame_after_a_frame_and_after_end() {
    let error = ExecutorError::DelayTooShort;
    let before_error_cases = [
        &[][..],
        &[frame(10, 0b0010)][..],
        &[PioCommandWord::END][..],
    ];
    for before_error in before_error_cases {
        let mut reader = PioCommandBlockReader::new(
            before_error
                .iter()
                .copied()
                .map(Ok)
                .chain([Err(error), Ok(PioCommandWord::END)]),
        );
        let mut buffer = [UNWRITTEN; 3];

        assert_eq!(reader.next_block(&mut buffer), Err(error));
        assert!(reader.is_finished());
        for (word, expected) in buffer.iter().zip(before_error) {
            assert_eq!(*word, expected.bits());
        }
        assert!(
            buffer[before_error.len()..]
                .iter()
                .all(|&word| word == UNWRITTEN)
        );
        assert_eq!(reader.next_block(&mut buffer), Ok(None));
    }
}

#[test]
fn finished_reader_does_not_consult_a_non_fused_source_again() {
    let reads = Cell::new(0);
    let mut reader = PioCommandBlockReader::new(iter::from_fn(|| {
        let read = reads.get();
        reads.set(read + 1);
        match read {
            0 => Some(Ok(PioCommandWord::END)),
            1 => None,
            _ => Some(Err(ExecutorError::CycleCountOverflow)),
        }
    }));
    let mut buffer = [UNWRITTEN];

    assert!(reader.next_block(&mut buffer).unwrap().unwrap().is_final);
    assert!(reader.is_finished());
    assert_eq!(reads.get(), 2);

    buffer.fill(UNWRITTEN);
    assert_eq!(reader.next_block(&mut buffer), Ok(None));
    assert_eq!(reader.next_block(&mut []), Ok(None));
    assert_eq!(buffer, [UNWRITTEN]);
    assert_eq!(reads.get(), 2);
}

#[test]
fn reconstructs_identical_words_across_block_capacities() {
    let commands = [
        frame(0, 0),
        frame(1, 0b1111),
        frame(PioCommandWord::MAX_WAIT_COUNTER, 0b0101),
        frame(5, 0b1010),
        frame(125, 0b0100),
        frame(10, 0b1000),
        PioCommandWord::END,
    ];
    let words = commands.map(|word| word.bits());
    let mut output = [UNWRITTEN; 12];
    for capacity in 1..=output.len() {
        let mut reader = PioCommandBlockReader::new(commands.into_iter().map(Ok));
        let mut reconstructed = words.map(|_| UNWRITTEN);
        let mut written = 0;
        while let Some(block) = reader.next_block(&mut output[..capacity]).unwrap() {
            let remaining = words.len() - written;
            assert_eq!(block.len, remaining.min(capacity));
            assert_eq!(block.is_final, remaining <= capacity);
            assert_eq!(reader.is_finished(), block.is_final);
            reconstructed[written..written + block.len].copy_from_slice(&output[..block.len]);
            written += block.len;
        }
        assert_eq!(written, words.len());
        assert_eq!(reconstructed, words);
    }
}

#[test]
fn preserves_all_twelve_words_of_firmware_trajectory_a() {
    let points = [[0.0, 0.0], [0.25, -0.25], [0.50, -0.50]]
        .map(|[first, second]| JointAngles::new(first, second).unwrap());
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let path =
        CoordinatedJointPathProfiles::new(&points, &[0.0; 3], &velocity, &acceleration).unwrap();
    let profiles = TimedCoordinatedJointPathProfiles::new(path).unwrap();
    let converter = JointToStepsConverter::new(8.0, 4.0).unwrap();
    let config = PioExecutionConfig {
        timebase: StepTimeBase::new(8).unwrap(),
        timing: StepDirTiming::new(125_000_000, 125_000_000, 1, 1).unwrap(),
        pio_frequency_hz: 1_000_000,
        initial_first_direction: StepDirection::Negative,
        initial_second_direction: StepDirection::Positive,
        first_positive_is_high: true,
        second_positive_is_high: true,
    };
    let mut boundaries = [0; 3];
    let mut operations = [PinOperationBatch::new(0); 8];
    let reference = build_verified_pio_commands(
        profiles,
        converter,
        config,
        &mut boundaries,
        &mut operations,
    )
    .unwrap()
    .map(|word| word.unwrap().bits())
    .collect::<Vec<_>>();
    assert_eq!(reference.len(), 12);
    assert_eq!(reference.last(), Some(&PioCommandWord::END.bits()));

    let commands = build_verified_pio_commands(
        profiles,
        converter,
        config,
        &mut boundaries,
        &mut operations,
    )
    .unwrap();
    let mut reader = PioCommandBlockReader::new(commands);
    let mut reconstructed = [[UNWRITTEN; 4]; 3];
    for (output, is_final) in reconstructed.iter_mut().zip([false, false, true]) {
        assert_eq!(
            reader.next_block(output),
            Ok(Some(PioCommandBlock { len: 4, is_final }))
        );
    }
    assert_eq!(reader.next_block(&mut [0; 4]), Ok(None));
    assert_eq!(reconstructed.as_flattened(), reference);
}
