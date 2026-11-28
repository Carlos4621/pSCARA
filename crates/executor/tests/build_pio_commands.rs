use executor::{
    ExecutionCommandIterator, ExecutorError, PIO_FRAME_OVERHEAD_CYCLES, PackedPinFrameIterator,
    PinFrameEncoder, PinFrameIterator, PioCommandIterator, PioCommandWord, PioExecutionConfig,
    build_pio_commands, build_verified_pio_commands, pio_frame_cycles, prepare_pio_trajectory,
};
use kinematics::JointAngles;
use planner::{
    CoordinatedJointPathProfiles, CoordinatedJointTrajectory, JointAccelerationLimits,
    JointToStepsConverter, JointVelocityLimits, PinOperationBatch, PinOperationBatchIterator,
    PlannerError, StepDirScheduleCursor, StepDirScheduleIterator, StepDirTiming,
    StepDirection::{Negative, Positive},
    StepTimeBase, TimedCoordinatedJointPathProfiles, TrajectoryStepEventIterator,
    generate_guarded_segment_boundaries, into_step_dir_events, plan_joint_path_speeds,
};

const TWO_MOTOR_PATH: [[f64; 2]; 2] = [[0.0, 0.0], [0.25, -0.25]];
const TWO_MOTOR_MOVEMENT: [(u32, u64); 6] = [
    (0xA000_0028, 20),
    (0x3000_0008, 30),
    (0x6000_0008, 40),
    (0x2000_0008, 50),
    (0x3000_0008, 60),
    (0x2000_0008, 70),
];

fn points<const N: usize>(coordinates: [[f64; 2]; N]) -> [JointAngles; N] {
    coordinates.map(|[first, second]| JointAngles::new(first, second).unwrap())
}

fn limits() -> (JointVelocityLimits, JointAccelerationLimits) {
    (
        JointVelocityLimits::new(10.0, 10.0).unwrap(),
        JointAccelerationLimits::new(1.0, 1.0).unwrap(),
    )
}

fn timed_path<'a>(
    points: &'a [JointAngles],
    speeds: &'a [f64],
    (velocity, acceleration): &'a (JointVelocityLimits, JointAccelerationLimits),
) -> TimedCoordinatedJointPathProfiles<'a> {
    let path = CoordinatedJointPathProfiles::new(points, speeds, velocity, acceleration).unwrap();
    TimedCoordinatedJointPathProfiles::new(path).unwrap()
}

fn two_motor_converter() -> JointToStepsConverter {
    JointToStepsConverter::new(8.0, 4.0).unwrap()
}

fn two_motor_config() -> PioExecutionConfig {
    PioExecutionConfig {
        timebase: StepTimeBase::new(8).unwrap(),
        timing: StepDirTiming::new(125_000_000, 125_000_000, 1, 1).unwrap(),
        pio_frequency_hz: 80,
        initial_first_direction: Negative,
        initial_second_direction: Positive,
        first_positive_is_high: true,
        second_positive_is_high: true,
    }
}

fn pico2_benchmark_config() -> PioExecutionConfig {
    PioExecutionConfig {
        timebase: StepTimeBase::new(100_000).unwrap(),
        timing: StepDirTiming::new(10_000, 10_000, 10_000, 10_000).unwrap(),
        pio_frequency_hz: 1_000_000,
        ..two_motor_config()
    }
}

fn uncached_binary_words(
    profiles: TimedCoordinatedJointPathProfiles<'_>,
    converter: JointToStepsConverter,
    config: PioExecutionConfig,
) -> Result<Vec<u32>, ExecutorError> {
    let trajectory = CoordinatedJointTrajectory::new(profiles)?;
    let position_at_tick = |tick| {
        let time = config.timebase.ticks_to_seconds(tick);
        converter.convert(trajectory.position_at_clamped(time)?)
    };
    let mut boundaries = [0; 8];
    let mut operations = [PinOperationBatch::new(0); 64];
    let count = generate_guarded_segment_boundaries(profiles, config.timebase, &mut boundaries)?;
    let mut events = Vec::new();
    // El iterador de un solo intervalo conserva la búsqueda binaria.
    for pair in boundaries[..count].windows(2) {
        let start = pair[0];
        events.extend(TrajectoryStepEventIterator::new(
            start,
            pair[1],
            position_at_tick(start)?,
            position_at_tick,
        )?);
    }
    let cursor = StepDirScheduleCursor::new(
        config.timing,
        config.timebase,
        Some(config.initial_first_direction),
        Some(config.initial_second_direction),
    )?;
    let batches = PinOperationBatchIterator::new(
        StepDirScheduleIterator::new(into_step_dir_events(events.into_iter()), cursor),
        config.timing.min_step_high_ticks(config.timebase)?,
        config.timing.min_dir_setup_ticks(config.timebase)?,
        &mut operations,
    )?;
    let encoder = PinFrameEncoder::new(
        config.initial_first_direction,
        config.initial_second_direction,
        config.first_positive_is_high,
        config.second_positive_is_high,
    );
    let levels = encoder.levels();
    let frames = PinFrameIterator::new(batches, encoder);
    let packed = PackedPinFrameIterator::new(frames, levels)?;
    let commands = ExecutionCommandIterator::new(packed, levels)?;
    PioCommandIterator::new(commands, levels, config.timebase, config.pio_frequency_hz)?
        .map(|command| command.map(|word| word.bits()))
        .collect()
}

fn assert_same_pio_words(
    profiles: TimedCoordinatedJointPathProfiles<'_>,
    converter: JointToStepsConverter,
    config: PioExecutionConfig,
    case: &str,
) -> Result<usize, ExecutorError> {
    let expected = uncached_binary_words(profiles, converter, config)?;
    let mut boundaries = [0; 8];
    let mut operations = [PinOperationBatch::new(0); 64];
    let commands = build_pio_commands(
        profiles,
        converter,
        config,
        &mut boundaries,
        &mut operations,
    )?;
    let actual = commands
        .map(|command| command.map(|word| word.bits()))
        .collect::<Result<Vec<_>, _>>()?;
    assert_eq!(actual.len(), expected.len(), "{case}");
    for (index, (&actual, &expected)) in actual.iter().zip(&expected).enumerate() {
        assert_eq!(
            actual, expected,
            "{case}, word {index}: {actual:#010x} vs {expected:#010x}"
        );
    }
    assert_eq!(
        actual.last().copied(),
        Some(PioCommandWord::END.bits()),
        "{case}"
    );

    let mut storage = vec![PioCommandWord::END; actual.len()];
    let prepared = prepare_pio_trajectory(
        profiles,
        converter,
        config,
        &mut boundaries,
        &mut operations,
        &mut storage,
    )?;
    assert!(
        prepared
            .commands()
            .iter()
            .map(PioCommandWord::bits)
            .eq(actual.iter().copied()),
        "{case}: las palabras preparadas difieren",
    );
    Ok(actual.len())
}

#[test]
fn optimized_sampling_preserves_every_pio_word_of_the_pico2_benchmark() -> Result<(), ExecutorError>
{
    let points = points([[0.0, 0.0], [1.0, -0.75]]);
    let steps_per_radian = (200.0 * 16.0 * 4.0) / core::f64::consts::TAU;
    let converter = JointToStepsConverter::new(steps_per_radian, steps_per_radian)?;
    let config = pico2_benchmark_config();
    for speed in [0.5, 1.0, 2.0, 4.0] {
        let limits = (
            JointVelocityLimits::new(speed, speed)?,
            JointAccelerationLimits::new(10.0, 10.0)?,
        );
        let profiles = timed_path(&points, &[0.0; 2], &limits);
        assert_eq!(
            assert_same_pio_words(profiles, converter, config, &format!("{speed} rad/s"))?,
            7133
        );
    }
    Ok(())
}

#[test]
fn optimized_sampling_preserves_pio_words_through_moving_junctions_and_reversals()
-> Result<(), ExecutorError> {
    let points = points([[0.0, 0.0], [0.25, -0.125], [0.5, -0.25], [0.25, 0.125]]);
    let limits = limits();
    let mut speeds = [0.0; 4];
    plan_joint_path_speeds(&points, &limits.0, &limits.1, &mut speeds)?;
    assert!(speeds[1] > 0.0);
    assert_eq!(speeds[2], 0.0);
    let profiles = timed_path(&points, &speeds, &limits);
    assert_same_pio_words(
        profiles,
        two_motor_converter(),
        pico2_benchmark_config(),
        "multi-segment path",
    )?;
    Ok(())
}

#[test]
fn both_builders_preserve_the_exact_words_and_timing_of_the_two_motor_path()
-> Result<(), ExecutorError> {
    let points = points(TWO_MOTOR_PATH);
    let limits = limits();
    let profiles = timed_path(&points, &[0.0; 2], &limits);
    let converter = two_motor_converter();
    let config = two_motor_config();
    let mut boundaries = [u64::MAX; 2];
    let mut operations = [PinOperationBatch::new(u64::MAX); 4];

    assert_eq!(config.initial_levels(), 0b1000);
    let commands = build_pio_commands(
        profiles,
        converter,
        config,
        &mut boundaries,
        &mut operations,
    )?
    .collect::<Result<Vec<_>, _>>()?;
    let verified = build_verified_pio_commands(
        profiles,
        converter,
        config,
        &mut boundaries,
        &mut operations,
    )?
    .collect::<Result<Vec<_>, _>>()?;

    assert_eq!(commands, verified);
    let (end, movement) = commands.split_last().unwrap();
    assert_eq!(*end, PioCommandWord::END);
    assert_eq!(end.bits(), 0x0000_0001);
    assert_eq!(movement.len(), TWO_MOTOR_MOVEMENT.len());
    let mut physical_cycles = 0;
    for (command, (bits, logical_cycles)) in movement.iter().zip(TWO_MOTOR_MOVEMENT) {
        assert_eq!(command.bits(), bits);
        physical_cycles += pio_frame_cycles(command.wait_counter().unwrap())?;
        assert_eq!(physical_cycles - PIO_FRAME_OVERHEAD_CYCLES, logical_cycles);
    }
    assert_eq!(boundaries, [0, 8]);
    Ok(())
}

#[test]
fn prepared_trajectory_matches_the_generator_and_only_borrows_word_storage()
-> Result<(), ExecutorError> {
    let mut storage = [PioCommandWord::END; TWO_MOTOR_MOVEMENT.len() + 1];
    let prepared = {
        let points = points(TWO_MOTOR_PATH);
        let limits = limits();
        let speeds = [0.0; 2];
        let profiles = timed_path(&points, &speeds, &limits);
        let mut boundaries = [0; 2];
        let mut operations = [PinOperationBatch::new(0); 4];
        let config = two_motor_config();
        let converter = two_motor_converter();
        let prepared = prepare_pio_trajectory(
            profiles,
            converter,
            config,
            &mut boundaries,
            &mut operations,
            &mut storage,
        )?;
        let expected = build_pio_commands(
            profiles,
            converter,
            config,
            &mut boundaries,
            &mut operations,
        )?;
        assert!(prepared.iter().eq(expected));
        prepared
    };

    assert_eq!(prepared.len(), TWO_MOTOR_MOVEMENT.len() + 1);
    assert_eq!(prepared.commands().last(), Some(&PioCommandWord::END));
    assert_eq!(prepared.initial_levels(), 0b1000);
    assert_eq!(prepared.final_levels(), 0b0010);
    assert_eq!(prepared.frame_cycles(), 70 + PIO_FRAME_OVERHEAD_CYCLES);
    assert_eq!(prepared.duration_seconds(), 0.95);
    Ok(())
}

#[test]
fn preparing_words_distinguishes_capacity_errors_from_planner_errors() {
    let points = points(TWO_MOTOR_PATH);
    let limits = limits();
    let profiles = timed_path(&points, &[0.0; 2], &limits);
    let converter = two_motor_converter();
    let config = two_motor_config();
    let mut boundaries = [0; 2];
    let mut operations = [PinOperationBatch::new(0); 4];
    let mut storage = [PioCommandWord::END; TWO_MOTOR_MOVEMENT.len()];

    let error = prepare_pio_trajectory(
        profiles,
        converter,
        config,
        &mut boundaries,
        &mut operations,
        &mut storage,
    )
    .unwrap_err();
    assert_eq!(error, ExecutorError::InsufficientCommandBuffer);

    let error = prepare_pio_trajectory(
        profiles,
        converter,
        config,
        &mut [],
        &mut operations,
        &mut [],
    )
    .unwrap_err();
    assert_eq!(error, ExecutorError::InsufficientCommandBuffer);

    let error = prepare_pio_trajectory(
        profiles,
        converter,
        config,
        &mut [],
        &mut operations,
        &mut storage,
    )
    .unwrap_err();
    assert_eq!(
        error,
        ExecutorError::Planner(PlannerError::InsufficientBoundaryBuffer)
    );
}

#[test]
fn emits_only_end_for_a_trajectory_without_steps_and_accepts_an_empty_operations_buffer()
-> Result<(), ExecutorError> {
    let points = points(TWO_MOTOR_PATH);
    let limits = limits();
    let profiles = timed_path(&points, &[0.0; 2], &limits);
    let converter = JointToStepsConverter::new(1.0, 1.0)?;
    let mut boundaries = [0; 2];
    let mut commands = build_verified_pio_commands(
        profiles,
        converter,
        two_motor_config(),
        &mut boundaries,
        &mut [],
    )?;

    assert_eq!(commands.next(), Some(Ok(PioCommandWord::END)));
    assert_eq!(commands.next(), None);
    Ok(())
}

#[test]
fn rejects_incompatible_pio_frequencies_during_construction() {
    let points = points(TWO_MOTOR_PATH);
    let limits = limits();
    let profiles = timed_path(&points, &[0.0; 2], &limits);
    let mut boundaries = [0; 2];
    let mut operations = [PinOperationBatch::new(0); 4];

    for pio_frequency_hz in [0, 40, 81] {
        let config = PioExecutionConfig {
            pio_frequency_hz,
            ..two_motor_config()
        };
        let commands = build_pio_commands(
            profiles,
            two_motor_converter(),
            config,
            &mut boundaries,
            &mut operations,
        );
        assert_eq!(commands.err(), Some(ExecutorError::InvalidPioClockRatio));
    }
}

#[test]
fn verifies_a_late_operations_buffer_overflow_before_returning_an_iterator()
-> Result<(), ExecutorError> {
    let points = points(TWO_MOTOR_PATH);
    let limits = limits();
    let profiles = timed_path(&points, &[0.0; 2], &limits);
    let converter = two_motor_converter();
    let config = two_motor_config();
    let mut boundaries = [0; 2];
    let mut operations = [PinOperationBatch::new(0); 3];
    let error = ExecutorError::Planner(PlannerError::InsufficientPinOperationBuffer);

    {
        let mut commands = build_pio_commands(
            profiles,
            converter,
            config,
            &mut boundaries,
            &mut operations,
        )?;
        for (bits, _) in TWO_MOTOR_MOVEMENT[..2].iter().copied() {
            assert_eq!(commands.next(), Some(Ok(PioCommandWord::from_bits(bits)?)));
        }
        assert_eq!(commands.next(), Some(Err(error)));
        assert_eq!(commands.next(), None);
    }

    let verified = build_verified_pio_commands(
        profiles,
        converter,
        config,
        &mut boundaries,
        &mut operations,
    );
    assert_eq!(verified.err(), Some(error));
    Ok(())
}

#[test]
fn verifies_step_intervals_before_returning_an_iterator() -> Result<(), ExecutorError> {
    let points = points(TWO_MOTOR_PATH);
    let limits = limits();
    let profiles = timed_path(&points, &[0.0; 2], &limits);
    let config = PioExecutionConfig {
        timing: StepDirTiming::new(250_000_000, 250_000_000, 1, 1)?,
        ..two_motor_config()
    };
    let mut boundaries = [0; 2];
    let mut operations = [PinOperationBatch::new(0); 8];

    let verified = build_verified_pio_commands(
        profiles,
        two_motor_converter(),
        config,
        &mut boundaries,
        &mut operations,
    );
    assert_eq!(
        verified.err(),
        Some(ExecutorError::Planner(PlannerError::StepIntervalTooShort))
    );
    Ok(())
}

#[test]
fn verifies_an_unschedulable_direction_reversal_before_returning_an_iterator()
-> Result<(), ExecutorError> {
    let points = points([[0.0, 0.0], [25.0 / 16.0, 0.0], [1.0, 0.0]]);
    let limits = limits();
    let profiles = timed_path(&points, &[0.0; 3], &limits);
    let converter = JointToStepsConverter::new(2.0, 2.0)?;
    let config = PioExecutionConfig {
        timebase: StepTimeBase::new(6)?,
        timing: StepDirTiming::new(1, 1, 1_000_000_000, 600_000_000)?,
        pio_frequency_hz: 60,
        initial_first_direction: Positive,
        ..two_motor_config()
    };
    let mut boundaries = [0; 3];
    let mut operations = [PinOperationBatch::new(0); 8];

    let verified = build_verified_pio_commands(
        profiles,
        converter,
        config,
        &mut boundaries,
        &mut operations,
    );
    assert_eq!(
        verified.err(),
        Some(ExecutorError::Planner(
            PlannerError::DirChangeNotSchedulable
        ))
    );
    Ok(())
}

#[test]
fn changes_only_the_corresponding_dir_bits_for_inverted_polarities() -> Result<(), ExecutorError> {
    let points = points(TWO_MOTOR_PATH);
    let limits = limits();
    let profiles = timed_path(&points, &[0.0; 2], &limits);
    let converter = two_motor_converter();
    let mut boundaries = [0; 2];
    let mut operations = [PinOperationBatch::new(0); 4];

    for (first_positive_is_high, second_positive_is_high, initial_levels, dir_mask) in [
        (false, true, 0b1010, 0x2000_0000),
        (true, false, 0b0000, 0x8000_0000),
        (false, false, 0b0010, 0xA000_0000),
    ] {
        let config = PioExecutionConfig {
            first_positive_is_high,
            second_positive_is_high,
            ..two_motor_config()
        };
        assert_eq!(config.initial_levels(), initial_levels);
        let mut commands = build_pio_commands(
            profiles,
            converter,
            config,
            &mut boundaries,
            &mut operations,
        )?;

        for (bits, _) in TWO_MOTOR_MOVEMENT {
            assert_eq!(
                commands.next(),
                Some(Ok(PioCommandWord::from_bits(bits ^ dir_mask)?))
            );
        }
        assert_eq!(commands.next(), Some(Ok(PioCommandWord::END)));
        assert_eq!(commands.next(), None);
    }
    Ok(())
}

#[test]
fn verifies_cycle_count_overflow_at_the_first_step_at_tick_two() -> Result<(), ExecutorError> {
    let points = points([[0.0, 0.0], [9.0 / 16.0, 0.0]]);
    let limits = limits();
    let profiles = timed_path(&points, &[0.0; 2], &limits);
    let converter = JointToStepsConverter::new(1.0, 1.0)?;
    let config = PioExecutionConfig {
        timebase: StepTimeBase::new(1)?,
        timing: StepDirTiming::new(1, 1, 1, 1)?,
        pio_frequency_hz: u64::MAX,
        initial_first_direction: Positive,
        ..two_motor_config()
    };
    let mut boundaries = [0; 2];
    let mut operations = [PinOperationBatch::new(0); 3];

    {
        let mut commands = build_pio_commands(
            profiles,
            converter,
            config,
            &mut boundaries,
            &mut operations,
        )?;
        assert_eq!(
            commands.next(),
            Some(Err(ExecutorError::CycleCountOverflow))
        );
        assert_eq!(commands.next(), None);
    }

    let verified = build_verified_pio_commands(
        profiles,
        converter,
        config,
        &mut boundaries,
        &mut operations,
    );
    assert_eq!(verified.err(), Some(ExecutorError::CycleCountOverflow));
    assert_eq!(
        prepare_pio_trajectory(
            profiles,
            converter,
            config,
            &mut boundaries,
            &mut operations,
            &mut [PioCommandWord::END; 1],
        )
        .unwrap_err(),
        ExecutorError::CycleCountOverflow
    );
    Ok(())
}
