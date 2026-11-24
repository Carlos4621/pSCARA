use planner::{
    JointToStepsConverter, PinOperationBatch, TimedCoordinatedJointPathProfiles,
    build_pin_operation_batches,
};

use crate::{
    ExecutionCommandIterator, ExecutorError, PackedPinFrameIterator, PinFrameIterator,
    PioCommandIterator, PioCommandWord, PioExecutionConfig,
};

/// Convierte perfiles articulares en palabras PIO bajo demanda con los búferes prestados.
pub fn build_pio_commands<'a>(
    profiles: TimedCoordinatedJointPathProfiles<'a>,
    converter: JointToStepsConverter,
    config: PioExecutionConfig,
    boundaries_buffer: &'a mut [u64],
    operations_buffer: &'a mut [PinOperationBatch],
) -> Result<impl Iterator<Item = Result<PioCommandWord, ExecutorError>> + 'a, ExecutorError> {
    let batches = build_pin_operation_batches(
        profiles,
        converter,
        config.timebase,
        config.timing,
        Some(config.initial_first_direction),
        Some(config.initial_second_direction),
        boundaries_buffer,
        operations_buffer,
    )?;
    let encoder = config.pin_frame_encoder();
    let initial_levels = encoder.levels();
    let frames = PinFrameIterator::new(batches, encoder);
    let packed_frames = PackedPinFrameIterator::new(frames, initial_levels)?;
    let commands = ExecutionCommandIterator::new(packed_frames, initial_levels)?;

    PioCommandIterator::new(
        commands,
        initial_levels,
        config.timebase,
        config.pio_frequency_hz,
    )
}

/// Verifica toda la secuencia sin almacenar palabras PIO y reconstruye el iterador.
/// Los búferes pueden contener datos de trabajo si la verificación devuelve un error.
pub fn build_verified_pio_commands<'a>(
    profiles: TimedCoordinatedJointPathProfiles<'a>,
    converter: JointToStepsConverter,
    config: PioExecutionConfig,
    boundaries_buffer: &'a mut [u64],
    operations_buffer: &'a mut [PinOperationBatch],
) -> Result<impl Iterator<Item = Result<PioCommandWord, ExecutorError>> + 'a, ExecutorError> {
    {
        let commands = build_pio_commands(
            profiles,
            converter,
            config,
            boundaries_buffer,
            operations_buffer,
        )?;
        for command in commands {
            command?;
        }
    }

    build_pio_commands(
        profiles,
        converter,
        config,
        boundaries_buffer,
        operations_buffer,
    )
}
