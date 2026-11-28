use planner::{JointToStepsConverter, PinOperationBatch, TimedCoordinatedJointPathProfiles};

use crate::{
    ExecutorError, PioCommandWord, PioExecutionConfig, build_pio_commands, pin_frame::STEP_MASK,
    pio_frame_cycles,
};

/// Secuencia PIO verificada, incluido END, que toma prestado el almacenamiento del llamador.
#[derive(Debug)]
pub struct PreparedPioTrajectory<'a> {
    commands: &'a [PioCommandWord],
    initial_levels: u8,
    final_levels: u8,
    frame_cycles: u64,
    pio_frequency_hz: u64,
}

impl PreparedPioTrajectory<'_> {
    pub fn commands(&self) -> &[PioCommandWord] {
        self.commands
    }

    pub fn len(&self) -> usize {
        self.commands.len()
    }

    pub fn is_empty(&self) -> bool {
        self.commands.is_empty()
    }

    pub fn initial_levels(&self) -> u8 {
        self.initial_levels
    }

    pub fn final_levels(&self) -> u8 {
        self.final_levels
    }

    pub fn frame_cycles(&self) -> u64 {
        self.frame_cycles
    }

    pub fn pio_frequency_hz(&self) -> u64 {
        self.pio_frequency_hz
    }

    /// Duración acumulada de las tramas PIO. Excluye la precarga, END y la recuperación de DMA.
    pub fn duration_seconds(&self) -> f64 {
        self.frame_cycles as f64 / self.pio_frequency_hz as f64
    }

    pub fn iter(&self) -> impl Iterator<Item = Result<PioCommandWord, ExecutorError>> + '_ {
        self.commands.iter().copied().map(Ok)
    }
}

/// Genera y verifica la trayectoria en una sola pasada.
///
/// La trayectoria preparada solo toma prestado `storage`. Si hay un error,
/// el búfer puede contener palabras parciales que deben descartarse.
pub fn prepare_pio_trajectory<'a>(
    profiles: TimedCoordinatedJointPathProfiles<'_>,
    converter: JointToStepsConverter,
    config: PioExecutionConfig,
    boundaries: &mut [u64],
    operations: &mut [PinOperationBatch],
    storage: &'a mut [PioCommandWord],
) -> Result<PreparedPioTrajectory<'a>, ExecutorError> {
    if storage.is_empty() {
        return Err(ExecutorError::InsufficientCommandBuffer);
    }

    let commands = build_pio_commands(profiles, converter, config, boundaries, operations)?;
    collect_commands(commands, config, storage)
}

fn collect_commands<I>(
    mut commands: I,
    config: PioExecutionConfig,
    storage: &mut [PioCommandWord],
) -> Result<PreparedPioTrajectory<'_>, ExecutorError>
where
    I: Iterator<Item = Result<PioCommandWord, ExecutorError>>,
{
    let initial_levels = config.initial_levels();
    let mut final_levels = initial_levels;
    let mut frame_cycles = 0;
    let mut len = 0;

    while let Some(command) = commands.next() {
        let command = command?;
        let destination = storage
            .get_mut(len)
            .ok_or(ExecutorError::InsufficientCommandBuffer)?;

        if command.is_end() {
            if final_levels & STEP_MASK != 0 {
                return Err(ExecutorError::StepPinsNotLow);
            }
            match commands.next() {
                None => {}
                Some(Ok(_)) => return Err(ExecutorError::UnexpectedCommandAfterEnd),
                Some(Err(error)) => return Err(error),
            }

            *destination = command;
            return Ok(PreparedPioTrajectory {
                commands: &storage[..=len],
                initial_levels,
                final_levels,
                frame_cycles,
                pio_frequency_hz: config.pio_frequency_hz,
            });
        }

        let wait_counter = command
            .wait_counter()
            .ok_or(ExecutorError::InvalidPioCommandWord)?;
        frame_cycles = accumulate_frame_cycles(frame_cycles, wait_counter)?;
        final_levels = command
            .levels()
            .ok_or(ExecutorError::InvalidPioCommandWord)?;

        *destination = command;
        len += 1;
    }

    Err(ExecutorError::MissingEndCommand)
}

fn accumulate_frame_cycles(frame_cycles: u64, wait_counter: u32) -> Result<u64, ExecutorError> {
    frame_cycles
        .checked_add(pio_frame_cycles(wait_counter)?)
        .ok_or(ExecutorError::CycleCountOverflow)
}

#[cfg(test)]
#[path = "../tests/unit/prepared_pio_trajectory.rs"]
mod tests;
