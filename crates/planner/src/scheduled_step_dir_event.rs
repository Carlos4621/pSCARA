use crate::{PlannerError, StepDirPinCommand, StepDirTiming, StepDirection, StepTimeBase};

/// Cambio de dirección con su propio tick de ejecución.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScheduledDirChange {
    pub tick: u64,
    pub direction: StepDirection,
}

/// Pulsos STEP de un mismo tick y los cambios de DIR que los preceden.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScheduledStepDirEvent {
    pub step_tick: u64,
    pub first_dir_change: Option<ScheduledDirChange>,
    pub second_dir_change: Option<ScheduledDirChange>,
    pub first_step: bool,
    pub second_step: bool,
}

impl ScheduledStepDirEvent {
    pub(crate) fn step_low_tick(&self, step_high_ticks: u64) -> Result<Option<u64>, PlannerError> {
        if self.first_step || self.second_step {
            return self
                .step_tick
                .checked_add(step_high_ticks)
                .map(Some)
                .ok_or(PlannerError::NumericalFailure);
        }
        Ok(None)
    }
}

/// Programa cada actualización de DIR de un comando en el último tick de su ventana válida.
/// Conserva el tick y los pulsos STEP del comando generado por [`crate::StepDirState::prepare`].
pub fn schedule_step_dir_command(
    command: StepDirPinCommand,
    previous_first_step: Option<u64>,
    previous_second_step: Option<u64>,
    timing: StepDirTiming,
    timebase: StepTimeBase,
) -> Result<ScheduledStepDirEvent, PlannerError> {
    let schedule_dir_change = |dir_update, previous_step| match dir_update {
        Some(direction) => timing
            .dir_change_window(previous_step, command.tick, timebase)
            .map(|(_, tick)| Some(ScheduledDirChange { tick, direction })),
        None => Ok(None),
    };

    Ok(ScheduledStepDirEvent {
        step_tick: command.tick,
        first_dir_change: schedule_dir_change(command.first_dir_update, previous_first_step)?,
        second_dir_change: schedule_dir_change(command.second_dir_update, previous_second_step)?,
        first_step: command.first_step,
        second_step: command.second_step,
    })
}
