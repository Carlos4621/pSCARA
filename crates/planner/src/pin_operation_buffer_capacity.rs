use crate::{PlannerError, StepDirTiming, StepTimeBase};

/// Calcula una capacidad suficiente en grupos para [`crate::PinOperationBatchIterator`].
/// Suma los ticks de setup DIR y STEP HIGH y reserva tres grupos para el evento entrante.
/// Devuelve [`PlannerError::NumericalFailure`] si el cálculo o la conversión a `usize` desborda.
pub fn pin_operation_buffer_capacity_bound(
    timing: StepDirTiming,
    timebase: StepTimeBase,
) -> Result<usize, PlannerError> {
    let dir_setup_ticks = timing.min_dir_setup_ticks(timebase)?;
    let step_high_ticks = timing.min_step_high_ticks(timebase)?;
    let capacity = dir_setup_ticks
        .checked_add(step_high_ticks)
        .and_then(|pending| pending.checked_add(3))
        .ok_or(PlannerError::NumericalFailure)?;

    usize::try_from(capacity).map_err(|_| PlannerError::NumericalFailure)
}
