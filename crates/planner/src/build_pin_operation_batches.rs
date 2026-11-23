use crate::{
    JointToStepsConverter, PinOperationBatch, PinOperationBatchIterator, PlannerError,
    StepDirScheduleCursor, StepDirScheduleIterator, StepDirTiming, StepDirection, StepTimeBase,
    TimedCoordinatedJointPathProfiles, build_trajectory_step_events, into_step_dir_events,
};

/// Construye la secuencia de operaciones de pines bajo demanda en los búferes prestados.
#[allow(
    clippy::too_many_arguments,
    reason = "La API recibe dos direcciones iniciales y dos búferes independientes"
)]
pub fn build_pin_operation_batches<'a>(
    profiles: TimedCoordinatedJointPathProfiles<'a>,
    converter: JointToStepsConverter,
    timebase: StepTimeBase,
    timing: StepDirTiming,
    initial_first_direction: Option<StepDirection>,
    initial_second_direction: Option<StepDirection>,
    boundaries_buffer: &'a mut [u64],
    operations_buffer: &'a mut [PinOperationBatch],
) -> Result<impl Iterator<Item = Result<PinOperationBatch, PlannerError>> + 'a, PlannerError> {
    let step_high_ticks = timing.min_step_high_ticks(timebase)?;
    let dir_setup_ticks = timing.min_dir_setup_ticks(timebase)?;
    let events = build_trajectory_step_events(profiles, converter, timebase, boundaries_buffer)?;
    let events = into_step_dir_events(events);
    let cursor = StepDirScheduleCursor::new(
        timing,
        timebase,
        initial_first_direction,
        initial_second_direction,
    )?;
    let scheduled = StepDirScheduleIterator::new(events, cursor);

    PinOperationBatchIterator::new(
        scheduled,
        step_high_ticks,
        dir_setup_ticks,
        operations_buffer,
    )
}

/// Valida toda la secuencia, incluida la capacidad del búfer de operaciones, antes de devolverla.
/// Reconstruye el iterador con la misma configuración y reutiliza ambos búferes.
/// Los búferes pueden contener datos de trabajo si la validación devuelve un error.
#[allow(
    clippy::too_many_arguments,
    reason = "La API recibe dos direcciones iniciales y dos búferes independientes"
)]
pub fn build_verified_pin_operation_batches<'a>(
    profiles: TimedCoordinatedJointPathProfiles<'a>,
    converter: JointToStepsConverter,
    timebase: StepTimeBase,
    timing: StepDirTiming,
    initial_first_direction: Option<StepDirection>,
    initial_second_direction: Option<StepDirection>,
    boundaries_buffer: &'a mut [u64],
    operations_buffer: &'a mut [PinOperationBatch],
) -> Result<impl Iterator<Item = Result<PinOperationBatch, PlannerError>> + 'a, PlannerError> {
    {
        let batches = build_pin_operation_batches(
            profiles,
            converter,
            timebase,
            timing,
            initial_first_direction,
            initial_second_direction,
            boundaries_buffer,
            operations_buffer,
        )?;
        for batch in batches {
            batch?;
        }
    }

    build_pin_operation_batches(
        profiles,
        converter,
        timebase,
        timing,
        initial_first_direction,
        initial_second_direction,
        boundaries_buffer,
        operations_buffer,
    )
}
