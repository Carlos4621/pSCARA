use crate::{
    PlannerError, StepDirEvent, StepDirScheduleCursor, StepDirTiming, StepDirection, StepTimeBase,
};

/// Valida los intervalos STEP discretos y las ventanas de cambio de DIR en un solo recorrido.
/// Los ticks deben ser estrictamente crecientes.
/// En cada evento comprueba los intervalos de ambos motores antes de evaluar DIR.
/// Las direcciones iniciales conocidas se consideran estables desde el tick cero.
pub fn validate_step_dir_schedule<I>(
    events: I,
    timing: StepDirTiming,
    timebase: StepTimeBase,
    initial_first_direction: Option<StepDirection>,
    initial_second_direction: Option<StepDirection>,
) -> Result<(), PlannerError>
where
    I: IntoIterator<Item = Result<StepDirEvent, PlannerError>>,
{
    let mut cursor = StepDirScheduleCursor::new(
        timing,
        timebase,
        initial_first_direction,
        initial_second_direction,
    )?;

    for event in events {
        cursor.schedule(event?)?;
    }

    Ok(())
}
