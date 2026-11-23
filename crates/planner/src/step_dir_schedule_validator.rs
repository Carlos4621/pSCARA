use crate::{PlannerError, StepDirEvent, StepDirState, StepDirTiming, StepDirection, StepTimeBase};

/// Valida los intervalos STEP y las ventanas de cambio de DIR en un solo recorrido.
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
    let min_interval = timing.min_step_interval_ticks(timebase)?;
    let mut state = StepDirState::new(initial_first_direction, initial_second_direction);
    let mut previous_tick = None;
    let mut previous_first_step = None;
    let mut previous_second_step = None;

    for event in events {
        let event = event?;
        let tick = event.tick;
        if previous_tick.is_some_and(|previous| tick <= previous) {
            return Err(PlannerError::InvalidTicks);
        }

        let command = state.prepare(event);
        let motors = [
            (
                command.first_step,
                command.first_dir_update,
                &mut previous_first_step,
            ),
            (
                command.second_step,
                command.second_dir_update,
                &mut previous_second_step,
            ),
        ];

        if motors.iter().any(|(step, _, previous_step)| {
            *step && previous_step.is_some_and(|previous| tick - previous < min_interval)
        }) {
            return Err(PlannerError::StepIntervalTooShort);
        }

        for (step, dir_update, previous_step) in motors {
            if !step {
                continue;
            }
            if dir_update.is_some() {
                timing.dir_change_window(*previous_step, tick, timebase)?;
            }
            *previous_step = Some(tick);
        }

        previous_tick = Some(tick);
    }

    Ok(())
}
