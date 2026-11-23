use crate::{PlannerError, StepDirEvent, StepDirState, StepDirTiming, StepDirection, StepTimeBase};

/// Comprueba que cada cambio de DIR tenga una ventana válida entre los pulsos de su motor.
/// Los ticks deben ser estrictamente crecientes y las direcciones iniciales conocidas
/// se consideran estables desde el tick cero. Las desconocidas pueden configurarse
/// a partir de ese tick.
pub fn validate_step_dir_changes<I>(
    events: I,
    timing: StepDirTiming,
    timebase: StepTimeBase,
    initial_first_direction: Option<StepDirection>,
    initial_second_direction: Option<StepDirection>,
) -> Result<(), PlannerError>
where
    I: IntoIterator<Item = Result<StepDirEvent, PlannerError>>,
{
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
        for (dir_update, step, previous_step) in [
            (
                command.first_dir_update,
                command.first_step,
                &mut previous_first_step,
            ),
            (
                command.second_dir_update,
                command.second_step,
                &mut previous_second_step,
            ),
        ] {
            if dir_update.is_some() {
                timing.dir_change_window(*previous_step, tick, timebase)?;
            }
            if step {
                *previous_step = Some(tick);
            }
        }

        previous_tick = Some(tick);
    }

    Ok(())
}
