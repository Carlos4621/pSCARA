use crate::{PlannerError, StepDirEvent, StepDirTiming, StepTimeBase};

/// Comprueba la separación entre pulsos STEP de cada motor en una sola pasada.
/// Los ticks de los eventos deben ser estrictamente crecientes.
pub fn validate_step_dir_intervals<I>(
    events: I,
    timing: StepDirTiming,
    timebase: StepTimeBase,
) -> Result<(), PlannerError>
where
    I: IntoIterator<Item = Result<StepDirEvent, PlannerError>>,
{
    let min_interval = timing.min_step_interval_ticks(timebase)?;
    let mut previous_tick = None;
    let mut previous_first_step = None;
    let mut previous_second_step = None;

    for event in events {
        let StepDirEvent {
            tick,
            first_direction,
            second_direction,
        } = event?;

        if previous_tick.is_some_and(|previous| tick <= previous) {
            return Err(PlannerError::InvalidTicks);
        }

        // El orden de los eventos garantiza que la resta de ticks nunca sea negativa.
        for (direction, previous_step) in [
            (first_direction, &mut previous_first_step),
            (second_direction, &mut previous_second_step),
        ] {
            if direction.is_none() {
                continue;
            }
            if previous_step.is_some_and(|previous| tick - previous < min_interval) {
                return Err(PlannerError::StepIntervalTooShort);
            }
            *previous_step = Some(tick);
        }

        previous_tick = Some(tick);
    }

    Ok(())
}
