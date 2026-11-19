use crate::{MotorStepMove, MotorSteps, PlannerError};

/// Encuentra el primer tick en `(start_tick, end_tick]` cuya posición difiere
/// de `initial_position`, la posición conocida en `start_tick`.
///
/// Para que la búsqueda binaria sea válida, una vez que la posición difiere
/// de la inicial debe seguir siendo distinta hasta `end_tick`. La función
/// de muestreo debe admitir consultas fuera de orden.
///
/// Devuelve `None` si el intervalo está vacío o la posición final es igual a
/// la inicial. Propaga los errores de muestreo y devuelve
/// `PlannerError::EndBeforeStart` si el intervalo está invertido.
pub fn find_first_transition_tick<F>(
    start_tick: u64,
    end_tick: u64,
    initial_position: MotorSteps,
    mut position_at_tick: F,
) -> Result<Option<(u64, MotorStepMove)>, PlannerError>
where
    F: FnMut(u64) -> Result<MotorSteps, PlannerError>,
{
    if end_tick < start_tick {
        return Err(PlannerError::EndBeforeStart);
    }
    if end_tick == start_tick {
        return Ok(None);
    }

    let mut right_position = position_at_tick(end_tick)?;

    if initial_position == right_position {
        return Ok(None);
    }

    let mut left = start_tick;
    let mut right = end_tick;

    while right - left > 1 {
        let mid = left + (right - left) / 2;

        let position = position_at_tick(mid)?;

        if position == initial_position {
            left = mid;
        } else {
            right = mid;
            right_position = position;
        }
    }

    let movement =         MotorStepMove::new(initial_position, right_position);

    if movement.first().pulses > 1 || movement.second().pulses > 1 {
        return Err(PlannerError::MultipleStepsOnTick);
    }

    Ok(Some((
        right,
        movement,
    )))
}
