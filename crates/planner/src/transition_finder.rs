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

    let right_position = position_at_tick(end_tick)?;

    if initial_position == right_position {
        return Ok(None);
    }

    bisect_transition(
        start_tick,
        end_tick,
        initial_position,
        right_position,
        position_at_tick,
    )
}

/// Encuentra la primera transición en `(start_tick, end_tick]` acotando la
/// búsqueda con distancias crecientes desde `start_tick`.
///
/// Al igual que [`find_first_transition_tick`], requiere que la posición
/// permanezca distinta de la inicial después de la transición y que el
/// muestreador admita consultas fuera de orden. Propaga los errores de los
/// ticks que consulta.
pub fn find_first_transition_tick_exponential<F>(
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

    let remaining = end_tick - start_tick;
    let mut left = start_tick;
    let mut offset = 1;
    loop {
        // El desplazamiento queda acotado por end_tick - start_tick.
        let probe = start_tick + offset;
        let position = position_at_tick(probe)?;
        if position != initial_position {
            return bisect_transition(left, probe, initial_position, position, position_at_tick);
        }
        if probe == end_tick {
            return Ok(None);
        }
        left = probe;
        offset = offset.saturating_mul(2).min(remaining);
    }
}

/// Comprueba una estimación de la primera transición en `(start_tick, end_tick]`.
/// Si el tick estimado y el anterior no confirman la transición, usa la
/// búsqueda exponencial. Requiere el mismo muestreo fuera de orden y que la
/// posición permanezca distinta de la inicial después de la transición.
/// Propaga los errores de los ticks consultados, incluidos los de verificación.
#[cfg(feature = "transition-estimation")]
pub fn find_first_transition_tick_hinted<F>(
    start_tick: u64,
    end_tick: u64,
    initial_position: MotorSteps,
    hint: Option<u64>,
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

    if let Some(tick) = hint.filter(|&tick| start_tick < tick && tick <= end_tick) {
        let position = position_at_tick(tick)?;
        if position != initial_position {
            let previous_position = if tick - 1 == start_tick {
                initial_position
            } else {
                position_at_tick(tick - 1)?
            };
            if previous_position == initial_position {
                // La ventana de un tick permite validar el movimiento sin volver a muestrear.
                return bisect_transition(
                    tick - 1,
                    tick,
                    initial_position,
                    position,
                    position_at_tick,
                );
            }
        }
    }

    find_first_transition_tick_exponential(start_tick, end_tick, initial_position, position_at_tick)
}

fn bisect_transition<F>(
    mut left: u64,
    mut right: u64,
    initial_position: MotorSteps,
    mut right_position: MotorSteps,
    mut position_at_tick: F,
) -> Result<Option<(u64, MotorStepMove)>, PlannerError>
where
    F: FnMut(u64) -> Result<MotorSteps, PlannerError>,
{
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

    let movement = MotorStepMove::new(initial_position, right_position);

    if movement.first().pulses > 1 || movement.second().pulses > 1 {
        return Err(PlannerError::MultipleStepsOnTick);
    }

    Ok(Some((right, movement)))
}

/// Compara estimaciones con los ticks verificados mediante muestreo.
#[cfg(feature = "transition-estimation")]
#[derive(Debug, Default)]
pub struct TransitionEstimationStatistics {
    pub transitions: u64,
    pub estimates: u64,
    pub exact_estimates: u64,
    /// Incluye las estimaciones exactas.
    pub estimates_within_one_tick: u64,
    /// Incluye las estimaciones exactas y las que difieren un tick.
    pub estimates_within_two_ticks: u64,
    pub max_error_ticks: u64,
}

#[cfg(feature = "transition-estimation")]
impl TransitionEstimationStatistics {
    pub fn record(&mut self, actual_tick: u64, estimated_tick: Option<u64>) {
        self.transitions += 1;
        if let Some(estimated_tick) = estimated_tick {
            let error = actual_tick.abs_diff(estimated_tick);
            self.estimates += 1;
            self.exact_estimates += u64::from(error == 0);
            self.estimates_within_one_tick += u64::from(error <= 1);
            self.estimates_within_two_ticks += u64::from(error <= 2);
            self.max_error_ticks = self.max_error_ticks.max(error);
        }
    }
}
