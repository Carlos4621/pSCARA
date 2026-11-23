use crate::{MotorStepMove, PlannerError, StepDirection};

/// Orden lógica STEP/DIR para ambos motores en un mismo tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StepDirEvent {
    pub tick: u64,
    /// `None` indica que el primer motor conserva DIR y omite el pulso STEP.
    pub first_direction: Option<StepDirection>,
    /// `None` indica que el segundo motor conserva DIR y omite el pulso STEP.
    pub second_direction: Option<StepDirection>,
}

impl StepDirEvent {
    pub fn new(tick: u64, movement: MotorStepMove) -> Result<Self, PlannerError> {
        let first = movement.first();
        let second = movement.second();
        if first.pulses > 1 || second.pulses > 1 {
            return Err(PlannerError::MultipleStepsOnTick);
        }

        Ok(Self {
            tick,
            first_direction: (first.pulses == 1).then_some(first.direction),
            second_direction: (second.pulses == 1).then_some(second.direction),
        })
    }
}

/// Convierte los movimientos en eventos STEP/DIR bajo demanda y propaga sus errores.
pub fn into_step_dir_events<I>(
    events: I,
) -> impl Iterator<Item = Result<StepDirEvent, PlannerError>>
where
    I: Iterator<Item = Result<(u64, MotorStepMove), PlannerError>>,
{
    events.map(|event| event.and_then(|(tick, movement)| StepDirEvent::new(tick, movement)))
}
