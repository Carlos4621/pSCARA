use crate::{MotorAxis, PinOperation, PlannerError, StepDirection, TimedPinOperation};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepPinLevel {
    High,
    Low,
}

/// Operaciones de los pines de ambos motores en un mismo tick.
/// `None` conserva el estado actual del pin correspondiente.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PinOperationBatch {
    pub tick: u64,
    pub first_direction: Option<StepDirection>,
    pub second_direction: Option<StepDirection>,
    pub first_step: Option<StepPinLevel>,
    pub second_step: Option<StepPinLevel>,
}

impl PinOperationBatch {
    pub fn new(tick: u64) -> Self {
        Self {
            tick,
            first_direction: None,
            second_direction: None,
            first_step: None,
            second_step: None,
        }
    }

    /// Añade una operación de este tick y acepta repeticiones idénticas.
    /// Si devuelve un error, el grupo conserva todos sus campos.
    pub fn insert(&mut self, operation: TimedPinOperation) -> Result<(), PlannerError> {
        if operation.tick != self.tick {
            return Err(PlannerError::InvalidTicks);
        }

        let (direction, step) = match operation.motor {
            MotorAxis::First => (&mut self.first_direction, &mut self.first_step),
            MotorAxis::Second => (&mut self.second_direction, &mut self.second_step),
        };

        match operation.operation {
            PinOperation::SetDirection(value) => insert_pin_value(direction, value),
            PinOperation::StepHigh => insert_pin_value(step, StepPinLevel::High),
            PinOperation::StepLow => insert_pin_value(step, StepPinLevel::Low),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.first_direction.is_none()
            && self.second_direction.is_none()
            && self.first_step.is_none()
            && self.second_step.is_none()
    }
}

fn insert_pin_value<T: Eq>(slot: &mut Option<T>, value: T) -> Result<(), PlannerError> {
    match slot {
        Some(current) if *current == value => Ok(()),
        Some(_) => Err(PlannerError::ConflictingPinOperations),
        None => {
            *slot = Some(value);
            Ok(())
        }
    }
}
