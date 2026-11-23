use crate::{PlannerError, ScheduledStepDirEvent, StepDirection};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorAxis {
    First,
    Second,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinOperation {
    SetDirection(StepDirection),
    StepHigh,
    StepLow,
}

/// Operación sobre un pin de un motor en un tick concreto.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimedPinOperation {
    pub tick: u64,
    pub motor: MotorAxis,
    pub operation: PinOperation,
}

/// Expande un evento en tres pares de posiciones para DIR, STEP HIGH y STEP LOW.
/// Cada par comienza con la posición del primer motor. Las posiciones vacías contienen `None`.
/// Devuelve [`PlannerError::InvalidStepDirTiming`] si HIGH dura cero ticks y
/// [`PlannerError::NumericalFailure`] si el flanco de bajada supera `u64::MAX`.
pub fn expand_step_dir_event(
    event: ScheduledStepDirEvent,
    step_high_ticks: u64,
) -> Result<[Option<TimedPinOperation>; 6], PlannerError> {
    if step_high_ticks == 0 {
        return Err(PlannerError::InvalidStepDirTiming);
    }

    let step_low_tick = event.step_low_tick(step_high_ticks)?;
    let mut operations = [None; 6];
    for (index, motor, dir_change, step) in [
        (
            0,
            MotorAxis::First,
            event.first_dir_change,
            event.first_step,
        ),
        (
            1,
            MotorAxis::Second,
            event.second_dir_change,
            event.second_step,
        ),
    ] {
        operations[index] = dir_change.map(|change| TimedPinOperation {
            tick: change.tick,
            motor,
            operation: PinOperation::SetDirection(change.direction),
        });
        if let Some(tick) = step_low_tick.filter(|_| step) {
            operations[index + 2] = Some(TimedPinOperation {
                tick: event.step_tick,
                motor,
                operation: PinOperation::StepHigh,
            });
            operations[index + 4] = Some(TimedPinOperation {
                tick,
                motor,
                operation: PinOperation::StepLow,
            });
        }
    }

    Ok(operations)
}
