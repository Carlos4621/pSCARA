use crate::{MotorStepMove, MotorSteps, PlannerError};

#[derive(Debug, Clone, Copy)]
pub struct MotorSpeedInterval {
    start_tick: u64,
    end_tick: u64,
    movement: MotorStepMove,
}

impl MotorSpeedInterval {
    pub fn new(start_tick: u64, end_tick: u64, origin: MotorSteps, target: MotorSteps) -> Result<Self, PlannerError> {
        if end_tick <= start_tick {
            return Err(PlannerError::EndBeforeStart);
        }
        
        Ok(Self { start_tick, end_tick, movement: MotorStepMove::new(origin, target) })
    }

    pub fn start_tick(&self) -> u64 {
        self.start_tick
    }

    pub fn end_tick(&self) -> u64 {
        self.end_tick
    }

    pub fn duration_ticks(&self) -> u64 {
        self.end_tick - self.start_tick
    }

    pub fn movement(&self) -> MotorStepMove {
        self.movement
    }
}