use crate::{MotorStepMove, MotorSteps, PlannerError};

#[derive(Debug)]
pub struct StepTransitionDetector {
    last_tick: u64,
    last_position: MotorSteps,
}

impl StepTransitionDetector {
    pub fn new(initial_tick: u64, initial_position: MotorSteps) -> Self {
        Self {
            last_tick: initial_tick,
            last_position: initial_position,
        }
    }

    pub fn update(
        &mut self,
        tick: u64,
        position: MotorSteps,
    ) -> Result<Option<MotorStepMove>, PlannerError> {
        if tick <= self.last_tick {
            return Err(PlannerError::InvalidTicks);
        }

        if position == self.last_position {
            self.last_tick = tick;
            return Ok(None);
        }

        let motion = MotorStepMove::new(self.last_position, position);

        if motion.first().pulses > 1 || motion.second().pulses > 1 {
            return Err(PlannerError::MultipleStepsOnTick);
        }
        
        self.last_tick = tick;
        self.last_position = position;

        Ok(Some(motion))
    }
}
