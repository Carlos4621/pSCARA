use crate::{MotorSteps, PlannerError};

#[derive(Debug, Clone, Copy)]
pub struct TrajectoryStepEventIterator<F> {
    position_at_tick: F,

    current_tick: u64,
    end_tick: u64,

    current_position: MotorSteps,

    finished: bool,
}

impl<F> TrajectoryStepEventIterator<F> {
    pub fn new(
        start_tick: u64,
        end_tick: u64,
        initial_position: MotorSteps,
        position_at_tick: F,
    ) -> Result<Self, PlannerError> {
        if start_tick > end_tick {
            return Err(PlannerError::EndBeforeStart);
        }

        Ok(Self {
            position_at_tick,
            current_tick: start_tick,
            end_tick,
            current_position: initial_position,
            finished: start_tick == end_tick,
        })
    }
}

#[cfg(test)]
#[path = "../tests/unit/trajectory_step_event_iterator.rs"]
mod tests;
