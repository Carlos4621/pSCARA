use core::iter::FusedIterator;

use crate::{
    AxisStepMove, MotorStepMove, MotorSteps, PlannerError, StepDirection,
    find_first_transition_tick,
};

#[derive(Debug)]
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

    fn apply_axis_step(position: &mut i64, movement: AxisStepMove) {
        match (movement.pulses, movement.direction) {
            (0, _) => {}

            (1, StepDirection::Positive) => {
                *position = position.checked_add(1).expect("Position overflow");
            }

            (1, StepDirection::Negative) => {
                *position = position.checked_sub(1).expect("Position underflow");
            }

            _ => unreachable!("More than one step per axis"),
        }
    }
}

impl<F> Iterator for TrajectoryStepEventIterator<F>
where
    F: FnMut(u64) -> Result<MotorSteps, PlannerError>,
{
    type Item = Result<(u64, MotorStepMove), PlannerError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.finished {
            return None;
        }

        let result = find_first_transition_tick(
            self.current_tick,
            self.end_tick,
            self.current_position,
            &mut self.position_at_tick,
        );

        let result = match result {
            Ok(ok) => ok,
            Err(err) => {
                self.finished = true;
                return Some(Err(err));
            }
        };

        let Some((tick, movement)) = result else {
            self.finished = true;
            return None;
        };

        Self::apply_axis_step(&mut self.current_position.first, movement.first());
        Self::apply_axis_step(&mut self.current_position.second, movement.second());

        self.current_tick = tick;

        Some(Ok((tick, movement)))
    }
}

impl<F> FusedIterator for TrajectoryStepEventIterator<F> where
    F: FnMut(u64) -> Result<MotorSteps, PlannerError>
{
}

#[cfg(test)]
#[path = "../tests/unit/trajectory_step_event_iterator.rs"]
mod tests;
