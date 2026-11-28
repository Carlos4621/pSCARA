use core::iter::FusedIterator;

use crate::{
    MotorStepMove, MotorSteps, PlannerError, TrajectoryStepEventIterator,
    find_first_transition_tick_exponential,
};

#[derive(Debug)]
pub struct SegmentedTrajectoryStepEventIterator<'a, F> {
    boundaries: &'a [u64],
    segment_index: usize,

    current_tick: u64,
    current_position: MotorSteps,

    position_at_tick: F,
    finished: bool,
}

impl<'a, F> SegmentedTrajectoryStepEventIterator<'a, F> {
    pub fn new(
        boundaries: &'a [u64],
        initial_position: MotorSteps,
        position_at_tick: F,
    ) -> Result<Self, PlannerError> {
        if boundaries.len() < 2 {
            return Err(PlannerError::BoundariesWithoutFrontiers);
        }

        if !boundaries.windows(2).all(|w| w[0] < w[1]) {
            return Err(PlannerError::InvalidTicks);
        }

        Ok(Self {
            boundaries,
            segment_index: 0,
            current_tick: boundaries[0],
            current_position: initial_position,
            position_at_tick,
            finished: false,
        })
    }
}

impl<'a, F> Iterator for SegmentedTrajectoryStepEventIterator<'a, F>
where
    F: FnMut(u64) -> Result<MotorSteps, PlannerError>,
{
    type Item = Result<(u64, MotorStepMove), PlannerError>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.segment_index >= self.boundaries.len() - 1 {
                self.finished = true;
            }

            if self.finished {
                return None;
            }

            let end_tick = self.boundaries[self.segment_index + 1];

            if self.current_tick == end_tick {
                self.segment_index += 1;
                continue;
            }

            let result = find_first_transition_tick_exponential(
                self.current_tick,
                end_tick,
                self.current_position,
                &mut self.position_at_tick,
            );

            match result {
                Ok(Some((tick, movement))) => {
                    TrajectoryStepEventIterator::<F>::apply_axis_step(
                        &mut self.current_position.first,
                        movement.first(),
                    );
                    TrajectoryStepEventIterator::<F>::apply_axis_step(
                        &mut self.current_position.second,
                        movement.second(),
                    );

                    self.current_tick = tick;
                    return Some(Ok((tick, movement)));
                }

                Ok(None) => {
                    self.current_tick = end_tick;
                    self.segment_index += 1;
                    continue;
                }

                Err(error) => {
                    self.finished = true;
                    return Some(Err(error));
                }
            }
        }
    }
}

impl<F> FusedIterator for SegmentedTrajectoryStepEventIterator<'_, F> where
    F: FnMut(u64) -> Result<MotorSteps, PlannerError>
{
}

#[cfg(test)]
#[path = "../tests/unit/segmented_trajectory_step_event_iterator.rs"]
mod tests;
