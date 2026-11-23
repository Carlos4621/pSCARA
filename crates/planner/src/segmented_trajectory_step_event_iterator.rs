use crate::{MotorSteps, PlannerError};

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

#[cfg(test)]
#[path = "../tests/unit/segmented_trajectory_step_event_iterator.rs"]
mod tests;
