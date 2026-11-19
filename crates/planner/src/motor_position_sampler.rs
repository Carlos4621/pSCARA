use crate::{
    CoordinatedJointTrajectoryCursor, JointToStepsConverter, MotorSteps, PlannerError, StepTimeBase,
};

#[derive(Debug)]
pub struct MotorPositionSampler<'a> {
    cursor: CoordinatedJointTrajectoryCursor<'a>,
    converter: JointToStepsConverter,
    timebase: StepTimeBase,

    last_tick: Option<u64>,
}

impl<'a> MotorPositionSampler<'a> {
    pub fn new(
        cursor: CoordinatedJointTrajectoryCursor<'a>,
        converter: JointToStepsConverter,
        timebase: StepTimeBase,
    ) -> Self {
        Self {
            cursor,
            converter,
            timebase,
            last_tick: None,
        }
    }

    pub fn sample_at_tick(&mut self, ticks: u64) -> Result<MotorSteps, PlannerError> {
        if self.last_tick.is_some_and(|last| ticks < last) {
            return Err(PlannerError::TimeWentBackwards);
        }

        let seconds = self.timebase.ticks_to_seconds(ticks);
        let sample = self.cursor.sample_at(seconds)?;

        self.last_tick = Some(ticks);

        self.converter.convert(sample.position)
    }
}
