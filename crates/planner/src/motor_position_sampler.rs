use crate::{CoordinatedJointTrajectoryCursor, JointToStepsConverter, MotorSteps, PlannerError, StepTimeBase};

#[derive(Debug)]
pub struct MotorPositionSampler<'a> {
    cursor: CoordinatedJointTrajectoryCursor<'a>,
    converter: JointToStepsConverter,
    timebase: StepTimeBase,
}

impl<'a> MotorPositionSampler<'a> {
    pub fn new(cursor: CoordinatedJointTrajectoryCursor<'a>, converter: JointToStepsConverter, timebase: StepTimeBase) -> Self {
        Self { cursor, converter, timebase }
    }

    pub fn sample_at_tick(&mut self, ticks: u64) -> Result<MotorSteps, PlannerError> {
        let seconds = self.timebase.ticks_to_seconds(ticks);
        let sample = self.cursor.sample_at(seconds)?;

        self.converter.convert(sample.position)
    }
}