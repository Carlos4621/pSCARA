use crate::{
    CoordinatedJointTrajectory, JointToStepsConverter, MotorSteps, PlannerError, SegmentedTrajectoryStepEventIterator, StepTimeBase, TimedCoordinatedJointPathProfiles, build_segmented_step_events,
};

pub fn build_trajectory_step_events<'a>(
    profiles: TimedCoordinatedJointPathProfiles<'a>,
    converter: JointToStepsConverter,
    timebase: StepTimeBase,
    boundaries_buffer: &'a mut [u64],
) -> Result<SegmentedTrajectoryStepEventIterator<'a, impl Fn(u64) -> Result<MotorSteps, PlannerError>>, PlannerError> {
    let trajectory = CoordinatedJointTrajectory::new(profiles)?;

    let position_at_tick = move |tick: u64| -> Result<MotorSteps, PlannerError> {
        let time = timebase.ticks_to_seconds(tick);

        let angles = trajectory.position_at(time)?;

        converter.convert(angles)
    };

    let initial_position = position_at_tick(0)?;

    build_segmented_step_events(
        profiles,
        timebase,
        boundaries_buffer,
        initial_position,
        position_at_tick,
    )
}
