use crate::{
    MotorSteps, PlannerError, SegmentedTrajectoryStepEventIterator, StepTimeBase,
    StepTransitionSearch, TimedCoordinatedJointPathProfiles, generate_guarded_segment_boundaries,
};

pub fn build_segmented_step_events<'a, F>(
    profiles: TimedCoordinatedJointPathProfiles<'_>,
    timebase: StepTimeBase,
    boundaries_buffer: &'a mut [u64],
    initial_position: MotorSteps,
    transition_search: F,
) -> Result<SegmentedTrajectoryStepEventIterator<'a, F>, PlannerError>
where
    F: StepTransitionSearch,
{
    let count = generate_guarded_segment_boundaries(profiles, timebase, boundaries_buffer)?;

    let boundaries = &boundaries_buffer[..count];

    SegmentedTrajectoryStepEventIterator::new(boundaries, initial_position, transition_search)
}
