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

#[cfg_attr(
    not(test),
    expect(dead_code, reason = "Se usará al validar fronteras entre ticks")
)]
fn validate_subtick_excursion(
    before: MotorSteps,
    at_boundary: MotorSteps,
    after: MotorSteps,
) -> Result<(), PlannerError> {
    let first_travel = u128::from(before.first.abs_diff(at_boundary.first))
        + u128::from(at_boundary.first.abs_diff(after.first));
    let second_travel = u128::from(before.second.abs_diff(at_boundary.second))
        + u128::from(at_boundary.second.abs_diff(after.second));

    if first_travel > 1 || second_travel > 1 {
        return Err(PlannerError::SubtickMotionNotRepresentable);
    }
    Ok(())
}

#[cfg(test)]
#[path = "../tests/unit/build_trajectory_step_events.rs"]
mod tests;
