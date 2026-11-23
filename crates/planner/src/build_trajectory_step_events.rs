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

    validate_motor_position_range(profiles, &converter)?;
    validate_subtick_boundaries(profiles, &trajectory, &converter, timebase)?;
    validate_step_frequencies(profiles, &converter, timebase)?;

    let position_at_tick = move |tick: u64| -> Result<MotorSteps, PlannerError> {
        let time = timebase.ticks_to_seconds(tick);

        let angles = trajectory.position_at_clamped(time)?;

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

/// Comprueba que la trayectoria pueda generar todos sus eventos STEP.
///
/// El búfer queda intacto ante un error de construcción. Si el error aparece
/// durante el recorrido del iterador, puede contener las fronteras generadas.
pub fn preflight_trajectory_step_events<'a>(
    profiles: TimedCoordinatedJointPathProfiles<'a>,
    converter: JointToStepsConverter,
    timebase: StepTimeBase,
    boundaries_buffer: &'a mut [u64],
) -> Result<(), PlannerError> {
    for event in build_trajectory_step_events(profiles, converter, timebase, boundaries_buffer)? {
        event?;
    }
    Ok(())
}

fn validate_motor_position_range(
    profiles: TimedCoordinatedJointPathProfiles<'_>,
    converter: &JointToStepsConverter,
) -> Result<(), PlannerError> {
    for profile in profiles {
        let profile = profile?;
        let initial_position = profile.position_at(profile.start_time())?;
        let final_position = profile.position_at(profile.end_time()?)?;

        converter.convert(initial_position)?;
        converter.convert(final_position)?;
    }
    Ok(())
}

fn validate_step_frequencies(
    profiles: TimedCoordinatedJointPathProfiles<'_>,
    converter: &JointToStepsConverter,
    timebase: StepTimeBase,
) -> Result<(), PlannerError> {
    let clock_frequency = timebase.ticks_per_second();
    for profile in profiles {
        let joint_velocities = profile?.peak_joint_velocities()?;
        let (first, second) = converter.peak_step_frequencies(joint_velocities)?;

        // Las frecuencias son finitas y no negativas. Truncarlas conserva
        // la comparación con un umbral entero, y saturar a u64::MAX también
        // rechaza las frecuencias que superan el rango de u64.
        if first as u64 >= clock_frequency || second as u64 >= clock_frequency {
            return Err(PlannerError::InsufficientTicksForSteps);
        }
    }
    Ok(())
}

#[cfg_attr(
    not(test),
    expect(dead_code, reason = "Las pruebas comparan ambos validadores")
)]
fn validate_subtick_boundary(
    boundary_time: f64,
    trajectory: &CoordinatedJointTrajectory<'_>,
    converter: &JointToStepsConverter,
    timebase: StepTimeBase,
) -> Result<(), PlannerError> {
    let (lower, upper) = timebase.bracketing_ticks(boundary_time)?;
    if lower == upper {
        return Ok(());
    }

    let position_at = |seconds| converter.convert(trajectory.position_at_clamped(seconds)?);
    let before = position_at(timebase.ticks_to_seconds(lower))?;
    let at_boundary = position_at(boundary_time)?;
    let after = position_at(timebase.ticks_to_seconds(upper))?;
    validate_subtick_excursion(before, at_boundary, after)
}

fn validate_subtick_excursion(
    before: MotorSteps,
    at_boundary: MotorSteps,
    after: MotorSteps,
) -> Result<(), PlannerError> {
    let mut travel = SubtickTravel::new(before);
    travel.add_position(at_boundary);
    travel.add_position(after);
    travel.validate()
}

fn validate_subtick_boundaries(
    profiles: TimedCoordinatedJointPathProfiles<'_>,
    trajectory: &CoordinatedJointTrajectory<'_>,
    converter: &JointToStepsConverter,
    timebase: StepTimeBase,
) -> Result<(), PlannerError> {
    let position_at = |seconds| converter.convert(trajectory.position_at_clamped(seconds)?);
    let position_at_tick = |tick| position_at(timebase.ticks_to_seconds(tick));
    let finish_interval = |upper, mut travel: SubtickTravel| {
        travel.add_position(position_at_tick(upper)?);
        travel.validate()
    };

    let mut active_interval = None;
    let mut profiles = profiles.peekable();
    while let Some(profile) = profiles.next() {
        let profile = profile?;
        if profiles.peek().is_none() {
            break;
        }

        let boundary_time = profile.end_time()?;
        let (lower, upper) = timebase.bracketing_ticks(boundary_time)?;
        if lower == upper {
            continue;
        }

        let mut travel = match active_interval {
            Some((interval, travel)) if interval == (lower, upper) => travel,
            previous_interval => {
                if let Some(((_, previous_upper), travel)) = previous_interval {
                    finish_interval(previous_upper, travel)?;
                }
                SubtickTravel::new(position_at_tick(lower)?)
            }
        };
        travel.add_position(position_at(boundary_time)?);
        active_interval = Some(((lower, upper), travel));
    }

    if let Some(((_, upper), travel)) = active_interval {
        finish_interval(upper, travel)?;
    }
    Ok(())
}

struct SubtickTravel {
    previous: MotorSteps,
    first_travel: u128,
    second_travel: u128,
}

impl SubtickTravel {
    fn new(position: MotorSteps) -> Self {
        Self {
            previous: position,
            first_travel: 0,
            second_travel: 0,
        }
    }

    fn add_position(&mut self, position: MotorSteps) {
        self.first_travel += u128::from(self.previous.first.abs_diff(position.first));
        self.second_travel += u128::from(self.previous.second.abs_diff(position.second));
        self.previous = position;
    }

    fn validate(&self) -> Result<(), PlannerError> {
        if self.first_travel > 1 || self.second_travel > 1 {
            return Err(PlannerError::SubtickMotionNotRepresentable);
        }
        Ok(())
    }
}

#[cfg(test)]
#[path = "../tests/unit/build_trajectory_step_events.rs"]
mod tests;
