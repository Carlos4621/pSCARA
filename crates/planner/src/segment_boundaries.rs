use crate::{
    PlannerError,
    StepTimeBase,
    TimedCoordinatedJointPathProfiles,
};

pub fn generate_segment_boundaries(
    profiles: TimedCoordinatedJointPathProfiles<'_>,
    timebase: StepTimeBase,
    boundaries: &mut [u64],
) -> Result<usize, PlannerError> {
    if boundaries.len() < 2 {
        return Err(PlannerError::InsufficientBoundaryBuffer);
    }

    let mut count = 1;
    let mut previous_tick = 0_u64;

    for profile in profiles {
        let profile = profile?;

        if count == 1 && profile.start_time() != 0.0 {
            return Err(PlannerError::InvalidTime);
        }

        let end_time = profile.end_time()?;
        let end_tick = timebase.seconds_to_ticks(end_time)?;

        if end_tick <= previous_tick {
            return Err(PlannerError::InvalidTicks);
        }

        if timebase.ticks_to_seconds(end_tick) != end_time {
            return Err(PlannerError::InvalidTicks);
        }

        if count >= boundaries.len() {
            return Err(PlannerError::InsufficientBoundaryBuffer);
        }

        previous_tick = end_tick;
        count += 1;
    }

    // No se encontraron perfiles.
    if count == 1 {
        return Err(PlannerError::InsufficientPoints);
    }
    
    boundaries[0] = 0;

    for (index, profile) in profiles.enumerate() {
        let end_time = profile?.end_time()?;
        let end_tick = timebase.seconds_to_ticks(end_time)?;

        boundaries[index + 1] = end_tick;
    }

    Ok(count)
}