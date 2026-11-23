use crate::{PlannerError, StepTimeBase, TimedCoordinatedJointPathProfiles};

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

/// Genera fronteras estrictamente crecientes desde el tick cero.
///
/// Cada frontera interna aporta los ticks que la rodean, omitiendo duplicados.
/// El final de la trayectoria debe coincidir con un tick.
/// Devuelve el número de fronteras escritas y deja el búfer intacto si hay un error.
pub fn generate_guarded_segment_boundaries(
    profiles: TimedCoordinatedJointPathProfiles<'_>,
    timebase: StepTimeBase,
    boundaries: &mut [u64],
) -> Result<usize, PlannerError> {
    let capacity = boundaries.len();
    if capacity < 2 {
        return Err(PlannerError::InsufficientBoundaryBuffer);
    }

    visit_guarded_boundaries(profiles, timebase, capacity, |_, _| {})?;
    visit_guarded_boundaries(profiles, timebase, capacity, |index, tick| {
        boundaries[index] = tick;
    })
}

fn visit_guarded_boundaries(
    profiles: TimedCoordinatedJointPathProfiles<'_>,
    timebase: StepTimeBase,
    capacity: usize,
    mut visit: impl FnMut(usize, u64),
) -> Result<usize, PlannerError> {
    let mut profiles = profiles.enumerate().peekable();
    let mut count = 1;
    let mut previous_tick = 0;
    visit(0, previous_tick);

    while let Some((index, profile)) = profiles.next() {
        let profile = profile?;
        if index == 0 && profile.start_time() != 0.0 {
            return Err(PlannerError::InvalidTime);
        }

        let (lower, upper) = timebase.bracketing_ticks(profile.end_time()?)?;
        if profiles.peek().is_none() && (lower != upper || upper == 0) {
            return Err(PlannerError::InvalidTicks);
        }

        for tick in [lower, upper] {
            if tick <= previous_tick {
                continue;
            }
            if count == capacity {
                return Err(PlannerError::InsufficientBoundaryBuffer);
            }

            visit(count, tick);
            previous_tick = tick;
            count += 1;
        }
    }

    if count == 1 {
        return Err(PlannerError::InsufficientPoints);
    }
    Ok(count)
}
