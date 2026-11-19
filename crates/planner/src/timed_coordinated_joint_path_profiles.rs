use core::iter::FusedIterator;

use crate::{CoordinatedJointPathProfiles, PlannerError, TimedCoordinatedJointProfile};

/// Recorre los perfiles pendientes y les asigna tiempos globales desde cero.
/// Tras el primer error, el iterador queda agotado.
#[derive(Debug, Clone, Copy)]
pub struct TimedCoordinatedJointPathProfiles<'a> {
    profiles: CoordinatedJointPathProfiles<'a>,
    elapsed_time: f64,
    is_finished: bool,
}

impl<'a> TimedCoordinatedJointPathProfiles<'a> {
    pub fn new(profiles: CoordinatedJointPathProfiles<'a>) -> Result<Self, PlannerError> {
        profiles.total_path_time()?;

        Ok(Self {
            profiles,
            elapsed_time: 0.0,
            is_finished: false,
        })
    }
}

impl Iterator for TimedCoordinatedJointPathProfiles<'_> {
    type Item = Result<TimedCoordinatedJointProfile, PlannerError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.is_finished {
            return None;
        }

        let Some(profile) = self.profiles.next() else {
            self.is_finished = true;
            return None;
        };

        let result = profile.and_then(|profile| {
            let timed = TimedCoordinatedJointProfile::new(self.elapsed_time, profile)?;
            self.elapsed_time = timed.end_time()?;
            Ok(timed)
        });

        if result.is_err() {
            self.is_finished = true;
        }

        Some(result)
    }
}

impl FusedIterator for TimedCoordinatedJointPathProfiles<'_> {}

#[cfg(test)]
#[path = "../tests/unit/timed_coordinated_joint_path_profiles.rs"]
mod tests;
