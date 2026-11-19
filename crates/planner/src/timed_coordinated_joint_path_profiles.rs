use crate::{CoordinatedJointPathProfiles, PlannerError};

#[derive(Debug, Clone, Copy)]
pub struct TimedCoordinatedJointPathProfiles<'a> {
    profiles: CoordinatedJointPathProfiles<'a>,
    elapsed_time: f64,
}

impl<'a> TimedCoordinatedJointPathProfiles<'a> {
    pub fn new(profiles: CoordinatedJointPathProfiles<'a>) -> Result<Self, PlannerError> {
        profiles.total_path_time()?;

        Ok(Self {
            profiles,
            elapsed_time: 0.0,
        })
    }
}

#[cfg(test)]
#[path = "../tests/unit/timed_coordinated_joint_path_profiles.rs"]
mod tests;
