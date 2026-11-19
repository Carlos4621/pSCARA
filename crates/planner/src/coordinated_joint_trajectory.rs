use kinematics::JointAngles;

use crate::{PlannerError, TimedCoordinatedJointPathProfiles, TimedCoordinatedJointProfile};

#[derive(Debug, Clone, Copy)]
pub struct CoordinatedJointTrajectory<'a> {
    profiles: TimedCoordinatedJointPathProfiles<'a>,
    total_time: f64,
}

impl<'a> CoordinatedJointTrajectory<'a> {
    pub fn new(profiles: TimedCoordinatedJointPathProfiles<'a>) -> Result<Self, PlannerError> {
        let mut end_time = 0.0;

        for i in profiles {
            end_time = i?.end_time()?;
        }

        Ok(Self { profiles, total_time: end_time })
    }

    pub fn position_at(&self, global_time: f64) -> Result<JointAngles, PlannerError> {
        self.find_profile(global_time)?.position_at(global_time)
    }

    pub fn acceleration_at(&self, global_time: f64) -> Result<(f64, f64), PlannerError> {
        self.find_profile(global_time)?.acceleration_at(global_time)
    }

    pub fn velocity_at(&self, global_time: f64) -> Result<(f64, f64), PlannerError> {
        self.find_profile(global_time)?.velocity_at(global_time)
    }

    fn find_profile(&self, global_time: f64) -> Result<TimedCoordinatedJointProfile, PlannerError> {
        utilities::validate_finite_positive(global_time).map_err(|_| PlannerError::InvalidTime)?;
    
        if global_time > self.total_time {
            return Err(PlannerError::InvalidTime);
        }

        for i in self.profiles {
            let i = i?;
            if i.start_time() <= global_time && i.end_time()? >= global_time {
                return Ok(i);
            }
        }

        Err(PlannerError::ProfileInstanceNotFound)
    }
}