use kinematics::JointAngles;

use crate::{PlannerError, TimedCoordinatedJointPathProfiles};

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
        utilities::validate_finite_positive(global_time).map_err(|_| PlannerError::InvalidTime)?;
    
        if global_time > self.total_time {
            return Err(PlannerError::InvalidTime);
        }
    
        for i in self.profiles {
            let i = i?;
            if i.start_time() <= global_time && i.end_time()? >= global_time {
                return i.position_at(global_time);
            }
        }

        Err(PlannerError::ProfileInstanceNotFound)
    }
}