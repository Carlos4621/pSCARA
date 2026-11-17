use kinematics::JointAngles;

use crate::{CoordinatedJointMotionProfile, PlannerError};

#[derive(Debug)]
pub struct TimedCoordinatedJointProfile {
    start_time: f64,
    profile: CoordinatedJointMotionProfile,
}

impl TimedCoordinatedJointProfile {
    pub fn new(start_time: f64, profile: CoordinatedJointMotionProfile) -> Self {
        Self { start_time, profile }
    }

    pub fn start_time(&self) -> f64 {
        self.start_time
    }

    pub fn end_time(&self) -> Result<f64, PlannerError> {
        let end_time = self.start_time + self.profile.total_time();

        utilities::validate_finite(end_time).map_err(|_| PlannerError::NumericalFailure)?;

        if self.profile.total_time() > 0.0 && end_time == self.start_time {
            return Err(PlannerError::TimeElapsedPrecisionLost);
        }

        Ok(end_time)
    } 

    pub fn position_at(&self, global_time: f64) -> Result<JointAngles, PlannerError> {
        utilities::validate_finite_positive(global_time).map_err(|_| PlannerError::InvalidTime)?;

        let local_time = global_time - self.start_time;

        self.validate_local_time(local_time)?;

        self.profile.position_at(local_time)
    }

    fn validate_local_time(&self, time: f64) -> Result<(), PlannerError> {
        if !(0.0..=self.end_time()?).contains(&time) {
            return Err(PlannerError::InvalidTime);
        }

        Ok(())
    }
}