use kinematics::JointAngles;

use crate::{CoordinatedJointMotionProfile, PlannerError};

#[derive(Debug)]
pub struct TimedCoordinatedJointProfile {
    start_time: f64,
    profile: CoordinatedJointMotionProfile,
}

impl TimedCoordinatedJointProfile {
    pub fn new(
        start_time: f64,
        profile: CoordinatedJointMotionProfile,
    ) -> Result<Self, PlannerError> {
        utilities::validate_finite_positive(start_time).map_err(|_| PlannerError::InvalidTime)?;

        Ok(Self {
            start_time,
            profile,
        })
    }

    pub fn start_time(&self) -> f64 {
        self.start_time
    }

    pub fn end_time(&self) -> Result<f64, PlannerError> {
        let duration = self.profile.total_time();
        utilities::validate_finite_positive(duration).map_err(|_| PlannerError::InvalidTime)?;

        let end_time = self.start_time + duration;
        utilities::validate_finite_positive(end_time)
            .map_err(|_| PlannerError::TimeElapsedOverflow)?;

        if self.start_time > 0.0
            && duration > 0.0
            && (end_time == self.start_time || end_time == duration)
        {
            return Err(PlannerError::TimeElapsedPrecisionLost);
        }

        Ok(end_time)
    }

    pub fn position_at(&self, global_time: f64) -> Result<JointAngles, PlannerError> {
        utilities::validate_finite_positive(global_time).map_err(|_| PlannerError::InvalidTime)?;

        let end_time = self.end_time()?;

        if global_time < self.start_time || global_time > end_time {
            return Err(PlannerError::InvalidTime);
        }

        if global_time == end_time {
            return self.profile.position_at(self.profile.total_time());
        }

        let local_time = global_time - self.start_time;

        self.validate_local_time(local_time)?;

        self.profile.position_at(local_time)
    }

    fn validate_local_time(&self, time: f64) -> Result<(), PlannerError> {
        if !(0.0..=self.profile.total_time()).contains(&time) {
            return Err(PlannerError::InvalidTime);
        }

        Ok(())
    }
}
