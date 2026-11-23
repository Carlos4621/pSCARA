use kinematics::JointAngles;

use crate::{CoordinatedJointMotionProfile, PlannerError};

#[derive(Debug)]
pub struct TimedCoordinatedJointProfile {
    start_time: f64,
    profile: CoordinatedJointMotionProfile,
}

impl TimedCoordinatedJointProfile {
    /// Crea un perfil y valida que su instante final sea representable.
    pub fn new(
        start_time: f64,
        profile: CoordinatedJointMotionProfile,
    ) -> Result<Self, PlannerError> {
        let timed = Self {
            start_time,
            profile,
        };
        timed.end_time()?;
        Ok(timed)
    }

    pub fn start_time(&self) -> f64 {
        self.start_time
    }

    pub fn end_time(&self) -> Result<f64, PlannerError> {
        utilities::checked_time_sumatory(&[self.start_time, self.profile.total_time()])
            .map_err(PlannerError::from_time_error)
    }

    pub fn position_at(&self, global_time: f64) -> Result<JointAngles, PlannerError> {
        self.profile.position_at(self.get_local_time(global_time)?)
    }

    pub fn velocity_at(&self, global_time: f64) -> Result<(f64, f64), PlannerError> {
        self.profile.velocity_at(self.get_local_time(global_time)?)
    }

    /// Devuelve las magnitudes máximas de velocidad del perfil en rad/s.
    pub fn peak_joint_velocities(&self) -> Result<(f64, f64), PlannerError> {
        self.profile.peak_joint_velocities()
    }

    pub fn acceleration_at(&self, global_time: f64) -> Result<(f64, f64), PlannerError> {
        self.profile.acceleration_at(self.get_local_time(global_time)?)
    }  

    fn validate_local_time(&self, time: f64) -> Result<(), PlannerError> {
        if !(0.0..=self.profile.total_time()).contains(&time) {
            return Err(PlannerError::InvalidTime);
        }

        Ok(())
    }

    fn get_local_time(&self, global_time: f64) -> Result<f64, PlannerError> {
        utilities::validate_finite_positive(global_time).map_err(|_| PlannerError::InvalidTime)?;

        let end_time = self.end_time()?;

        if global_time < self.start_time || global_time > end_time {
            return Err(PlannerError::InvalidTime);
        }

        if global_time == end_time {
            return Ok(self.profile.total_time());
        }

        let local_time = global_time - self.start_time;

        self.validate_local_time(local_time)?;

        Ok(local_time)
    }
}
