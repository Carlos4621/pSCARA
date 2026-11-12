use kinematics::JointAngles;

use crate::{
    JointVelocityLimits, PlannerError, ValidatedTrajectory,
    validated_timed_trajectory::ValidatedTimedTrajectory,
};

#[derive(Debug, Clone, Copy)]
pub struct TimedJointSample {
    pub angle: JointAngles,
    pub time_elapsed: f64,
}

#[derive(Debug, Clone)]
pub struct TimedJointTrajectory<'a> {
    trajectory: ValidatedTrajectory<'a>,
    velocity_limits: JointVelocityLimits,
    previous_angle: Option<JointAngles>,
    time_elapsed: f64,
    is_finished: bool,
    has_started: bool,
}

impl<'a> TimedJointTrajectory<'a> {
    pub fn new(trajectory: ValidatedTrajectory<'a>, velocity_limits: JointVelocityLimits) -> Self {
        Self {
            trajectory,
            velocity_limits,
            previous_angle: None,
            time_elapsed: 0.0,
            is_finished: false,
            has_started: false,
        }
    }

    pub fn validate(self) -> Result<(), PlannerError> {
        if self.has_started {
            return Err(PlannerError::TrajectoryAlreadyStarted);
        }

        for n in self {
            n?;
        }

        Ok(())
    }

    pub fn into_validated(self) -> Result<ValidatedTimedTrajectory<'a>, PlannerError> {
        self.clone().validate()?;

        Ok(ValidatedTimedTrajectory::new(self))
    }
}

impl Iterator for TimedJointTrajectory<'_> {
    type Item = Result<TimedJointSample, PlannerError>;

    fn next(&mut self) -> Option<Self::Item> {
        if !self.has_started {
            self.has_started = true;
        }

        if self.is_finished {
            return None;
        }

        let Some(next_angle) = self.trajectory.next() else {
            self.is_finished = true;
            return None;
        };

        let result = next_angle.and_then(|angle| {
            let delta_time = match self.previous_angle {
                Some(previous) => self.velocity_limits.min_duration_between(previous, angle)?,
                None => 0.0,
            };

            let new_time = self.time_elapsed + delta_time;

            utilities::validate_finite(new_time).map_err(|_| PlannerError::TimeElapsedOverflow)?;

            if delta_time > 0.0 && new_time <= self.time_elapsed {
                return Err(PlannerError::TimeElapsedPrecisionLost);
            }

            self.previous_angle = Some(angle);
            self.time_elapsed = new_time;

            Ok(TimedJointSample {
                angle,
                time_elapsed: new_time,
            })
        });

        if result.is_err() {
            self.is_finished = true;
        }

        Some(result)
    }
}

#[cfg(test)]
#[path = "../tests/unit/timed_joint_trajectory.rs"]
mod tests;
