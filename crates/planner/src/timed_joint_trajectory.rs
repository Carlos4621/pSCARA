use kinematics::JointAngles;

use crate::{JointVelocityLimits, PlannerError, ValidatedTrajectory};

#[derive(Debug, Clone, Copy)]
pub struct TimedJointSample {
    pub angle: JointAngles,
    pub time_elapsed: f64,
}

#[derive(Debug)]
pub struct TimedJointTrajectory<'a> {
    trajectory: ValidatedTrajectory<'a>,
    velocity_limits: JointVelocityLimits,
    previous_angle: Option<JointAngles>,
    time_elapsed: f64,
    is_finished: bool,
}

impl<'a> TimedJointTrajectory<'a> {
    pub fn new(trajectory: ValidatedTrajectory<'a>, velocity_limits: JointVelocityLimits) -> Self {
        Self {
            trajectory,
            velocity_limits,
            previous_angle: None,
            time_elapsed: 0.0,
            is_finished: false,
        }
    }
}

impl Iterator for TimedJointTrajectory<'_> {
    type Item = Result<TimedJointSample, PlannerError>;

    fn next(&mut self) -> Option<Self::Item> {
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

            if !new_time.is_finite() {
                return Err(PlannerError::TimeElapsedOverflow);
            }

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
