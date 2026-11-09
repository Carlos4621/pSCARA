use kinematics::JointAngles;

use crate::{JointVelocityLimits, PlannerError, ValidatedTrajectory};

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

        let next_angle = match self.trajectory.next()? {
            Ok(ok) => ok,
            Err(err) => return Some(Err(err)),
        };

        let previous_angle = self.previous_angle.unwrap_or(next_angle);

        let time_to_reach = self
            .velocity_limits
            .min_duration_between(previous_angle, next_angle);

        match time_to_reach {
            Ok(ok) => {
                self.previous_angle = Some(next_angle);
                self.time_elapsed += ok;

                if !self.time_elapsed.is_finite() {
                    self.is_finished = true;
                    return Some(Err(PlannerError::TimeElapsedOverflow));
                }

                Some(Ok(TimedJointSample {
                    angle: next_angle,
                    time_elapsed: self.time_elapsed,
                }))
            }
            Err(err) => {
                self.is_finished = true;
                Some(Err(err))
            }
        }
    }
}

#[cfg(test)]
#[path = "../tests/unit/timed_joint_trajectory.rs"]
mod tests;
