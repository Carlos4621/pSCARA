use kinematics::JointAngles;

use crate::{
    CoordinatedJointMotionProfile, JointAccelerationLimits, JointVelocityLimits, PlannerError,
};

#[derive(Debug)]
pub struct CoordinatedJointPathProfiles<'a> {
    points: &'a [JointAngles],
    speeds: &'a [f64],

    velocity_limits: &'a JointVelocityLimits,
    acceleration_limits: &'a JointAccelerationLimits,

    current_segment: usize,
}

impl<'a> CoordinatedJointPathProfiles<'a> {
    pub fn new(
        points: &'a [JointAngles],
        speeds: &'a [f64],
        velocity_limits: &'a JointVelocityLimits,
        acceleration_limits: &'a JointAccelerationLimits,
    ) -> Result<Self, PlannerError> {
        if points.len() < 2 {
            return Err(PlannerError::InsufficientPoints);
        }

        if points.len() != speeds.len() {
            return Err(PlannerError::MorePointsThanSpeedsBuffer);
        }

        for &speed in speeds {
            utilities::validate_finite_positive(speed)
                .map_err(|_| PlannerError::InvalidBoundarySpeed)?;
        }

        if speeds[0] != 0.0 || speeds[speeds.len() - 1] != 0.0 {
            return Err(PlannerError::InvalidBoundarySpeed);
        }

        for i in 1..points.len() - 1 {
            if speeds[i] == 0.0 {
                continue;
            }

            if !CoordinatedJointMotionProfile::are_segments_same_direction(
                points[i - 1],
                points[i],
                points[i + 1],
            )? {
                return Err(PlannerError::DiscontinuousJointVelocity);
            }
        }

        for i in 0..points.len() - 1 {
            Self::build_segment_profile(
                points[i],
                points[i + 1],
                speeds[i],
                speeds[i + 1],
                velocity_limits,
                acceleration_limits,
            )?;
        }

        Ok(Self {
            points,
            speeds,
            velocity_limits,
            acceleration_limits,
            current_segment: 0,
        })
    }

    fn build_segment_profile(
        start: JointAngles,
        end: JointAngles,
        initial_speed: f64,
        final_speed: f64,
        velocity_limits: &JointVelocityLimits,
        acceleration_limits: &JointAccelerationLimits,
    ) -> Result<CoordinatedJointMotionProfile, PlannerError> {
        let d1 = end.first_theta() - start.first_theta();
        let d2 = end.second_theta() - start.second_theta();

        for i in [d1, d2] {
            utilities::validate_finite(i).map_err(|_| PlannerError::NumericalFailure)?;
        }

        let length = d1.abs().max(d2.abs());

        utilities::validate_non_zero_finite_positive(length)
            .map_err(|_| PlannerError::ZeroDisplacementMotion)?;

        let initial_progress_speed = initial_speed / length;
        let final_progress_speed = final_speed / length;

        for (speed, progress_speed) in [
            (initial_speed, initial_progress_speed),
            (final_speed, final_progress_speed),
        ] {
            utilities::validate_finite_positive(progress_speed)
                .map_err(|_| PlannerError::NumericalFailure)?;
            // A positive requested speed must not silently become a stop.
            if speed > 0.0 && progress_speed == 0.0 {
                return Err(PlannerError::NumericalFailure);
            }
        }

        CoordinatedJointMotionProfile::new(
            start,
            end,
            velocity_limits,
            acceleration_limits,
            initial_progress_speed,
            final_progress_speed,
        )
    }
}

impl Iterator for CoordinatedJointPathProfiles<'_> {
    type Item = Result<CoordinatedJointMotionProfile, PlannerError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.current_segment >= self.points.len() - 1 {
            return None;
        }

        let i = self.current_segment;

        let start = self.points[i];
        let end = self.points[i + 1];

        let initial_speed = self.speeds[i];
        let final_speed = self.speeds[i + 1];

        self.current_segment += 1;

        Some(Self::build_segment_profile(
            start,
            end,
            initial_speed,
            final_speed,
            self.velocity_limits,
            self.acceleration_limits,
        ))
    }
}

#[cfg(test)]
#[path = "../tests/unit/coordinated_joint_path_profiles.rs"]
mod tests;
