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
            let start = points[i];
            let end = points[i + 1];

            let d1 = end.first_theta() - start.first_theta();
            let d2 = end.second_theta() - start.second_theta();

            for displacement in [d1, d2] {
                utilities::validate_finite(displacement)
                    .map_err(|_| PlannerError::NumericalFailure)?;
            }

            let length = d1.abs().max(d2.abs());

            utilities::validate_non_zero_finite_positive(length)
                .map_err(|_| PlannerError::ZeroDisplacementMotion)?;

            let initial_progress_speed = speeds[i] / length;
            let final_progress_speed = speeds[i + 1] / length;

            for (speed, progress_speed) in [
                (speeds[i], initial_progress_speed),
                (speeds[i + 1], final_progress_speed),
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
}
