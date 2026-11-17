use kinematics::JointAngles;

use crate::{
    JointAccelerationLimits, JointVelocityLimits, MotionProfile, PlannerError,
    profile_math::sqrt_product_ratio,
};

const MAX_ROUNDING_ADJUSTMENTS: u8 = 4;

pub fn plan_joint_path_speeds(
    points: &[JointAngles],
    velocity_limits: &JointVelocityLimits,
    acceleration_limits: &JointAccelerationLimits,
    speeds: &mut [f64],
) -> Result<(), PlannerError> {
    todo!()
}

fn limit_reachable_speed(
    initial_speed: f64,
    acceleration: f64,
    distance: f64,
    speed_limit: f64,
) -> Result<f64, PlannerError> {
    for value in [initial_speed, acceleration, distance, speed_limit] {
        utilities::validate_finite_positive(value).map_err(|_| PlannerError::NumericalFailure)?;
    }

    let calculated_speed = libm::hypot(
        initial_speed,
        sqrt_product_ratio(&[2.0, acceleration, distance], &[]),
    );

    let mut speed = calculated_speed.min(speed_limit);
    for adjustment in 0..=MAX_ROUNDING_ADJUSTMENTS {
        if speed <= initial_speed
            || MotionProfile::speed_change_distance(speed, initial_speed, acceleration) <= distance
        {
            return Ok(speed);
        }
        if adjustment < MAX_ROUNDING_ADJUSTMENTS {
            speed = speed.next_down();
        }
    }

    Err(PlannerError::NumericalFailure)
}

#[cfg(test)]
#[path = "../tests/unit/joint_path_velocity_planner.rs"]
mod tests;
