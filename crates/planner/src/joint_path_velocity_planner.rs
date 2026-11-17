use kinematics::JointAngles;

use crate::{
    JointAccelerationLimits, JointVelocityLimits, MotionProfile, PlannerError, profile_math::{product_ratio, sqrt_product_ratio},
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

fn segment_path_limits(
    start: JointAngles,
    end: JointAngles,
    velocity_limits: &JointVelocityLimits,
    acceleration_limits: &JointAccelerationLimits,
) -> Result<(f64, f64, f64), PlannerError> {
    let d1 = end.first_theta() - start.first_theta();
    let d2 = end.second_theta() - start.second_theta();

    for displacement in [d1, d2] {
        utilities::validate_finite(displacement).map_err(|_| PlannerError::NumericalFailure)?;
    }

    let length = d1.abs().max(d2.abs());

    if length == 0.0 {
        return Err(PlannerError::ZeroDisplacementMotion);
    }

    let joints = [
        (
            d1.abs(),
            velocity_limits.first_joint_max_velocity(),
            acceleration_limits.first_joint_max_acceleration(),
        ),
        (
            d2.abs(),
            velocity_limits.second_joint_max_velocity(),
            acceleration_limits.second_joint_max_acceleration(),
        ),
    ];

    let mut max_velocity = f64::MAX;
    let mut max_acceleration = f64::MAX;

    for (displacement, velocity, acceleration) in joints {
        if displacement == 0.0 {
            continue;
        }

        let velocity_cap = product_ratio(
            &[velocity, length],
            &[displacement],
        );

        let acceleration_cap = product_ratio(
            &[acceleration, length],
            &[displacement],
        );

        max_velocity = max_velocity.min(velocity_cap);
        max_acceleration = max_acceleration.min(acceleration_cap);
    }

    for value in [max_velocity, max_acceleration] {
        utilities::validate_non_zero_finite_positive(value)
            .map_err(|_| PlannerError::NumericalFailure)?;
    }

    Ok((length, max_velocity, max_acceleration))
}

#[cfg(test)]
#[path = "../tests/unit/joint_path_velocity_planner.rs"]
mod tests;
