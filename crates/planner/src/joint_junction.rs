use kinematics::JointAngles;

use crate::{
    CoordinatedJointMotionProfile, JointAccelerationLimits, JointVelocityLimits, MotionProfile,
    PlannerError,
    profile_math::{product_ratio, sqrt_product_ratio},
};

pub fn calculate_max_junction_progress_speeds(
    a: JointAngles,
    b: JointAngles,
    c: JointAngles,
    velocity_limits: &JointVelocityLimits,
    acceleration_limits: JointAccelerationLimits,
) -> Result<(f64, f64), PlannerError> {
    if !CoordinatedJointMotionProfile::are_segments_same_direction(a, b, c)? {
        return Ok((0.0, 0.0));
    }

    let (first_velocity, first_acceleration) =
        MotionProfile::calculate_progress_limits(a, b, velocity_limits, &acceleration_limits)?;
    let (second_velocity, second_acceleration) =
        MotionProfile::calculate_progress_limits(b, c, velocity_limits, &acceleration_limits)?;

    let first_cap = first_velocity.min(sqrt_product_ratio(&[2.0, first_acceleration], &[]));
    let second_cap = second_velocity.min(sqrt_product_ratio(&[2.0, second_acceleration], &[]));

    // Work in the first segment's progress units. Keep all factors separate:
    // the displacement ratio can overflow even when the scaled speed fits.
    let (first_delta, second_delta) = if b.first_theta() != a.first_theta() {
        (
            (b.first_theta() - a.first_theta()).abs(),
            (c.first_theta() - b.first_theta()).abs(),
        )
    } else {
        (
            (b.second_theta() - a.second_theta()).abs(),
            (c.second_theta() - b.second_theta()).abs(),
        )
    };
    let mut first_speed = first_cap.min(product_ratio(&[second_cap, second_delta], &[first_delta]));

    loop {
        utilities::validate_non_zero_finite_positive(first_speed)
            .map_err(|_| PlannerError::NumericalFailure)?;
        let second_speed = product_ratio(&[first_speed, first_delta], &[second_delta]);
        utilities::validate_non_zero_finite_positive(second_speed)
            .map_err(|_| PlannerError::NumericalFailure)?;

        if second_speed <= second_cap {
            let first = MotionProfile::with_boundary_speeds(
                0.0,
                1.0,
                first_acceleration,
                first_velocity,
                0.0,
                first_speed,
            );
            let second = MotionProfile::with_boundary_speeds(
                0.0,
                1.0,
                second_acceleration,
                second_velocity,
                second_speed,
                0.0,
            );
            let mut feasible = true;
            for profile in [first, second] {
                match profile {
                    Ok(_) => {}
                    Err(PlannerError::InfeasibleMotion) => feasible = false,
                    Err(error) => return Err(error),
                }
            }
            if feasible {
                return Ok((first_speed, second_speed));
            }
        }
        first_speed = first_speed.next_down();
    }
}
