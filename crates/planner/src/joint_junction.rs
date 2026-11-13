use kinematics::JointAngles;

use crate::{
    CoordinatedJointMotionProfile, JointAccelerationLimits, JointVelocityLimits, MotionProfile,
    PlannerError,
    profile_math::{product_ratio, sqrt_product_ratio},
};

const MAX_ROUNDING_ADJUSTMENTS: u8 = 4;

/// Tries the theoretical junction speed, then at most four adjacent smaller
/// representable speeds to correct boundary rounding. Both profiles must be
/// feasible; exhausting the adjustments returns `PlannerError::NumericalFailure`.
pub fn calculate_max_junction_progress_speeds(
    a: JointAngles,
    b: JointAngles,
    c: JointAngles,
    velocity_limits: &JointVelocityLimits,
    acceleration_limits: &JointAccelerationLimits,
) -> Result<(f64, f64), PlannerError> {
    calculate_with_rounding_limit(
        a,
        b,
        c,
        velocity_limits,
        acceleration_limits,
        MAX_ROUNDING_ADJUSTMENTS,
    )
}

fn calculate_with_rounding_limit(
    a: JointAngles,
    b: JointAngles,
    c: JointAngles,
    velocity_limits: &JointVelocityLimits,
    acceleration_limits: &JointAccelerationLimits,
    max_rounding_adjustments: u8,
) -> Result<(f64, f64), PlannerError> {
    if !CoordinatedJointMotionProfile::are_segments_same_direction(a, b, c)? {
        return Ok((0.0, 0.0));
    }

    let (first_velocity, first_acceleration) =
        MotionProfile::calculate_progress_limits(a, b, velocity_limits, acceleration_limits)?;
    let (second_velocity, second_acceleration) =
        MotionProfile::calculate_progress_limits(b, c, velocity_limits, acceleration_limits)?;

    let first_cap = first_velocity.min(sqrt_product_ratio(&[2.0, first_acceleration], &[]));
    let second_cap = second_velocity.min(sqrt_product_ratio(&[2.0, second_acceleration], &[]));

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

    for adjustment in 0..=max_rounding_adjustments {
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
                    // The candidate is bounded by sqrt(2*a), so an
                    // infeasible unit-distance ramp here is boundary rounding.
                    Err(PlannerError::InfeasibleMotion) => feasible = false,
                    Err(error) => return Err(error),
                }
            }
            if feasible {
                return Ok((first_speed, second_speed));
            }
        }
        if adjustment < max_rounding_adjustments {
            first_speed = first_speed.next_down();
        }
    }

    Err(PlannerError::NumericalFailure)
}

#[cfg(test)]
#[path = "../tests/unit/joint_junction.rs"]
mod tests;
