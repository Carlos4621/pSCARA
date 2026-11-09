use kinematics::JointAngles;

use crate::PlannerError;

#[derive(Debug)]
pub struct JointVelocity {
    first_joint_max_velocity: f64,
    second_joint_max_velocity: f64,
}

impl JointVelocity {
    pub fn new(
        first_joint_max_velocity: f64,
        second_joint_max_velocity: f64,
    ) -> Result<Self, PlannerError> {
        if !first_joint_max_velocity.is_finite()
            || !second_joint_max_velocity.is_finite()
            || first_joint_max_velocity <= 0.0
            || second_joint_max_velocity <= 0.0
        {
            return Err(PlannerError::InvalidJointVelocity);
        }

        Ok(Self {
            first_joint_max_velocity,
            second_joint_max_velocity,
        })
    }

    pub fn min_duration_between(
        &self,
        start: JointAngles,
        end: JointAngles,
    ) -> Result<f64, PlannerError> {
        let first_delta = (end.first_theta() - start.first_theta()).abs();
        let second_delta = (end.second_theta() - start.second_theta()).abs();

        if first_delta == 0.0 && second_delta == 0.0 {
            return Ok(0.0); // El único caso válido para cero
        }

        if !first_delta.is_finite() || !second_delta.is_finite() {
            return Err(PlannerError::AngleNonFinite);
        }

        let first_joint_time = first_delta / self.first_joint_max_velocity;
        let second_joint_time = second_delta / self.second_joint_max_velocity;
        let max_time = first_joint_time.max(second_joint_time);

        if !max_time.is_finite() || max_time == 0.0 {
            return Err(PlannerError::NumericalFailure);
        }

        Ok((first_delta / self.first_joint_max_velocity)
            .max(second_delta / self.second_joint_max_velocity))
    }
}
