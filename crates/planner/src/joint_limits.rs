use kinematics::JointAngles;

use crate::{JointRange, PlannerError};

#[derive(Debug)]
pub struct JointLimits {
    first: JointRange,
    second: JointRange,
}

impl JointLimits {
    pub fn new(first: JointRange, second: JointRange) -> Self {
        Self { first, second }
    }

    pub fn contains(&self, angles: JointAngles) -> Result<bool, PlannerError> {
        let first_contains = self.first.contains(angles.first_theta)?;
        let second_contains = self.second.contains(angles.second_theta)?;

        // Evitamos cortocircuito
        Ok(first_contains && second_contains)
    }
}
