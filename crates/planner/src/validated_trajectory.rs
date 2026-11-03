use kinematics::JointAngles;

use crate::{JointTrajectory, PlannerError};

#[derive(Debug)]
pub struct ValidatedTrajectory<'a> {
    trajectory: JointTrajectory<'a>,
}

impl<'a> ValidatedTrajectory<'a> {
    pub(crate) fn new(trajectory: JointTrajectory<'a>) -> Self {
        Self { trajectory }
    }
}

impl Iterator for ValidatedTrajectory<'_> {
    type Item = Result<JointAngles, PlannerError>;

    fn next(&mut self) -> Option<Self::Item> {
        self.trajectory.next()
    }
}
