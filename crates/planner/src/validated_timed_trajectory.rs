use crate::{PlannerError, TimedJointSample, TimedJointTrajectory};

/// A timed iterator checked through [`TimedJointTrajectory::into_validated`].
///
/// External callers cannot bypass validation using the internal constructor:
///
/// ```compile_fail
/// use planner::{TimedJointTrajectory, ValidatedTimedTrajectory};
///
/// fn bypass<'a>(trajectory: TimedJointTrajectory<'a>) -> ValidatedTimedTrajectory<'a> {
///     ValidatedTimedTrajectory::new(trajectory)
/// }
/// ```
///
/// The wrapped iterator is private as well:
///
/// ```compile_fail
/// use planner::{TimedJointTrajectory, ValidatedTimedTrajectory};
///
/// fn bypass<'a>(trajectory: TimedJointTrajectory<'a>) -> ValidatedTimedTrajectory<'a> {
///     ValidatedTimedTrajectory { trajectory }
/// }
/// ```
#[derive(Debug)]
pub struct ValidatedTimedTrajectory<'a> {
    trajectory: TimedJointTrajectory<'a>,
}

impl<'a> ValidatedTimedTrajectory<'a> {
    pub(crate) fn new(trajectory: TimedJointTrajectory<'a>) -> Self {
        Self { trajectory }
    }
}

impl Iterator for ValidatedTimedTrajectory<'_> {
    type Item = Result<TimedJointSample, PlannerError>;

    fn next(&mut self) -> Option<Self::Item> {
        self.trajectory.next()
    }
}
