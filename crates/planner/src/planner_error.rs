use kinematics::KinematicsError;
use trajectory::TrajectoryError;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlannerError {
    Trajectory(TrajectoryError),
    Kinematics(KinematicsError),
    AngleNonFinite,
}
