use kinematics::KinematicsError;
use trajectory::TrajectoryError;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlannerError {
    Trajectory(TrajectoryError),
    Kinematics(KinematicsError),
    AngleNonFinite,
    MinGreaterThanMax,
    AngleOutOfLimits,
    InitialPositionNotInStart,
    InitialConfigurationMismatch,
    TrajectoryAlreadyStarted,
    InvalidMaxJointStep,
    MaxJointStepExceeded,
    MaxRefinementAttemptsReached,
    SegmentUnreachable,
    InvalidJointVelocity,
    NumericalFailure,
    TimeElapsedOverflow,
    TimeElapsedPrecisionLost,
    InvalidJointAcceleration,
    InvalidMaxVelocity,
    InvalidMaxAcceleration,
    InvalidTime,
    InvalidInitialSpeed,
    InvalidFinalSpeed,
    InvalidBoundarySpeed,
    InfeasibleMotion,
    ZeroDisplacementMotion,
}
