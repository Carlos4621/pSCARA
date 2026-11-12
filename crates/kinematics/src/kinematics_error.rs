#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KinematicsError {
    UnreachablePoint,
    InvalidArmLength,
    InvalidPoint,
    NumericalFailure,
    InvalidAngle,
    InvalidTolerance,
    InvalidScale,
}
