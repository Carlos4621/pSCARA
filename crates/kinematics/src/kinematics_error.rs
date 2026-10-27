#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KinematicsError {
    UnreachablePoint,
    ArmWithInvalidLength,
    InvalidPoint,
    NumericalFailure,
}
