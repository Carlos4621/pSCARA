#[derive(Debug)]
pub enum KinematicsError {
    UnreachablePoint,
    ArmWithInvalidLength,
    InvalidPoint,
    NumericalFailure,
}
