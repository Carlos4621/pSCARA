use utilities::UtilitiesErrors;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KinematicsError {
    UnreachablePoint,
    InvalidArmLength,
    InvalidPoint,
    NumericalFailure,
    InvalidAngle,
    InvalidTolerance,
    InvalidScale,
    Utilities(UtilitiesErrors),
}

impl From<UtilitiesErrors> for KinematicsError {
    fn from(error: UtilitiesErrors) -> Self {
        Self::Utilities(error)
    }
}
