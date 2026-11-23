use kinematics::KinematicsError;
use trajectory::TrajectoryError;
use utilities::UtilitiesErrors;

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
    /// La suma de tiempos excedió el rango finito.
    TimeElapsedOverflow,
    /// El redondeo borró un operando positivo al sumar tiempos.
    TimeElapsedPrecisionLost,
    /// El cursor de ejecución recibió un tiempo anterior al último válido.
    TimeWentBackwards,
    InvalidJointAcceleration,
    InvalidMaxVelocity,
    InvalidMaxAcceleration,
    InvalidTime,
    InvalidInitialSpeed,
    InvalidFinalSpeed,
    InvalidBoundarySpeed,
    InfeasibleMotion,
    ZeroDisplacementMotion,
    MorePointsThanSpeedsBuffer,
    InsufficientPoints,
    DiscontinuousJointVelocity,
    ProfileInstanceNotFound,
    InvalidSteps,
    InvalidTicks,
    EndBeforeStart,
    InsufficientTicksForSteps,
    MultipleStepsOnTick,
    BoundariesWithoutFrontiers,
    InsufficientBoundaryBuffer,
    /// Un motor recorre más de un micropaso al pasar por una frontera entre ticks.
    SubtickMotionNotRepresentable,
    /// Al menos un requisito temporal de STEP/DIR es cero.
    InvalidStepDirTiming,
    /// Dos flancos STEP consecutivos no respetan el intervalo mínimo.
    StepIntervalTooShort,
    /// No existe un tick válido para cambiar DIR entre dos pulsos.
    DirChangeNotSchedulable,
    /// Dos operaciones contradictorias sobre el mismo pin en un tick.
    ConflictingPinOperations,
}

impl PlannerError {
    pub(crate) fn from_time_error(error: UtilitiesErrors) -> Self {
        match error {
            UtilitiesErrors::InvalidTime => Self::InvalidTime,
            UtilitiesErrors::TimeOverflow => Self::TimeElapsedOverflow,
            UtilitiesErrors::TimePrecisionLost => Self::TimeElapsedPrecisionLost,
            _ => Self::NumericalFailure,
        }
    }
}
