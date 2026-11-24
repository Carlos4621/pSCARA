use planner::PlannerError;

/// Errores al preparar instrucciones de ejecución.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExecutorError {
    Planner(PlannerError),
    InvalidPinLevels,
    DelayOutOfRange,
    /// Uno o ambos pines STEP permanecen en nivel alto
    /// al iniciar o finalizar una secuencia.
    StepPinsNotLow,
    WaitCounterOutOfRange,
    InvalidPioCommandWord,
}

impl From<PlannerError> for ExecutorError {
    fn from(error: PlannerError) -> Self {
        Self::Planner(error)
    }
}
