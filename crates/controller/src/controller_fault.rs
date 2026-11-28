/// Motivo por el que se perdió la referencia de posición.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControllerFault {
    ExecutionFailed,
    Cancelled,
    PositionLost,
}
