/// Error al solicitar una operación al controlador.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControllerError {
    InvalidState,
    StartPositionMismatch,
}
