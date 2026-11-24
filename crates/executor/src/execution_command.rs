use crate::PackedPinFrame;

/// Comando lógico de una secuencia de ejecución.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExecutionCommand {
    Frame(PackedPinFrame),
    /// Indica que la secuencia terminó normalmente con ambos pines STEP en LOW.
    End,
}
