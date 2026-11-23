use crate::PlannerError;

/// Tiempos mínimos de las señales STEP/DIR en nanosegundos.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StepDirTiming {
    step_high_ns: u64,
    step_low_ns: u64,
    dir_setup_ns: u64,
    dir_hold_ns: u64,
}

impl StepDirTiming {
    /// Devuelve [`PlannerError::InvalidStepDirTiming`] si algún tiempo es cero.
    pub fn new(
        step_high_ns: u64,
        step_low_ns: u64,
        dir_setup_ns: u64,
        dir_hold_ns: u64,
    ) -> Result<Self, PlannerError> {
        if step_high_ns == 0 || step_low_ns == 0 || dir_setup_ns == 0 || dir_hold_ns == 0 {
            return Err(PlannerError::InvalidStepDirTiming);
        }

        Ok(Self {
            step_high_ns,
            step_low_ns,
            dir_setup_ns,
            dir_hold_ns,
        })
    }

    /// Duración mínima del nivel alto de STEP.
    pub fn step_high_ns(&self) -> u64 {
        self.step_high_ns
    }

    /// Duración mínima del nivel bajo de STEP entre pulsos.
    pub fn step_low_ns(&self) -> u64 {
        self.step_low_ns
    }

    /// Tiempo mínimo entre establecer DIR y el flanco activo de STEP.
    pub fn dir_setup_ns(&self) -> u64 {
        self.dir_setup_ns
    }

    /// Tiempo mínimo que DIR debe conservar su valor después del flanco activo de STEP.
    pub fn dir_hold_ns(&self) -> u64 {
        self.dir_hold_ns
    }
}
