use crate::PlannerError;

/// Determina qué operaciones preceden a todas las operaciones de los eventos futuros.
/// Supone eventos con ticks estrictamente crecientes y cambios DIR en el último tick permitido.
#[derive(Debug, Clone, Copy)]
pub struct PinOperationReleaseCursor {
    dir_setup_ticks: u64,
    last_step_tick: Option<u64>,
    finished: bool,
}

impl PinOperationReleaseCursor {
    pub fn new(dir_setup_ticks: u64) -> Result<Self, PlannerError> {
        if dir_setup_ticks == 0 {
            return Err(PlannerError::InvalidStepDirTiming);
        }

        Ok(Self {
            dir_setup_ticks,
            last_step_tick: None,
            finished: false,
        })
    }

    /// Registra el tick de un evento procesado, incluso si el evento carece de pulsos.
    pub fn observe(&mut self, step_tick: u64) -> Result<(), PlannerError> {
        if self.finished || self.last_step_tick.is_some_and(|last| step_tick <= last) {
            return Err(PlannerError::InvalidTicks);
        }

        self.last_step_tick = Some(step_tick);
        Ok(())
    }

    /// Indica si el tick está dentro del límite inclusivo de extracción segura.
    /// Antes de terminar, el límite se obtiene restando el setup de DIR al último tick observado.
    pub fn can_release(&self, tick: u64) -> bool {
        self.finished
            || self
                .last_step_tick
                .and_then(|last| last.checked_sub(self.dir_setup_ticks))
                .is_some_and(|safe_tick| tick <= safe_tick)
    }

    /// Marca la fuente como agotada correctamente y habilita la extracción de todos los ticks.
    pub fn finish(&mut self) {
        self.finished = true;
    }
}
