use crate::{StepDirEvent, StepDirection};

/// Operaciones pendientes de DIR y STEP asociadas a un tick lógico.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StepDirPinCommand {
    pub tick: u64,
    pub first_dir_update: Option<StepDirection>,
    pub second_dir_update: Option<StepDirection>,
    pub first_step: bool,
    pub second_step: bool,
}

/// Conserva la dirección de cada motor al preparar comandos sucesivos.
#[derive(Debug, Clone, Copy)]
pub struct StepDirState {
    first_direction: Option<StepDirection>,
    second_direction: Option<StepDirection>,
}

impl StepDirState {
    /// Inicializa las direcciones conocidas de los pines DIR. `None` indica un estado desconocido.
    pub fn new(
        first_direction: Option<StepDirection>,
        second_direction: Option<StepDirection>,
    ) -> Self {
        Self {
            first_direction,
            second_direction,
        }
    }

    /// Prepara las operaciones del evento y registra los cambios de dirección solicitados.
    pub fn prepare(&mut self, event: StepDirEvent) -> StepDirPinCommand {
        StepDirPinCommand {
            tick: event.tick,
            first_dir_update: Self::update_direction(
                &mut self.first_direction,
                event.first_direction,
            ),
            second_dir_update: Self::update_direction(
                &mut self.second_direction,
                event.second_direction,
            ),
            first_step: event.first_direction.is_some(),
            second_step: event.second_direction.is_some(),
        }
    }

    fn update_direction(
        current: &mut Option<StepDirection>,
        requested: Option<StepDirection>,
    ) -> Option<StepDirection> {
        if requested.is_some() && requested != *current {
            *current = requested;
            requested
        } else {
            None
        }
    }
}
