use executor::PreparedPioTrajectory;
use planner::MotorSteps;

use crate::{ControllerError, ControllerFault, ControllerState};

/// Máquina de estados para la posición comandada y la autorización lógica de movimiento.
#[derive(Debug)]
pub struct MotionController {
    state: ControllerState,
}

impl MotionController {
    pub const fn new() -> Self {
        Self {
            state: ControllerState::Unreferenced,
        }
    }

    pub fn state(&self) -> ControllerState {
        self.state
    }

    /// Devuelve la posición comandada conocida mientras el robot está detenido y referenciado.
    pub fn commanded_position(&self) -> Option<MotorSteps> {
        match self.state {
            ControllerState::Idle { position } => Some(position),
            ControllerState::Ready { start, .. } => Some(start),
            _ => None,
        }
    }

    pub fn target_position(&self) -> Option<MotorSteps> {
        match self.state {
            ControllerState::Ready { target, .. } | ControllerState::Running { target, .. } => {
                Some(target)
            }
            _ => None,
        }
    }

    /// Registra una posición confirmada por la capa de referenciación.
    pub fn reference_position(&mut self, position: MotorSteps) -> Result<(), ControllerError> {
        match self.state {
            ControllerState::Unreferenced => {
                self.state = ControllerState::Idle { position };
                Ok(())
            }
            _ => Err(ControllerError::InvalidState),
        }
    }

    pub fn accept_prepared(
        &mut self,
        trajectory: &PreparedPioTrajectory<'_>,
    ) -> Result<(), ControllerError> {
        let ControllerState::Idle { position } = self.state else {
            return Err(ControllerError::InvalidState);
        };
        if position != trajectory.start_position() {
            return Err(ControllerError::StartPositionMismatch);
        }

        self.state = ControllerState::Ready {
            start: position,
            target: trajectory.target_position(),
        };
        Ok(())
    }

    /// Pasa a ejecución lógica. La capa de hardware debe iniciar la trayectoria aceptada.
    pub fn start(&mut self) -> Result<(), ControllerError> {
        let ControllerState::Ready { start, target } = self.state else {
            return Err(ControllerError::InvalidState);
        };
        self.state = ControllerState::Running { start, target };
        Ok(())
    }

    /// Confirma la posición comandada tras el éxito del ejecutor, incluida la ejecución de END.
    pub fn finish_success(&mut self) -> Result<(), ControllerError> {
        let ControllerState::Running { target, .. } = self.state else {
            return Err(ControllerError::InvalidState);
        };
        self.state = ControllerState::Idle { position: target };
        Ok(())
    }

    pub fn discard_prepared(&mut self) -> Result<(), ControllerError> {
        let ControllerState::Ready { start, .. } = self.state else {
            return Err(ControllerError::InvalidState);
        };
        self.state = ControllerState::Idle { position: start };
        Ok(())
    }

    /// Registra el fallo de una ejecución. Tras una cancelación, la capa de hardware debe haber
    /// detenido el ejecutor y recuperado sus recursos antes de confirmar el fallo.
    pub fn finish_failure(&mut self, reason: ControllerFault) -> Result<(), ControllerError> {
        if !matches!(self.state, ControllerState::Running { .. }) {
            return Err(ControllerError::InvalidState);
        }
        self.state = ControllerState::Fault { reason };
        Ok(())
    }

    pub fn clear_fault(&mut self) -> Result<(), ControllerError> {
        if !matches!(self.state, ControllerState::Fault { .. }) {
            return Err(ControllerError::InvalidState);
        }
        self.state = ControllerState::Unreferenced;
        Ok(())
    }

    pub fn invalidate_position(&mut self) -> Result<(), ControllerError> {
        match self.state {
            ControllerState::Idle { .. } | ControllerState::Ready { .. } => {
                self.state = ControllerState::Fault {
                    reason: ControllerFault::PositionLost,
                };
                Ok(())
            }
            _ => Err(ControllerError::InvalidState),
        }
    }
}

impl Default for MotionController {
    fn default() -> Self {
        Self::new()
    }
}
