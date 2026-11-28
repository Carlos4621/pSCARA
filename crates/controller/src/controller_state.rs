use planner::MotorSteps;

use crate::ControllerFault;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControllerState {
    Unreferenced,
    Idle {
        position: MotorSteps,
    },
    Ready {
        start: MotorSteps,
        target: MotorSteps,
    },
    Running {
        start: MotorSteps,
        target: MotorSteps,
    },
    Fault {
        reason: ControllerFault,
    },
}
