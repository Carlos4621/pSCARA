use crate::MotorSteps;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepDirection {
    Positive,
    Negative,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AxisStepMove {
    pub direction: StepDirection,
    pub pulses: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotorStepMove {
    first: AxisStepMove,
    second: AxisStepMove,
}

impl MotorStepMove {
    pub fn new(origin: MotorSteps, target: MotorSteps) -> Self {
        Self {
            first: Self::get_axis_move(origin.first, target.first),
            second: Self::get_axis_move(origin.second, target.second),
        }
    }

    pub fn first(&self) -> AxisStepMove {
        self.first
    }

    pub fn second(&self) -> AxisStepMove {
        self.second
    }

    fn get_axis_move(origin: i64, target: i64) -> AxisStepMove {
        AxisStepMove {
            direction: if origin <= target {
                StepDirection::Positive
            } else {
                StepDirection::Negative
            },
            pulses: target.abs_diff(origin),
        }
    }
}

#[cfg(test)]
#[path = "../tests/unit/motor_step_move.rs"]
mod tests;
