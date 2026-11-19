use core::iter::FusedIterator;

use crate::{AxisStepMove, MotorStepMove};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StepEvent {
    pub first_step: bool,
    pub second_step: bool,
}

#[derive(Debug)]
pub struct CoordinatedStepInterpolator {
    first_is_primary: bool,
    secondary_motor: AxisStepMove,
    total_events: u64,
    emitted_events: u64,
    error: u128,
}

impl CoordinatedStepInterpolator {
    pub fn new(movement: MotorStepMove) -> Self {
        let first_is_primary = movement.first().pulses >= movement.second().pulses;

        Self {
            first_is_primary,
            secondary_motor: if !first_is_primary {
                movement.first()
            } else {
                movement.second()
            },
            total_events: movement.first().pulses.max(movement.second().pulses),
            emitted_events: 0,
            error: 0,
        }
    }
}

impl Iterator for CoordinatedStepInterpolator {
    type Item = StepEvent;

    fn next(&mut self) -> Option<Self::Item> {
        if self.emitted_events >= self.total_events {
            return None;
        }

        let mut secondary_pulse = false;

        self.error += self.secondary_motor.pulses as u128;

        if self.error >= self.total_events.into() {
            secondary_pulse = true;
            self.error -= self.total_events as u128;
        }

        self.emitted_events += 1;

        Some(StepEvent {
            first_step: if self.first_is_primary {
                true
            } else {
                secondary_pulse
            },
            second_step: if !self.first_is_primary {
                true
            } else {
                secondary_pulse
            },
        })
    }
}

impl FusedIterator for CoordinatedStepInterpolator {}

#[cfg(test)]
#[path = "../tests/unit/coordinated_step_interpolator.rs"]
mod tests;
