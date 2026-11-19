use core::iter::FusedIterator;

use crate::{CoordinatedStepInterpolator, MotorStepInterval, PlannerError, StepEvent};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimedStepEvent {
    pub tick: u64,
    pub event: StepEvent,
}

#[derive(Debug)]
pub struct TimedStepInterpolator {
    interpolator: CoordinatedStepInterpolator,

    start_tick: u64,
    duration_ticks: u64,

    total_events: u64,
    emitted_events: u64,
}

impl TimedStepInterpolator {
    pub fn new(interval: MotorStepInterval) -> Result<Self, PlannerError> {
        let total_events = interval
            .movement()
            .first()
            .pulses
            .max(interval.movement().second().pulses);
        let duration_ticks = interval.duration_ticks();

        if total_events >= duration_ticks {
            return Err(PlannerError::InsufficientTicksForSteps);
        }

        Ok(Self {
            interpolator: CoordinatedStepInterpolator::new(interval.movement()),
            start_tick: interval.start_tick(),
            duration_ticks,
            total_events,
            emitted_events: 0,
        })
    }
}

impl Iterator for TimedStepInterpolator {
    type Item = TimedStepEvent;

    fn next(&mut self) -> Option<Self::Item> {
        let event = self.interpolator.next()?;

        self.emitted_events += 1;

        let offset = (u128::from(self.emitted_events) * u128::from(self.duration_ticks)).div_ceil(u128::from(self.total_events));

        let offset = u64::try_from(offset).expect("Invariante modificada");

        let tick = self.start_tick + offset;

        Some(TimedStepEvent { tick, event })
    }
}

impl FusedIterator for TimedStepInterpolator {}