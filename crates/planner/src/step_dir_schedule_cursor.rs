use crate::{
    PlannerError, ScheduledStepDirEvent, StepDirEvent, StepDirState, StepDirTiming, StepDirection,
    StepTimeBase, schedule_step_dir_command,
};

/// Programa eventos STEP/DIR en orden creciente y conserva el historial de cada motor.
#[derive(Debug)]
pub struct StepDirScheduleCursor {
    state: StepDirState,
    previous_event_tick: Option<u64>,
    previous_first_step: Option<u64>,
    previous_second_step: Option<u64>,
    timing: StepDirTiming,
    timebase: StepTimeBase,
    min_step_interval: u64,
}

impl StepDirScheduleCursor {
    /// Calcula el intervalo STEP con los tiempos HIGH y LOW redondeados por separado.
    /// Considera las direcciones iniciales conocidas estables desde el tick cero.
    pub fn new(
        timing: StepDirTiming,
        timebase: StepTimeBase,
        initial_first_direction: Option<StepDirection>,
        initial_second_direction: Option<StepDirection>,
    ) -> Result<Self, PlannerError> {
        let min_step_interval = timing.min_discrete_step_interval_ticks(timebase)?;

        Ok(Self {
            state: StepDirState::new(initial_first_direction, initial_second_direction),
            previous_event_tick: None,
            previous_first_step: None,
            previous_second_step: None,
            timing,
            timebase,
            min_step_interval,
        })
    }

    /// Programa un evento y actualiza el historial solo si supera todas las comprobaciones.
    pub fn schedule(&mut self, event: StepDirEvent) -> Result<ScheduledStepDirEvent, PlannerError> {
        let tick = event.tick;
        if self
            .previous_event_tick
            .is_some_and(|previous| tick <= previous)
        {
            return Err(PlannerError::InvalidTicks);
        }

        let mut next_state = self.state;
        let command = next_state.prepare(event);

        // Todo STEP anterior pertenece a un evento con un tick menor que el actual.
        for (step, previous_step) in [
            (command.first_step, self.previous_first_step),
            (command.second_step, self.previous_second_step),
        ] {
            if step
                && previous_step.is_some_and(|previous| tick - previous < self.min_step_interval)
            {
                return Err(PlannerError::StepIntervalTooShort);
            }
        }

        let scheduled = schedule_step_dir_command(
            command,
            self.previous_first_step,
            self.previous_second_step,
            self.timing,
            self.timebase,
        )?;

        self.state = next_state;
        if scheduled.first_step {
            self.previous_first_step = Some(tick);
        }
        if scheduled.second_step {
            self.previous_second_step = Some(tick);
        }
        self.previous_event_tick = Some(tick);
        Ok(scheduled)
    }
}
