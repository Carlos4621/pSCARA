use planner::{PinOperationBatch, PlannerError, StepDirection, StepPinLevel};

const FIRST_STEP_MASK: u8 = 1 << 0;
const FIRST_DIR_MASK: u8 = 1 << 1;
const SECOND_STEP_MASK: u8 = 1 << 2;
const SECOND_DIR_MASK: u8 = 1 << 3;
pub(crate) const STEP_MASK: u8 = FIRST_STEP_MASK | SECOND_STEP_MASK;

/// Estado completo de los pines tras una espera relativa en ticks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PinFrame {
    pub delay_ticks: u64,
    /// Los bits 0–3 representan STEP 1, DIR 1, STEP 2 y DIR 2. Los bits 4–7 valen cero.
    pub levels: u8,
}

/// Convierte grupos de operaciones en tramas con tiempos relativos y niveles completos.
pub struct PinFrameEncoder {
    levels: u8,
    previous_tick: Option<u64>,
    first_positive_is_high: bool,
    second_positive_is_high: bool,
}

impl PinFrameEncoder {
    /// Comienza con ambos pines STEP en LOW y aplica la polaridad de cada pin DIR.
    pub fn new(
        initial_first_direction: StepDirection,
        initial_second_direction: StepDirection,
        first_positive_is_high: bool,
        second_positive_is_high: bool,
    ) -> Self {
        let mut levels = 0;
        set_pin_level(
            &mut levels,
            FIRST_DIR_MASK,
            direction_is_high(initial_first_direction, first_positive_is_high),
        );
        set_pin_level(
            &mut levels,
            SECOND_DIR_MASK,
            direction_is_high(initial_second_direction, second_positive_is_high),
        );

        Self {
            levels,
            previous_tick: None,
            first_positive_is_high,
            second_positive_is_high,
        }
    }

    /// Devuelve la máscara actual de los cuatro pines.
    pub fn levels(&self) -> u8 {
        self.levels
    }

    /// Codifica un grupo y conserva el estado del encoder ante un tick inválido.
    pub fn encode(&mut self, batch: PinOperationBatch) -> Result<PinFrame, PlannerError> {
        let delay_ticks = match self.previous_tick {
            Some(previous_tick) if batch.tick <= previous_tick => {
                return Err(PlannerError::InvalidTicks);
            }
            Some(previous_tick) => batch.tick - previous_tick,
            None => batch.tick,
        };

        let updates = [
            (
                FIRST_STEP_MASK,
                batch.first_step.map(|level| level == StepPinLevel::High),
            ),
            (
                FIRST_DIR_MASK,
                batch
                    .first_direction
                    .map(|direction| direction_is_high(direction, self.first_positive_is_high)),
            ),
            (
                SECOND_STEP_MASK,
                batch.second_step.map(|level| level == StepPinLevel::High),
            ),
            (
                SECOND_DIR_MASK,
                batch
                    .second_direction
                    .map(|direction| direction_is_high(direction, self.second_positive_is_high)),
            ),
        ];
        let mut levels = self.levels;
        for (mask, high) in updates {
            if let Some(high) = high {
                set_pin_level(&mut levels, mask, high);
            }
        }

        self.levels = levels;
        self.previous_tick = Some(batch.tick);
        Ok(PinFrame {
            delay_ticks,
            levels,
        })
    }
}

fn direction_is_high(direction: StepDirection, positive_is_high: bool) -> bool {
    match direction {
        StepDirection::Positive => positive_is_high,
        StepDirection::Negative => !positive_is_high,
    }
}

fn set_pin_level(levels: &mut u8, mask: u8, high: bool) {
    if high {
        *levels |= mask;
    } else {
        *levels &= !mask;
    }
}
