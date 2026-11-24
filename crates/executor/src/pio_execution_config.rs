use planner::{StepDirTiming, StepDirection, StepTimeBase};

use crate::PinFrameEncoder;

/// Configuración temporal y estado inicial de los pines para una ejecución PIO.
#[derive(Debug, Clone, Copy)]
pub struct PioExecutionConfig {
    pub timebase: StepTimeBase,
    pub timing: StepDirTiming,
    pub pio_frequency_hz: u64,

    pub initial_first_direction: StepDirection,
    pub initial_second_direction: StepDirection,

    pub first_positive_is_high: bool,
    pub second_positive_is_high: bool,
}

impl PioExecutionConfig {
    /// Devuelve los niveles físicos iniciales de DIR y ambos pines STEP en LOW.
    pub fn initial_levels(&self) -> u8 {
        self.pin_frame_encoder().levels()
    }

    pub(crate) fn pin_frame_encoder(&self) -> PinFrameEncoder {
        PinFrameEncoder::new(
            self.initial_first_direction,
            self.initial_second_direction,
            self.first_positive_is_high,
            self.second_positive_is_high,
        )
    }
}
