use core::iter::FusedIterator;

use crate::{
    ExecutorError, PIO_FRAME_OVERHEAD_CYCLES, PIO_MAX_FRAME_CYCLES, PioCommandWord,
    packed_pin_frame::LEVEL_MASK, pio_wait_counter,
};

/// Divide una espera en palabras PIO y aplica los niveles objetivo en la última.
pub struct PioFrameSplitter {
    previous_levels: u8,
    target_levels: u8,
    remaining_cycles: u64,
    first_frame: bool,
    finished: bool,
}

impl PioFrameSplitter {
    /// `first_frame` indica que la primera palabra inicia la trayectoria.
    /// En ese caso, los ciclos de arranque quedan fuera de `delay_cycles`.
    pub fn new(
        previous_levels: u8,
        target_levels: u8,
        delay_cycles: u64,
        first_frame: bool,
    ) -> Result<Self, ExecutorError> {
        if previous_levels > LEVEL_MASK || target_levels > LEVEL_MASK {
            return Err(ExecutorError::InvalidPinLevels);
        }
        if !first_frame && delay_cycles < PIO_FRAME_OVERHEAD_CYCLES {
            return Err(ExecutorError::DelayTooShort);
        }

        Ok(Self {
            previous_levels,
            target_levels,
            remaining_cycles: delay_cycles,
            first_frame,
            finished: false,
        })
    }
}

impl Iterator for PioFrameSplitter {
    type Item = PioCommandWord;

    fn next(&mut self) -> Option<Self::Item> {
        if self.finished {
            return None;
        }

        let startup_cycles = if self.first_frame {
            PIO_FRAME_OVERHEAD_CYCLES
        } else {
            0
        };
        let max_cycles = PIO_MAX_FRAME_CYCLES - startup_cycles;
        let chunk = if self.remaining_cycles > max_cycles {
            // Reserva el mínimo que consumirá la siguiente palabra.
            max_cycles.min(self.remaining_cycles - PIO_FRAME_OVERHEAD_CYCLES)
        } else {
            self.remaining_cycles
        };
        let wait_counter = pio_wait_counter(chunk + startup_cycles)
            .expect("El fragmento debe ser representable en una palabra PIO");

        self.remaining_cycles -= chunk;
        self.first_frame = false;
        self.finished = self.remaining_cycles == 0;
        let levels = if self.finished {
            self.target_levels
        } else {
            self.previous_levels
        };

        Some(
            PioCommandWord::frame(wait_counter, levels)
                .expect("El contador y los niveles ya están validados"),
        )
    }
}

impl FusedIterator for PioFrameSplitter {}
