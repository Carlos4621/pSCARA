use core::iter::FusedIterator;

use crate::{ExecutorError, PackedPinFrame, PinFrame, packed_pin_frame::LEVEL_MASK};

/// Divide una espera bajo demanda y aplica los niveles objetivo en la última trama.
pub struct PackedPinFrameSplitter {
    previous_levels: u8,
    target_levels: u8,
    remaining_ticks: u64,
    finished: bool,
}

impl PackedPinFrameSplitter {
    pub fn new(previous_levels: u8, frame: PinFrame) -> Result<Self, ExecutorError> {
        if previous_levels > LEVEL_MASK || frame.levels > LEVEL_MASK {
            return Err(ExecutorError::InvalidPinLevels);
        }

        Ok(Self {
            previous_levels,
            target_levels: frame.levels,
            remaining_ticks: frame.delay_ticks,
            finished: false,
        })
    }
}

impl Iterator for PackedPinFrameSplitter {
    type Item = PackedPinFrame;

    fn next(&mut self) -> Option<Self::Item> {
        if self.finished {
            return None;
        }

        let delay_ticks = self.remaining_ticks.min(PackedPinFrame::MAX_DELAY_TICKS);
        self.remaining_ticks -= delay_ticks;
        self.finished = self.remaining_ticks == 0;
        let levels = if self.finished {
            self.target_levels
        } else {
            self.previous_levels
        };

        Some(PackedPinFrame::from_valid_parts(delay_ticks, levels))
    }
}

impl FusedIterator for PackedPinFrameSplitter {}
