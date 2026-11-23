use crate::{ExecutorError, PinFrame};

const LEVEL_BITS: u32 = 4;
const LEVEL_MASK: u8 = (1 << LEVEL_BITS) - 1;

/// Trama con 28 bits de espera en la parte alta y cuatro bits de niveles en la parte baja.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(transparent)]
pub struct PackedPinFrame(u32);

impl PackedPinFrame {
    pub const MAX_DELAY_TICKS: u64 = (1_u64 << (u32::BITS - LEVEL_BITS)) - 1;

    pub const fn from_bits(bits: u32) -> Self {
        Self(bits)
    }

    pub const fn bits(&self) -> u32 {
        self.0
    }

    pub const fn levels(&self) -> u8 {
        (self.0 & LEVEL_MASK as u32) as u8
    }

    pub const fn delay_ticks(&self) -> u64 {
        (self.0 >> LEVEL_BITS) as u64
    }
}

impl TryFrom<PinFrame> for PackedPinFrame {
    type Error = ExecutorError;

    fn try_from(frame: PinFrame) -> Result<Self, Self::Error> {
        if frame.levels > LEVEL_MASK {
            return Err(ExecutorError::InvalidPinLevels);
        }
        if frame.delay_ticks > Self::MAX_DELAY_TICKS {
            return Err(ExecutorError::DelayOutOfRange);
        }

        let delay_bits = (frame.delay_ticks as u32) << LEVEL_BITS;
        Ok(Self(delay_bits | u32::from(frame.levels)))
    }
}

impl From<PackedPinFrame> for PinFrame {
    fn from(frame: PackedPinFrame) -> Self {
        Self {
            delay_ticks: frame.delay_ticks(),
            levels: frame.levels(),
        }
    }
}
