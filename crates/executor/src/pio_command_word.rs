use crate::{ExecutorError, packed_pin_frame::LEVEL_MASK};

const TYPE_MASK: u32 = 1;
const WAIT_COUNTER_SHIFT: u32 = 1;
const LEVELS_SHIFT: u32 = 28;

/// Palabra del protocolo PIO con el tipo en el bit 0.
///
/// Las tramas de movimiento guardan el contador de espera en los bits 1–27 y
/// los cuatro niveles de pin en los bits 28–31. El contador pertenece al programa PIO.
/// La única palabra de control definida es [`Self::END`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(transparent)]
pub struct PioCommandWord(u32);

impl PioCommandWord {
    pub const MAX_WAIT_COUNTER: u32 = (1_u32 << 27) - 1;

    pub const END: Self = Self(1);

    pub fn frame(wait_counter: u32, levels: u8) -> Result<Self, ExecutorError> {
        if levels > LEVEL_MASK {
            return Err(ExecutorError::InvalidPinLevels);
        }
        if wait_counter > Self::MAX_WAIT_COUNTER {
            return Err(ExecutorError::WaitCounterOutOfRange);
        }

        let bits = (u32::from(levels) << LEVELS_SHIFT) | (wait_counter << WAIT_COUNTER_SHIFT);
        Ok(Self(bits))
    }

    pub const fn bits(&self) -> u32 {
        self.0
    }

    pub const fn is_end(&self) -> bool {
        self.0 == Self::END.0
    }

    pub const fn levels(&self) -> Option<u8> {
        if self.is_end() {
            None
        } else {
            Some((self.0 >> LEVELS_SHIFT) as u8)
        }
    }

    pub const fn wait_counter(&self) -> Option<u32> {
        if self.is_end() {
            None
        } else {
            Some((self.0 >> WAIT_COUNTER_SHIFT) & Self::MAX_WAIT_COUNTER)
        }
    }

    pub fn from_bits(bits: u32) -> Result<Self, ExecutorError> {
        if bits & TYPE_MASK != 0 && bits != Self::END.0 {
            return Err(ExecutorError::InvalidPioCommandWord);
        }

        Ok(Self(bits))
    }
}
