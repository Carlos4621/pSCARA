use core::iter::FusedIterator;

use planner::PlannerError;

use crate::{
    ExecutorError, PackedPinFrame, PackedPinFrameSplitter, PinFrame, packed_pin_frame::LEVEL_MASK,
};

/// Empaqueta tramas bajo demanda y termina tras el primer error o el agotamiento de la fuente.
pub struct PackedPinFrameIterator<I> {
    source: I,
    current_levels: u8,
    splitter: Option<PackedPinFrameSplitter>,
    finished: bool,
}

impl<I> PackedPinFrameIterator<I>
where
    I: Iterator<Item = Result<PinFrame, PlannerError>>,
{
    /// Valida los niveles iniciales de los pines sin avanzar la fuente.
    pub fn new(source: I, initial_levels: u8) -> Result<Self, ExecutorError> {
        if initial_levels > LEVEL_MASK {
            return Err(ExecutorError::InvalidPinLevels);
        }

        Ok(Self {
            source,
            current_levels: initial_levels,
            splitter: None,
            finished: false,
        })
    }
}

impl<I> Iterator for PackedPinFrameIterator<I>
where
    I: Iterator<Item = Result<PinFrame, PlannerError>>,
{
    type Item = Result<PackedPinFrame, ExecutorError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.finished {
            return None;
        }

        loop {
            if let Some(packed) = self.splitter.as_mut().and_then(Iterator::next) {
                self.current_levels = packed.levels();
                return Some(Ok(packed));
            }
            self.splitter = None;

            let Some(frame) = self.source.next() else {
                self.finished = true;
                return None;
            };
            let splitter = frame
                .map_err(ExecutorError::from)
                .and_then(|frame| PackedPinFrameSplitter::new(self.current_levels, frame));

            match splitter {
                Ok(splitter) => self.splitter = Some(splitter),
                Err(error) => {
                    self.finished = true;
                    return Some(Err(error));
                }
            }
        }
    }
}

impl<I> FusedIterator for PackedPinFrameIterator<I> where
    I: Iterator<Item = Result<PinFrame, PlannerError>>
{
}
