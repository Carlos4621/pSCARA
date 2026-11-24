use core::iter::FusedIterator;

use crate::{
    ExecutionCommand, ExecutorError, PackedPinFrame, packed_pin_frame::LEVEL_MASK,
    pin_frame::STEP_MASK,
};

/// Emite tramas bajo demanda y produce `End` al agotar la fuente con ambos pines STEP en LOW.
pub struct ExecutionCommandIterator<I> {
    source: I,
    current_levels: u8,
    finished: bool,
}

impl<I> ExecutionCommandIterator<I>
where
    I: Iterator<Item = Result<PackedPinFrame, ExecutorError>>,
{
    /// Valida los niveles iniciales y exige ambos pines STEP en LOW sin avanzar la fuente.
    pub fn new(source: I, initial_levels: u8) -> Result<Self, ExecutorError> {
        if initial_levels > LEVEL_MASK {
            return Err(ExecutorError::InvalidPinLevels);
        }
        if initial_levels & STEP_MASK != 0 {
            return Err(ExecutorError::StepPinsNotLow);
        }

        Ok(Self {
            source,
            current_levels: initial_levels,
            finished: false,
        })
    }
}

impl<I> Iterator for ExecutionCommandIterator<I>
where
    I: Iterator<Item = Result<PackedPinFrame, ExecutorError>>,
{
    type Item = Result<ExecutionCommand, ExecutorError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.finished {
            return None;
        }

        let result = match self.source.next() {
            Some(Ok(frame)) => {
                self.current_levels = frame.levels();
                return Some(Ok(ExecutionCommand::Frame(frame)));
            }
            Some(Err(error)) => Err(error),
            None if self.current_levels & STEP_MASK != 0 => Err(ExecutorError::StepPinsNotLow),
            None => Ok(ExecutionCommand::End),
        };

        self.finished = true;
        Some(result)
    }
}

impl<I> FusedIterator for ExecutionCommandIterator<I> where
    I: Iterator<Item = Result<PackedPinFrame, ExecutorError>>
{
}
