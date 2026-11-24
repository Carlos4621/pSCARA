use core::iter::FusedIterator;

use planner::StepTimeBase;

use crate::{
    ExecutionCommand, ExecutorError, PioCommandWord, PioFrameSplitter,
    packed_pin_frame::LEVEL_MASK, pin_frame::STEP_MASK, pio_cycles_per_tick,
};

/// Convierte una secuencia lógica en palabras PIO bajo demanda hasta `End` o el primer error.
pub struct PioCommandIterator<I> {
    source: I,
    cycles_per_tick: u64,
    current_levels: u8,
    splitter: Option<PioFrameSplitter>,
    first_frame: bool,
    finished: bool,
}

impl<I> PioCommandIterator<I>
where
    I: Iterator<Item = Result<ExecutionCommand, ExecutorError>>,
{
    /// Valida los niveles iniciales y la relación de frecuencias sin avanzar la fuente.
    pub fn new(
        source: I,
        initial_levels: u8,
        timebase: StepTimeBase,
        pio_frequency_hz: u64,
    ) -> Result<Self, ExecutorError> {
        if initial_levels > LEVEL_MASK {
            return Err(ExecutorError::InvalidPinLevels);
        }
        if initial_levels & STEP_MASK != 0 {
            return Err(ExecutorError::StepPinsNotLow);
        }

        Ok(Self {
            source,
            cycles_per_tick: pio_cycles_per_tick(timebase, pio_frequency_hz)?,
            current_levels: initial_levels,
            splitter: None,
            first_frame: true,
            finished: false,
        })
    }
}

impl<I> Iterator for PioCommandIterator<I>
where
    I: Iterator<Item = Result<ExecutionCommand, ExecutorError>>,
{
    type Item = Result<PioCommandWord, ExecutorError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.finished {
            return None;
        }

        let result = loop {
            if let Some(word) = self.splitter.as_mut().and_then(Iterator::next) {
                self.current_levels = word
                    .levels()
                    .expect("El divisor solo emite palabras de movimiento");
                self.first_frame = false;
                return Some(Ok(word));
            }
            self.splitter = None;

            let frame = match self.source.next() {
                Some(Ok(ExecutionCommand::Frame(frame))) => frame,
                Some(Ok(ExecutionCommand::End)) if self.current_levels & STEP_MASK != 0 => {
                    break Err(ExecutorError::StepPinsNotLow);
                }
                Some(Ok(ExecutionCommand::End)) => break Ok(PioCommandWord::END),
                Some(Err(error)) => break Err(error),
                None => break Err(ExecutorError::MissingEndCommand),
            };

            let splitter = frame
                .delay_ticks()
                .checked_mul(self.cycles_per_tick)
                .ok_or(ExecutorError::CycleCountOverflow)
                .and_then(|delay_cycles| {
                    PioFrameSplitter::new(
                        self.current_levels,
                        frame.levels(),
                        delay_cycles,
                        self.first_frame,
                    )
                });
            match splitter {
                Ok(splitter) => self.splitter = Some(splitter),
                Err(error) => break Err(error),
            }
        };

        self.finished = true;
        Some(result)
    }
}

impl<I> FusedIterator for PioCommandIterator<I> where
    I: Iterator<Item = Result<ExecutionCommand, ExecutorError>>
{
}
