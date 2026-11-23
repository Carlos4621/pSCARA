use core::iter::FusedIterator;

use planner::{PinOperationBatch, PlannerError};

use crate::{PinFrame, PinFrameEncoder};

/// Codifica grupos bajo demanda y termina tras el primer error o el agotamiento de la fuente.
pub struct PinFrameIterator<I> {
    source: I,
    encoder: PinFrameEncoder,
    finished: bool,
}

impl<I> PinFrameIterator<I>
where
    I: Iterator<Item = Result<PinOperationBatch, PlannerError>>,
{
    /// Usa un encoder inicializado con las mismas direcciones que el planificador y los pines.
    pub fn new(source: I, encoder: PinFrameEncoder) -> Self {
        Self {
            source,
            encoder,
            finished: false,
        }
    }
}

impl<I> Iterator for PinFrameIterator<I>
where
    I: Iterator<Item = Result<PinOperationBatch, PlannerError>>,
{
    type Item = Result<PinFrame, PlannerError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.finished {
            return None;
        }

        let Some(batch) = self.source.next() else {
            self.finished = true;
            return None;
        };

        let frame = batch.and_then(|batch| self.encoder.encode(batch));
        if frame.is_err() {
            self.finished = true;
        }
        Some(frame)
    }
}

impl<I> FusedIterator for PinFrameIterator<I> where
    I: Iterator<Item = Result<PinOperationBatch, PlannerError>>
{
}
