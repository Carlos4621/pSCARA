use core::iter::FusedIterator;

use crate::{
    PendingPinOperationBuffer, PinOperationBatch, PinOperationReleaseCursor, PlannerError,
    ScheduledStepDirEvent, expand_step_dir_event,
};

/// Agrupa las operaciones de los eventos programados y las entrega en orden cronológico.
/// Usa el búfer prestado para retener los grupos que aún pueden recibir operaciones.
/// El primer error termina la iteración. El agotamiento normal entrega los grupos pendientes.
pub struct PinOperationBatchIterator<'a, I> {
    source: I,
    pending: PendingPinOperationBuffer<'a>,
    release_cursor: PinOperationReleaseCursor,
    step_high_ticks: u64,
    dir_setup_ticks: u64,
    source_finished: bool,
    finished: bool,
}

impl<'a, I> PinOperationBatchIterator<'a, I>
where
    I: Iterator<Item = Result<ScheduledStepDirEvent, PlannerError>>,
{
    /// Usa las duraciones HIGH y setup de DIR de la configuración que programó la fuente.
    pub fn new(
        source: I,
        step_high_ticks: u64,
        dir_setup_ticks: u64,
        buffer: &'a mut [PinOperationBatch],
    ) -> Result<Self, PlannerError> {
        if step_high_ticks == 0 {
            return Err(PlannerError::InvalidStepDirTiming);
        }

        Ok(Self {
            source,
            pending: PendingPinOperationBuffer::new(buffer),
            release_cursor: PinOperationReleaseCursor::new(dir_setup_ticks)?,
            step_high_ticks,
            dir_setup_ticks,
            source_finished: false,
            finished: false,
        })
    }

    fn enqueue_event(&mut self, event: ScheduledStepDirEvent) -> Result<(), PlannerError> {
        let mut release_cursor = self.release_cursor;
        release_cursor.observe(event.step_tick)?;

        let dir_tick = event.step_tick.checked_sub(self.dir_setup_ticks);
        for (step, change) in [
            (event.first_step, event.first_dir_change),
            (event.second_step, event.second_dir_change),
        ] {
            if change.is_some_and(|change| !step || Some(change.tick) != dir_tick) {
                return Err(PlannerError::DirChangeNotSchedulable);
            }
        }

        let operations = expand_step_dir_event(event, self.step_high_ticks)?;
        for operation in operations.into_iter().flatten() {
            self.pending.insert(operation)?;
        }

        self.release_cursor = release_cursor;
        Ok(())
    }
}

impl<I> Iterator for PinOperationBatchIterator<'_, I>
where
    I: Iterator<Item = Result<ScheduledStepDirEvent, PlannerError>>,
{
    type Item = Result<PinOperationBatch, PlannerError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.finished {
            return None;
        }

        loop {
            if self
                .pending
                .peek_first()
                .is_some_and(|batch| self.release_cursor.can_release(batch.tick))
            {
                return self.pending.pop_first().map(Ok);
            }

            if self.source_finished {
                self.finished = true;
                return None;
            }

            match self.source.next() {
                Some(event) => {
                    if let Err(error) = event.and_then(|event| self.enqueue_event(event)) {
                        self.finished = true;
                        return Some(Err(error));
                    }
                }
                None => {
                    self.source_finished = true;
                    self.release_cursor.finish();
                }
            }
        }
    }
}

impl<I> FusedIterator for PinOperationBatchIterator<'_, I> where
    I: Iterator<Item = Result<ScheduledStepDirEvent, PlannerError>>
{
}
