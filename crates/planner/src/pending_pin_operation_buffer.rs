use crate::{PinOperationBatch, PlannerError, TimedPinOperation};

/// Búfer prestado de grupos pendientes con ticks estrictamente crecientes.
/// Los grupos ocupados se almacenan en el prefijo de longitud `len()`.
#[derive(Debug)]
pub struct PendingPinOperationBuffer<'a> {
    buffer: &'a mut [PinOperationBatch],
    len: usize,
}

impl<'a> PendingPinOperationBuffer<'a> {
    pub fn new(buffer: &'a mut [PinOperationBatch]) -> Self {
        Self { buffer, len: 0 }
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn capacity(&self) -> usize {
        self.buffer.len()
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Inserta en orden por tick y combina las operaciones que comparten un tick.
    /// Si la inserción falla, el búfer conserva su contenido y su longitud.
    pub fn insert(&mut self, operation: TimedPinOperation) -> Result<(), PlannerError> {
        match self.buffer[..self.len].binary_search_by_key(&operation.tick, |batch| batch.tick) {
            Ok(index) => self.buffer[index].insert(operation),
            Err(index) => {
                if self.len == self.capacity() {
                    return Err(PlannerError::InsufficientPinOperationBuffer);
                }

                let mut batch = PinOperationBatch::new(operation.tick);
                batch.insert(operation)?;

                self.buffer.copy_within(index..self.len, index + 1);
                self.buffer[index] = batch;
                self.len += 1;
                Ok(())
            }
        }
    }

    pub fn peek_first(&self) -> Option<PinOperationBatch> {
        self.buffer[..self.len].first().copied()
    }

    pub fn pop_first(&mut self) -> Option<PinOperationBatch> {
        let first = self.peek_first()?;
        self.buffer.copy_within(1..self.len, 0);
        self.len -= 1;
        Some(first)
    }
}
