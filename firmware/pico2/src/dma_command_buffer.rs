use executor::{ExecutorError, PioCommandBlock, PioCommandBlockReader, PioCommandWord};
use rp235x_hal::dma::ReadTarget;

// TRANS_COUNT reserva los cuatro bits superiores para el modo de transferencia.
const MAX_DMA_WORDS: usize = 0x0fff_ffff;

/// Almacenamiento exclusivo para las palabras PIO que leerá DMA.
#[must_use]
pub struct DmaCommandBuffer {
    storage: &'static mut [u32],
    len: usize,
}

impl DmaCommandBuffer {
    pub fn new(storage: &'static mut [u32]) -> Self {
        assert!(storage.len() <= MAX_DMA_WORDS);
        Self { storage, len: 0 }
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn capacity(&self) -> usize {
        self.storage.len()
    }

    pub fn clear(&mut self) {
        self.len = 0;
    }

    pub fn as_slice(&self) -> &[u32] {
        &self.storage[..self.len]
    }

    #[allow(
        dead_code,
        reason = "la prueba carga los búferes mediante el lector de bloques"
    )]
    pub fn push(&mut self, command: PioCommandWord) -> bool {
        if self.len == self.capacity() {
            return false;
        }
        self.storage[self.len] = command.bits();
        self.len += 1;
        true
    }

    /// Lee el siguiente bloque directamente en la memoria que usará DMA.
    pub fn fill_from<I>(
        &mut self,
        reader: &mut PioCommandBlockReader<I>,
    ) -> Result<Option<PioCommandBlock>, ExecutorError>
    where
        I: Iterator<Item = Result<PioCommandWord, ExecutorError>>,
    {
        self.clear();
        let block = reader.next_block(self.storage)?;
        if let Some(block) = block {
            self.len = block.len;
        }
        Ok(block)
    }
}

// SAFETY: El slice mutable posee almacenamiento exclusivo y estático en una
// dirección estable. El rango de lectura se limita a len, que nunca supera la
// capacidad. El HAL toma el búfer por valor durante la transferencia, de modo
// que el llamador solo puede modificarlo tras recuperar su propiedad.
unsafe impl ReadTarget for DmaCommandBuffer {
    type ReceivedWord = u32;

    fn rx_treq() -> Option<u8> {
        None
    }

    fn rx_address_count(&self) -> (u32, u32) {
        (self.storage.as_ptr() as u32, self.len as u32)
    }

    fn rx_increment(&self) -> bool {
        true
    }
}
