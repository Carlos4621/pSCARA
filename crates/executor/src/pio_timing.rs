use crate::{ExecutorError, PioCommandWord};

/// Sobrecoste por trama de `step_dir` en ciclos de la máquina de estados.
/// Son cinco instrucciones fijas más la última iteración de `jmp x--`.
pub const PIO_FRAME_OVERHEAD_CYCLES: u64 = 6;

/// Mayor intervalo entre actualizaciones consecutivas de GPIO sin bloqueo de la FIFO TX.
pub const PIO_MAX_FRAME_CYCLES: u64 =
    PioCommandWord::MAX_WAIT_COUNTER as u64 + PIO_FRAME_OVERHEAD_CYCLES;

/// Calcula el intervalo entre actualizaciones consecutivas con datos disponibles en la FIFO TX.
pub fn pio_frame_cycles(wait_counter: u32) -> Result<u64, ExecutorError> {
    if wait_counter > PioCommandWord::MAX_WAIT_COUNTER {
        return Err(ExecutorError::WaitCounterOutOfRange);
    }

    Ok(u64::from(wait_counter) + PIO_FRAME_OVERHEAD_CYCLES)
}

/// Calcula el contador para un intervalo entre actualizaciones consecutivas de GPIO.
pub fn pio_wait_counter(cycles: u64) -> Result<u32, ExecutorError> {
    if cycles < PIO_FRAME_OVERHEAD_CYCLES {
        return Err(ExecutorError::DelayTooShort);
    }
    if cycles > PIO_MAX_FRAME_CYCLES {
        return Err(ExecutorError::WaitCounterOutOfRange);
    }

    Ok((cycles - PIO_FRAME_OVERHEAD_CYCLES) as u32)
}
