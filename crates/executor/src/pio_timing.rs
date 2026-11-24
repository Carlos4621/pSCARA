use planner::StepTimeBase;

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

/// Devuelve la cantidad exacta de ciclos PIO por tick del planificador.
/// La frecuencia PIO debe ser un múltiplo de la frecuencia del planificador
/// y permitir al menos [`PIO_FRAME_OVERHEAD_CYCLES`] ciclos por tick.
pub fn pio_cycles_per_tick(
    timebase: StepTimeBase,
    pio_frequency_hz: u64,
) -> Result<u64, ExecutorError> {
    let ticks_per_second = timebase.ticks_per_second();
    if pio_frequency_hz == 0 || !pio_frequency_hz.is_multiple_of(ticks_per_second) {
        return Err(ExecutorError::InvalidPioClockRatio);
    }

    let cycles_per_tick = pio_frequency_hz / ticks_per_second;
    if cycles_per_tick < PIO_FRAME_OVERHEAD_CYCLES {
        return Err(ExecutorError::InvalidPioClockRatio);
    }

    Ok(cycles_per_tick)
}
