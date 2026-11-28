use executor::{ExecutorError, PioCommandWord, pio_wait_counter};
use rp235x_hal::{
    pac::PIO0,
    pio::PIO,
    timer::{CopyableTimer0, Timer},
};

use crate::streaming_dma_executor::{StreamingDmaExecutor, StreamingPoll};

const SUPERVISION_TIMEOUT_US: u64 = 15_000_000;
pub const SYNTHETIC_INITIAL_LEVELS: u8 = 0b1010;

/// Genera pulsos simultáneos para los dos motores, con ambos DIR en alto.
pub fn synthetic_step_commands(
    pulse_count: usize,
    half_period_cycles: u64,
) -> Result<impl Iterator<Item = Result<PioCommandWord, ExecutorError>>, ExecutorError> {
    let wait_counter = pio_wait_counter(half_period_cycles)?;
    let frame_count = pulse_count
        .checked_mul(2)
        .ok_or(ExecutorError::CycleCountOverflow)?;

    Ok((0..frame_count)
        .map(move |index| {
            let levels = if index.is_multiple_of(2) {
                0b1111
            } else {
                SYNTHETIC_INITIAL_LEVELS
            };
            PioCommandWord::frame(wait_counter, levels)
        })
        .chain(core::iter::once(Ok(PioCommandWord::END))))
}

/// Conserva todas las palabras solo cuando se pudo recorrer la secuencia completa.
pub fn prepare_commands<I>(
    storage: &mut [PioCommandWord],
    commands: Result<I, ExecutorError>,
) -> Option<&[PioCommandWord]>
where
    I: Iterator<Item = Result<PioCommandWord, ExecutorError>>,
{
    let prepared = commands.and_then(|commands| {
        let mut len = 0;
        for command in commands {
            let word = command?;
            let Some(slot) = storage.get_mut(len) else {
                defmt::warn!("Capacidad de preparacion insuficiente: {} palabras", len);
                return Ok(None);
            };
            *slot = word;
            len += 1;
        }
        Ok(Some(&storage[..len]))
    });
    match prepared {
        Ok(words) => words,
        Err(error) => {
            defmt::warn!("Verificacion fallida: {:?}", defmt::Debug2Format(&error));
            None
        }
    }
}

/// FNV-1a de los bytes little-endian de cada palabra, incluido END.
pub fn command_checksum(commands: &[PioCommandWord]) -> u64 {
    const OFFSET: u64 = 0xcbf29ce484222325;
    const PRIME: u64 = 0x100000001b3;

    let mut hash = OFFSET;
    for command in commands {
        for byte in command.bits().to_le_bytes() {
            hash ^= u64::from(byte);
            hash = hash.wrapping_mul(PRIME);
        }
    }
    hash
}

pub fn measure_execution<I>(
    executor: StreamingDmaExecutor,
    pio: &mut PIO<PIO0>,
    timer: &Timer<CopyableTimer0>,
    commands: I,
    initial_levels: u8,
) -> StreamingDmaExecutor
where
    I: Iterator<Item = Result<PioCommandWord, ExecutorError>>,
{
    let started_at = timer.get_counter().ticks();
    let mut active =
        match executor.start(pio, timer, commands, initial_levels, SUPERVISION_TIMEOUT_US) {
            Ok(active) => active,
            Err((executor, error)) => {
                let elapsed_us = timer.get_counter().ticks().wrapping_sub(started_at);
                defmt::warn!(
                    "Inicio fallido tras {} us: {:?}",
                    elapsed_us,
                    defmt::Debug2Format(&error)
                );
                return executor;
            }
        };

    let mut poll_count = 0u64;
    let mut max_poll_us = 0;
    let (executor, report) = loop {
        let poll_started = timer.get_counter().ticks();
        let state = active.poll(pio, timer);
        let poll_elapsed = timer.get_counter().ticks().wrapping_sub(poll_started);
        poll_count += 1;
        max_poll_us = max_poll_us.max(poll_elapsed);

        match state {
            StreamingPoll::Pending(next) => active = next,
            StreamingPoll::Finished { executor, report } => break (executor, report),
        }
    };
    let elapsed_us = timer.get_counter().ticks().wrapping_sub(started_at);

    let stats = report.statistics;
    defmt::info!(
        "Duracion={} us, bloques={}, final={}, recargas={}, recarga total={} us, recarga maxima={} us",
        elapsed_us,
        report.blocks_queued,
        report.final_queued,
        stats.refill_count,
        stats.total_refill_us,
        stats.max_refill_us,
    );
    if stats.refill_count != 0 {
        let mean_refill_us = stats.total_refill_us as f64 / stats.refill_count as f64;
        defmt::info!("Recarga media={} us", mean_refill_us);
    }
    defmt::info!(
        "Consultas={}, poll maximo={} us, resultado={:?}",
        poll_count,
        max_poll_us,
        defmt::Debug2Format(&report.result),
    );
    executor
}
