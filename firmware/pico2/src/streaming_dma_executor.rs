use executor::{ExecutorError, PioCommandBlock, PioCommandBlockReader, PioCommandWord};
use rp235x_hal::{
    dma::{CH0, CH1, Channel},
    pac::PIO0,
    pio::{
        Buffers, InstalledProgram, PIO, PIOBuilder, PinDir, PinState, Rx, SM0, ShiftDirection,
        UninitStateMachine,
    },
    timer::{CopyableTimer0, Timer},
};

use crate::{
    dma_command_buffer::DmaCommandBuffer,
    dma_executor::{
        ExecutionPhase, ExecutionStatus, IRQ_END, IRQ_INVALID_COMMAND, PRELOAD_TIMEOUT_US,
        PioExecutionError, TX_FIFO_CAPACITY, pio_has_stalled, valid_initial_levels,
    },
    dma_pair::{DmaPairTransfer, RefillError},
};

type StepDirSM = (PIO0, SM0);

/// Posee los recursos de PIO y DMA, incluidos los dos búferes reutilizables.
pub struct StreamingDmaExecutor {
    sm: UninitStateMachine<StepDirSM>,
    program: InstalledProgram<PIO0>,
    channel_a: Channel<CH0>,
    channel_b: Channel<CH1>,
    buffer_a: DmaCommandBuffer,
    buffer_b: DmaCommandBuffer,
    output_gpios: [u8; 4],
    clock_divisor: u16,
}

#[must_use = "llama a poll hasta finalizar o a cancel para recuperar el ejecutor"]
pub struct ActiveStreamingExecution<I> {
    phase: ExecutionPhase,
    transfer: DmaPairTransfer,
    reader: PioCommandBlockReader<I>,
    rx: Rx<StepDirSM>,
    output_gpios: [u8; 4],
    clock_divisor: u16,
    timeout_us: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StreamingExecutionReport {
    pub blocks_queued: usize,
    pub final_queued: bool,
    pub result: Result<(), PioExecutionError>,
}

#[must_use]
pub enum StreamingPoll<I> {
    Pending(ActiveStreamingExecution<I>),
    Finished {
        executor: StreamingDmaExecutor,
        report: StreamingExecutionReport,
    },
}

impl StreamingDmaExecutor {
    /// Los GPIO deben ser consecutivos, en orden STEP 1, DIR 1, STEP 2, DIR 2.
    #[allow(
        clippy::too_many_arguments,
        reason = "recibe los recursos exclusivos de PIO y DMA"
    )]
    pub fn new(
        sm: UninitStateMachine<StepDirSM>,
        program: InstalledProgram<PIO0>,
        channel_a: Channel<CH0>,
        channel_b: Channel<CH1>,
        buffer_a: DmaCommandBuffer,
        buffer_b: DmaCommandBuffer,
        output_gpios: [u8; 4],
        clock_divisor: u16,
    ) -> Self {
        assert_eq!(buffer_a.capacity(), buffer_b.capacity());
        // Dos bloques intermedios completos deben exceder la capacidad de la FIFO.
        assert!(buffer_a.capacity() > TX_FIFO_CAPACITY / 2);

        Self {
            sm,
            program,
            channel_a,
            channel_b,
            buffer_a,
            buffer_b,
            output_gpios,
            clock_divisor,
        }
    }

    /// Inicia la precarga con un iterador de `build_verified_pio_commands` o con
    /// una única palabra END para una ejecución sin movimiento.
    pub fn start<I>(
        mut self,
        pio: &mut PIO<PIO0>,
        timer: &Timer<CopyableTimer0>,
        commands: I,
        initial_levels: u8,
        timeout_us: u64,
    ) -> Result<ActiveStreamingExecution<I>, (Self, PioExecutionError)>
    where
        I: Iterator<Item = Result<PioCommandWord, ExecutorError>>,
    {
        if !valid_initial_levels(initial_levels) {
            return Err((self, PioExecutionError::InvalidInitialLevels));
        }

        let mut reader = PioCommandBlockReader::new(commands);
        let first = match fill_block(&mut self.buffer_a, &mut reader) {
            Ok(block) => block,
            Err(error) => return Err((self, error)),
        };
        self.buffer_b.clear();
        if !first.is_final
            && let Err(error) = fill_block(&mut self.buffer_b, &mut reader)
        {
            return Err((self, error));
        }

        let Self {
            sm,
            program,
            channel_a,
            channel_b,
            buffer_a,
            buffer_b,
            output_gpios,
            clock_divisor,
        } = self;
        let (mut sm, rx, tx) = PIOBuilder::from_installed_program(program)
            .out_pins(output_gpios[0], output_gpios.len() as u8)
            .out_shift_direction(ShiftDirection::Right)
            .autopull(false)
            .clock_divisor_fixed_point(clock_divisor, 0)
            .buffers(Buffers::OnlyTx)
            .build(sm);

        sm.clear_fifos();
        sm.set_pins(output_gpios.into_iter().enumerate().map(|(bit, pin)| {
            let state = if initial_levels & (1 << bit) != 0 {
                PinState::High
            } else {
                PinState::Low
            };
            (pin, state)
        }));
        sm.set_pindirs(output_gpios.map(|pin| (pin, PinDir::Output)));
        pio.clear_irq(IRQ_END | IRQ_INVALID_COMMAND);
        tx.clear_stalled_flag();

        let started_at = timer.get_counter().ticks();
        let transfer = DmaPairTransfer::start(channel_a, channel_b, buffer_a, buffer_b, tx);

        Ok(ActiveStreamingExecution {
            phase: ExecutionPhase::Preloading { sm, started_at },
            transfer,
            reader,
            rx,
            output_gpios,
            clock_divisor,
            timeout_us,
        })
    }
}

fn fill_block<I>(
    buffer: &mut DmaCommandBuffer,
    reader: &mut PioCommandBlockReader<I>,
) -> Result<PioCommandBlock, PioExecutionError>
where
    I: Iterator<Item = Result<PioCommandWord, ExecutorError>>,
{
    buffer
        .fill_from(reader)
        .map_err(|error| PioExecutionError::BlockRefill(RefillError::Source(error)))?
        .ok_or(PioExecutionError::BlockRefill(RefillError::MissingBlock))
}

impl<I> ActiveStreamingExecution<I>
where
    I: Iterator<Item = Result<PioCommandWord, ExecutorError>>,
{
    pub fn status(&self) -> ExecutionStatus {
        self.phase.status()
    }

    pub fn running_elapsed_us(&self, timer: &Timer<CopyableTimer0>) -> Option<u64> {
        self.phase.running_elapsed_us(timer)
    }

    /// Consulta el estado y recarga un bloque si DMA ya liberó su búfer.
    pub fn poll(mut self, pio: &mut PIO<PIO0>, timer: &Timer<CopyableTimer0>) -> StreamingPoll<I> {
        let now = timer.get_counter().ticks();
        if self.transfer.has_bus_error() {
            return self.finish(Err(PioExecutionError::DmaBusError));
        }
        if pio_has_stalled() {
            return self.finish(Err(PioExecutionError::FifoUnderrun));
        }

        match self.phase {
            ExecutionPhase::Preloading { sm, .. }
                if self.transfer.is_done() || !pio.irq0().raw().sm0_tx_not_full() =>
            {
                self.phase = ExecutionPhase::Running {
                    sm: sm.start(),
                    started_at: now,
                };
            }
            ExecutionPhase::Preloading { started_at, .. } => {
                if now.wrapping_sub(started_at) >= PRELOAD_TIMEOUT_US {
                    return self.finish(Err(PioExecutionError::PreloadFailed));
                }
            }
            ExecutionPhase::Running { started_at, .. } => {
                let irqs = pio.get_irq_raw();
                if irqs & IRQ_INVALID_COMMAND != 0 {
                    return self.finish(Err(PioExecutionError::InvalidCommand));
                }
                if irqs & IRQ_END != 0 {
                    if !self.transfer.final_queued() {
                        return self.finish(Err(PioExecutionError::PrematureEnd));
                    }
                    if self.transfer.is_done() {
                        return self.finish(Ok(()));
                    }
                }
                if now.wrapping_sub(started_at) >= self.timeout_us {
                    return self.finish(Err(PioExecutionError::Timeout));
                }
                if let Err(error) = self.transfer.refill_next(&mut self.reader) {
                    return self.finish(Err(PioExecutionError::BlockRefill(error)));
                }
            }
        }

        StreamingPoll::Pending(self)
    }

    /// Cancela la ejecución y recupera el ejecutor. Al interrumpir un movimiento,
    /// la posición mecánica final puede diferir de la posición planificada.
    pub fn cancel(self) -> (StreamingDmaExecutor, StreamingExecutionReport) {
        self.finalize(Err(PioExecutionError::Cancelled))
    }

    fn finish(self, result: Result<(), PioExecutionError>) -> StreamingPoll<I> {
        let (executor, report) = self.finalize(result);
        StreamingPoll::Finished { executor, report }
    }

    fn finalize(
        self,
        mut result: Result<(), PioExecutionError>,
    ) -> (StreamingDmaExecutor, StreamingExecutionReport) {
        let Self {
            phase,
            transfer,
            rx,
            output_gpios,
            clock_divisor,
            ..
        } = self;
        let mut sm = match phase {
            ExecutionPhase::Preloading { sm, .. } => sm,
            ExecutionPhase::Running { sm, .. } => sm.stop(),
        };
        sm.set_pins([
            (output_gpios[0], PinState::Low),
            (output_gpios[2], PinState::Low),
        ]);

        let blocks_queued = transfer.blocks_queued();
        let final_queued = transfer.final_queued();
        let resources = transfer.stop();

        if resources.bus_error {
            result = Err(PioExecutionError::DmaBusError);
        } else if resources.tx.has_stalled() {
            result = Err(PioExecutionError::FifoUnderrun);
        } else if result.is_ok() {
            if !resources.completed {
                result = Err(PioExecutionError::DmaIncomplete);
            } else if !resources.tx.is_empty() {
                result = Err(PioExecutionError::PrematureEnd);
            }
        }

        sm.clear_fifos();
        let (sm, program) = sm.uninit(rx, resources.tx);
        let executor = StreamingDmaExecutor {
            sm,
            program,
            channel_a: resources.channel_a,
            channel_b: resources.channel_b,
            buffer_a: resources.buffer_a,
            buffer_b: resources.buffer_b,
            output_gpios,
            clock_divisor,
        };
        let report = StreamingExecutionReport {
            blocks_queued,
            final_queued,
            result,
        };
        (executor, report)
    }
}
