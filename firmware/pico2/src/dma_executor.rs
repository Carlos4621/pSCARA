use executor::PioCommandWord;
use rp235x_hal::{
    dma::{CH0, Channel, Pace, SingleChannel, single_buffer},
    pac::{self, PIO0},
    pio::{
        Buffers, InstalledProgram, PIO, PIOBuilder, PinDir, PinState, Running, Rx, SM0,
        ShiftDirection, StateMachine, Stopped, Tx, UninitStateMachine,
    },
    timer::{CopyableTimer0, Timer},
};

use crate::dma_command_buffer::DmaCommandBuffer;

pub(super) const IRQ_END: u8 = 1 << 0;
pub(super) const IRQ_INVALID_COMMAND: u8 = 1 << 1;
pub(super) const TX_FIFO_CAPACITY: usize = 8;
pub(super) const PRELOAD_TIMEOUT_US: u64 = 10_000;
const STEP_MASK: u8 = 0b0101;
const LEVEL_MASK: u8 = 0b1111;

type StepDirSM = (PIO0, SM0);
type DmaTransfer = single_buffer::Transfer<Channel<CH0>, DmaCommandBuffer, Tx<StepDirSM>>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PioExecutionError {
    InvalidInitialLevels,
    InvalidCommandSequence,
    PreloadFailed,
    FifoUnderrun,
    InvalidCommand,
    PrematureEnd,
    Timeout,
    DmaIncomplete,
    DmaBusError,
    Cancelled,
}

pub struct DmaExecutor {
    sm: UninitStateMachine<StepDirSM>,
    program: InstalledProgram<PIO0>,
    dma_channel: Channel<CH0>,
    output_gpios: [u8; 4],
    clock_divisor: u16,
}

enum ExecutionPhase {
    Preloading {
        sm: StateMachine<StepDirSM, Stopped>,
        started_at: u64,
    },
    Running {
        sm: StateMachine<StepDirSM, Running>,
        started_at: u64,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExecutionStatus {
    Preloading,
    Running,
}

/// Posee la máquina PIO, el canal DMA y el búfer hasta finalizar o cancelar la ejecución.
#[must_use]
pub struct ActiveDmaExecution {
    phase: ExecutionPhase,
    transfer: DmaTransfer,
    rx: Rx<StepDirSM>,
    output_gpios: [u8; 4],
    clock_divisor: u16,
    timeout_us: u64,
}

#[must_use]
pub enum ExecutionPoll {
    Pending(ActiveDmaExecution),
    Finished {
        executor: DmaExecutor,
        buffer: DmaCommandBuffer,
        result: Result<(), PioExecutionError>,
    },
}

pub(super) fn pio_has_stalled() -> bool {
    // SAFETY: lectura volátil de FDEBUG de PIO0 sin efectos laterales.
    unsafe { (&*pac::PIO0::ptr()).fdebug().read().txstall().bits() & 1 != 0 }
}

fn dma_has_bus_error() -> bool {
    // SAFETY: lectura volátil del estado de DMA0 sin efectos laterales.
    unsafe {
        (&*pac::DMA::ptr())
            .ch(0)
            .ch_ctrl_trig()
            .read()
            .ahb_error()
            .bit_is_set()
    }
}

impl DmaExecutor {
    /// Los GPIO deben ser consecutivos, en orden STEP 1, DIR 1, STEP 2, DIR 2.
    pub fn new(
        sm: UninitStateMachine<StepDirSM>,
        program: InstalledProgram<PIO0>,
        dma_channel: Channel<CH0>,
        output_gpios: [u8; 4],
        clock_divisor: u16,
    ) -> Self {
        Self {
            sm,
            program,
            dma_channel,
            output_gpios,
            clock_divisor,
        }
    }

    fn validate(commands: &[u32], initial_levels: u8) -> Result<(), PioExecutionError> {
        if initial_levels & !LEVEL_MASK != 0 || initial_levels & STEP_MASK != 0 {
            return Err(PioExecutionError::InvalidInitialLevels);
        }

        let Some((last, frames)) = commands.split_last() else {
            return Err(PioExecutionError::InvalidCommandSequence);
        };
        if *last != PioCommandWord::END.bits() {
            return Err(PioExecutionError::InvalidCommandSequence);
        }

        let mut final_levels = initial_levels;
        for &bits in frames {
            let word = PioCommandWord::from_bits(bits)
                .map_err(|_| PioExecutionError::InvalidCommandSequence)?;
            final_levels = word
                .levels()
                .ok_or(PioExecutionError::InvalidCommandSequence)?;
        }
        if final_levels & STEP_MASK != 0 {
            return Err(PioExecutionError::InvalidCommandSequence);
        }

        Ok(())
    }

    /// Configura PIO e inicia la precarga por DMA.
    pub fn start(
        self,
        pio: &mut PIO<PIO0>,
        timer: &Timer<CopyableTimer0>,
        buffer: DmaCommandBuffer,
        initial_levels: u8,
        timeout_us: u64,
    ) -> Result<ActiveDmaExecution, (Self, DmaCommandBuffer, PioExecutionError)> {
        if let Err(error) = Self::validate(buffer.as_slice(), initial_levels) {
            return Err((self, buffer, error));
        }

        let Self {
            sm,
            program,
            dma_channel,
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

        let mut dma_config = single_buffer::Config::new(dma_channel, buffer, tx);
        dma_config.pace(Pace::PreferSink);
        let started_at = timer.get_counter().ticks();
        let transfer = dma_config.start();

        Ok(ActiveDmaExecution {
            phase: ExecutionPhase::Preloading { sm, started_at },
            transfer,
            rx,
            output_gpios,
            clock_divisor,
            timeout_us,
        })
    }

    /// Ejecuta la secuencia hasta terminar y devuelve los recursos y el búfer.
    pub fn execute(
        self,
        pio: &mut PIO<PIO0>,
        timer: &Timer<CopyableTimer0>,
        buffer: DmaCommandBuffer,
        initial_levels: u8,
        timeout_us: u64,
    ) -> (Self, DmaCommandBuffer, Result<(), PioExecutionError>) {
        let mut active = match self.start(pio, timer, buffer, initial_levels, timeout_us) {
            Ok(active) => active,
            Err((executor, buffer, error)) => return (executor, buffer, Err(error)),
        };

        loop {
            match active.poll(pio, timer) {
                ExecutionPoll::Pending(next) => active = next,
                ExecutionPoll::Finished {
                    executor,
                    buffer,
                    result,
                } => return (executor, buffer, result),
            }
        }
    }
}

impl ActiveDmaExecution {
    pub fn status(&self) -> ExecutionStatus {
        match &self.phase {
            ExecutionPhase::Preloading { .. } => ExecutionStatus::Preloading,
            ExecutionPhase::Running { .. } => ExecutionStatus::Running,
        }
    }

    pub fn running_elapsed_us(&self, timer: &Timer<CopyableTimer0>) -> Option<u64> {
        match &self.phase {
            ExecutionPhase::Preloading { .. } => None,
            ExecutionPhase::Running { started_at, .. } => {
                Some(timer.get_counter().ticks().wrapping_sub(*started_at))
            }
        }
    }

    /// Cancela la ejecución y recupera los recursos y el búfer.
    /// La posición mecánica resultante no está garantizada.
    pub fn cancel(self) -> (DmaExecutor, DmaCommandBuffer, Result<(), PioExecutionError>) {
        self.finalize(Err(PioExecutionError::Cancelled))
    }

    /// Consulta el estado y devuelve el control mientras la ejecución sigue activa.
    pub fn poll(mut self, pio: &mut PIO<PIO0>, timer: &Timer<CopyableTimer0>) -> ExecutionPoll {
        let now = timer.get_counter().ticks();
        if dma_has_bus_error() {
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
                // DMA puede terminar mientras PIO aún ejecuta las últimas palabras.
                if irqs & IRQ_END != 0 && self.transfer.is_done() {
                    return self.finish(Ok(()));
                }
                if now.wrapping_sub(started_at) >= self.timeout_us {
                    return self.finish(Err(PioExecutionError::Timeout));
                }
            }
        }

        ExecutionPoll::Pending(self)
    }

    fn finish(self, result: Result<(), PioExecutionError>) -> ExecutionPoll {
        let (executor, buffer, result) = self.finalize(result);
        ExecutionPoll::Finished {
            executor,
            buffer,
            result,
        }
    }

    fn finalize(
        self,
        mut result: Result<(), PioExecutionError>,
    ) -> (DmaExecutor, DmaCommandBuffer, Result<(), PioExecutionError>) {
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

        let step_pins_low = [
            (output_gpios[0], PinState::Low),
            (output_gpios[2], PinState::Low),
        ];
        let execution_failed = result.is_err();
        if execution_failed {
            sm.set_pins(step_pins_low);
        }

        let (dma_channel, buffer, tx) = if transfer.is_done() {
            transfer.wait()
        } else {
            transfer.abort()
        };

        let dma_registers = dma_channel.ch();
        let dma_status = dma_registers.ch_ctrl_trig().read();
        if dma_status.ahb_error().bit_is_set() {
            // Reconocer los flags persistentes para que la siguiente llamada empiece limpia.
            dma_registers.ch_al1_ctrl().modify(|_, w| {
                w.read_error()
                    .clear_bit_by_one()
                    .write_error()
                    .clear_bit_by_one()
            });
            result = Err(PioExecutionError::DmaBusError);
        } else if tx.has_stalled() {
            result = Err(PioExecutionError::FifoUnderrun);
        } else if result.is_ok() {
            if dma_registers.ch_trans_count().read().count().bits() != 0 {
                result = Err(PioExecutionError::DmaIncomplete);
            } else if !tx.is_empty() {
                result = Err(PioExecutionError::PrematureEnd);
            }
        }

        if !execution_failed && result.is_err() {
            sm.set_pins(step_pins_low);
        }
        sm.clear_fifos();
        let (sm, program) = sm.uninit(rx, tx);

        (
            DmaExecutor {
                sm,
                program,
                dma_channel,
                output_gpios,
                clock_divisor,
            },
            buffer,
            result,
        )
    }
}
