use executor::PioCommandWord;
use rp235x_hal::{
    dma::{CH0, Channel, Pace, SingleChannel, single_buffer},
    pac::PIO0,
    pio::{
        Buffers, InstalledProgram, PIO, PIOBuilder, PinDir, PinState, SM0, ShiftDirection,
        UninitStateMachine,
    },
    timer::{CopyableTimer0, Timer},
};

const IRQ_END: u8 = 1 << 0;
const IRQ_INVALID_COMMAND: u8 = 1 << 1;
pub(super) const TX_FIFO_CAPACITY: usize = 8;
const PRELOAD_TIMEOUT_US: u64 = 10_000;
const STEP_MASK: u8 = 0b0101;
const LEVEL_MASK: u8 = 0b1111;

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
}

pub struct DmaExecutor {
    sm: UninitStateMachine<(PIO0, SM0)>,
    program: InstalledProgram<PIO0>,
    dma_channel: Channel<CH0>,
    output_gpios: [u8; 4],
    clock_divisor: u16,
}

impl DmaExecutor {
    /// Los GPIO deben ser consecutivos, en orden STEP 1, DIR 1, STEP 2, DIR 2.
    pub fn new(
        sm: UninitStateMachine<(PIO0, SM0)>,
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

    /// Ejecuta la secuencia y devuelve los recursos para la siguiente llamada.
    pub fn execute(
        self,
        pio: &mut PIO<PIO0>,
        timer: &Timer<CopyableTimer0>,
        commands: &'static [u32],
        initial_levels: u8,
        timeout_us: u64,
    ) -> (Self, Result<(), PioExecutionError>) {
        if let Err(error) = Self::validate(commands, initial_levels) {
            return (self, Err(error));
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

        let mut dma_config = single_buffer::Config::new(dma_channel, commands, tx);
        dma_config.pace(Pace::PreferSink);
        let transfer = dma_config.start();

        // DMA llena la FIFO con la máquina detenida. Una secuencia corta cabe completa.
        let preload_started = timer.get_counter().ticks();
        let preload_ok = loop {
            if transfer.is_done() || !pio.irq0().raw().sm0_tx_not_full() {
                break true;
            }
            if timer.get_counter().ticks().wrapping_sub(preload_started) >= PRELOAD_TIMEOUT_US {
                break false;
            }
        };

        let (mut sm, mut result) = if preload_ok {
            let started_at = timer.get_counter().ticks();
            let running_sm = sm.start();
            let result = loop {
                let irqs = pio.get_irq_raw();
                if irqs & IRQ_INVALID_COMMAND != 0 {
                    break Err(PioExecutionError::InvalidCommand);
                }
                // DMA puede terminar mientras PIO aún ejecuta las últimas palabras.
                if irqs & IRQ_END != 0 {
                    if !transfer.is_done() {
                        break Err(PioExecutionError::DmaIncomplete);
                    }
                    break Ok(());
                }
                if timer.get_counter().ticks().wrapping_sub(started_at) >= timeout_us {
                    break Err(PioExecutionError::Timeout);
                }
            };
            (running_sm.stop(), result)
        } else {
            (sm, Err(PioExecutionError::PreloadFailed))
        };

        let step_pins_low = [
            (output_gpios[0], PinState::Low),
            (output_gpios[2], PinState::Low),
        ];
        let execution_failed = result.is_err();
        if execution_failed {
            sm.set_pins(step_pins_low);
        }

        let (dma_channel, _, tx) = if transfer.is_done() {
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
            Self {
                sm,
                program,
                dma_channel,
                output_gpios,
                clock_divisor,
            },
            result,
        )
    }
}
