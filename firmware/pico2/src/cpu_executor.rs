use executor::PioCommandWord;
use rp235x_hal::{
    pac::PIO0,
    pio::{
        Buffers, InstalledProgram, PIO, PIOBuilder, PinDir, PinState, SM0, ShiftDirection,
        UninitStateMachine,
    },
    timer::{CopyableTimer0, Timer},
};

const IRQ_END: u8 = 1 << 0;
const IRQ_INVALID_COMMAND: u8 = 1 << 1;
// Wokwi marca TX como llena con cuatro palabras incluso con FJOIN_TX activo.
const TX_FIFO_CAPACITY: usize = 4;
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
}

pub struct CpuExecutor {
    sm: UninitStateMachine<(PIO0, SM0)>,
    program: InstalledProgram<PIO0>,
    output_gpios: [u8; 4],
    clock_divisor: u16,
}

impl CpuExecutor {
    /// Los GPIO deben ser consecutivos, en orden STEP 1, DIR 1, STEP 2, DIR 2.
    pub fn new(
        sm: UninitStateMachine<(PIO0, SM0)>,
        program: InstalledProgram<PIO0>,
        output_gpios: [u8; 4],
        clock_divisor: u16,
    ) -> Self {
        Self {
            sm,
            program,
            output_gpios,
            clock_divisor,
        }
    }

    fn validate(commands: &[PioCommandWord], initial_levels: u8) -> Result<(), PioExecutionError> {
        if initial_levels & !LEVEL_MASK != 0 || initial_levels & STEP_MASK != 0 {
            return Err(PioExecutionError::InvalidInitialLevels);
        }

        let Some((last, frames)) = commands.split_last() else {
            return Err(PioExecutionError::InvalidCommandSequence);
        };
        if !last.is_end() || frames.iter().any(|word| word.is_end()) {
            return Err(PioExecutionError::InvalidCommandSequence);
        }

        let final_levels = frames
            .last()
            .and_then(|word| word.levels())
            .unwrap_or(initial_levels);
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
        commands: &[PioCommandWord],
        initial_levels: u8,
        timeout_us: u64,
    ) -> (Self, Result<(), PioExecutionError>) {
        if let Err(error) = Self::validate(commands, initial_levels) {
            return (self, Err(error));
        }

        let Self {
            sm,
            program,
            output_gpios,
            clock_divisor,
        } = self;
        let (mut sm, rx, mut tx) = PIOBuilder::from_installed_program(program)
            .out_pins(output_gpios[0], output_gpios.len() as u8)
            .out_shift_direction(ShiftDirection::Right)
            .autopull(false)
            .clock_divisor_fixed_point(clock_divisor, 0)
            .buffers(Buffers::RxTx)
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

        let preload_count = commands.len().min(TX_FIFO_CAPACITY);
        let mut next_command = 0;
        while next_command < preload_count && tx.write(commands[next_command].bits()) {
            next_command += 1;
        }

        pio.clear_irq(IRQ_END | IRQ_INVALID_COMMAND);
        tx.clear_stalled_flag();

        let (mut sm, result) = if next_command != preload_count {
            (sm, Err(PioExecutionError::PreloadFailed))
        } else {
            let started_at = timer.get_counter().ticks();
            let running_sm = sm.start();
            let result = loop {
                let irqs = pio.get_irq_raw();
                // TXSTALL se conserva incluso si PIO ya alcanzó IRQ_END.
                if tx.has_stalled() {
                    break Err(PioExecutionError::FifoUnderrun);
                }
                if irqs & IRQ_INVALID_COMMAND != 0 {
                    break Err(PioExecutionError::InvalidCommand);
                }
                if irqs & IRQ_END != 0 {
                    if next_command != commands.len() || !tx.is_empty() {
                        break Err(PioExecutionError::PrematureEnd);
                    }
                    break Ok(());
                }
                if timer.get_counter().ticks().wrapping_sub(started_at) >= timeout_us {
                    break Err(PioExecutionError::Timeout);
                }
                if next_command < commands.len() && tx.write(commands[next_command].bits()) {
                    next_command += 1;
                }
            };
            (running_sm.stop(), result)
        };

        if result.is_err() {
            sm.set_pins([
                (output_gpios[0], PinState::Low),
                (output_gpios[2], PinState::Low),
            ]);
        }
        sm.clear_fifos();
        let (sm, program) = sm.uninit(rx, tx);

        (
            Self {
                sm,
                program,
                output_gpios,
                clock_divisor,
            },
            result,
        )
    }
}
