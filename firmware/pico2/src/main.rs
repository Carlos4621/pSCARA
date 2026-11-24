#![no_std]
#![no_main]

mod dma_command_buffer;
#[allow(
    dead_code,
    reason = "la prueba de banco configura el par DMA directamente"
)]
mod dma_executor;
mod dma_pair;

use defmt_rtt as _;
use dma_command_buffer::DmaCommandBuffer;
use dma_executor::{
    IRQ_END, IRQ_INVALID_COMMAND, PRELOAD_TIMEOUT_US, PioExecutionError, pio_has_stalled,
};
use dma_pair::{DmaPairResources, DmaPairTransfer};
use embedded_hal::digital::OutputPin;
use executor::{
    PioCommandBlock, PioCommandBlockReader, PioExecutionConfig, build_verified_pio_commands,
};
use kinematics::JointAngles;
use panic_probe as _;
use planner::{
    CoordinatedJointPathProfiles, JointAccelerationLimits, JointToStepsConverter,
    JointVelocityLimits, PinOperationBatch, StepDirTiming, StepDirection, StepTimeBase,
    TimedCoordinatedJointPathProfiles,
};
use rp235x_hal::{
    self as hal, Clock,
    clocks::init_clocks_and_plls,
    dma::DMAExt,
    entry,
    gpio::{FunctionPio0, PinState as GpioPinState},
    pac::{self, PIO0},
    pio::{Buffers, PIO, PIOBuilder, PIOExt, PinDir, PinState, ShiftDirection},
    timer::{CopyableTimer0, Timer},
};

const XOSC_HZ: u32 = 12_000_000;
const PIO_HZ: u32 = 1_000_000;
const BLOCK_CAPACITY: usize = 6;
const SUPERVISION_TIMEOUT_US: u64 = 3_000_000;

#[unsafe(link_section = ".start_block")]
#[used]
static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

#[unsafe(link_section = ".bi_entries")]
#[used]
static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 3] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    #[cfg(feature = "adafruit-feather-rp2350")]
    hal::binary_info::rp_program_description!(
        c"SCARA planned STEP/DIR bench test (Adafruit Feather RP2350, A0-A3)"
    ),
    #[cfg(not(feature = "adafruit-feather-rp2350"))]
    hal::binary_info::rp_program_description!(c"SCARA planned STEP/DIR bench test"),
];

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = init_clocks_and_plls(
        XOSC_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .expect("no se pudieron inicializar los relojes");
    let system_clock_hz = clocks.system_clock.freq().to_Hz();
    assert!(system_clock_hz.is_multiple_of(PIO_HZ));
    let divisor = u16::try_from(system_clock_hz / PIO_HZ).unwrap();
    assert_ne!(divisor, 0);
    defmt::info!(
        "clk_sys={} Hz, PIO={} Hz, divisor={}",
        system_clock_hz,
        PIO_HZ,
        divisor
    );

    let timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    // PIO usa cuatro GPIO consecutivos, en el orden STEP 1, DIR 1, STEP 2, DIR 2.
    #[cfg(feature = "adafruit-feather-rp2350")]
    let (status_led, step_1, dir_1, step_2, dir_2) = (
        pins.gpio7,
        pins.gpio26,
        pins.gpio27,
        pins.gpio28,
        pins.gpio29,
    );
    #[cfg(not(feature = "adafruit-feather-rp2350"))]
    let (status_led, step_1, dir_1, step_2, dir_2) =
        (pins.gpio25, pins.gpio2, pins.gpio3, pins.gpio4, pins.gpio5);
    let mut led = status_led.into_push_pull_output_in_state(GpioPinState::Low);
    let output_gpios = [
        step_1.id().num,
        dir_1.id().num,
        step_2.id().num,
        dir_2.id().num,
    ];
    let mut step_1 = step_1.into_function::<FunctionPio0>();
    let _dir_1 = dir_1.into_function::<FunctionPio0>();
    let mut step_2 = step_2.into_function::<FunctionPio0>();
    let _dir_2 = dir_2.into_function::<FunctionPio0>();
    step_1.set_input_enable(true);
    step_2.set_input_enable(true);
    let step_masks = [1u32 << output_gpios[0], 1u32 << output_gpios[2]];

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let program = pio_proc::pio_file!(
        "../../crates/executor/pio/step_dir.pio",
        select_program("step_dir")
    );
    let installed = pio.install(&program.program).unwrap();
    let dma = pac.DMA.split(&mut pac.RESETS);
    let points = [
        JointAngles::new(0.0, 0.0).unwrap(),
        JointAngles::new(0.25, -0.25).unwrap(),
        JointAngles::new(0.50, -0.50).unwrap(),
    ];
    let config = PioExecutionConfig {
        timebase: StepTimeBase::new(8).unwrap(),
        timing: StepDirTiming::new(125_000_000, 125_000_000, 1, 1).unwrap(),
        pio_frequency_hz: u64::from(PIO_HZ),
        initial_first_direction: StepDirection::Negative,
        initial_second_direction: StepDirection::Positive,
        first_positive_is_high: true,
        second_positive_is_high: true,
    };

    let velocity_limits = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration_limits = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let path = CoordinatedJointPathProfiles::new(
        &points,
        &[0.0; 3],
        &velocity_limits,
        &acceleration_limits,
    )
    .unwrap();
    let profiles = TimedCoordinatedJointPathProfiles::new(path).unwrap();
    let converter = JointToStepsConverter::new(8.0, 4.0).unwrap();
    let mut boundaries = [0; 3];
    let mut operations = [PinOperationBatch::new(0); 8];
    let commands = build_verified_pio_commands(
        profiles,
        converter,
        config,
        &mut boundaries,
        &mut operations,
    )
    .expect("no se pudieron verificar las palabras PIO");
    let mut reader = PioCommandBlockReader::new(commands);
    let storage_a = cortex_m::singleton!(: [u32; BLOCK_CAPACITY] = [0; BLOCK_CAPACITY]).unwrap();
    let storage_b = cortex_m::singleton!(: [u32; BLOCK_CAPACITY] = [0; BLOCK_CAPACITY]).unwrap();
    let mut buffer_a = DmaCommandBuffer::new(storage_a);
    let mut buffer_b = DmaCommandBuffer::new(storage_b);
    assert_eq!(
        buffer_a.fill_from(&mut reader),
        Ok(Some(PioCommandBlock {
            len: BLOCK_CAPACITY,
            is_final: false,
        }))
    );
    assert_eq!(
        buffer_b.fill_from(&mut reader),
        Ok(Some(PioCommandBlock {
            len: BLOCK_CAPACITY,
            is_final: true,
        }))
    );
    assert!(reader.is_finished());

    let (mut sm, rx, tx) = PIOBuilder::from_installed_program(installed)
        .out_pins(output_gpios[0], output_gpios.len() as u8)
        .out_shift_direction(ShiftDirection::Right)
        .autopull(false)
        .clock_divisor_fixed_point(divisor, 0)
        .buffers(Buffers::OnlyTx)
        .build(sm0);

    let initial_levels = config.initial_levels();
    sm.set_pins(output_gpios.into_iter().enumerate().map(|(bit, pin)| {
        let state = if initial_levels & (1 << bit) != 0 {
            PinState::High
        } else {
            PinState::Low
        };
        (pin, state)
    }));
    sm.set_pindirs(output_gpios.map(|pin| (pin, PinDir::Output)));
    sm.clear_fifos();
    pio.clear_irq(IRQ_END | IRQ_INVALID_COMMAND);
    tx.clear_stalled_flag();

    let step_pins_low = [
        (output_gpios[0], PinState::Low),
        (output_gpios[2], PinState::Low),
    ];
    let step_mask = step_masks[0] | step_masks[1];

    // Con PIO detenido, la FIFO se llena tras las primeras dos palabras de CH1.
    // Se cancela en ese punto y se reutilizan los recursos en la prueba normal.
    let transfer = DmaPairTransfer::start(dma.ch0, dma.ch1, buffer_a, buffer_b, tx);
    let preload_result = wait_for_preload(&transfer, &pio, &timer);
    let reached_channel_b = !transfer.channel_a_busy() && transfer.channel_b_busy();
    let remaining_at_abort = transfer.remaining_words();
    sm.set_pins(step_pins_low);
    let DmaPairResources {
        channel_a,
        channel_b,
        buffer_a,
        buffer_b,
        tx,
        completed,
        bus_error,
    } = transfer.stop();
    let preload_stalled = tx.has_stalled();
    sm.clear_fifos();
    assert_eq!(preload_result, Ok(()));
    assert!(reached_channel_b);
    assert_eq!(remaining_at_abort, [0, 4]);
    assert!(!completed && !bus_error && !preload_stalled);
    assert_eq!(hal::Sio::read_bank0() & step_mask, 0);
    defmt::info!("Precarga cancelada con CH1 activo");

    pio.clear_irq(IRQ_END | IRQ_INVALID_COMMAND);
    tx.clear_stalled_flag();
    let transfer = DmaPairTransfer::start(channel_a, channel_b, buffer_a, buffer_b, tx);
    let preload_result = wait_for_preload(&transfer, &pio, &timer);
    let mut observed_steps = [0u32; 2];
    let (mut sm, result) = if preload_result.is_ok() {
        let mut previous = hal::Sio::read_bank0();
        let started_at = timer.get_counter().ticks();
        let running_sm = sm.start();
        let result = loop {
            let pins = hal::Sio::read_bank0();
            let rising = pins & !previous;
            previous = pins;
            for (steps, mask) in observed_steps.iter_mut().zip(step_masks) {
                *steps += u32::from(rising & mask != 0);
            }

            if transfer.has_bus_error() {
                break Err(PioExecutionError::DmaBusError);
            }
            if pio_has_stalled() {
                break Err(PioExecutionError::FifoUnderrun);
            }
            let irqs = pio.get_irq_raw();
            if irqs & IRQ_INVALID_COMMAND != 0 {
                break Err(PioExecutionError::InvalidCommand);
            }
            if irqs & IRQ_END != 0 && transfer.is_done() {
                break Ok(());
            }
            if timer.get_counter().ticks().wrapping_sub(started_at) >= SUPERVISION_TIMEOUT_US {
                break Err(PioExecutionError::Timeout);
            }
        };
        (running_sm.stop(), result)
    } else {
        (sm, preload_result)
    };

    sm.set_pins(step_pins_low);
    let DmaPairResources {
        channel_a: _channel_a,
        channel_b: _channel_b,
        buffer_a,
        buffer_b,
        tx,
        completed,
        bus_error,
    } = transfer.stop();
    let fifo_empty = tx.is_empty();
    let stalled = tx.has_stalled();
    sm.clear_fifos();
    let (_sm, _program) = sm.uninit(rx, tx);

    assert_eq!(result, Ok(()));
    assert!(completed && !bus_error);
    assert!(fifo_empty && !stalled);
    assert_eq!(buffer_a.len(), BLOCK_CAPACITY);
    assert_eq!(buffer_b.len(), BLOCK_CAPACITY);
    assert_eq!(observed_steps, [4, 2]);
    assert_eq!(hal::Sio::read_bank0() & step_mask, 0);
    defmt::info!("Trayectoria A ejecutada con CH0 y CH1, cuatro pulsos STEP1 y dos STEP2");

    led.set_high().unwrap();
    loop {
        cortex_m::asm::wfi();
    }
}

fn wait_for_preload(
    transfer: &DmaPairTransfer,
    pio: &PIO<PIO0>,
    timer: &Timer<CopyableTimer0>,
) -> Result<(), PioExecutionError> {
    let started_at = timer.get_counter().ticks();
    loop {
        if transfer.has_bus_error() {
            return Err(PioExecutionError::DmaBusError);
        }
        if transfer.is_done() || !pio.irq0().raw().sm0_tx_not_full() {
            return Ok(());
        }
        if timer.get_counter().ticks().wrapping_sub(started_at) >= PRELOAD_TIMEOUT_US {
            return Err(PioExecutionError::PreloadFailed);
        }
    }
}
