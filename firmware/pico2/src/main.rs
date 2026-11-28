#![no_std]
#![no_main]

mod benchmark;
mod dma_command_buffer;
#[allow(
    dead_code,
    reason = "la prueba de banco usa el ejecutor DMA por bloques"
)]
mod dma_executor;
mod dma_pair;
mod streaming_dma_executor;

use benchmark::{
    SYNTHETIC_INITIAL_LEVELS, measure_execution, prepare_commands, synthetic_step_commands,
};
use defmt_rtt as _;
use dma_command_buffer::DmaCommandBuffer;
use embedded_hal::{delay::DelayNs, digital::OutputPin};
use executor::{PioCommandWord, PioExecutionConfig, build_pio_commands};
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
    pac,
    pio::PIOExt,
};
use streaming_dma_executor::StreamingDmaExecutor;

const XOSC_HZ: u32 = 12_000_000;
const PIO_HZ: u32 = 1_000_000;
const BLOCK_CAPACITY: usize = 128;
const PULSE_COUNT: usize = 10_000;
const PREPARED_CAPACITY: usize = 8_192;
const BENCH_SEPARATION_MS: u32 = 1_000;

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
        c"SCARA STEP/DIR performance benchmark (Adafruit Feather RP2350, A0-A3)"
    ),
    #[cfg(not(feature = "adafruit-feather-rp2350"))]
    hal::binary_info::rp_program_description!(c"SCARA STEP/DIR performance benchmark"),
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

    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
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

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let program = pio_proc::pio_file!(
        "../../crates/executor/pio/step_dir.pio",
        select_program("step_dir")
    );
    let installed = pio.install(&program.program).unwrap();
    let dma = pac.DMA.split(&mut pac.RESETS);
    let storage_a = cortex_m::singleton!(: [u32; BLOCK_CAPACITY] = [0; BLOCK_CAPACITY]).unwrap();
    let storage_b = cortex_m::singleton!(: [u32; BLOCK_CAPACITY] = [0; BLOCK_CAPACITY]).unwrap();
    let mut executor = StreamingDmaExecutor::new(
        sm0,
        installed,
        dma.ch0,
        dma.ch1,
        DmaCommandBuffer::new(storage_a),
        DmaCommandBuffer::new(storage_b),
        output_gpios,
        divisor,
    );

    defmt::info!("DMA/PIO sintetico: {} pulsos por motor", PULSE_COUNT);
    for frequency in [1_000u64, 5_000, 10_000, 20_000, 50_000] {
        let half_period_cycles = u64::from(PIO_HZ) / (2 * frequency);
        defmt::info!(
            "Frecuencia solicitada={} Hz, semiperiodo={} ciclos",
            frequency,
            half_period_cycles
        );
        match synthetic_step_commands(PULSE_COUNT, half_period_cycles) {
            Ok(commands) => {
                executor = measure_execution(
                    executor,
                    &mut pio,
                    &timer,
                    commands,
                    SYNTHETIC_INITIAL_LEVELS,
                );
            }
            Err(error) => defmt::warn!(
                "Generacion sintetica fallida: {:?}",
                defmt::Debug2Format(&error)
            ),
        }
        timer.delay_ms(BENCH_SEPARATION_MS);
    }

    let config = PioExecutionConfig {
        timebase: StepTimeBase::new(100_000).unwrap(),
        timing: StepDirTiming::new(10_000, 10_000, 10_000, 10_000).unwrap(),
        pio_frequency_hz: u64::from(PIO_HZ),
        initial_first_direction: StepDirection::Negative,
        initial_second_direction: StepDirection::Positive,
        first_positive_is_high: true,
        second_positive_is_high: true,
    };
    let steps_per_radian = (200.0 * 16.0 * 4.0) / core::f64::consts::TAU;
    let converter = JointToStepsConverter::new(steps_per_radian, steps_per_radian).unwrap();
    let points = [
        JointAngles::new(0.0, 0.0).unwrap(),
        JointAngles::new(1.0, -0.75).unwrap(),
    ];
    let speeds = [0.0; 2];
    let acceleration_limits = JointAccelerationLimits::new(10.0, 10.0).unwrap();
    let mut boundaries = [0; 2];
    let mut operations = [PinOperationBatch::new(0); 64];
    let storage = cortex_m::singleton!(: [PioCommandWord; PREPARED_CAPACITY] =
        [PioCommandWord::END; PREPARED_CAPACITY])
    .unwrap();

    defmt::info!("Planificador: {} micropasos/rad", steps_per_radian);
    for velocity in [0.5, 1.0, 2.0, 4.0] {
        defmt::info!("Limite de velocidad={} rad/s", velocity);
        let velocity_limits = JointVelocityLimits::new(velocity, velocity).unwrap();
        let profiles = match CoordinatedJointPathProfiles::new(
            &points,
            &speeds,
            &velocity_limits,
            &acceleration_limits,
        )
        .and_then(TimedCoordinatedJointPathProfiles::new)
        {
            Ok(profiles) => profiles,
            Err(error) => {
                defmt::warn!(
                    "Construccion de perfiles fallida: {:?}",
                    defmt::Debug2Format(&error)
                );
                continue;
            }
        };

        let preparation_started = timer.get_counter().ticks();
        let commands = build_pio_commands(
            profiles,
            converter,
            config,
            &mut boundaries,
            &mut operations,
        );
        let Some(prepared) = prepare_commands(storage, commands) else {
            continue;
        };
        let preparation_elapsed = timer
            .get_counter()
            .ticks()
            .wrapping_sub(preparation_started);
        defmt::info!(
            "Preparacion y verificacion={} us, palabras={}",
            preparation_elapsed,
            prepared.len()
        );

        defmt::info!("Trayectoria con generacion durante la ejecucion");
        match build_pio_commands(
            profiles,
            converter,
            config,
            &mut boundaries,
            &mut operations,
        ) {
            Ok(commands) => {
                executor = measure_execution(
                    executor,
                    &mut pio,
                    &timer,
                    commands,
                    config.initial_levels(),
                );
            }
            Err(error) => defmt::warn!("Reconstruccion fallida: {:?}", defmt::Debug2Format(&error)),
        }
        timer.delay_ms(BENCH_SEPARATION_MS);

        defmt::info!("Trayectoria con palabras preparadas en RAM");
        executor = measure_execution(
            executor,
            &mut pio,
            &timer,
            prepared.iter().copied().map(Ok),
            config.initial_levels(),
        );
        timer.delay_ms(BENCH_SEPARATION_MS);
    }

    defmt::info!("Benchmark terminado");
    led.set_high().unwrap();
    loop {
        cortex_m::asm::wfi();
    }
}
