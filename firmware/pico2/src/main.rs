#![no_std]
#![no_main]

#[cfg(not(feature = "wokwi-test"))]
mod benchmark;
#[cfg(feature = "wokwi-pio-test")]
mod cpu_executor;
#[cfg(not(feature = "wokwi-pio-test"))]
mod dma_command_buffer;
#[cfg(not(feature = "wokwi-pio-test"))]
#[allow(
    dead_code,
    reason = "la prueba de banco usa el ejecutor DMA por bloques"
)]
mod dma_executor;
#[cfg(not(feature = "wokwi-pio-test"))]
mod dma_pair;
#[cfg(not(feature = "wokwi-pio-test"))]
mod streaming_dma_executor;
#[cfg(feature = "wokwi-test")]
mod wokwi_test;

#[cfg(not(feature = "wokwi-test"))]
use benchmark::{
    SYNTHETIC_INITIAL_LEVELS, measure_execution, prepare_commands, synthetic_step_commands,
};
use defmt_rtt as _;
#[cfg(not(feature = "wokwi-pio-test"))]
use dma_command_buffer::DmaCommandBuffer;
#[cfg(not(feature = "wokwi-test"))]
use embedded_hal::{delay::DelayNs, digital::OutputPin};
#[cfg(not(feature = "wokwi-test"))]
use executor::{PioCommandWord, PioExecutionConfig, build_pio_commands};
#[cfg(not(feature = "wokwi-test"))]
use kinematics::JointAngles;
use panic_probe as _;
#[cfg(not(feature = "wokwi-test"))]
use planner::{
    CoordinatedJointPathProfiles, JointAccelerationLimits, JointToStepsConverter,
    JointVelocityLimits, PinOperationBatch, StepDirTiming, StepDirection, StepTimeBase,
    TimedCoordinatedJointPathProfiles,
};
#[cfg(not(feature = "wokwi-pio-test"))]
use rp235x_hal::dma::DMAExt;
#[cfg(not(feature = "wokwi-test"))]
use rp235x_hal::gpio::PinState as GpioPinState;
use rp235x_hal::{
    self as hal, Clock, clocks::init_clocks_and_plls, entry, gpio::FunctionPio0, pac, pio::PIOExt,
};
#[cfg(not(feature = "wokwi-pio-test"))]
use streaming_dma_executor::StreamingDmaExecutor;

const XOSC_HZ: u32 = 12_000_000;
const PIO_HZ: u32 = 1_000_000;
const BLOCK_CAPACITY: usize = 128;
#[cfg(not(feature = "wokwi-test"))]
const PULSE_COUNT: usize = 10_000;
#[cfg(not(feature = "wokwi-test"))]
const PREPARED_CAPACITY: usize = 8_192;
#[cfg(not(feature = "wokwi-test"))]
const BENCH_SEPARATION_MS: u32 = 1_000;
#[cfg(feature = "wokwi-test")]
const _: () = {
    assert!(PIO_HZ as u64 == scara_wokwi::PIO_HZ);
    assert!(BLOCK_CAPACITY == scara_wokwi::BLOCK_WORDS);
};

#[unsafe(link_section = ".start_block")]
#[used]
static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

#[unsafe(link_section = ".bi_entries")]
#[used]
static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 3] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    #[cfg(all(feature = "adafruit-feather-rp2350", not(feature = "wokwi-test")))]
    hal::binary_info::rp_program_description!(
        c"SCARA STEP/DIR performance benchmark (Adafruit Feather RP2350, A0-A3)"
    ),
    #[cfg(not(any(feature = "adafruit-feather-rp2350", feature = "wokwi-test")))]
    hal::binary_info::rp_program_description!(c"SCARA STEP/DIR performance benchmark"),
    #[cfg(feature = "wokwi-test")]
    hal::binary_info::rp_program_description!(c"SCARA Pico 2 STEP/DIR capture scenarios"),
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
    #[cfg(feature = "wokwi-test")]
    assert_eq!(system_clock_hz, scara_wokwi::SYSTEM_CLOCK_HZ);
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
    #[cfg(all(feature = "adafruit-feather-rp2350", not(feature = "wokwi-test")))]
    let (status_led, step_1, dir_1, step_2, dir_2) = (
        pins.gpio7,
        pins.gpio26,
        pins.gpio27,
        pins.gpio28,
        pins.gpio29,
    );
    #[cfg(not(any(feature = "adafruit-feather-rp2350", feature = "wokwi-test")))]
    let (status_led, step_1, dir_1, step_2, dir_2) =
        (pins.gpio25, pins.gpio2, pins.gpio3, pins.gpio4, pins.gpio5);
    #[cfg(feature = "wokwi-test")]
    let (marker_pin, step_1, dir_1, step_2, dir_2) =
        (pins.gpio6, pins.gpio2, pins.gpio3, pins.gpio4, pins.gpio5);
    #[cfg(not(feature = "wokwi-test"))]
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

    let (mut pio, sm0, _sm1, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let program = pio_proc::pio_file!(
        "../../crates/executor/pio/step_dir.pio",
        select_program("step_dir")
    );
    let installed = pio.install(&program.program).unwrap();
    #[cfg(feature = "wokwi-pio-test")]
    let executor = cpu_executor::CpuExecutor::new(sm0, installed, output_gpios, divisor);
    #[cfg(not(feature = "wokwi-pio-test"))]
    let executor = {
        let dma = pac.DMA.split(&mut pac.RESETS);
        let storage_a =
            cortex_m::singleton!(: [u32; BLOCK_CAPACITY] = [0; BLOCK_CAPACITY]).unwrap();
        let storage_b =
            cortex_m::singleton!(: [u32; BLOCK_CAPACITY] = [0; BLOCK_CAPACITY]).unwrap();
        StreamingDmaExecutor::new(
            sm0,
            installed,
            dma.ch0,
            dma.ch1,
            DmaCommandBuffer::new(storage_a),
            DmaCommandBuffer::new(storage_b),
            output_gpios,
            divisor,
        )
    };

    #[cfg(feature = "wokwi-test")]
    {
        use hal::pio::{PIOBuilder, PinDir, PinState, ShiftDirection};

        let marker = marker_pin.into_function::<FunctionPio0>().id().num;
        let program = pio_proc::pio_asm!("pull block", "out pins, 1");
        let installed = pio.install(&program.program).unwrap();
        let (mut marker_sm, _, mut marker_tx) = PIOBuilder::from_installed_program(installed)
            .out_pins(marker, 1)
            .out_shift_direction(ShiftDirection::Right)
            .clock_divisor_fixed_point(divisor, 0)
            .build(_sm1);
        marker_sm.set_pins([(marker, PinState::Low)]);
        marker_sm.set_pindirs([(marker, PinDir::Output)]);
        let _marker_sm = marker_sm.start();
        wokwi_test::run(executor, pio, timer, |high| {
            assert!(marker_tx.write(u32::from(high)));
        });
    }
    #[cfg(not(feature = "wokwi-test"))]
    {
        run_benchmark(executor, pio, timer);
        led.set_high().unwrap();
    }
    loop {
        cortex_m::asm::wfi();
    }
}

#[cfg(not(feature = "wokwi-test"))]
fn run_benchmark(
    mut executor: StreamingDmaExecutor,
    mut pio: hal::pio::PIO<pac::PIO0>,
    mut timer: hal::timer::Timer<hal::timer::CopyableTimer0>,
) {
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
}
