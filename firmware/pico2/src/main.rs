#![no_std]
#![no_main]

mod dma_executor;

use defmt_rtt as _;
use dma_executor::{
    DmaExecutor, ExecutionPoll, ExecutionStatus, PioExecutionError, TX_FIFO_CAPACITY,
};
use embedded_hal::digital::OutputPin;
use executor::{PioExecutionConfig, build_verified_pio_commands};
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

const XOSC_HZ: u32 = 12_000_000;
const PIO_HZ: u32 = 1_000_000;
const COMMAND_BUFFER_CAPACITY: usize = 32;
const SUPERVISION_TIMEOUT_US: u64 = 3_000_000;
const CANCEL_AFTER_US: u64 = 400_000;

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
    let _step_1 = step_1.into_function::<FunctionPio0>();
    let _dir_1 = dir_1.into_function::<FunctionPio0>();
    let _step_2 = step_2.into_function::<FunctionPio0>();
    let _dir_2 = dir_2.into_function::<FunctionPio0>();

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let program = pio_proc::pio_file!(
        "../../crates/executor/pio/step_dir.pio",
        select_program("step_dir")
    );
    let installed = pio.install(&program.program).unwrap();
    let dma = pac.DMA.split(&mut pac.RESETS);
    let mut executor = DmaExecutor::new(sm0, installed, dma.ch0, output_gpios, divisor);

    let points = [
        JointAngles::new(0.0, 0.0).unwrap(),
        JointAngles::new(0.25, -0.25).unwrap(),
        JointAngles::new(0.50, -0.50).unwrap(),
    ];
    let speeds = [0.0; 3];
    let velocity_limits = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration_limits = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let path =
        CoordinatedJointPathProfiles::new(&points, &speeds, &velocity_limits, &acceleration_limits)
            .unwrap();
    let profiles = TimedCoordinatedJointPathProfiles::new(path).unwrap();
    // Factores de prueba que producen cuatro pasos en el primer eje y dos en el segundo.
    let converter = JointToStepsConverter::new(8.0, 4.0).unwrap();
    let config = PioExecutionConfig {
        timebase: StepTimeBase::new(8).unwrap(),
        timing: StepDirTiming::new(125_000_000, 125_000_000, 1, 1).unwrap(),
        pio_frequency_hz: u64::from(PIO_HZ),
        initial_first_direction: StepDirection::Negative,
        initial_second_direction: StepDirection::Positive,
        first_positive_is_high: true,
        second_positive_is_high: true,
    };

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
    let command_buffer =
        cortex_m::singleton!(: [u32; COMMAND_BUFFER_CAPACITY] = [0; COMMAND_BUFFER_CAPACITY])
            .unwrap();
    let mut command_count = 0;
    for command in commands {
        assert!(
            command_count < command_buffer.len(),
            "la trayectoria excede la capacidad del búfer PIO"
        );
        command_buffer[command_count] = command.expect("no se pudo generar una palabra PIO").bits();
        command_count += 1;
    }
    let commands: &'static [u32] = &command_buffer[..command_count];
    assert!(command_count > TX_FIFO_CAPACITY);

    let active = match executor.start(
        &mut pio,
        &timer,
        commands,
        config.initial_levels(),
        SUPERVISION_TIMEOUT_US,
    ) {
        Ok(active) => active,
        Err((_, error)) => panic!("no se pudo iniciar la precarga DMA: {error:?}"),
    };
    assert_eq!(active.status(), ExecutionStatus::Preloading);
    assert_eq!(active.running_elapsed_us(&timer), None);
    let (returned_executor, result) = active.cancel();
    executor = returned_executor;
    assert_eq!(result, Err(PioExecutionError::Cancelled));
    defmt::info!("Cancelación durante precarga correcta");

    let mut active = match executor.start(
        &mut pio,
        &timer,
        commands,
        config.initial_levels(),
        SUPERVISION_TIMEOUT_US,
    ) {
        Ok(active) => active,
        Err((_, error)) => panic!("no se pudo iniciar la ejecución DMA: {error:?}"),
    };
    let (returned_executor, result) = loop {
        if active
            .running_elapsed_us(&timer)
            .is_some_and(|elapsed| elapsed >= CANCEL_AFTER_US)
        {
            assert_eq!(active.status(), ExecutionStatus::Running);
            break active.cancel();
        }

        match active.poll(&mut pio, &timer) {
            ExecutionPoll::Pending(next) => active = next,
            ExecutionPoll::Finished { executor, result } => break (executor, result),
        }
    };
    executor = returned_executor;
    assert_eq!(result, Err(PioExecutionError::Cancelled));
    defmt::info!("Cancelación durante movimiento correcta");

    for iteration in 0..3 {
        let mut active = match executor.start(
            &mut pio,
            &timer,
            commands,
            config.initial_levels(),
            SUPERVISION_TIMEOUT_US,
        ) {
            Ok(active) => active,
            Err((_, error)) => panic!("no se pudo iniciar la ejecución DMA: {error:?}"),
        };
        let mut cpu_iterations = 0u64;
        let (returned_executor, result) = loop {
            match active.poll(&mut pio, &timer) {
                ExecutionPoll::Pending(next) => active = next,
                ExecutionPoll::Finished { executor, result } => break (executor, result),
            }
            cpu_iterations += 1;
        };
        executor = returned_executor;

        match result {
            Ok(()) => {
                assert!(cpu_iterations > 0);
                defmt::info!(
                    "Ejecución {} completada con {} iteraciones disponibles para otras tareas",
                    iteration + 1,
                    cpu_iterations
                );
            }
            Err(error) => {
                defmt::error!(
                    "Fallo de la prueba STEP/DIR: {:?}",
                    defmt::Debug2Format(&error)
                );
                loop {
                    cortex_m::asm::wfi();
                }
            }
        }
    }

    led.set_high().unwrap();
    loop {
        cortex_m::asm::wfi();
    }
}
