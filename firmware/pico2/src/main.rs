#![no_std]
#![no_main]

use defmt_rtt as _;
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
    entry,
    gpio::{FunctionPio0, PinState as GpioPinState},
    pac,
    pio::{Buffers, PIOBuilder, PIOExt, PinDir, PinState, ShiftDirection},
};

const XOSC_HZ: u32 = 12_000_000;
const PIO_HZ: u32 = 1_000_000;
const SUPERVISION_TIMEOUT_US: u64 = 2_000_000;

const EXPECTED_WORDS: [u32; 7] = [
    0xA007_A120,
    0x3003_D084,
    0x6003_D084,
    0x2003_D084,
    0x3003_D084,
    0x2003_D084,
    0x0000_0001,
];

const IRQ_END: u8 = 1 << 0;
const IRQ_INVALID_COMMAND: u8 = 1 << 1;

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
    let (mut sm, _rx, mut tx) = PIOBuilder::from_installed_program(installed)
        .out_pins(output_gpios[0], output_gpios.len() as u8)
        .out_shift_direction(ShiftDirection::Right)
        .autopull(false)
        .clock_divisor_fixed_point(divisor, 0)
        .buffers(Buffers::OnlyTx)
        .build(sm0);

    let points = [
        JointAngles::new(0.0, 0.0).unwrap(),
        JointAngles::new(0.25, -0.25).unwrap(),
    ];
    let speeds = [0.0; 2];
    let velocity_limits = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration_limits = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let path =
        CoordinatedJointPathProfiles::new(&points, &speeds, &velocity_limits, &acceleration_limits)
            .unwrap();
    let profiles = TimedCoordinatedJointPathProfiles::new(path).unwrap();
    // Factores de prueba que producen dos pasos en el primer eje y uno en el segundo.
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

    let initial_levels = config.initial_levels();
    assert_eq!(initial_levels, 0b1000);
    sm.set_pins(output_gpios.map(|pin| {
        let bit = pin - output_gpios[0];
        let state = if initial_levels & (1 << bit) != 0 {
            PinState::High
        } else {
            PinState::Low
        };
        (pin, state)
    }));
    sm.set_pindirs(output_gpios.map(|pin| (pin, PinDir::Output)));

    let mut boundaries = [0; 2];
    let mut operations = [PinOperationBatch::new(0); 4];
    let commands = build_verified_pio_commands(
        profiles,
        converter,
        config,
        &mut boundaries,
        &mut operations,
    )
    .expect("no se pudieron verificar las palabras PIO");
    let mut command_count = 0;
    for command in commands {
        let word = command.expect("no se pudo generar una palabra PIO").bits();
        let expected = EXPECTED_WORDS
            .get(command_count)
            .expect("el ejecutor generó más palabras PIO de las esperadas");
        assert_eq!(word, *expected, "palabra PIO {command_count}");
        assert!(tx.write(word), "la FIFO TX no admitió todas las palabras");
        command_count += 1;
    }
    assert_eq!(command_count, EXPECTED_WORDS.len());

    pio.clear_irq(IRQ_END | IRQ_INVALID_COMMAND);
    tx.clear_stalled_flag();
    let started_at = timer.get_counter().ticks();
    let sm = sm.start();

    let result = loop {
        let irqs = pio.get_irq_raw();
        // TXSTALL se conserva incluso si PIO ya alcanzó IRQ_END.
        if tx.has_stalled() {
            break Err("la FIFO TX se agotó durante la ejecución");
        }
        if irqs & IRQ_INVALID_COMMAND != 0 {
            break Err("PIO recibió una instrucción de control inválida");
        }
        if irqs & IRQ_END != 0 {
            break Ok(());
        }
        if timer.get_counter().ticks().wrapping_sub(started_at) >= SUPERVISION_TIMEOUT_US {
            break Err("PIO no informó la finalización dentro del tiempo límite");
        }
    };
    let mut sm = sm.stop();

    match result {
        Ok(()) => {
            led.set_high().unwrap();
            defmt::info!("Secuencia STEP/DIR finalizada");
        }
        Err(reason) => {
            sm.set_pins([
                (output_gpios[0], PinState::Low),
                (output_gpios[2], PinState::Low),
            ]);
            defmt::error!("Fallo de la prueba STEP/DIR: {=str}", reason);
        }
    }

    loop {
        cortex_m::asm::wfi();
    }
}
