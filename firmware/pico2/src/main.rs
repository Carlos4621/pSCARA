#![no_std]
#![no_main]

use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use executor::PioCommandWord;
use panic_probe as _;
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
const SUPERVISION_TIMEOUT_US: u64 = 10_000;

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
        c"SCARA STEP/DIR bench test (Adafruit Feather RP2350, A0-A3)"
    ),
    #[cfg(not(feature = "adafruit-feather-rp2350"))]
    hal::binary_info::rp_program_description!(c"SCARA STEP/DIR bench test"),
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

    sm.set_pins(output_gpios.map(|pin| (pin, PinState::Low)));
    sm.set_pindirs(output_gpios.map(|pin| (pin, PinDir::Output)));

    let commands = [
        PioCommandWord::frame(0, 0b0010).unwrap(),
        PioCommandWord::frame(94, 0b0011).unwrap(),
        PioCommandWord::frame(14, 0b0010).unwrap(),
        PioCommandWord::frame(74, 0b1010).unwrap(),
        PioCommandWord::frame(94, 0b1111).unwrap(),
        PioCommandWord::frame(14, 0b1010).unwrap(),
        PioCommandWord::END,
    ];
    for command in commands {
        assert!(
            tx.write(command.bits()),
            "la FIFO TX no admitió todas las palabras"
        );
    }

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
