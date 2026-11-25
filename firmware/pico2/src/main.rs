#![no_std]
#![no_main]

mod dma_command_buffer;
#[allow(
    dead_code,
    reason = "la prueba de banco usa el ejecutor DMA por bloques"
)]
mod dma_executor;
mod dma_pair;
mod streaming_dma_executor;

use defmt_rtt as _;
use dma_command_buffer::DmaCommandBuffer;
use dma_executor::{ExecutionStatus, PioExecutionError};
use embedded_hal::{delay::DelayNs, digital::OutputPin};
use executor::{ExecutorError, PioCommandWord, PioExecutionConfig, build_verified_pio_commands};
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
    pio::{PIO, PIOExt},
    timer::{CopyableTimer0, Timer},
};
use streaming_dma_executor::{ActiveStreamingExecution, StreamingDmaExecutor, StreamingPoll};

const XOSC_HZ: u32 = 12_000_000;
const PIO_HZ: u32 = 1_000_000;
const BLOCK_CAPACITY: usize = 5;
const SUPERVISION_TIMEOUT_US: u64 = 6_000_000;
const SHORT_TIMEOUT_US: u64 = 1_000;
const PRELOAD_CANCEL_DELAY_US: u32 = 100;
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
    let step_masks = [1u32 << output_gpios[0], 1u32 << output_gpios[2]];

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

    let points = [
        JointAngles::new(0.0, 0.0).unwrap(),
        JointAngles::new(0.25, -0.25).unwrap(),
        JointAngles::new(0.50, -0.50).unwrap(),
        JointAngles::new(0.75, -0.75).unwrap(),
        JointAngles::new(1.00, -1.00).unwrap(),
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
    let speeds = [0.0; 5];
    let converter = JointToStepsConverter::new(8.0, 4.0).unwrap();

    for (trajectory, case) in [
        (&points[..3], BenchCase::CancelPreloading),
        (&points[..3], BenchCase::CancelOnFirstStep),
        (&points[..3], BenchCase::Timeout),
        (
            &points[..2],
            BenchCase::Complete {
                blocks: 2,
                steps: [2, 1],
            },
        ),
        (
            &points[..3],
            BenchCase::Complete {
                blocks: 3,
                steps: [4, 2],
            },
        ),
        (
            &points[..],
            BenchCase::Complete {
                blocks: 5,
                steps: [8, 4],
            },
        ),
    ] {
        let path = CoordinatedJointPathProfiles::new(
            trajectory,
            &speeds[..trajectory.len()],
            &velocity_limits,
            &acceleration_limits,
        )
        .unwrap();
        let profiles = TimedCoordinatedJointPathProfiles::new(path).unwrap();
        let mut boundaries = [0; 5];
        let mut operations = [PinOperationBatch::new(0); 16];
        let commands = build_verified_pio_commands(
            profiles,
            converter,
            config,
            &mut boundaries,
            &mut operations,
        )
        .expect("no se pudieron verificar las palabras PIO");
        let timeout_us = if matches!(case, BenchCase::Timeout) {
            SHORT_TIMEOUT_US
        } else {
            SUPERVISION_TIMEOUT_US
        };
        let active = executor
            .start(
                &mut pio,
                &timer,
                commands,
                config.initial_levels(),
                timeout_us,
            )
            .unwrap_or_else(|(_, error)| panic!("Error de inicio: {error:?}"));
        executor = check_bench_execution(active, &mut pio, &mut timer, step_masks, case);
        timer.delay_ms(BENCH_SEPARATION_MS);
    }

    let active = executor
        .start(
            &mut pio,
            &timer,
            core::iter::once(Ok(PioCommandWord::END)),
            config.initial_levels(),
            SUPERVISION_TIMEOUT_US,
        )
        .unwrap_or_else(|(_, error)| panic!("Error de inicio: {error:?}"));
    let _executor = check_bench_execution(
        active,
        &mut pio,
        &mut timer,
        step_masks,
        BenchCase::Complete {
            blocks: 1,
            steps: [0, 0],
        },
    );

    led.set_high().unwrap();
    loop {
        cortex_m::asm::wfi();
    }
}

enum BenchCase {
    CancelPreloading,
    CancelOnFirstStep,
    Timeout,
    Complete { blocks: usize, steps: [u32; 2] },
}

fn check_bench_execution<I>(
    mut active: ActiveStreamingExecution<I>,
    pio: &mut PIO<PIO0>,
    timer: &mut Timer<CopyableTimer0>,
    step_masks: [u32; 2],
    case: BenchCase,
) -> StreamingDmaExecutor
where
    I: Iterator<Item = Result<PioCommandWord, ExecutorError>>,
{
    assert_eq!(active.status(), ExecutionStatus::Preloading);
    assert_eq!(active.running_elapsed_us(timer), None);
    let step_mask = step_masks[0] | step_masks[1];
    let mut observed_steps = [0; 2];
    let (executor, report) = if matches!(case, BenchCase::CancelPreloading) {
        // Dar tiempo a DMA para llenar la FIFO mientras la máquina PIO sigue detenida.
        timer.delay_us(PRELOAD_CANCEL_DELAY_US);
        assert_eq!(hal::Sio::read_bank0() & step_mask, 0);
        active.cancel()
    } else {
        let mut previous = hal::Sio::read_bank0();
        loop {
            let pins = hal::Sio::read_bank0();
            let rising = pins & !previous;
            previous = pins;
            for (count, mask) in observed_steps.iter_mut().zip(step_masks) {
                *count += u32::from(rising & mask != 0);
            }

            if matches!(case, BenchCase::CancelOnFirstStep) && rising & step_mask != 0 {
                assert_eq!(active.status(), ExecutionStatus::Running);
                assert!(active.running_elapsed_us(timer).is_some());
                break active.cancel();
            }

            match active.poll(pio, timer) {
                StreamingPoll::Pending(next) => active = next,
                StreamingPoll::Finished { executor, report } => break (executor, report),
            }
        }
    };

    assert_eq!(hal::Sio::read_bank0() & step_mask, 0);
    match case {
        BenchCase::CancelPreloading => {
            assert_eq!(report.result, Err(PioExecutionError::Cancelled));
            assert_eq!(report.blocks_queued, 2);
            assert!(!report.final_queued);
            defmt::info!("Cancelación durante la precarga verificada");
        }
        BenchCase::CancelOnFirstStep => {
            assert_eq!(report.result, Err(PioExecutionError::Cancelled));
            assert_ne!(observed_steps, [0, 0]);
            defmt::info!("Cancelación durante el movimiento verificada");
        }
        BenchCase::Timeout => {
            assert_eq!(report.result, Err(PioExecutionError::Timeout));
            assert_eq!(observed_steps, [0, 0]);
            defmt::info!("Tiempo límite insuficiente verificado");
        }
        BenchCase::Complete { blocks, steps } => {
            assert_eq!(report.result, Ok(()));
            assert!(report.final_queued);
            assert_eq!(report.blocks_queued, blocks);
            assert_eq!(observed_steps, steps);
            defmt::info!("Ejecución de {} bloques verificada", report.blocks_queued);
        }
    }
    executor
}
