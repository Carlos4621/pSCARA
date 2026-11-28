use embedded_hal::delay::DelayNs;
use executor::PioCommandWord;
use rp235x_hal::{
    pac::PIO0,
    pio::PIO,
    timer::{CopyableTimer0, Timer},
};
#[cfg(not(feature = "wokwi-pio-test"))]
use scara_wokwi::STARVE_US;
use scara_wokwi::{AFTER_WINDOW_US, BEFORE_WINDOW_US, COMMAND_CAPACITY, Case, cases, config};

#[cfg(feature = "wokwi-pio-test")]
use crate::cpu_executor::CpuExecutor as Executor;
#[cfg(not(feature = "wokwi-pio-test"))]
use crate::{
    dma_executor::{ExecutionStatus, PioExecutionError},
    streaming_dma_executor::{StreamingDmaExecutor as Executor, StreamingPoll},
};

const EXECUTION_TIMEOUT_US: u64 = 5_000_000;
// La SM del marcador necesita dos ciclos PIO para consumir la orden y publicar GP6.
const MARKER_SETTLE_US: u32 = 3;

pub fn run(
    mut executor: Executor,
    mut pio: PIO<PIO0>,
    mut timer: Timer<CopyableTimer0>,
    mut mark_window: impl FnMut(bool),
) {
    let mut storage = [PioCommandWord::END; COMMAND_CAPACITY];
    for &case in cases(!cfg!(feature = "wokwi-pio-test")) {
        let commands = case.prepare(&mut storage).unwrap();
        executor = drive(
            executor,
            &mut pio,
            &mut timer,
            &[PioCommandWord::END],
            false,
        );
        timer.delay_us(BEFORE_WINDOW_US);
        mark_window(true);
        timer.delay_us(MARKER_SETTLE_US);
        executor = drive(
            executor,
            &mut pio,
            &mut timer,
            commands,
            case == Case::Starved,
        );
        timer.delay_us(AFTER_WINDOW_US);
        mark_window(false);
        timer.delay_us(MARKER_SETTLE_US);
    }
}

#[cfg(feature = "wokwi-pio-test")]
fn drive(
    executor: Executor,
    pio: &mut PIO<PIO0>,
    timer: &mut Timer<CopyableTimer0>,
    commands: &[PioCommandWord],
    _starve: bool,
) -> Executor {
    let (executor, result) = executor.execute(
        pio,
        timer,
        commands,
        config().initial_levels(),
        EXECUTION_TIMEOUT_US,
    );
    assert_eq!(result, Ok(()));
    executor
}

#[cfg(not(feature = "wokwi-pio-test"))]
fn drive(
    executor: Executor,
    pio: &mut PIO<PIO0>,
    timer: &mut Timer<CopyableTimer0>,
    commands: &[PioCommandWord],
    starve: bool,
) -> Executor {
    let mut active = executor
        .start(
            pio,
            timer,
            commands.iter().copied().map(Ok),
            config().initial_levels(),
            EXECUTION_TIMEOUT_US,
        )
        .unwrap_or_else(|(_, error)| panic!("No se pudo iniciar la captura: {error:?}"));
    let mut inject_starvation = starve;
    loop {
        if inject_starvation && active.status() == ExecutionStatus::Running {
            timer.delay_us(STARVE_US);
            inject_starvation = false;
        }
        match active.poll(pio, timer) {
            StreamingPoll::Pending(next) => active = next,
            StreamingPoll::Finished { executor, report } => {
                let expected = if starve {
                    Err(PioExecutionError::FifoUnderrun)
                } else {
                    Ok(())
                };
                assert_eq!(report.result, expected);
                return executor;
            }
        }
    }
}
