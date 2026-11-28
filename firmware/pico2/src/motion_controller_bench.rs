use controller::{ControllerError, ControllerFault, ControllerState};
use executor::{ExecutorError, PioCommandWord, PioExecutionConfig};
use kinematics::JointAngles;
use planner::{
    CoordinatedJointPathProfiles, JointAccelerationLimits, JointToStepsConverter,
    JointVelocityLimits, MotorSteps, PinOperationBatch, StepDirTiming, StepDirection, StepTimeBase,
    TimedCoordinatedJointPathProfiles,
};

use crate::{
    PIO_HZ, PREPARED_CAPACITY,
    dma_executor::PioExecutionError,
    rp2350_motion_controller::{MotionError, MotionResources, Rp2350MotionController},
    streaming_dma_executor::StreamingExecutionReport,
};

const ORIGIN: MotorSteps = MotorSteps {
    first: 0,
    second: 0,
};
const OUTBOUND_TARGET: MotorSteps = MotorSteps {
    first: 2037,
    second: -1528,
};
const INVALID_STATE: MotionError = MotionError::Controller(ControllerError::InvalidState);
const TIMEOUT_US: u64 = 5_000_000;
const CANCEL_AFTER_US: u64 = 250_000;
const STEP_OBSERVATION_US: u64 = 10_000;

pub fn run(resources: MotionResources, system_clock_hz: u64, mut steps_low: impl FnMut() -> bool) {
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
    let (executor, pio, timer) = resources;
    let incompatible_config = PioExecutionConfig {
        pio_frequency_hz: 2 * u64::from(PIO_HZ),
        ..config
    };
    let resources = match Rp2350MotionController::new(
        executor,
        pio,
        timer,
        converter,
        incompatible_config,
        system_clock_hz,
    ) {
        Err((resources, MotionError::FrequencyMismatch)) => resources,
        _ => panic!("se esperaba FrequencyMismatch"),
    };
    defmt::info!("Frecuencia PIO incompatible: FrequencyMismatch, recursos recuperados");

    let velocity = JointVelocityLimits::new(0.5, 0.5).unwrap();
    let acceleration = JointAccelerationLimits::new(10.0, 10.0).unwrap();
    let points =
        [[0.0, 0.0], [1.0, -0.75]].map(|[first, second]| JointAngles::new(first, second).unwrap());
    let reversed = [points[1], points[0]];
    let speeds = [0.0; 2];
    let timed_path = |points| {
        let profiles =
            CoordinatedJointPathProfiles::new(points, &speeds, &velocity, &acceleration).unwrap();
        TimedCoordinatedJointPathProfiles::new(profiles).unwrap()
    };
    let outbound = timed_path(&points);
    let inbound = timed_path(&reversed);
    let storage_a = cortex_m::singleton!(: [PioCommandWord; PREPARED_CAPACITY] =
        [PioCommandWord::END; PREPARED_CAPACITY])
    .unwrap();
    let storage_b = cortex_m::singleton!(: [PioCommandWord; PREPARED_CAPACITY] =
        [PioCommandWord::END; PREPARED_CAPACITY])
    .unwrap();

    let mut controller = new_controller(resources, converter, config, system_clock_hz);
    check_preparation(&mut controller, outbound, storage_a, storage_b);
    let resources = release(controller);

    let mut controller = new_controller(resources, converter, config, system_clock_hz);
    assert_eq!(controller.state(), ControllerState::Unreferenced);
    defmt::info!("Estado: Unreferenced");
    defmt::info!("Referenciacion simulada: (0, 0)");
    controller.reference_position(ORIGIN).unwrap();
    check_idle(&controller, ORIGIN);
    run_move(
        &mut controller,
        "Ida",
        outbound,
        storage_a,
        ORIGIN,
        OUTBOUND_TARGET,
    );
    run_move(
        &mut controller,
        "Vuelta",
        inbound,
        storage_b,
        OUTBOUND_TARGET,
        ORIGIN,
    );
    let mut resources = release(controller);

    let mut observe_step_low = || {
        let start = timer.get_counter().ticks();
        let mut samples = 0u32;
        loop {
            assert!(steps_low(), "ambas señales STEP deben quedar en LOW");
            samples += 1;
            if timer.get_counter().ticks().wrapping_sub(start) >= STEP_OBSERVATION_US {
                break;
            }
        }
        defmt::info!(
            "STEP 1 y STEP 2 en LOW: {} lecturas GPIO durante {} us",
            samples,
            STEP_OBSERVATION_US
        );
    };
    for scenario in [
        Interruption::Preload,
        Interruption::Timeout,
        Interruption::Running,
    ] {
        let mut controller = new_controller(resources, converter, config, system_clock_hz);
        controller.reference_position(ORIGIN).unwrap();
        prepare(&mut controller, outbound, storage_a).unwrap();
        check_interruption(&mut controller, scenario, &mut observe_step_low);
        resources = release(controller);
    }

    defmt::info!("Benchmark terminado");
}

fn new_controller<'a>(
    (executor, pio, timer): MotionResources,
    converter: JointToStepsConverter,
    config: PioExecutionConfig,
    system_clock_hz: u64,
) -> Rp2350MotionController<'a> {
    match Rp2350MotionController::new(executor, pio, timer, converter, config, system_clock_hz) {
        Ok(controller) => controller,
        Err((_, error)) => panic!("no se pudo construir el controlador: {error:?}"),
    }
}

fn release(controller: Rp2350MotionController<'_>) -> MotionResources {
    match controller.into_resources() {
        Ok(resources) => resources,
        Err(mut controller) => {
            let _ = controller.cancel();
            panic!("se intentaron recuperar recursos durante el movimiento");
        }
    }
}

fn prepare<'a>(
    controller: &mut Rp2350MotionController<'a>,
    profiles: TimedCoordinatedJointPathProfiles<'_>,
    storage: &'a mut [PioCommandWord],
) -> Result<(), MotionError> {
    controller.prepare(
        profiles,
        &mut [0; 2],
        &mut [PinOperationBatch::new(0); 64],
        storage,
    )
}

fn check_preparation<'a>(
    controller: &mut Rp2350MotionController<'a>,
    outbound: TimedCoordinatedJointPathProfiles<'_>,
    storage_a: &'a mut [PioCommandWord],
    storage_b: &'a mut [PioCommandWord],
) {
    assert_eq!(controller.start(TIMEOUT_US), Err(INVALID_STATE));
    assert_eq!(prepare(controller, outbound, &mut []), Err(INVALID_STATE));
    let wrong_position = MotorSteps {
        first: 1,
        second: 0,
    };
    controller.reference_position(wrong_position).unwrap();
    assert_eq!(
        prepare(controller, outbound, storage_a),
        Err(MotionError::Controller(
            ControllerError::StartPositionMismatch
        )),
    );
    assert_eq!(
        controller.state(),
        ControllerState::Idle {
            position: wrong_position
        }
    );
    controller.invalidate_position().unwrap();
    controller.clear_fault().unwrap();
    controller.reference_position(ORIGIN).unwrap();
    assert_eq!(controller.start(TIMEOUT_US), Err(INVALID_STATE));
    assert_eq!(
        prepare(controller, outbound, &mut []),
        Err(MotionError::Preparation(
            ExecutorError::InsufficientCommandBuffer
        )),
    );
    assert_eq!(
        controller.state(),
        ControllerState::Idle { position: ORIGIN }
    );

    prepare(controller, outbound, storage_b).unwrap();
    controller.discard_prepared().unwrap();
    assert_eq!(controller.commanded_position(), Some(ORIGIN));
    assert_eq!(controller.target_position(), None);
    assert_eq!(
        controller.state(),
        ControllerState::Idle { position: ORIGIN }
    );
    defmt::info!("Casos de referencia, posicion incompatible, capacidad y descarte: OK");
}

fn check_idle(controller: &Rp2350MotionController<'_>, position: MotorSteps) {
    assert_eq!(controller.state(), ControllerState::Idle { position });
    assert_eq!(controller.commanded_position(), Some(position));
    defmt::info!(
        "Estado: Idle; posicion comandada: ({}, {})",
        position.first,
        position.second
    );
}

fn run_move<'a>(
    controller: &mut Rp2350MotionController<'a>,
    name: &str,
    profiles: TimedCoordinatedJointPathProfiles<'_>,
    storage: &'a mut [PioCommandWord],
    start: MotorSteps,
    target: MotorSteps,
) {
    defmt::info!("Preparando {}", name);
    prepare(controller, profiles, storage).unwrap();
    assert_eq!(controller.state(), ControllerState::Ready { start, target });
    assert_eq!(controller.target_position(), Some(target));
    defmt::info!(
        "Estado: Ready; objetivo: ({}, {})",
        target.first,
        target.second
    );
    controller.start(TIMEOUT_US).unwrap();
    let running = controller.state();
    let duplicate_start = controller.start(TIMEOUT_US);
    let prepare_running = prepare(controller, profiles, &mut []);
    defmt::info!("Estado: {:?}", defmt::Debug2Format(&running));
    let report = wait_for_report(controller);

    assert_eq!(running, ControllerState::Running { start, target });
    assert_eq!(duplicate_start, Err(INVALID_STATE));
    assert_eq!(prepare_running, Err(INVALID_STATE));
    assert_eq!(report.result, Ok(()));
    assert!(report.final_queued);
    defmt::info!(
        "{}: PIO Ok(()), {} bloques, END encolado",
        name,
        report.blocks_queued
    );
    check_idle(controller, target);
}

enum Interruption {
    Preload,
    Timeout,
    Running,
}

fn check_interruption(
    controller: &mut Rp2350MotionController<'_>,
    scenario: Interruption,
    observe_step_low: &mut impl FnMut(),
) {
    let (report, expected_error, reason) = match scenario {
        Interruption::Preload => {
            controller.start(TIMEOUT_US).unwrap();
            let elapsed = controller.running_elapsed_us();
            let report = controller.cancel().unwrap();
            assert_eq!(elapsed, None);
            defmt::info!("Cancelacion durante la precarga");
            (
                report,
                PioExecutionError::Cancelled,
                ControllerFault::Cancelled,
            )
        }
        Interruption::Timeout => {
            controller.start(1_000).unwrap();
            let report = wait_for_report(controller);
            (
                report,
                PioExecutionError::Timeout,
                ControllerFault::ExecutionFailed,
            )
        }
        Interruption::Running => {
            controller.start(TIMEOUT_US).unwrap();
            let (report, elapsed) = loop {
                assert!(
                    poll(controller).is_none(),
                    "PIO termino antes de la cancelacion"
                );
                if let Some(elapsed) = controller.running_elapsed_us()
                    && elapsed >= CANCEL_AFTER_US
                {
                    break (controller.cancel().unwrap(), elapsed);
                }
            };
            observe_step_low();
            defmt::info!(
                "Cancelacion solicitada tras {} us de ejecucion PIO",
                elapsed
            );
            (
                report,
                PioExecutionError::Cancelled,
                ControllerFault::Cancelled,
            )
        }
    };
    assert_eq!(report.result, Err(expected_error));
    assert_eq!(controller.state(), ControllerState::Fault { reason });
    assert_eq!(controller.commanded_position(), None);
    assert_eq!(controller.target_position(), None);
    assert_eq!(controller.start(TIMEOUT_US), Err(INVALID_STATE));
    defmt::info!(
        "Resultado PIO: {:?}; estado: {:?}; posicion comandada: None",
        defmt::Debug2Format(&report.result),
        defmt::Debug2Format(&controller.state()),
    );
    controller.clear_fault().unwrap();
    assert_eq!(controller.state(), ControllerState::Unreferenced);
    assert_eq!(controller.start(TIMEOUT_US), Err(INVALID_STATE));
    defmt::info!("Fallo limpiado: Unreferenced. Iniciar devuelve InvalidState");
    controller.reference_position(ORIGIN).unwrap();
    assert_eq!(controller.commanded_position(), Some(ORIGIN));
    defmt::info!("Nueva referencia simulada aceptada; recursos recuperados");
}

fn poll(controller: &mut Rp2350MotionController<'_>) -> Option<StreamingExecutionReport> {
    match controller.poll() {
        Ok(report) => report,
        Err(error) => {
            let _ = controller.cancel();
            panic!("fallo al supervisar PIO: {error:?}");
        }
    }
}

fn wait_for_report(controller: &mut Rp2350MotionController<'_>) -> StreamingExecutionReport {
    loop {
        if let Some(report) = poll(controller) {
            return report;
        }
    }
}
