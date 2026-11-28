use controller::{ControllerError, ControllerFault, ControllerState, MotionController};
use executor::{PioCommandWord, PioExecutionConfig, PreparedPioTrajectory, prepare_pio_trajectory};
use kinematics::JointAngles;
use planner::{
    CoordinatedJointPathProfiles, JointAccelerationLimits, JointToStepsConverter,
    JointVelocityLimits, MotorSteps, PinOperationBatch, StepDirTiming, StepDirection, StepTimeBase,
    TimedCoordinatedJointPathProfiles,
};

const START: MotorSteps = MotorSteps {
    first: 8,
    second: -2,
};
const TARGET: MotorSteps = MotorSteps {
    first: 10,
    second: -3,
};
const IDLE: ControllerState = ControllerState::Idle { position: START };
const READY: ControllerState = ControllerState::Ready {
    start: START,
    target: TARGET,
};
const RUNNING: ControllerState = ControllerState::Running {
    start: START,
    target: TARGET,
};
const EXECUTION_FAULT: ControllerState = ControllerState::Fault {
    reason: ControllerFault::ExecutionFailed,
};
const LOST_POSITION: ControllerState = ControllerState::Fault {
    reason: ControllerFault::PositionLost,
};

fn prepare(
    storage: &mut [PioCommandWord],
    start: MotorSteps,
    target: MotorSteps,
) -> PreparedPioTrajectory<'_> {
    let points = [start, target].map(|position| {
        JointAngles::new(position.first as f64 / 8.0, position.second as f64 / 4.0).unwrap()
    });
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let profiles =
        CoordinatedJointPathProfiles::new(&points, &[0.0; 2], &velocity, &acceleration).unwrap();
    let profiles = TimedCoordinatedJointPathProfiles::new(profiles).unwrap();
    let config = PioExecutionConfig {
        timebase: StepTimeBase::new(8).unwrap(),
        timing: StepDirTiming::new(125_000_000, 125_000_000, 1, 1).unwrap(),
        pio_frequency_hz: 80,
        initial_first_direction: StepDirection::Negative,
        initial_second_direction: StepDirection::Positive,
        first_positive_is_high: true,
        second_positive_is_high: true,
    };
    prepare_pio_trajectory(
        profiles,
        JointToStepsConverter::new(8.0, 4.0).unwrap(),
        config,
        &mut [0; 2],
        &mut [PinOperationBatch::new(0); 8],
        storage,
    )
    .unwrap()
}

fn controller_in(
    state: ControllerState,
    trajectory: &PreparedPioTrajectory<'_>,
) -> MotionController {
    let mut controller = MotionController::new();
    if state == ControllerState::Unreferenced {
        return controller;
    }
    controller.reference_position(START).unwrap();
    if state == IDLE {
        return controller;
    }
    controller.accept_prepared(trajectory).unwrap();
    if state == READY {
        return controller;
    }
    controller.start().unwrap();
    if let ControllerState::Fault { reason } = state {
        controller.finish_failure(reason).unwrap();
    }
    controller
}

fn assert_state(controller: &MotionController, expected: ControllerState) {
    assert_eq!(controller.state(), expected);
    let (commanded, target) = match expected {
        ControllerState::Unreferenced | ControllerState::Fault { .. } => (None, None),
        ControllerState::Idle { position } => (Some(position), None),
        ControllerState::Ready { start, target } => (Some(start), Some(target)),
        ControllerState::Running { target, .. } => (None, Some(target)),
    };
    assert_eq!(controller.commanded_position(), commanded);
    assert_eq!(controller.target_position(), target);
}

#[test]
fn new_and_default_require_a_reference() {
    assert_state(&MotionController::new(), ControllerState::Unreferenced);
    assert_state(&MotionController::default(), ControllerState::Unreferenced);
}

#[test]
fn tests_each_operation_from_each_state_and_preserves_state_on_error() {
    type Operation =
        fn(&mut MotionController, &PreparedPioTrajectory<'_>) -> Result<(), ControllerError>;
    let operations: [(&str, Operation); 8] = [
        ("reference", |c, _| c.reference_position(START)),
        ("accept", MotionController::accept_prepared),
        ("start", |c, _| c.start()),
        ("finish_success", |c, _| c.finish_success()),
        ("discard", |c, _| c.discard_prepared()),
        ("finish_failure", |c, _| {
            c.finish_failure(ControllerFault::ExecutionFailed)
        }),
        ("clear_fault", |c, _| c.clear_fault()),
        ("invalidate", |c, _| c.invalidate_position()),
    ];

    let mut storage = [PioCommandWord::END; 32];
    let trajectory = prepare(&mut storage, START, TARGET);
    let cases = [
        (
            ControllerState::Unreferenced,
            [Some(IDLE), None, None, None, None, None, None, None],
        ),
        (
            IDLE,
            [
                None,
                Some(READY),
                None,
                None,
                None,
                None,
                None,
                Some(LOST_POSITION),
            ],
        ),
        (
            READY,
            [
                None,
                None,
                Some(RUNNING),
                None,
                Some(IDLE),
                None,
                None,
                Some(LOST_POSITION),
            ],
        ),
        (
            RUNNING,
            [
                None,
                None,
                None,
                Some(ControllerState::Idle { position: TARGET }),
                None,
                Some(EXECUTION_FAULT),
                None,
                None,
            ],
        ),
        (
            EXECUTION_FAULT,
            [
                None,
                None,
                None,
                None,
                None,
                None,
                Some(ControllerState::Unreferenced),
                None,
            ],
        ),
    ];

    for (state, next_states) in cases {
        for ((name, operate), next) in operations.into_iter().zip(next_states) {
            let mut controller = controller_in(state, &trajectory);
            assert_state(&controller, state);
            let result = operate(&mut controller, &trajectory);
            assert_eq!(
                result,
                next.map_or(Err(ControllerError::InvalidState), |_| Ok(())),
                "state={state:?}, operation={name}",
            );
            assert_state(&controller, next.unwrap_or(state));
        }
    }
}

#[test]
fn rejects_a_start_mismatch_on_either_motor() {
    let mut storage = [PioCommandWord::END; 32];
    let trajectory = prepare(&mut storage, START, TARGET);
    for position in [
        MotorSteps {
            first: START.first + 1,
            ..START
        },
        MotorSteps {
            second: START.second - 1,
            ..START
        },
    ] {
        let mut controller = MotionController::new();
        controller.reference_position(position).unwrap();
        assert_eq!(
            controller.accept_prepared(&trajectory),
            Err(ControllerError::StartPositionMismatch)
        );
        assert_state(&controller, ControllerState::Idle { position });
    }
}

#[test]
fn successful_execution_allows_a_new_trajectory_only_from_the_completed_target() {
    let mut storage = [PioCommandWord::END; 32];
    let trajectory = prepare(&mut storage, START, TARGET);
    let mut controller = controller_in(RUNNING, &trajectory);
    controller.finish_success().unwrap();
    assert_eq!(
        controller.accept_prepared(&trajectory),
        Err(ControllerError::StartPositionMismatch)
    );
    assert_state(&controller, ControllerState::Idle { position: TARGET });

    let mut return_storage = [PioCommandWord::END; 32];
    let return_trajectory = prepare(&mut return_storage, TARGET, START);
    controller.accept_prepared(&return_trajectory).unwrap();
    assert_state(
        &controller,
        ControllerState::Ready {
            start: TARGET,
            target: START,
        },
    );
    controller.start().unwrap();
    controller.finish_success().unwrap();
    assert_state(&controller, IDLE);
}

#[test]
fn cancellation_execution_failure_and_lost_position_require_a_new_reference() {
    let mut storage = [PioCommandWord::END; 32];
    let trajectory = prepare(&mut storage, START, TARGET);
    for (initial, reason) in [
        (RUNNING, ControllerFault::Cancelled),
        (RUNNING, ControllerFault::ExecutionFailed),
        (IDLE, ControllerFault::PositionLost),
        (READY, ControllerFault::PositionLost),
    ] {
        let mut controller = controller_in(initial, &trajectory);
        if reason == ControllerFault::PositionLost {
            controller.invalidate_position().unwrap();
        } else {
            controller.finish_failure(reason).unwrap();
        }
        assert_state(&controller, ControllerState::Fault { reason });
        assert_eq!(controller.start(), Err(ControllerError::InvalidState));
        assert_eq!(
            controller.reference_position(START),
            Err(ControllerError::InvalidState)
        );
        controller.clear_fault().unwrap();
        assert_state(&controller, ControllerState::Unreferenced);
        assert_eq!(
            controller.accept_prepared(&trajectory),
            Err(ControllerError::InvalidState)
        );

        let measured = MotorSteps {
            first: i64::MIN,
            second: i64::MAX,
        };
        controller.reference_position(measured).unwrap();
        assert_state(&controller, ControllerState::Idle { position: measured });
    }
}
