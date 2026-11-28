use controller::{ControllerError, ControllerFault, ControllerState, MotionController};
use executor::{
    ExecutorError, PioCommandWord, PioExecutionConfig, PreparedPioCommands, PreparedPioTrajectory,
    prepare_pio_trajectory,
};
use planner::{
    JointToStepsConverter, MotorSteps, PinOperationBatch, TimedCoordinatedJointPathProfiles,
};
use rp235x_hal::{
    pac::PIO0,
    pio::PIO,
    timer::{CopyableTimer0, Timer},
};

use crate::{
    dma_executor::PioExecutionError,
    streaming_dma_executor::{
        ActiveStreamingExecution, StreamingDmaExecutor, StreamingExecutionReport, StreamingPoll,
    },
};

pub type MotionResources = (StreamingDmaExecutor, PIO<PIO0>, Timer<CopyableTimer0>);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionError {
    Controller(ControllerError),
    Preparation(ExecutorError),
    Execution(PioExecutionError),
    FrequencyMismatch,
    InterMoveGuard,
}

impl From<ControllerError> for MotionError {
    fn from(error: ControllerError) -> Self {
        Self::Controller(error)
    }
}

impl From<ExecutorError> for MotionError {
    fn from(error: ExecutorError) -> Self {
        Self::Preparation(error)
    }
}

impl From<PioExecutionError> for MotionError {
    fn from(error: PioExecutionError) -> Self {
        Self::Execution(error)
    }
}

#[must_use = "llama a poll hasta finalizar o a cancel antes de abandonar el controlador"]
pub struct Rp2350MotionController<'a> {
    controller: MotionController,
    executor: Option<StreamingDmaExecutor>,
    prepared: Option<PreparedPioTrajectory<'a>>,
    active: Option<ActiveStreamingExecution<PreparedPioCommands<'a>>>,
    pio: PIO<PIO0>,
    timer: Timer<CopyableTimer0>,
    converter: JointToStepsConverter,
    config: PioExecutionConfig,
    last_stop_us: Option<u64>,
}

impl<'a> Rp2350MotionController<'a> {
    pub fn new(
        executor: StreamingDmaExecutor,
        pio: PIO<PIO0>,
        timer: Timer<CopyableTimer0>,
        converter: JointToStepsConverter,
        config: PioExecutionConfig,
        system_clock_hz: u64,
    ) -> Result<Self, (MotionResources, MotionError)> {
        let divisor = u64::from(executor.clock_divisor());
        let valid_frequency = divisor != 0
            && system_clock_hz != 0
            && system_clock_hz.is_multiple_of(divisor)
            && system_clock_hz / divisor == config.pio_frequency_hz;
        if !valid_frequency {
            return Err(((executor, pio, timer), MotionError::FrequencyMismatch));
        }

        Ok(Self {
            controller: MotionController::new(),
            executor: Some(executor),
            prepared: None,
            active: None,
            pio,
            timer,
            converter,
            config,
            last_stop_us: None,
        })
    }

    pub fn state(&self) -> ControllerState {
        self.controller.state()
    }

    pub fn commanded_position(&self) -> Option<MotorSteps> {
        self.controller.commanded_position()
    }

    pub fn target_position(&self) -> Option<MotorSteps> {
        self.controller.target_position()
    }

    pub fn reference_position(&mut self, position: MotorSteps) -> Result<(), MotionError> {
        self.controller.reference_position(position)?;
        Ok(())
    }

    pub fn clear_fault(&mut self) -> Result<(), MotionError> {
        self.controller.clear_fault()?;
        Ok(())
    }

    pub fn discard_prepared(&mut self) -> Result<(), MotionError> {
        self.controller.discard_prepared()?;
        self.prepared = None;
        Ok(())
    }

    pub fn invalidate_position(&mut self) -> Result<(), MotionError> {
        self.controller.invalidate_position()?;
        self.prepared = None;
        Ok(())
    }

    pub fn prepare(
        &mut self,
        profiles: TimedCoordinatedJointPathProfiles<'_>,
        boundaries: &mut [u64],
        operations: &mut [PinOperationBatch],
        storage: &'a mut [PioCommandWord],
    ) -> Result<(), MotionError> {
        if !matches!(self.state(), ControllerState::Idle { .. }) {
            return Err(ControllerError::InvalidState.into());
        }

        let trajectory = prepare_pio_trajectory(
            profiles,
            self.converter,
            self.config,
            boundaries,
            operations,
            storage,
        )?;
        self.controller.accept_prepared(&trajectory)?;
        self.prepared = Some(trajectory);
        Ok(())
    }

    pub fn start(&mut self, timeout_us: u64) -> Result<(), MotionError> {
        if !matches!(self.state(), ControllerState::Ready { .. }) {
            return Err(ControllerError::InvalidState.into());
        }
        let now = self.timer.get_counter().ticks();
        if let Some(last_stop) = self.last_stop_us
            && now.wrapping_sub(last_stop) < self.inter_move_guard_us()
        {
            return Err(MotionError::InterMoveGuard);
        }
        let setup_guard = self.initial_setup_guard_us();
        self.controller.start()?;
        let trajectory = self
            .prepared
            .take()
            .expect("Ready requiere una trayectoria");
        let executor = self.executor.take().expect("Ready requiere un ejecutor");
        let initial_levels = trajectory.initial_levels();
        match executor.start_guarded(
            &mut self.pio,
            &self.timer,
            trajectory.into_commands(),
            initial_levels,
            timeout_us,
            setup_guard,
        ) {
            Ok(active) => {
                self.active = Some(active);
                Ok(())
            }
            Err((executor, error)) => {
                self.restore_executor(executor);
                self.controller
                    .finish_failure(ControllerFault::ExecutionFailed)
                    .expect("el controlador debe estar en Running");
                Err(error.into())
            }
        }
    }

    pub fn running_elapsed_us(&self) -> Option<u64> {
        self.active.as_ref()?.running_elapsed_us(&self.timer)
    }

    pub fn poll(&mut self) -> Result<Option<StreamingExecutionReport>, MotionError> {
        if !matches!(self.state(), ControllerState::Running { .. }) {
            return Err(ControllerError::InvalidState.into());
        }

        let active = self
            .active
            .take()
            .expect("Running requiere una ejecución activa");
        match active.poll(&mut self.pio, &self.timer) {
            StreamingPoll::Pending(active) => {
                self.active = Some(active);
                Ok(None)
            }
            StreamingPoll::Finished { executor, report } => {
                self.restore_executor(executor);
                if report.result.is_ok() {
                    self.controller.finish_success()?;
                } else {
                    self.controller
                        .finish_failure(ControllerFault::ExecutionFailed)?;
                }
                Ok(Some(report))
            }
        }
    }

    pub fn cancel(&mut self) -> Result<StreamingExecutionReport, MotionError> {
        if !matches!(self.state(), ControllerState::Running { .. }) {
            return Err(ControllerError::InvalidState.into());
        }

        let active = self
            .active
            .take()
            .expect("Running requiere una ejecución activa");
        let (executor, report) = active.cancel();
        self.restore_executor(executor);
        let reason = match report.result {
            Err(PioExecutionError::Cancelled) => ControllerFault::Cancelled,
            _ => ControllerFault::ExecutionFailed,
        };
        self.controller.finish_failure(reason)?;
        Ok(report)
    }

    /// Consume un controlador detenido para reutilizar sus recursos en otro caso del banco.
    #[allow(
        clippy::result_large_err,
        reason = "devuelve el controlador activo intacto y sin asignar memoria"
    )]
    pub(crate) fn into_resources(self) -> Result<MotionResources, Self> {
        if self.active.is_some() {
            return Err(self);
        }
        let executor = self
            .executor
            .expect("un controlador detenido conserva el ejecutor");
        Ok((executor, self.pio, self.timer))
    }

    fn restore_executor(&mut self, executor: StreamingDmaExecutor) {
        self.executor = Some(executor);
        self.last_stop_us = Some(self.timer.get_counter().ticks());
    }

    fn inter_move_guard_us(&self) -> u64 {
        guard_us(
            self.config
                .timing
                .dir_hold_ns()
                .max(self.config.timing.step_low_ns()),
        )
    }

    fn initial_setup_guard_us(&self) -> u64 {
        guard_us(
            self.config
                .timing
                .dir_setup_ns()
                .max(self.config.timing.step_low_ns()),
        )
    }
}

fn guard_us(nanoseconds: u64) -> u64 {
    nanoseconds.div_ceil(1000) + 1
}
