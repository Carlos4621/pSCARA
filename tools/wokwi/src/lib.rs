#![no_std]

use executor::{
    ExecutorError, PioCommandWord, PioExecutionConfig, build_pio_commands, pio_wait_counter,
};
use kinematics::{ElbowConfiguration, JointAngles, Point, Scara};
use planner::{
    CoordinatedJointPathProfiles, JointAccelerationLimits, JointToStepsConverter, JointTrajectory,
    JointVelocityLimits, PinOperationBatch, PlannerError, StepDirTiming, StepDirection,
    StepTimeBase, TimedCoordinatedJointPathProfiles,
};
use trajectory::LineSegment;

pub const PIO_HZ: u64 = 1_000_000;
pub const SYSTEM_CLOCK_HZ: u32 = 150_000_000;
pub const BLOCK_WORDS: usize = 128;
pub const COMMAND_CAPACITY: usize = 384;
pub const BEFORE_WINDOW_US: u32 = 100;
pub const AFTER_WINDOW_US: u32 = 100;
pub const STARVE_US: u32 = 5_000;
const CRUISE_RAD_S: f64 = 12.5;
const ACCELERATION_RAD_S2: f64 = 2_500.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Case {
    Idle,
    First,
    Second,
    Both,
    Reverse,
    Ramp,
    Cartesian,
    Single,
    Fast,
    LongWait,
    Starved,
}

pub const CASES: [Case; 11] = [
    Case::Idle,
    Case::First,
    Case::Second,
    Case::Both,
    Case::Reverse,
    Case::Ramp,
    Case::Cartesian,
    Case::Single,
    Case::Fast,
    Case::LongWait,
    Case::Starved,
];

pub fn cases(include_starvation: bool) -> &'static [Case] {
    if include_starvation {
        &CASES
    } else {
        &CASES[..CASES.len() - 1]
    }
}

#[derive(Debug)]
pub enum ScenarioError {
    Planning(PlannerError),
    Encoding(ExecutorError),
    Geometry,
    Capacity,
}

impl From<PlannerError> for ScenarioError {
    fn from(error: PlannerError) -> Self {
        Self::Planning(error)
    }
}

impl From<ExecutorError> for ScenarioError {
    fn from(error: ExecutorError) -> Self {
        Self::Encoding(error)
    }
}

pub fn config() -> PioExecutionConfig {
    PioExecutionConfig {
        timebase: StepTimeBase::new(100_000).unwrap(),
        timing: StepDirTiming::new(10_000, 10_000, 10_000, 10_000).unwrap(),
        pio_frequency_hz: PIO_HZ,
        initial_first_direction: StepDirection::Negative,
        initial_second_direction: StepDirection::Positive,
        first_positive_is_high: true,
        second_positive_is_high: true,
    }
}

impl Case {
    pub fn name(self) -> &'static str {
        match self {
            Self::Idle => "reposo",
            Self::First => "motor-1",
            Self::Second => "motor-2",
            Self::Both => "simultaneo",
            Self::Reverse => "inversion",
            Self::Ramp => "rampa",
            Self::Cartesian => "cartesiana",
            Self::Single => "un-paso",
            Self::Fast => "frecuencia-maxima",
            Self::LongWait => "espera-larga",
            Self::Starved => "fifo-vacia",
        }
    }

    /// Conteo y desplazamiento derivados de la geometría o del estímulo explícito.
    pub fn expected_steps(self) -> ([usize; 2], [i64; 2]) {
        match self {
            Self::Idle => ([0, 0], [0, 0]),
            Self::First => ([3, 0], [3, 0]),
            Self::Second => ([0, 3], [0, -3]),
            Self::Both => ([6, 2], [6, 2]),
            Self::Reverse => ([6, 0], [0, 0]),
            Self::Ramp => ([16, 0], [16, 0]),
            Self::Cartesian => ([7, 2], [-7, -2]),
            Self::Single => ([1, 0], [1, 0]),
            Self::Fast => ([140, 140], [-140, 140]),
            Self::LongWait => ([2, 0], [-2, 0]),
            Self::Starved => (
                [BLOCK_WORDS; 2],
                [-(BLOCK_WORDS as i64), BLOCK_WORDS as i64],
            ),
        }
    }

    /// Recorre y conserva la secuencia completa antes de activar las salidas.
    pub fn prepare(
        self,
        storage: &mut [PioCommandWord],
    ) -> Result<&[PioCommandWord], ScenarioError> {
        let zero = angles(0.0, 0.0)?;
        match self {
            Self::Idle => store(storage, [Ok(PioCommandWord::END)]),
            Self::First => planned(storage, &[zero, angles(0.375, 0.0)?], 8.0, CRUISE_RAD_S),
            Self::Second => planned(storage, &[zero, angles(0.0, -0.375)?], 8.0, CRUISE_RAD_S),
            Self::Both => planned(storage, &[zero, angles(0.75, 0.25)?], 8.0, CRUISE_RAD_S),
            Self::Reverse => planned(
                storage,
                &[zero, angles(0.375, 0.0)?, zero],
                8.0,
                CRUISE_RAD_S,
            ),
            Self::Ramp => planned(storage, &[zero, angles(0.5, 0.0)?], 32.0, 25.0),
            Self::Cartesian => {
                let (points, count) = cartesian_joints()?;
                planned(storage, &points[..count], 32.0, CRUISE_RAD_S)
            }
            Self::Single => planned(storage, &[zero, angles(0.125, 0.0)?], 8.0, CRUISE_RAD_S),
            Self::Fast | Self::Starved => {
                let initial = config().initial_levels();
                let high = PioCommandWord::frame(pio_wait_counter(10)?, initial | 0b0101)?;
                let low = PioCommandWord::frame(pio_wait_counter(10)?, initial)?;
                store(
                    storage,
                    [high, low]
                        .into_iter()
                        .cycle()
                        .take(280)
                        .chain([PioCommandWord::END])
                        .map(Ok),
                )
            }
            Self::LongWait => {
                let initial = config().initial_levels();
                let high = PioCommandWord::frame(pio_wait_counter(10)?, initial | 1)?;
                let low = PioCommandWord::frame(pio_wait_counter(10)?, initial)?;
                let second = PioCommandWord::frame(pio_wait_counter(50_000)?, initial | 1)?;
                store(
                    storage,
                    [high, low, second, low, PioCommandWord::END]
                        .into_iter()
                        .map(Ok),
                )
            }
        }
    }

    /// Palabras que deben llegar al PIO cuando se deja de recargar después de la precarga.
    pub fn delivered_words(self, prepared: &[PioCommandWord]) -> &[PioCommandWord] {
        if self == Self::Starved {
            &prepared[..2 * BLOCK_WORDS]
        } else {
            prepared
        }
    }
}

const MAX_POINTS: usize = 8;

fn cartesian_joints() -> Result<([JointAngles; MAX_POINTS], usize), ScenarioError> {
    let scara = Scara::new(1.0, 1.0).map_err(|_| ScenarioError::Geometry)?;
    let line = LineSegment::new(
        Point::new(1.0, 1.0).map_err(|_| ScenarioError::Geometry)?,
        Point::new(1.25, 0.75).map_err(|_| ScenarioError::Geometry)?,
    )
    .map_err(|_| ScenarioError::Geometry)?;
    let mut points = [angles(0.0, 0.0)?; MAX_POINTS];
    let mut count = 0;
    for point in JointTrajectory::new(&scara, &line, 0.2, ElbowConfiguration::Positive)? {
        *points.get_mut(count).ok_or(ScenarioError::Capacity)? = point?;
        count += 1;
    }
    Ok((points, count))
}

pub fn cartesian_endpoint(displacement: [i64; 2]) -> Result<Point, ScenarioError> {
    let scara = Scara::new(1.0, 1.0).map_err(|_| ScenarioError::Geometry)?;
    let initial = JointToStepsConverter::new(32.0, 32.0)?.convert(cartesian_joints()?.0[0])?;
    let first = initial
        .first
        .checked_add(displacement[0])
        .ok_or(ScenarioError::Geometry)?;
    let second = initial
        .second
        .checked_add(displacement[1])
        .ok_or(ScenarioError::Geometry)?;
    scara
        .forward(angles(first as f64 / 32.0, second as f64 / 32.0)?)
        .map_err(|_| ScenarioError::Geometry)
}

fn angles(first: f64, second: f64) -> Result<JointAngles, ScenarioError> {
    JointAngles::new(first, second).map_err(|_| ScenarioError::Geometry)
}

fn planned<'a>(
    storage: &'a mut [PioCommandWord],
    points: &[JointAngles],
    steps_per_rad: f64,
    velocity: f64,
) -> Result<&'a [PioCommandWord], ScenarioError> {
    let speed = JointVelocityLimits::new(velocity, velocity)?;
    let acceleration = JointAccelerationLimits::new(ACCELERATION_RAD_S2, ACCELERATION_RAD_S2)?;
    let speeds = [0.0; MAX_POINTS];
    let path =
        CoordinatedJointPathProfiles::new(points, &speeds[..points.len()], &speed, &acceleration)?;
    let profiles = TimedCoordinatedJointPathProfiles::new(path)?;
    let mut boundaries = [0; MAX_POINTS];
    let mut operations = [PinOperationBatch::new(0); 64];
    let converter = JointToStepsConverter::new(steps_per_rad, steps_per_rad)?;
    let commands = build_pio_commands(
        profiles,
        converter,
        config(),
        &mut boundaries,
        &mut operations,
    )?;
    store(storage, commands)
}

fn store(
    storage: &mut [PioCommandWord],
    commands: impl IntoIterator<Item = Result<PioCommandWord, ExecutorError>>,
) -> Result<&[PioCommandWord], ScenarioError> {
    let mut count = 0;
    for word in commands {
        *storage.get_mut(count).ok_or(ScenarioError::Capacity)? = word?;
        count += 1;
    }
    Ok(&storage[..count])
}
