use crate::{PlannerError, StepTimeBase};

const NANOS_PER_SECOND: u128 = 1_000_000_000;

/// Tiempos mínimos de las señales STEP/DIR en nanosegundos.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StepDirTiming {
    step_high_ns: u64,
    step_low_ns: u64,
    dir_setup_ns: u64,
    dir_hold_ns: u64,
}

impl StepDirTiming {
    /// Devuelve [`PlannerError::InvalidStepDirTiming`] si algún tiempo es cero.
    pub fn new(
        step_high_ns: u64,
        step_low_ns: u64,
        dir_setup_ns: u64,
        dir_hold_ns: u64,
    ) -> Result<Self, PlannerError> {
        if step_high_ns == 0 || step_low_ns == 0 || dir_setup_ns == 0 || dir_hold_ns == 0 {
            return Err(PlannerError::InvalidStepDirTiming);
        }

        Ok(Self {
            step_high_ns,
            step_low_ns,
            dir_setup_ns,
            dir_hold_ns,
        })
    }

    /// Duración mínima del nivel alto de STEP.
    pub fn step_high_ns(&self) -> u64 {
        self.step_high_ns
    }

    /// Duración mínima del nivel bajo de STEP entre pulsos.
    pub fn step_low_ns(&self) -> u64 {
        self.step_low_ns
    }

    /// Tiempo mínimo entre establecer DIR y el flanco activo de STEP.
    pub fn dir_setup_ns(&self) -> u64 {
        self.dir_setup_ns
    }

    /// Tiempo mínimo que DIR debe conservar su valor después del flanco activo de STEP.
    pub fn dir_hold_ns(&self) -> u64 {
        self.dir_hold_ns
    }

    /// Intervalo mínimo entre flancos activos de STEP del mismo motor, en ticks redondeados hacia arriba.
    /// Devuelve [`PlannerError::NumericalFailure`] si el intervalo no cabe en `u64`.
    pub fn min_step_interval_ticks(&self, timebase: StepTimeBase) -> Result<u64, PlannerError> {
        let total_ns = u128::from(self.step_high_ns) + u128::from(self.step_low_ns);
        let ticks_per_second = u128::from(timebase.ticks_per_second());

        // Los segundos completos caben en 36 bits y el resto en 30.
        // Ambos productos y su suma caben en u128 incluso a la frecuencia máxima.
        let whole_seconds = total_ns / NANOS_PER_SECOND;
        let remaining_ns = total_ns % NANOS_PER_SECOND;
        let whole_ticks = whole_seconds * ticks_per_second;
        let remaining_ticks = (remaining_ns * ticks_per_second).div_ceil(NANOS_PER_SECOND);

        u64::try_from(whole_ticks + remaining_ticks).map_err(|_| PlannerError::NumericalFailure)
    }

    /// Tiempo mínimo de establecimiento de DIR en ticks, redondeado hacia arriba.
    /// Devuelve [`PlannerError::NumericalFailure`] si el resultado no cabe en `u64`.
    pub fn min_dir_setup_ticks(&self, timebase: StepTimeBase) -> Result<u64, PlannerError> {
        nanoseconds_to_ticks(self.dir_setup_ns, timebase)
    }

    /// Tiempo mínimo de mantenimiento de DIR en ticks, redondeado hacia arriba.
    /// Devuelve [`PlannerError::NumericalFailure`] si el resultado no cabe en `u64`.
    pub fn min_dir_hold_ticks(&self, timebase: StepTimeBase) -> Result<u64, PlannerError> {
        nanoseconds_to_ticks(self.dir_hold_ns, timebase)
    }

    /// Devuelve los límites inclusivos de la ventana para cambiar DIR.
    /// Sin un pulso anterior, la ventana empieza en el tick cero.
    ///
    /// Devuelve [`PlannerError::InvalidTicks`] si los pulsos tienen ticks iguales o decrecientes,
    /// y [`PlannerError::DirChangeNotSchedulable`] si la ventana queda vacía.
    /// Propaga los errores al convertir los requisitos aplicables a ticks.
    pub fn dir_change_window(
        &self,
        previous_step_tick: Option<u64>,
        next_step_tick: u64,
        timebase: StepTimeBase,
    ) -> Result<(u64, u64), PlannerError> {
        if previous_step_tick.is_some_and(|previous| previous >= next_step_tick) {
            return Err(PlannerError::InvalidTicks);
        }

        let earliest_tick = match previous_step_tick {
            Some(previous) => previous
                .checked_add(self.min_dir_hold_ticks(timebase)?)
                .ok_or(PlannerError::DirChangeNotSchedulable)?,
            None => 0,
        };
        let latest_tick = next_step_tick
            .checked_sub(self.min_dir_setup_ticks(timebase)?)
            .ok_or(PlannerError::DirChangeNotSchedulable)?;

        if earliest_tick > latest_tick {
            return Err(PlannerError::DirChangeNotSchedulable);
        }

        Ok((earliest_tick, latest_tick))
    }
}

fn nanoseconds_to_ticks(nanoseconds: u64, timebase: StepTimeBase) -> Result<u64, PlannerError> {
    let numerator = u128::from(nanoseconds) * u128::from(timebase.ticks_per_second());
    let ticks = numerator.div_ceil(NANOS_PER_SECOND);
    u64::try_from(ticks).map_err(|_| PlannerError::NumericalFailure)
}
