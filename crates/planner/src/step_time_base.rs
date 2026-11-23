use crate::PlannerError;

#[derive(Debug, Clone, Copy)]
pub struct StepTimeBase {
    ticks_per_second: u64,
}

impl StepTimeBase {
    pub fn new(ticks_per_second: u64) -> Result<Self, PlannerError> {
        if ticks_per_second == 0 {
            return Err(PlannerError::InvalidTicks);
        }

        Ok(Self { ticks_per_second })
    }

    /// Convierte ticks a segundos como el `f64` más cercano al cociente exacto.
    /// En caso de empate, elige el valor con significando par.
    pub fn ticks_to_seconds(&self, ticks: u64) -> f64 {
        if ticks == 0 {
            return 0.0;
        }

        let numerator = u128::from(ticks);
        let denominator = u128::from(self.ticks_per_second);
        let mut exponent = ticks.ilog2() as i32 - self.ticks_per_second.ilog2() as i32;
        // The difference in bit lengths is at most one above floor(log2(ticks / rate)).
        let below_power = if exponent >= 0 {
            numerator < (denominator << exponent)
        } else {
            (numerator << exponent.unsigned_abs()) < denominator
        };
        exponent -= i32::from(below_power);

        let shift = 52 - exponent;
        let (numerator, denominator) = if shift >= 0 {
            (numerator << shift, denominator)
        } else {
            (numerator, denominator << shift.unsigned_abs())
        };
        let mut significand = numerator / denominator;
        let twice_remainder = 2 * (numerator % denominator);
        if twice_remainder > denominator || (twice_remainder == denominator && significand & 1 != 0)
        {
            significand += 1;
        }

        libm::scalbn(significand as f64, exponent - 52)
    }

    /// Convierte el valor exacto de `seconds` al tick más cercano.
    /// Los empates de medio tick se redondean hacia arriba.
    /// Rechaza tiempos negativos o no finitos con [`PlannerError::InvalidTime`]
    /// y resultados que no caben en `u64` con [`PlannerError::NumericalFailure`].
    pub fn seconds_to_ticks(&self, seconds: f64) -> Result<u64, PlannerError> {
        utilities::validate_finite_positive(seconds).map_err(|_| PlannerError::InvalidTime)?;

        let bits = seconds.to_bits();
        let encoded_exponent = ((bits >> 52) & 0x7ff) as i32;
        let fraction = bits & ((1_u64 << 52) - 1);
        let (significand, exponent) = if encoded_exponent == 0 {
            (fraction, -1074)
        } else {
            (fraction | (1_u64 << 52), encoded_exponent - 1023 - 52)
        };

        let product = u128::from(significand) * u128::from(self.ticks_per_second);
        if product == 0 {
            return Ok(0);
        }

        let rounded = if exponent >= 0 {
            let shift = exponent as u32;
            if shift >= u64::BITS || product > u128::from(u64::MAX >> shift) {
                return Err(PlannerError::NumericalFailure);
            }
            product << shift
        } else {
            let shift = exponent.unsigned_abs();
            if shift >= u128::BITS {
                return Ok(0);
            }

            let round_up = (product >> (shift - 1)) & 1;
            (product >> shift) + round_up
        };

        u64::try_from(rounded).map_err(|_| PlannerError::NumericalFailure)
    }

    /// Devuelve los ticks que rodean un instante temporal.
    ///
    /// Si el instante coincide con un tick representable,
    /// devuelve el mismo tick en ambas posiciones.
    ///
    /// Si está entre dos ticks, devuelve (anterior, siguiente).
    pub fn bracketing_ticks(&self, seconds: f64) -> Result<(u64, u64), PlannerError> {
        let nearest_tick = self.seconds_to_ticks(seconds)?;
        let nearest_time = self.ticks_to_seconds(nearest_tick);

        if nearest_time == seconds {
            return Ok((nearest_tick, nearest_tick));
        }

        if nearest_time < seconds {
            let next_tick = nearest_tick
                .checked_add(1)
                .ok_or(PlannerError::NumericalFailure)?;

            if self.ticks_to_seconds(next_tick) == seconds {
                return Ok((next_tick, next_tick));
            }

            Ok((nearest_tick, next_tick))
        } else {
            let previous_tick = nearest_tick
                .checked_sub(1)
                .ok_or(PlannerError::NumericalFailure)?;

            Ok((previous_tick, nearest_tick))
        }
    }
}
