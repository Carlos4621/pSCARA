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
}
