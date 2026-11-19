use crate::{UtilitiesErrors, validate_finite_positive};

/// Suma tiempos finitos y no negativos en el orden del slice.
///
/// Un slice vacío suma cero. Devuelve el primer error: un término inválido,
/// un desbordamiento o una suma en la que desaparece un operando positivo.
pub fn checked_time_sumatory(times: &[f64]) -> Result<f64, UtilitiesErrors> {
    let mut total = 0.0;

    for &duration in times {
        validate_finite_positive(duration).map_err(|_| UtilitiesErrors::InvalidTime)?;

        let next = total + duration;
        if !next.is_finite() {
            return Err(UtilitiesErrors::TimeOverflow);
        }
        if total > 0.0 && duration > 0.0 && (next == total || next == duration) {
            return Err(UtilitiesErrors::TimePrecisionLost);
        }
        total = next;
    }

    Ok(total)
}
