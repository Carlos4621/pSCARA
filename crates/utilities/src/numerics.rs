use crate::UtilitiesErrors;

pub fn validate_finite(value: f64) -> Result<(), UtilitiesErrors> {
    if !value.is_finite() {
        return Err(UtilitiesErrors::NonFiniteF64);
    }

    Ok(())
}

pub fn validate_finite_positive(value: f64) -> Result<(), UtilitiesErrors> {
    validate_finite(value)?;

    if value < 0.0 {
        return Err(UtilitiesErrors::NonPositiveF64);
    }

    Ok(())
}

pub fn validate_finite_negativo(value: f64) -> Result<(), UtilitiesErrors> {
    validate_finite(value)?;

    if value > 0.0 {
        return Err(UtilitiesErrors::NonNegativeF64);
    }

    Ok(())
}

pub fn validate_non_zero_finite(value: f64) -> Result<(), UtilitiesErrors> {
    validate_finite(value)?;

    if value == 0.0 {
        return Err(UtilitiesErrors::NonZeroFiniteF64);
    }

    Ok(())
}
