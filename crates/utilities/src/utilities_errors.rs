/// Errores de validación de valores `f64` y de acumulación de tiempos.
///
/// Los validadores comprueban primero la finitud y después el signo: NaN y
/// ambos infinitos siempre producen [`Self::NonFiniteF64`]. Los errores de signo
/// se refieren a dominios inclusivos; el cero se admite en ambos.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum UtilitiesErrors {
    /// El valor validado es NaN o infinito.
    NonFiniteF64,
    /// Se pasó un valor finito menor que cero a [`crate::validate_finite_positive`].
    NonPositiveF64,
    /// Se pasó un valor finito mayor que cero a [`crate::validate_finite_negativo`].
    NonNegativeF64,
    NonZeroFiniteF64,
    NonZeroFinitePositiveF64,
    /// Un tiempo de entrada es negativo o no finito.
    InvalidTime,
    /// Un operando positivo se perdió por redondeo durante la suma de tiempos.
    TimePrecisionLost,
    /// La suma de tiempos válidos excedió el rango finito.
    TimeOverflow,
}
