/// Errores al construir, convertir u operar los números finitos del crate.
///
/// Los constructores comprueban primero la finitud y después el signo: NaN y
/// ambos infinitos siempre producen [`Self::NonFiniteF64`]. Los errores de signo
/// se refieren a dominios inclusivos; el cero se admite en ambos.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum UtilitiesErrors {
    /// La entrada o el resultado es NaN o infinito, incluida la división por cero.
    NonFiniteF64,
    /// Se intentó construir un [`crate::FinitePositiveF64`] con un valor menor que cero.
    NonPositiveF64,
    /// Se intentó construir un [`crate::FiniteNegativeF64`] con un valor mayor que cero.
    NonNegativeF64,
}
