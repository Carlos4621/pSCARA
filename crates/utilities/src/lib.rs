#![no_std]

//! Números `f64` finitos con validación de signo y operaciones comprobadas.
//!
//! Este crate no requiere `std` ni asignaciones de memoria. Sus tipos mantienen
//! sus invariantes mediante campos privados y constructores que devuelven
//! [`Result`].
//!
//! | Tipo | Valores admitidos |
//! | --- | --- |
//! | [`FiniteF64`] | Cualquier `f64` finito |
//! | [`FinitePositiveF64`] | Valores finitos mayores o iguales a cero |
//! | [`FiniteNegativeF64`] | Valores finitos menores o iguales a cero |
//!
//! Los tres tipos admiten `+0.0` y `-0.0`, conservando su representación.
//! La igualdad y el orden siguen las comparaciones numéricas de `f64`:
//! ambos ceros son iguales aunque sus bits sean distintos.
//!
//! # Aritmética y errores
//!
//! Las operaciones usan el redondeo de `f64`. Las operaciones binarias devuelven
//! `Result`: un resultado infinito o NaN produce [`UtilitiesErrors::NonFiniteF64`].
//! Esto incluye la división entre cero. El subdesbordamiento a un subnormal o
//! a cero es válido; estos tipos garantizan finitud, no aritmética exacta.
//!
//! ```
//! use utilities::{FiniteF64, FinitePositiveF64, UtilitiesErrors};
//!
//! fn sumar(a: f64, b: f64) -> Result<FiniteF64, UtilitiesErrors> {
//!     let a = FiniteF64::try_from(a)?;
//!     let b: FiniteF64 = b.try_into()?;
//!     a + b
//! }
//!
//! assert_eq!(sumar(1.5, 2.25)?.get_value(), 3.75);
//! assert_eq!(sumar(f64::MAX, f64::MAX), Err(UtilitiesErrors::NonFiniteF64));
//!
//! let positivo = FinitePositiveF64::try_from(2.5)?;
//! let general: FiniteF64 = positivo.into();
//! let original: f64 = general.into();
//! assert_eq!(original, 2.5);
//! # Ok::<(), UtilitiesErrors>(())
//! ```
//!
//! # Conversiones
//!
//! [`TryFrom<f64>`] y [`TryInto`] validan la entrada mediante los constructores.
//! [`From`] e [`Into`] permiten extraer un `f64` de cualquiera de los tres tipos
//! o convertir un tipo con signo en [`FiniteF64`] sin perder bits.
//! Para restringir el signo de un [`FiniteF64`], utiliza
//! [`FiniteF64::to_finite_positive`] o [`FiniteF64::to_finite_negative`].

mod finite_f64;
pub use finite_f64::FiniteF64;

mod finite_positive_f64;
pub use finite_positive_f64::FinitePositiveF64;

mod utilities_errors;
pub use utilities_errors::UtilitiesErrors;

mod finite_negative_f64;
pub use finite_negative_f64::FiniteNegativeF64;
