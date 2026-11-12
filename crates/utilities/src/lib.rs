#![no_std]

//! Validación de valores `f64` sin depender de `std` ni asignar memoria.
//!
//! Las funciones devuelven `Ok(())` cuando la entrada satisface la condición,
//! o un [`UtilitiesErrors`] que indica el motivo del rechazo. No convierten
//! ni almacenan el número, y no validan automáticamente operaciones posteriores.
//!
//! [`validate_finite`] admite cualquier valor finito. Los dominios de
//! [`validate_finite_positive`] y [`validate_finite_negativo`] son inclusivos:
//! admiten, respectivamente, valores mayores o iguales a cero y menores o
//! iguales a cero. Ambos ceros con signo son válidos en estos tres validadores.
//! La finitud se comprueba antes que el signo.
//!
//! ```
//! use utilities::{validate_finite, validate_finite_positive, validate_finite_negativo, UtilitiesErrors};
//!
//! fn sumar(a: f64, b: f64) -> Result<f64, UtilitiesErrors> {
//!     validate_finite(a)?;
//!     validate_finite(b)?;
//!     let result = a + b;
//!     validate_finite(result)?;
//!     Ok(result)
//! }
//!
//! assert_eq!(sumar(1.5, 2.25)?, 3.75);
//! assert_eq!(sumar(f64::MAX, f64::MAX), Err(UtilitiesErrors::NonFiniteF64));
//! assert_eq!(validate_finite_positive(-0.0), Ok(()));
//! assert_eq!(validate_finite_negativo(0.0), Ok(()));
//! assert_eq!(validate_finite_positive(-1.0), Err(UtilitiesErrors::NonPositiveF64));
//! # Ok::<(), UtilitiesErrors>(())
//! ```

mod numerics;
pub use numerics::*;

mod utilities_errors;
pub use utilities_errors::UtilitiesErrors;
