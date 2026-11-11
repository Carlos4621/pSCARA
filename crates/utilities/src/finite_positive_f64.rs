use core::ops::{Add, Div, Mul, Neg, Sub};

use crate::{FiniteF64, UtilitiesErrors, finite_negative_f64::FiniteNegativeF64};

/// Un `f64` finito mayor o igual a cero.
///
/// El dominio es inclusivo: pese al nombre, admite `+0.0` y `-0.0` y conserva
/// sus bits. Sus operaciones no exigen resultados estrictamente positivos;
/// el subdesbordamiento a cero es válido.
///
/// Suma, producto y cociente devuelven `Result<Self, UtilitiesErrors>`.
/// La resta devuelve un [`FiniteF64`], pues puede tener cualquier signo.
/// La negación devuelve `Result<FiniteNegativeF64, UtilitiesErrors>`.
///
/// ```
/// use utilities::{FiniteF64, FinitePositiveF64, UtilitiesErrors};
///
/// let a = FinitePositiveF64::new(2.0)?;
/// let b = FinitePositiveF64::new(4.0)?;
/// assert_eq!((a + b)?.get_value(), 6.0);
/// let difference: FiniteF64 = (a - b)?;
/// assert_eq!(difference.get_value(), -2.0);
/// assert_eq!((a * b)?.get_value(), 8.0);
/// assert_eq!((a / b)?.get_value(), 0.5);
/// assert_eq!((-a)?.get_value(), -2.0);
/// # Ok::<(), UtilitiesErrors>(())
/// ```
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct FinitePositiveF64 {
    value: FiniteF64,
}

impl FinitePositiveF64 {
    /// Construye un valor finito no negativo, conservando su representación.
    ///
    /// # Errors
    ///
    /// Comprueba primero la finitud: NaN y ambos infinitos producen
    /// [`UtilitiesErrors::NonFiniteF64`]. Un valor finito menor que cero produce
    /// [`UtilitiesErrors::NonPositiveF64`]. Ambos ceros con signo son válidos.
    ///
    /// ```
    /// use utilities::{FinitePositiveF64, UtilitiesErrors};
    ///
    /// assert_eq!(FinitePositiveF64::new(0.0)?.get_value(), 0.0);
    /// assert_eq!(FinitePositiveF64::new(-1.0), Err(UtilitiesErrors::NonPositiveF64));
    /// assert_eq!(FinitePositiveF64::new(f64::NEG_INFINITY), Err(UtilitiesErrors::NonFiniteF64));
    /// # Ok::<(), UtilitiesErrors>(())
    /// ```
    pub fn new(value: f64) -> Result<Self, UtilitiesErrors> {
        let value = FiniteF64::new(value)?;

        if value.get_value() < 0.0 {
            return Err(UtilitiesErrors::NonPositiveF64);
        }

        Ok(Self { value })
    }

    /// Devuelve el valor almacenado sin modificarlo ni perder bits.
    ///
    /// ```
    /// use utilities::FinitePositiveF64;
    ///
    /// let number = FinitePositiveF64::new(-0.0).unwrap();
    /// assert_eq!(number.get_value().to_bits(), (-0.0_f64).to_bits());
    /// ```
    pub fn get_value(&self) -> f64 {
        self.value.get_value()
    }
}

impl Neg for FinitePositiveF64 {
    type Output = Result<FiniteNegativeF64, UtilitiesErrors>;

    /// Invierte el signo y devuelve un [`FiniteNegativeF64`].
    ///
    /// La firma devuelve `Result`, pero para cualquier instancia válida siempre
    /// devuelve `Ok`: negar un valor finito conserva la finitud.
    /// También invierte el signo del cero.
    ///
    /// ```
    /// use utilities::{FinitePositiveF64, UtilitiesErrors};
    ///
    /// let zero = FinitePositiveF64::new(0.0)?;
    /// assert_eq!((-zero)?.get_value().to_bits(), (-0.0_f64).to_bits());
    /// # Ok::<(), UtilitiesErrors>(())
    /// ```
    fn neg(self) -> Self::Output {
        FiniteNegativeF64::new(-self.get_value())
    }
}

impl Add for FinitePositiveF64 {
    type Output = Result<Self, UtilitiesErrors>;

    /// Suma dos valores no negativos.
    /// Devuelve [`UtilitiesErrors::NonFiniteF64`] si el resultado desborda.
    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.get_value() + rhs.get_value())
    }
}

impl Sub for FinitePositiveF64 {
    type Output = Result<FiniteF64, UtilitiesErrors>;

    /// Resta y devuelve un [`FiniteF64`] que puede tener cualquier signo.
    ///
    /// La diferencia entre dos valores finitos del mismo dominio es finita;
    /// esta operación siempre devuelve `Ok` para instancias válidas.
    fn sub(self, rhs: Self) -> Self::Output {
        FiniteF64::new(self.get_value() - rhs.get_value())
    }
}

impl Mul for FinitePositiveF64 {
    type Output = Result<Self, UtilitiesErrors>;

    /// Multiplica y conserva el dominio no negativo.
    /// Devuelve [`UtilitiesErrors::NonFiniteF64`] si desborda; admite subdesbordamiento.
    fn mul(self, rhs: Self) -> Self::Output {
        Self::new(self.get_value() * rhs.get_value())
    }
}

impl Div for FinitePositiveF64 {
    type Output = Result<Self, UtilitiesErrors>;

    /// Divide y conserva el dominio no negativo.
    ///
    /// Devuelve [`UtilitiesErrors::NonFiniteF64`] si el divisor es `+0.0` o
    /// `-0.0`, o si el cociente desborda. El subdesbordamiento se admite.
    ///
    /// ```
    /// use utilities::{FinitePositiveF64, UtilitiesErrors};
    ///
    /// let one = FinitePositiveF64::new(1.0)?;
    /// for raw_zero in [0.0, -0.0] {
    ///     assert_eq!(one / FinitePositiveF64::new(raw_zero)?, Err(UtilitiesErrors::NonFiniteF64));
    /// }
    /// # Ok::<(), UtilitiesErrors>(())
    /// ```
    fn div(self, rhs: Self) -> Self::Output {
        Self::new(self.get_value() / rhs.get_value())
    }
}
