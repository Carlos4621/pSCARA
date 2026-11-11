use core::ops::{Add, Div, Mul, Neg, Sub};

use crate::{FiniteF64, FinitePositiveF64, UtilitiesErrors};

/// Un `f64` finito menor o igual a cero.
///
/// El dominio es inclusivo: admite `+0.0` y `-0.0` y conserva sus bits.
/// La suma devuelve `Result<Self, UtilitiesErrors>`. La resta devuelve un
/// [`FiniteF64`]; el producto y el cociente devuelven un [`FinitePositiveF64`]
/// dentro de `Result`, porque dos valores no positivos producen un resultado
/// no negativo cuando este es finito. La negación también devuelve un
/// `Result<FinitePositiveF64, UtilitiesErrors>`.
///
/// ```
/// use utilities::{FiniteF64, FiniteNegativeF64, FinitePositiveF64, UtilitiesErrors};
///
/// let a = FiniteNegativeF64::new(-2.0)?;
/// let b = FiniteNegativeF64::new(-4.0)?;
/// assert_eq!((a + b)?.get_value(), -6.0);
/// let difference: FiniteF64 = (a - b)?;
/// assert_eq!(difference.get_value(), 2.0);
/// let product: FinitePositiveF64 = (a * b)?;
/// assert_eq!(product.get_value(), 8.0);
/// assert_eq!((a / b)?.get_value(), 0.5);
/// assert_eq!((-a)?.get_value(), 2.0);
/// # Ok::<(), UtilitiesErrors>(())
/// ```
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct FiniteNegativeF64 {
    value: FiniteF64,
}

impl FiniteNegativeF64 {
    /// Construye un valor finito no positivo, conservando su representación.
    ///
    /// # Errors
    ///
    /// Comprueba primero la finitud: NaN y ambos infinitos producen
    /// [`UtilitiesErrors::NonFiniteF64`]. Un valor finito mayor que cero produce
    /// [`UtilitiesErrors::NonNegativeF64`]. Ambos ceros con signo son válidos.
    ///
    /// ```
    /// use utilities::{FiniteNegativeF64, UtilitiesErrors};
    ///
    /// assert_eq!(FiniteNegativeF64::new(0.0)?.get_value(), 0.0);
    /// assert_eq!(FiniteNegativeF64::new(1.0), Err(UtilitiesErrors::NonNegativeF64));
    /// assert_eq!(FiniteNegativeF64::new(f64::INFINITY), Err(UtilitiesErrors::NonFiniteF64));
    /// # Ok::<(), UtilitiesErrors>(())
    /// ```
    pub fn new(value: f64) -> Result<Self, UtilitiesErrors> {
        let value = FiniteF64::new(value)?;

        if value.get_value() > 0.0 {
            return Err(UtilitiesErrors::NonNegativeF64);
        }

        Ok(Self { value })
    }

    /// Devuelve el valor almacenado sin modificarlo ni perder bits.
    ///
    /// ```
    /// use utilities::FiniteNegativeF64;
    ///
    /// let number = FiniteNegativeF64::new(-0.0).unwrap();
    /// assert_eq!(number.get_value().to_bits(), (-0.0_f64).to_bits());
    /// ```
    pub fn get_value(&self) -> f64 {
        self.value.get_value()
    }
}

impl Neg for FiniteNegativeF64 {
    type Output = Result<FinitePositiveF64, UtilitiesErrors>;

    /// Invierte el signo y devuelve un [`FinitePositiveF64`].
    ///
    /// La firma devuelve `Result`, pero para cualquier instancia válida siempre
    /// devuelve `Ok`: negar un valor finito conserva la finitud.
    /// También invierte el signo del cero.
    ///
    /// ```
    /// use utilities::{FiniteNegativeF64, UtilitiesErrors};
    ///
    /// let zero = FiniteNegativeF64::new(-0.0)?;
    /// assert_eq!((-zero)?.get_value().to_bits(), 0.0_f64.to_bits());
    /// # Ok::<(), UtilitiesErrors>(())
    /// ```
    fn neg(self) -> Self::Output {
        FinitePositiveF64::new(-self.get_value())
    }
}

impl Add for FiniteNegativeF64 {
    type Output = Result<Self, UtilitiesErrors>;

    /// Suma dos valores no positivos.
    /// Devuelve [`UtilitiesErrors::NonFiniteF64`] si el resultado desborda.
    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.get_value() + rhs.get_value())
    }
}

impl Sub for FiniteNegativeF64 {
    type Output = Result<FiniteF64, UtilitiesErrors>;

    /// Resta y devuelve un [`FiniteF64`] que puede tener cualquier signo.
    ///
    /// La diferencia entre dos valores finitos del mismo dominio es finita;
    /// esta operación siempre devuelve `Ok` para instancias válidas.
    fn sub(self, rhs: Self) -> Self::Output {
        FiniteF64::new(self.get_value() - rhs.get_value())
    }
}

impl Mul for FiniteNegativeF64 {
    type Output = Result<FinitePositiveF64, UtilitiesErrors>;

    /// Multiplica y devuelve un [`FinitePositiveF64`].
    /// Devuelve [`UtilitiesErrors::NonFiniteF64`] si desborda; admite subdesbordamiento.
    fn mul(self, rhs: Self) -> Self::Output {
        FinitePositiveF64::new(self.get_value() * rhs.get_value())
    }
}

impl Div for FiniteNegativeF64 {
    type Output = Result<FinitePositiveF64, UtilitiesErrors>;

    /// Divide y devuelve un [`FinitePositiveF64`].
    ///
    /// Devuelve [`UtilitiesErrors::NonFiniteF64`] si el divisor es `+0.0` o
    /// `-0.0`, o si el cociente desborda. El subdesbordamiento se admite.
    ///
    /// ```
    /// use utilities::{FiniteNegativeF64, UtilitiesErrors};
    ///
    /// let one = FiniteNegativeF64::new(-1.0)?;
    /// for raw_zero in [0.0, -0.0] {
    ///     assert_eq!(one / FiniteNegativeF64::new(raw_zero)?, Err(UtilitiesErrors::NonFiniteF64));
    /// }
    /// # Ok::<(), UtilitiesErrors>(())
    /// ```
    fn div(self, rhs: Self) -> Self::Output {
        FinitePositiveF64::new(self.get_value() / rhs.get_value())
    }
}
