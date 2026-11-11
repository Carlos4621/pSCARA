use core::ops::{Add, Div, Mul, Neg, Sub};

use crate::{FinitePositiveF64, UtilitiesErrors, finite_negative_f64::FiniteNegativeF64};

/// Un `f64` que siempre contiene un valor finito, de cualquier signo.
///
/// Conserva los bits de la entrada, incluidos los de `-0.0`. Implementa
/// `Copy`, y su igualdad y orden son los numéricos de `f64`.
/// Los operadores binarios devuelven `Result<Self, UtilitiesErrors>` y rechazan
/// resultados no finitos; la negación devuelve directamente `Self`.
///
/// ```
/// use utilities::{FiniteF64, UtilitiesErrors};
///
/// let a = FiniteF64::new(6.0)?;
/// let b = FiniteF64::new(2.0)?;
/// assert_eq!((a + b)?.get_value(), 8.0);
/// assert_eq!((a - b)?.get_value(), 4.0);
/// assert_eq!((a * b)?.get_value(), 12.0);
/// assert_eq!((a / b)?.get_value(), 3.0);
/// assert_eq!((-a).get_value(), -6.0);
/// # Ok::<(), UtilitiesErrors>(())
/// ```
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct FiniteF64 {
    value: f64,
}

impl FiniteF64 {
    /// Construye un número finito sin alterar su representación.
    ///
    /// # Errors
    ///
    /// Devuelve [`UtilitiesErrors::NonFiniteF64`] para NaN o cualquier infinito.
    ///
    /// ```
    /// use utilities::{FiniteF64, UtilitiesErrors};
    ///
    /// assert_eq!(FiniteF64::new(-2.5)?.get_value(), -2.5);
    /// for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
    ///     assert_eq!(FiniteF64::new(value), Err(UtilitiesErrors::NonFiniteF64));
    /// }
    /// # Ok::<(), UtilitiesErrors>(())
    /// ```
    pub fn new(value: f64) -> Result<Self, UtilitiesErrors> {
        if !value.is_finite() {
            return Err(UtilitiesErrors::NonFiniteF64);
        }

        Ok(Self { value })
    }

    /// Devuelve el `f64` almacenado sin modificarlo, conservando sus bits.
    ///
    /// ```
    /// use utilities::FiniteF64;
    ///
    /// let number = FiniteF64::new(-0.0).unwrap();
    /// assert_eq!(number.get_value().to_bits(), (-0.0_f64).to_bits());
    /// ```
    pub fn get_value(&self) -> f64 {
        self.value
    }

    /// Compara la diferencia absoluta con una tolerancia inclusiva.
    ///
    /// Evalúa `abs(self - other) <= tolerance` con el redondeo de `f64`.
    /// La tolerancia es absoluta, no relativa. Con tolerancia cero comprueba
    /// igualdad numérica, por lo que `+0.0` y `-0.0` coinciden.
    /// Si la resta desborda, devuelve `false`: ninguna tolerancia finita puede
    /// cubrir ese resultado. La comparación no devuelve errores.
    ///
    /// ```
    /// use utilities::{FiniteF64, FinitePositiveF64, UtilitiesErrors};
    ///
    /// let a = FiniteF64::new(1.0)?;
    /// let b = FiniteF64::new(1.25)?;
    /// assert!(a.is_aproximated_equal_to(b, FinitePositiveF64::new(0.25)?));
    /// assert!(!a.is_aproximated_equal_to(b, FinitePositiveF64::new(0.125)?));
    /// let maximum = FiniteF64::new(f64::MAX)?;
    /// assert!(!maximum.is_aproximated_equal_to(-maximum, FinitePositiveF64::new(f64::MAX)?));
    /// # Ok::<(), UtilitiesErrors>(())
    /// ```
    pub fn is_aproximated_equal_to(&self, other: FiniteF64, tolerance: FinitePositiveF64) -> bool {
        match *self - other {
            Ok(difference) => difference.value.abs() <= tolerance.get_value(),
            Err(_) =>
            // Un desbordamiento indica valores totalmente distintos
            {
                false
            }
        }
    }

    /// Restringe el valor al dominio mayor o igual a cero, conservando sus bits.
    ///
    /// # Errors
    ///
    /// Devuelve [`UtilitiesErrors::NonPositiveF64`] si el valor es menor que cero.
    /// Ambos ceros con signo se admiten.
    ///
    /// ```
    /// use utilities::{FiniteF64, UtilitiesErrors};
    ///
    /// let zero = FiniteF64::new(-0.0)?.to_finite_positive()?;
    /// assert_eq!(zero.get_value().to_bits(), (-0.0_f64).to_bits());
    /// assert_eq!(
    ///     FiniteF64::new(-1.0)?.to_finite_positive(),
    ///     Err(UtilitiesErrors::NonPositiveF64),
    /// );
    /// # Ok::<(), UtilitiesErrors>(())
    /// ```
    pub fn to_finite_positive(self) -> Result<FinitePositiveF64, UtilitiesErrors> {
        FinitePositiveF64::new(self.value)
    }

    /// Restringe el valor al dominio menor o igual a cero, conservando sus bits.
    ///
    /// # Errors
    ///
    /// Devuelve [`UtilitiesErrors::NonNegativeF64`] si el valor es mayor que cero.
    /// Ambos ceros con signo se admiten.
    ///
    /// ```
    /// use utilities::{FiniteF64, UtilitiesErrors};
    ///
    /// assert_eq!(FiniteF64::new(-3.0)?.to_finite_negative()?.get_value(), -3.0);
    /// assert_eq!(
    ///     FiniteF64::new(1.0)?.to_finite_negative(),
    ///     Err(UtilitiesErrors::NonNegativeF64),
    /// );
    /// # Ok::<(), UtilitiesErrors>(())
    /// ```
    pub fn to_finite_negative(self) -> Result<FiniteNegativeF64, UtilitiesErrors> {
        FiniteNegativeF64::new(self.value)
    }
}

impl Neg for FiniteF64 {
    type Output = Self;

    /// Invierte el signo sin poder desbordar; también intercambia `+0.0` y `-0.0`.
    fn neg(self) -> Self::Output {
        Self { value: -self.value }
    }
}

impl Mul for FiniteF64 {
    type Output = Result<Self, UtilitiesErrors>;

    /// Multiplica y rechaza resultados no finitos con [`UtilitiesErrors::NonFiniteF64`].
    /// El subdesbordamiento, incluido un cero con signo, se admite.
    fn mul(self, rhs: Self) -> Self::Output {
        Self::new(self.value * rhs.value)
    }
}

impl Add for FiniteF64 {
    type Output = Result<Self, UtilitiesErrors>;

    /// Suma y devuelve [`UtilitiesErrors::NonFiniteF64`] si el resultado desborda.
    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.value + rhs.value)
    }
}

impl Sub for FiniteF64 {
    type Output = Result<Self, UtilitiesErrors>;

    /// Resta y devuelve [`UtilitiesErrors::NonFiniteF64`] si el resultado desborda.
    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.value - rhs.value)
    }
}

impl Div for FiniteF64 {
    type Output = Result<Self, UtilitiesErrors>;

    /// Divide y rechaza resultados no finitos con [`UtilitiesErrors::NonFiniteF64`].
    ///
    /// Ambos ceros divisores producen un error. Un resultado finito, incluso
    /// un subnormal o cero por subdesbordamiento, se admite.
    ///
    /// ```
    /// use utilities::{FiniteF64, UtilitiesErrors};
    ///
    /// let one = FiniteF64::new(1.0)?;
    /// assert_eq!(one / FiniteF64::new(-0.0)?, Err(UtilitiesErrors::NonFiniteF64));
    /// let smallest = FiniteF64::new(f64::from_bits(1))?;
    /// assert_eq!((smallest / FiniteF64::new(2.0)?)?.get_value(), 0.0);
    /// # Ok::<(), UtilitiesErrors>(())
    /// ```
    fn div(self, rhs: Self) -> Self::Output {
        Self::new(self.value / rhs.value)
    }
}

impl From<FinitePositiveF64> for FiniteF64 {
    /// Amplía el dominio sin validar de nuevo ni alterar los bits del valor.
    ///
    /// ```
    /// use utilities::{FiniteF64, FinitePositiveF64};
    ///
    /// let source = FinitePositiveF64::new(-0.0).unwrap();
    /// let target = FiniteF64::from(source);
    /// let via_into: FiniteF64 = source.into();
    /// assert_eq!(target.get_value().to_bits(), (-0.0_f64).to_bits());
    /// assert_eq!(target, via_into);
    /// ```
    fn from(value: FinitePositiveF64) -> Self {
        Self {
            value: value.get_value(),
        }
    }
}

impl From<FiniteNegativeF64> for FiniteF64 {
    /// Amplía el dominio sin validar de nuevo ni alterar los bits del valor.
    ///
    /// ```
    /// use utilities::{FiniteF64, FiniteNegativeF64};
    ///
    /// let source = FiniteNegativeF64::new(-2.5).unwrap();
    /// let target: FiniteF64 = source.into();
    /// assert_eq!(target.get_value(), -2.5);
    /// ```
    fn from(value: FiniteNegativeF64) -> Self {
        Self {
            value: value.get_value(),
        }
    }
}

impl From<FiniteF64> for f64 {
    /// Extrae el valor original sin pérdida de precisión ni de bits.
    ///
    /// ```
    /// use utilities::FiniteF64;
    ///
    /// let source = FiniteF64::new(-0.0).unwrap();
    /// let raw = f64::from(source);
    /// assert_eq!(raw.to_bits(), (-0.0_f64).to_bits());
    /// ```
    fn from(value: FiniteF64) -> Self {
        value.get_value()
    }
}

impl From<FinitePositiveF64> for f64 {
    /// Extrae el valor no negativo, conservando también el signo del cero.
    ///
    /// ```
    /// use utilities::FinitePositiveF64;
    ///
    /// let raw: f64 = FinitePositiveF64::new(2.5).unwrap().into();
    /// assert_eq!(raw, 2.5);
    /// ```
    fn from(value: FinitePositiveF64) -> Self {
        value.get_value()
    }
}

impl From<FiniteNegativeF64> for f64 {
    /// Extrae el valor no positivo, conservando también el signo del cero.
    ///
    /// ```
    /// use utilities::FiniteNegativeF64;
    ///
    /// let raw: f64 = FiniteNegativeF64::new(-2.5).unwrap().into();
    /// assert_eq!(raw, -2.5);
    /// ```
    fn from(value: FiniteNegativeF64) -> Self {
        value.get_value()
    }
}

impl TryFrom<f64> for FiniteF64 {
    type Error = UtilitiesErrors;

    /// Valida mediante [`FiniteF64::new`]; permite usar también [`TryInto`].
    ///
    /// # Errors
    ///
    /// Devuelve [`UtilitiesErrors::NonFiniteF64`] para NaN o infinitos.
    ///
    /// ```
    /// use utilities::{FiniteF64, UtilitiesErrors};
    ///
    /// let number: FiniteF64 = 3.5_f64.try_into()?;
    /// assert_eq!(number.get_value(), 3.5);
    /// assert_eq!(FiniteF64::try_from(f64::NAN), Err(UtilitiesErrors::NonFiniteF64));
    /// # Ok::<(), UtilitiesErrors>(())
    /// ```
    fn try_from(value: f64) -> Result<Self, Self::Error> {
        Self::new(value)
    }
}

impl TryFrom<f64> for FinitePositiveF64 {
    type Error = UtilitiesErrors;

    /// Valida mediante [`FinitePositiveF64::new`]; permite usar también [`TryInto`].
    ///
    /// # Errors
    ///
    /// Devuelve [`UtilitiesErrors::NonFiniteF64`] para NaN o infinitos, y
    /// [`UtilitiesErrors::NonPositiveF64`] para valores finitos menores que cero.
    ///
    /// ```
    /// use utilities::{FinitePositiveF64, UtilitiesErrors};
    ///
    /// let number: FinitePositiveF64 = 0.0_f64.try_into()?;
    /// assert_eq!(number.get_value(), 0.0);
    /// assert_eq!(FinitePositiveF64::try_from(-1.0), Err(UtilitiesErrors::NonPositiveF64));
    /// # Ok::<(), UtilitiesErrors>(())
    /// ```
    fn try_from(value: f64) -> Result<Self, Self::Error> {
        FinitePositiveF64::new(value)
    }
}

impl TryFrom<f64> for FiniteNegativeF64 {
    type Error = UtilitiesErrors;

    /// Valida mediante [`FiniteNegativeF64::new`]; permite usar también [`TryInto`].
    ///
    /// # Errors
    ///
    /// Devuelve [`UtilitiesErrors::NonFiniteF64`] para NaN o infinitos, y
    /// [`UtilitiesErrors::NonNegativeF64`] para valores finitos mayores que cero.
    ///
    /// ```
    /// use utilities::{FiniteNegativeF64, UtilitiesErrors};
    ///
    /// let number: FiniteNegativeF64 = (-3.5_f64).try_into()?;
    /// assert_eq!(number.get_value(), -3.5);
    /// assert_eq!(FiniteNegativeF64::try_from(1.0), Err(UtilitiesErrors::NonNegativeF64));
    /// # Ok::<(), UtilitiesErrors>(())
    /// ```
    fn try_from(value: f64) -> Result<Self, Self::Error> {
        FiniteNegativeF64::new(value)
    }
}
