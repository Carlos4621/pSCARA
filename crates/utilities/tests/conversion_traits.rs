use utilities::{FiniteF64, FiniteNegativeF64, FinitePositiveF64, UtilitiesErrors};

// Sample every finite exponent, both signs, and mantissa boundaries, including
// signed zeros, the smallest subnormals, MIN_POSITIVE and MAX.
fn finite_samples() -> impl Iterator<Item = f64> {
    (0_u64..0x7ff).flat_map(|exponent| {
        [0, 1, 1_u64 << 51, (1_u64 << 52) - 1]
            .into_iter()
            .flat_map(move |mantissa| {
                [0, 1_u64 << 63].map(|sign| f64::from_bits(sign | (exponent << 52) | mantissa))
            })
    })
}

fn nan_samples() -> impl Iterator<Item = f64> {
    [0, 1_u64 << 63].into_iter().flat_map(|sign| {
        [1, 2, (1_u64 << 51) - 1, 1_u64 << 51, (1_u64 << 52) - 1]
            .map(move |payload| f64::from_bits(sign | (0x7ff_u64 << 52) | payload))
    })
}

macro_rules! conversion_suite {
    ($module:ident, $number:ident, $allowed:expr) => {
        mod $module {
            use super::*;

            #[test]
            fn try_from_and_try_into_preserve_allowed_finite_bits() {
                for raw in finite_samples().filter($allowed) {
                    // Both spellings must expose the same concrete error type.
                    let explicit: Result<$number, UtilitiesErrors> = $number::try_from(raw);
                    let inferred: Result<$number, UtilitiesErrors> = raw.try_into();
                    for result in [explicit, inferred] {
                        assert_eq!(result.unwrap().get_value().to_bits(), raw.to_bits());
                    }
                }
            }

            #[test]
            fn from_and_into_f64_preserve_bits_and_allow_a_round_trip() {
                for raw in finite_samples().filter($allowed) {
                    let original = $number::new(raw).unwrap();
                    let explicit = f64::from(original);
                    let inferred: f64 = original.into();
                    assert_eq!(explicit.to_bits(), raw.to_bits());
                    assert_eq!(inferred.to_bits(), raw.to_bits());
                    let restored = $number::try_from(inferred).unwrap();
                    assert_eq!(restored.get_value().to_bits(), raw.to_bits());
                    assert_eq!(original.get_value().to_bits(), raw.to_bits());
                }
            }
        }
    };
}

conversion_suite!(finite, FiniteF64, |_: &f64| true);
conversion_suite!(positive, FinitePositiveF64, |raw: &f64| *raw >= 0.0);
conversion_suite!(negative, FiniteNegativeF64, |raw: &f64| *raw <= 0.0);

#[test]
fn positive_fallible_conversions_reject_every_sampled_negative_nonzero_value() {
    for raw in finite_samples().filter(|raw| *raw < 0.0) {
        let inferred: Result<FinitePositiveF64, UtilitiesErrors> = raw.try_into();
        assert_eq!(inferred, Err(UtilitiesErrors::NonPositiveF64));
        assert_eq!(FinitePositiveF64::try_from(raw), inferred);
    }
}

#[test]
fn negative_fallible_conversions_reject_every_sampled_positive_nonzero_value() {
    for raw in finite_samples().filter(|raw| *raw > 0.0) {
        let inferred: Result<FiniteNegativeF64, UtilitiesErrors> = raw.try_into();
        assert_eq!(inferred, Err(UtilitiesErrors::NonNegativeF64));
        assert_eq!(FiniteNegativeF64::try_from(raw), inferred);
    }
}

#[test]
fn finite_fallible_conversions_reject_infinities_and_nan_payloads() {
    for raw in nan_samples().chain([f64::NEG_INFINITY, f64::INFINITY]) {
        let inferred: Result<FiniteF64, UtilitiesErrors> = raw.try_into();
        assert_eq!(inferred, Err(UtilitiesErrors::NonFiniteF64));
        assert_eq!(FiniteF64::try_from(raw), inferred);
    }
}

#[test]
fn positive_fallible_conversions_reject_nonfinite_inputs_before_checking_sign() {
    for raw in nan_samples().chain([f64::NEG_INFINITY, f64::INFINITY]) {
        let inferred: Result<FinitePositiveF64, UtilitiesErrors> = raw.try_into();
        assert_eq!(inferred, Err(UtilitiesErrors::NonFiniteF64));
        assert_eq!(FinitePositiveF64::try_from(raw), inferred);
    }
}

#[test]
fn negative_fallible_conversions_reject_nonfinite_inputs_before_checking_sign() {
    for raw in nan_samples().chain([f64::NEG_INFINITY, f64::INFINITY]) {
        let inferred: Result<FiniteNegativeF64, UtilitiesErrors> = raw.try_into();
        assert_eq!(inferred, Err(UtilitiesErrors::NonFiniteF64));
        assert_eq!(FiniteNegativeF64::try_from(raw), inferred);
    }
}

#[test]
fn positive_from_and_into_finite_preserve_all_sampled_allowed_bits() {
    for raw in finite_samples().filter(|raw| *raw >= 0.0) {
        let original = FinitePositiveF64::new(raw).unwrap();
        let explicit = FiniteF64::from(original);
        let inferred: FiniteF64 = original.into();
        assert_eq!(explicit.get_value().to_bits(), raw.to_bits());
        assert_eq!(inferred.get_value().to_bits(), raw.to_bits());
        let restored = inferred.to_finite_positive().unwrap();
        assert_eq!(restored.get_value().to_bits(), raw.to_bits());
        assert_eq!(original.get_value().to_bits(), raw.to_bits());
    }
}

#[test]
fn negative_from_and_into_finite_preserve_all_sampled_allowed_bits() {
    for raw in finite_samples().filter(|raw| *raw <= 0.0) {
        let original = FiniteNegativeF64::new(raw).unwrap();
        let explicit = FiniteF64::from(original);
        let inferred: FiniteF64 = original.into();
        assert_eq!(explicit.get_value().to_bits(), raw.to_bits());
        assert_eq!(inferred.get_value().to_bits(), raw.to_bits());
        let restored = inferred.to_finite_negative().unwrap();
        assert_eq!(restored.get_value().to_bits(), raw.to_bits());
        assert_eq!(original.get_value().to_bits(), raw.to_bits());
    }
}

fn positive_sum(a: f64, b: f64) -> Result<FiniteF64, UtilitiesErrors> {
    let left: FinitePositiveF64 = a.try_into()?;
    let right = FinitePositiveF64::try_from(b)?;
    let sum = (left + right)?;
    Ok(sum.into())
}

#[test]
fn conversions_compose_with_question_mark_arithmetic_and_widening() {
    assert_eq!(positive_sum(1.5, 2.25).unwrap().get_value(), 3.75);
    assert_eq!(
        positive_sum(-0.0, -0.0).unwrap().get_value().to_bits(),
        (-0.0_f64).to_bits()
    );
    assert_eq!(
        positive_sum(-f64::from_bits(1), 1.0),
        Err(UtilitiesErrors::NonPositiveF64)
    );
    assert_eq!(
        positive_sum(1.0, -1.0),
        Err(UtilitiesErrors::NonPositiveF64)
    );
    for raw in nan_samples().chain([f64::NEG_INFINITY, f64::INFINITY]) {
        assert_eq!(positive_sum(raw, 1.0), Err(UtilitiesErrors::NonFiniteF64));
        assert_eq!(positive_sum(1.0, raw), Err(UtilitiesErrors::NonFiniteF64));
    }
    assert_eq!(
        positive_sum(f64::MAX, f64::MAX),
        Err(UtilitiesErrors::NonFiniteF64)
    );
    // A failure on the first conversion is returned before inspecting b.
    assert_eq!(
        positive_sum(-1.0, f64::NAN),
        Err(UtilitiesErrors::NonPositiveF64)
    );
}

#[test]
fn conversion_traits_work_through_generic_bounds_and_iterator_collection() {
    fn convert<T: TryFrom<f64, Error = UtilitiesErrors>>(raw: f64) -> Result<T, UtilitiesErrors> {
        raw.try_into()
    }

    let values: Result<Vec<FiniteNegativeF64>, UtilitiesErrors> =
        [-0.0, -1.5, -f64::MAX].into_iter().map(convert).collect();
    let values = values.unwrap();
    let raw: Vec<f64> = values.into_iter().map(f64::from).collect();
    for (actual, expected) in raw.into_iter().zip([-0.0_f64, -1.5, -f64::MAX]) {
        assert_eq!(actual.to_bits(), expected.to_bits());
    }

    let invalid_sign: Result<Vec<FiniteNegativeF64>, UtilitiesErrors> =
        [-1.0, 1.0, -2.0].into_iter().map(convert).collect();
    assert_eq!(invalid_sign, Err(UtilitiesErrors::NonNegativeF64));
    let nonfinite: Result<Vec<FiniteF64>, UtilitiesErrors> =
        [0.0, f64::NAN, 1.0].into_iter().map(convert).collect();
    assert_eq!(nonfinite, Err(UtilitiesErrors::NonFiniteF64));
}
