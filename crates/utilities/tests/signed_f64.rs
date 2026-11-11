use core::fmt::Debug;

use utilities::{FiniteF64, FiniteNegativeF64, FinitePositiveF64, UtilitiesErrors};

const SMALLEST: f64 = f64::from_bits(1);
const LARGEST_SUBNORMAL: f64 = f64::from_bits(f64::MIN_POSITIVE.to_bits() - 1);

fn magnitudes() -> [f64; 18] {
    [
        0.0,
        SMALLEST,
        2.0 * SMALLEST,
        3.0 * SMALLEST,
        LARGEST_SUBNORMAL,
        f64::MIN_POSITIVE,
        1e-300,
        1e-200,
        0.25,
        0.5,
        1.0,
        f64::from_bits(1.0_f64.to_bits() + 1),
        2.0,
        4.0,
        1e200,
        1e300,
        f64::from_bits(f64::MAX.to_bits() - 1),
        f64::MAX,
    ]
}

fn exponent_samples() -> impl Iterator<Item = f64> {
    (0_u64..0x7ff).flat_map(|exponent| {
        [0, 1, 1_u64 << 51, (1_u64 << 52) - 1]
            .map(|mantissa| f64::from_bits((exponent << 52) | mantissa))
    })
}

// Both constructors reject nonfinite inputs before checking the sign.
fn expected_signed(raw: f64, sign: f64) -> Result<f64, UtilitiesErrors> {
    if !raw.is_finite() {
        Err(UtilitiesErrors::NonFiniteF64)
    } else if sign > 0.0 && raw < 0.0 {
        Err(UtilitiesErrors::NonPositiveF64)
    } else if sign < 0.0 && raw > 0.0 {
        Err(UtilitiesErrors::NonNegativeF64)
    } else {
        expected_finite(raw)
    }
}

fn expected_finite(raw: f64) -> Result<f64, UtilitiesErrors> {
    if raw.is_finite() {
        Ok(raw)
    } else {
        Err(UtilitiesErrors::NonFiniteF64)
    }
}

fn assert_result<T: Debug>(
    actual: Result<T, UtilitiesErrors>,
    expected: Result<f64, UtilitiesErrors>,
    get: impl FnOnce(T) -> f64,
) {
    match (actual, expected) {
        (Ok(number), Ok(raw)) => {
            assert_eq!(get(number).to_bits(), raw.to_bits(), "expected {raw:?}")
        }
        (Err(actual), Err(expected)) => assert_eq!(actual, expected),
        (actual, expected) => panic!("unexpected result {actual:?}; expected {expected:?}"),
    }
}

macro_rules! signed_suite {
    ($module:ident, $number:ident, $opposite:ident, $sign:expr, $wrong_sign:ident) => {
        mod $module {
            use super::*;

            const SIGN: f64 = $sign;

            fn number(raw: f64) -> $number {
                $number::new(raw).expect("fixture must be finite and have the allowed sign")
            }

            fn check_pair(left: f64, right: f64) {
                let a = number(left);
                let b = number(right);
                // Explicit annotations also check public operator output types.
                let sum: Result<$number, UtilitiesErrors> = a + b;
                let difference: Result<FiniteF64, UtilitiesErrors> = a - b;
                assert_result(sum, expected_signed(left + right, SIGN), |n| n.get_value());
                assert_result(difference, expected_finite(left - right), |n| n.get_value());
                assert_result(a * b, expected_signed(left * right, 1.0), |n| n.get_value());
                assert_result(a / b, expected_signed(left / right, 1.0), |n| n.get_value());
                assert_eq!(a.get_value().to_bits(), left.to_bits());
                assert_eq!(b.get_value().to_bits(), right.to_bits());
            }

            #[test]
            fn constructor_preserves_boundary_values_and_both_zero_signs() {
                for raw in magnitudes()
                    .map(|m| SIGN * m)
                    .into_iter()
                    .chain([-0.0, 0.0])
                {
                    assert_eq!(number(raw).get_value().to_bits(), raw.to_bits());
                }
            }

            #[test]
            fn constructor_accepts_every_sampled_finite_exponent_and_mantissa_class() {
                for magnitude in exponent_samples() {
                    let raw = SIGN * magnitude;
                    assert_eq!(number(raw).get_value().to_bits(), raw.to_bits());
                }
            }

            #[test]
            fn constructor_rejects_every_sampled_nonzero_value_of_the_wrong_sign() {
                for magnitude in exponent_samples().filter(|raw| *raw > 0.0) {
                    assert_eq!(
                        $number::new(-SIGN * magnitude),
                        Err(UtilitiesErrors::$wrong_sign)
                    );
                }
            }

            #[test]
            fn constructor_rejects_infinities_and_nan_payloads_with_current_error_precedence() {
                assert_eq!(
                    $number::new(SIGN * f64::INFINITY),
                    Err(UtilitiesErrors::NonFiniteF64)
                );
                assert_eq!(
                    $number::new(-SIGN * f64::INFINITY),
                    Err(UtilitiesErrors::NonFiniteF64)
                );
                for sign in [0, 1_u64 << 63] {
                    for payload in [1, 2, (1_u64 << 51) - 1, 1_u64 << 51, (1_u64 << 52) - 1] {
                        let nan = f64::from_bits(sign | (0x7ff_u64 << 52) | payload);
                        assert_eq!($number::new(nan), Err(UtilitiesErrors::NonFiniteF64));
                    }
                }
            }

            #[test]
            fn copy_clone_and_debug_work_for_public_consumers() {
                fn cloned<T: Clone>(item: &T) -> T {
                    item.clone()
                }
                fn copied<T: Copy>(item: T) -> (T, T) {
                    (item, item)
                }
                for magnitude in magnitudes() {
                    let raw = SIGN * magnitude;
                    let original = number(raw);
                    let (first, second) = copied(original);
                    for item in [original, first, second, cloned(&original)] {
                        assert_eq!(item.get_value().to_bits(), raw.to_bits());
                    }
                    assert!(format!("{original:?}").contains(stringify!($number)));
                }
            }

            #[test]
            fn equality_and_ordering_match_numeric_order_including_signed_zero() {
                let values: Vec<_> = magnitudes()
                    .map(|m| SIGN * m)
                    .into_iter()
                    .chain([-0.0, 0.0])
                    .collect();
                for &left in &values {
                    for &right in &values {
                        let a = number(left);
                        let b = number(right);
                        assert_eq!(a == b, left == right);
                        assert_eq!(a != b, left != right);
                        assert_eq!(a.partial_cmp(&b), left.partial_cmp(&right));
                        assert_eq!(a < b, left < right);
                        assert_eq!(a <= b, left <= right);
                        assert_eq!(a > b, left > right);
                        assert_eq!(a >= b, left >= right);
                    }
                }
                assert_eq!(number(-0.0), number(0.0));
            }

            #[test]
            fn negation_returns_the_opposite_public_type_and_double_negation_preserves_bits() {
                for raw in exponent_samples().map(|m| SIGN * m).chain([-0.0, 0.0]) {
                    let original = number(raw);
                    let opposite: $opposite = (-original).unwrap();
                    assert_eq!(
                        opposite.get_value().to_bits(),
                        raw.to_bits() ^ (1_u64 << 63)
                    );
                    let recovered: $number = (-opposite).unwrap();
                    assert_eq!(recovered.get_value().to_bits(), raw.to_bits());
                    assert_eq!(original.get_value().to_bits(), raw.to_bits());
                }
            }

            #[test]
            fn addition_has_exact_results_in_the_same_sign_domain() {
                let sum: $number = (number(SIGN * 1.25) + number(SIGN * 2.5)).unwrap();
                assert_eq!(sum.get_value(), SIGN * 3.75);
            }

            #[test]
            fn subtraction_can_produce_either_sign_in_the_general_finite_type() {
                let small = number(SIGN * 1.0);
                let large = number(SIGN * 4.0);
                let forward: FiniteF64 = (small - large).unwrap();
                let reverse: FiniteF64 = (large - small).unwrap();
                assert_eq!(forward.get_value(), -SIGN * 3.0);
                assert_eq!(reverse.get_value(), SIGN * 3.0);
                assert_eq!(
                    (small - small).unwrap().get_value().to_bits(),
                    0.0_f64.to_bits()
                );
            }

            #[test]
            fn multiplication_returns_a_positive_public_type_for_either_input_domain() {
                let product: FinitePositiveF64 = (number(SIGN * 1.5) * number(SIGN * 2.0)).unwrap();
                assert_eq!(product.get_value(), 3.0);
            }

            #[test]
            fn division_returns_a_positive_public_type_and_respects_operand_order() {
                let large = number(SIGN * 8.0);
                let small = number(SIGN * 2.0);
                let forward: FinitePositiveF64 = (large / small).unwrap();
                let reverse: FinitePositiveF64 = (small / large).unwrap();
                assert_eq!(forward.get_value(), 4.0);
                assert_eq!(reverse.get_value(), 0.25);
            }

            #[test]
            fn every_boundary_pair_matches_ieee_results_and_return_type_invariants() {
                let values: Vec<_> = magnitudes()
                    .map(|m| SIGN * m)
                    .into_iter()
                    .chain([-0.0, 0.0])
                    .collect();
                for &left in &values {
                    for &right in &values {
                        check_pair(left, right);
                    }
                }
            }

            #[test]
            fn every_sampled_exponent_composes_with_zero_units_and_extremes() {
                for raw in exponent_samples() {
                    for operand in [0.0, SMALLEST, 0.5, 1.0, 2.0, f64::MAX] {
                        check_pair(SIGN * raw, SIGN * operand);
                    }
                }
            }

            #[test]
            fn reproducible_mixed_bit_patterns_match_ieee_arithmetic() {
                let mut state = 0xbb67_ae85_84ca_a73b_u64;
                let mut next = || {
                    state ^= state << 13;
                    state ^= state >> 7;
                    state ^= state << 17;
                    f64::from_bits(state & !(1_u64 << 63))
                };
                let mut checked = 0;
                for _ in 0..10_000 {
                    let (left, right) = (next(), next());
                    if left.is_finite() && right.is_finite() {
                        check_pair(SIGN * left, SIGN * right);
                        checked += 1;
                    }
                }
                assert!(checked > 9_900);
            }

            #[test]
            fn division_by_either_signed_zero_rejects_nonfinite_results() {
                assert_eq!(
                    number(SIGN) / number(SIGN * 0.0),
                    Err(UtilitiesErrors::NonFiniteF64)
                );
                assert_eq!(
                    number(SIGN) / number(-SIGN * 0.0),
                    Err(UtilitiesErrors::NonFiniteF64)
                );
                for numerator in [-0.0, 0.0] {
                    for denominator in [-0.0, 0.0] {
                        assert_eq!(
                            number(numerator) / number(denominator),
                            Err(UtilitiesErrors::NonFiniteF64)
                        );
                    }
                }
            }

            #[test]
            fn overflow_is_rejected_and_nearby_finite_operations_remain_valid() {
                let maximum = number(SIGN * f64::MAX);
                assert_eq!(maximum + maximum, Err(UtilitiesErrors::NonFiniteF64));
                assert_eq!(
                    maximum * number(SIGN * 2.0),
                    Err(UtilitiesErrors::NonFiniteF64)
                );
                assert_eq!(
                    maximum / number(SIGN * 0.5),
                    Err(UtilitiesErrors::NonFiniteF64)
                );
                assert_eq!(
                    (maximum + number(SIGN * 0.0)).unwrap().get_value(),
                    SIGN * f64::MAX
                );
                assert_eq!((maximum / maximum).unwrap().get_value(), 1.0);
                assert_eq!(
                    (maximum - number(SIGN * 0.0)).unwrap().get_value(),
                    SIGN * f64::MAX
                );
            }

            #[test]
            fn subnormal_arithmetic_preserves_values_across_the_normal_boundary() {
                let smallest = number(SIGN * SMALLEST);
                let largest_subnormal = number(SIGN * LARGEST_SUBNORMAL);
                assert_eq!(
                    (largest_subnormal + smallest).unwrap().get_value(),
                    SIGN * f64::MIN_POSITIVE
                );
                assert_eq!(
                    (number(SIGN * f64::MIN_POSITIVE) - smallest)
                        .unwrap()
                        .get_value(),
                    SIGN * LARGEST_SUBNORMAL
                );
                assert_eq!(
                    (smallest + smallest).unwrap().get_value(),
                    SIGN * 2.0 * SMALLEST
                );
            }

            #[test]
            fn multiplication_and_division_accept_underflow_and_ties_to_even() {
                for (units, expected_units) in [(1.0, 0.0), (2.0, 1.0), (3.0, 2.0), (5.0, 2.0)] {
                    let a = number(SIGN * units * SMALLEST);
                    let product: FinitePositiveF64 = (a * number(SIGN * 0.5)).unwrap();
                    let quotient: FinitePositiveF64 = (a / number(SIGN * 2.0)).unwrap();
                    let expected = expected_units * SMALLEST;
                    assert_eq!(product.get_value().to_bits(), expected.to_bits());
                    assert_eq!(quotient.get_value().to_bits(), expected.to_bits());
                }
            }

            #[test]
            fn all_signed_zero_pairs_obey_ieee_arithmetic() {
                for left in [-0.0, 0.0] {
                    for right in [-0.0, 0.0] {
                        check_pair(left, right);
                    }
                    for right in [SIGN * SMALLEST, SIGN, SIGN * f64::MAX] {
                        check_pair(left, right);
                        check_pair(right, left);
                    }
                }
            }

            #[test]
            fn nonzero_self_division_and_exact_add_sub_composition_hold() {
                for magnitude in magnitudes().into_iter().filter(|m| *m > 0.0) {
                    let a = number(SIGN * magnitude);
                    assert_eq!((a / a).unwrap().get_value(), 1.0);
                }
                for a in 0..=16 {
                    for b in 0..=16 {
                        let a = number(SIGN * f64::from(a));
                        let b = number(SIGN * f64::from(b));
                        let restored: FiniteF64 = ((a + b).unwrap() - b).unwrap();
                        assert_eq!(restored.get_value(), a.get_value());
                    }
                }
            }

            fn composed(a: $number, b: $number) -> Result<FinitePositiveF64, UtilitiesErrors> {
                let sum = (a + b)?;
                let product: FinitePositiveF64 = (sum * b)?;
                product / FinitePositiveF64::new(2.0)?
            }

            #[test]
            fn question_mark_composition_preserves_success_and_errors() {
                assert_eq!(
                    composed(number(SIGN * 2.0), number(SIGN * 4.0))
                        .unwrap()
                        .get_value(),
                    12.0
                );
                assert_eq!(
                    composed(number(SIGN * f64::MAX), number(SIGN * f64::MAX)),
                    Err(UtilitiesErrors::NonFiniteF64)
                );
                // The addition is finite, but the subsequent product overflows.
                assert_eq!(
                    composed(number(SIGN * f64::MAX / 4.0), number(SIGN * 8.0)),
                    Err(UtilitiesErrors::NonFiniteF64)
                );
            }

            #[test]
            fn failed_operations_do_not_consume_or_mutate_copied_operands() {
                let maximum = number(SIGN * f64::MAX);
                let zero = number(-0.0);
                assert_eq!(maximum + maximum, Err(UtilitiesErrors::NonFiniteF64));
                assert!((maximum / zero).is_err());
                assert_eq!((maximum - maximum).unwrap().get_value(), 0.0);
                assert_eq!(maximum.get_value().to_bits(), (SIGN * f64::MAX).to_bits());
                assert_eq!(zero.get_value().to_bits(), (-0.0_f64).to_bits());
            }
        }
    };
}

// The constructors currently use inclusive domains: >= 0 and <= 0.
signed_suite!(
    positive,
    FinitePositiveF64,
    FiniteNegativeF64,
    1.0,
    NonPositiveF64
);
signed_suite!(
    negative,
    FiniteNegativeF64,
    FinitePositiveF64,
    -1.0,
    NonNegativeF64
);

mod conversions {
    use super::*;

    #[test]
    fn finite_to_signed_conversions_preserve_all_sampled_allowed_values() {
        for magnitude in exponent_samples() {
            let positive: FinitePositiveF64 = FiniteF64::new(magnitude)
                .unwrap()
                .to_finite_positive()
                .unwrap();
            let negative: FiniteNegativeF64 = FiniteF64::new(-magnitude)
                .unwrap()
                .to_finite_negative()
                .unwrap();
            assert_eq!(positive.get_value().to_bits(), magnitude.to_bits());
            assert_eq!(negative.get_value().to_bits(), (-magnitude).to_bits());
        }
    }

    #[test]
    fn conversions_reject_every_sampled_nonzero_value_of_the_wrong_sign() {
        for raw in exponent_samples().filter(|raw| *raw > 0.0) {
            assert_eq!(
                FiniteF64::new(-raw).unwrap().to_finite_positive(),
                Err(UtilitiesErrors::NonPositiveF64)
            );
            assert_eq!(
                FiniteF64::new(raw).unwrap().to_finite_negative(),
                Err(UtilitiesErrors::NonNegativeF64)
            );
        }
    }

    #[test]
    fn both_signed_zeros_convert_to_either_domain_without_losing_the_sign_bit() {
        for raw in [-0.0_f64, 0.0] {
            let finite = FiniteF64::new(raw).unwrap();
            assert_eq!(
                finite.to_finite_positive().unwrap().get_value().to_bits(),
                raw.to_bits()
            );
            assert_eq!(
                finite.to_finite_negative().unwrap().get_value().to_bits(),
                raw.to_bits()
            );
            assert_eq!(finite.get_value().to_bits(), raw.to_bits());
        }
    }

    #[test]
    fn signed_values_reconstruct_the_original_general_finite_value() {
        for raw in magnitudes() {
            for value in [raw, -raw] {
                let original = FiniteF64::new(value).unwrap();
                let reconstructed = if value >= 0.0 {
                    FiniteF64::new(original.to_finite_positive().unwrap().get_value()).unwrap()
                } else {
                    FiniteF64::new(original.to_finite_negative().unwrap().get_value()).unwrap()
                };
                assert_eq!(
                    reconstructed.get_value().to_bits(),
                    original.get_value().to_bits()
                );
            }
        }
    }

    #[test]
    fn subtraction_results_can_be_converted_according_to_their_resulting_sign() {
        let one = FinitePositiveF64::new(1.0).unwrap();
        let four = FinitePositiveF64::new(4.0).unwrap();
        assert_eq!(
            (one - four)
                .unwrap()
                .to_finite_negative()
                .unwrap()
                .get_value(),
            -3.0
        );
        assert_eq!(
            (four - one)
                .unwrap()
                .to_finite_positive()
                .unwrap()
                .get_value(),
            3.0
        );
        assert_eq!(
            (one - four).unwrap().to_finite_positive(),
            Err(UtilitiesErrors::NonPositiveF64)
        );
        assert_eq!(
            (four - one).unwrap().to_finite_negative(),
            Err(UtilitiesErrors::NonNegativeF64)
        );
    }

    fn convert_and_negate(raw: f64) -> Result<FiniteNegativeF64, UtilitiesErrors> {
        let positive = FiniteF64::new(raw)?.to_finite_positive()?;
        -positive
    }

    #[test]
    fn conversion_chains_propagate_nonfinite_and_sign_errors() {
        assert_eq!(convert_and_negate(4.0).unwrap().get_value(), -4.0);
        assert_eq!(
            convert_and_negate(-1.0),
            Err(UtilitiesErrors::NonPositiveF64)
        );
        for raw in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert_eq!(convert_and_negate(raw), Err(UtilitiesErrors::NonFiniteF64));
        }
    }

    #[test]
    fn conversion_produces_a_valid_approximation_tolerance() {
        let a = FiniteF64::new(1.0).unwrap();
        let b = FiniteF64::new(1.25).unwrap();
        let tolerance = FiniteF64::new(0.25).unwrap().to_finite_positive().unwrap();
        assert!(a.is_aproximated_equal_to(b, tolerance));
        assert!(b.is_aproximated_equal_to(a, tolerance));
    }

    #[test]
    fn errors_remain_distinguishable_copyable_and_debuggable() {
        fn copy<T: Copy>(item: T) -> T {
            item
        }
        let errors = [
            UtilitiesErrors::NonFiniteF64,
            UtilitiesErrors::NonPositiveF64,
            UtilitiesErrors::NonNegativeF64,
        ];
        for (i, error) in errors.into_iter().enumerate() {
            assert_eq!(copy(error), error);
            assert!(!format!("{error:?}").is_empty());
            for (j, other) in errors.into_iter().enumerate() {
                assert_eq!(error == other, i == j);
            }
        }
    }
}
