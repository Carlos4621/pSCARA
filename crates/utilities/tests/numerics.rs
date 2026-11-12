use utilities::{
    UtilitiesErrors, validate_finite, validate_finite_negativo, validate_finite_positive,
    validate_non_zero_finite, validate_non_zero_finite_positive,
};

const SMALLEST: f64 = f64::from_bits(1);
const LARGEST_SUBNORMAL: f64 = f64::from_bits(f64::MIN_POSITIVE.to_bits() - 1);

fn finite_samples() -> impl Iterator<Item = f64> {
    // Every finite exponent, both signs, and representative mantissa boundaries.
    // This is not an enumeration of all possible f64 bit patterns.
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

fn boundary_magnitudes() -> [f64; 18] {
    [
        0.0,
        SMALLEST,
        2.0 * SMALLEST,
        LARGEST_SUBNORMAL,
        f64::MIN_POSITIVE,
        1e-300,
        1e-200,
        0.25,
        0.5,
        f64::from_bits(1.0_f64.to_bits() - 1),
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

fn raw_bit_samples() -> impl Iterator<Item = f64> {
    // Fixed seed keeps mixed-exponent/mantissa checks reproducible.
    let mut state = 0x6a09_e667_f3bc_c909_u64;
    (0..20_000).map(move |_| {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        f64::from_bits(state)
    })
}

macro_rules! nonfinite_suite {
    ($module:ident, $validate:ident) => {
        mod $module {
            use super::*;

            #[test]
            fn rejects_both_infinities_before_any_domain_check() {
                for raw in [f64::NEG_INFINITY, f64::INFINITY] {
                    assert_eq!($validate(raw), Err(UtilitiesErrors::NonFiniteF64));
                }
            }

            #[test]
            fn rejects_signed_quiet_and_signaling_nan_payloads() {
                for raw in nan_samples() {
                    assert!(raw.is_nan());
                    assert_eq!($validate(raw), Err(UtilitiesErrors::NonFiniteF64));
                }
            }
        }
    };
}

nonfinite_suite!(finite_nonfinite, validate_finite);
nonfinite_suite!(positive_nonfinite, validate_finite_positive);
nonfinite_suite!(negative_nonfinite, validate_finite_negativo);
nonfinite_suite!(nonzero_nonfinite, validate_non_zero_finite);
nonfinite_suite!(
    nonzero_positive_nonfinite,
    validate_non_zero_finite_positive
);

mod finite {
    use super::*;

    #[test]
    fn accepts_finite_boundaries_of_both_signs() {
        for magnitude in boundary_magnitudes() {
            for raw in [magnitude, -magnitude] {
                assert_eq!(validate_finite(raw), Ok(()), "input: {raw:?}");
            }
        }
    }

    #[test]
    fn accepts_every_sampled_finite_exponent_and_mantissa_class() {
        for raw in finite_samples() {
            assert_eq!(
                validate_finite(raw),
                Ok(()),
                "bits: {:#018x}",
                raw.to_bits()
            );
        }
    }

    #[test]
    fn accepts_both_signed_zeros() {
        for raw in [0.0_f64, -0.0] {
            assert_eq!(validate_finite(raw), Ok(()));
        }
    }

    #[test]
    fn accepts_finite_arithmetic_results_including_subnormal_underflow() {
        for raw in [
            1.5 + 2.25,
            f64::MAX - f64::MAX,
            f64::MIN_POSITIVE * 0.5,
            SMALLEST * 0.5,
            -SMALLEST / 2.0,
        ] {
            assert_eq!(validate_finite(raw), Ok(()));
        }
    }

    #[test]
    fn rejects_nonfinite_arithmetic_results() {
        let zero = 0.0_f64;
        for raw in [
            f64::MAX * 2.0,
            -f64::MAX - f64::MAX,
            1.0 / zero,
            zero / zero,
        ] {
            assert_eq!(validate_finite(raw), Err(UtilitiesErrors::NonFiniteF64));
        }
    }

    #[test]
    fn mixed_raw_bit_patterns_match_the_finite_domain() {
        for raw in raw_bit_samples() {
            let expected = if raw.is_finite() {
                Ok(())
            } else {
                Err(UtilitiesErrors::NonFiniteF64)
            };
            assert_eq!(
                validate_finite(raw),
                expected,
                "bits: {:#018x}",
                raw.to_bits()
            );
        }
    }
}

macro_rules! signed_suite {
    ($module:ident, $validate:ident, $sign:expr, $wrong_sign:ident) => {
        mod $module {
            use super::*;

            const SIGN: f64 = $sign;

            #[test]
            fn accepts_allowed_boundary_values() {
                for magnitude in boundary_magnitudes() {
                    assert_eq!($validate(SIGN * magnitude), Ok(()));
                }
            }

            #[test]
            fn rejects_wrong_sign_finite_nonzero_boundaries() {
                for magnitude in boundary_magnitudes().into_iter().filter(|raw| *raw > 0.0) {
                    assert_eq!(
                        $validate(-SIGN * magnitude),
                        Err(UtilitiesErrors::$wrong_sign)
                    );
                }
            }

            #[test]
            fn accepts_both_signed_zeros_as_an_inclusive_boundary() {
                assert_eq!($validate(0.0), Ok(()));
                assert_eq!($validate(-0.0), Ok(()));
            }

            #[test]
            fn partitions_all_sampled_finite_exponents_by_numeric_sign() {
                for magnitude in finite_samples().filter(|raw| *raw > 0.0) {
                    assert_eq!($validate(SIGN * magnitude), Ok(()));
                    assert_eq!(
                        $validate(-SIGN * magnitude),
                        Err(UtilitiesErrors::$wrong_sign)
                    );
                }
            }

            #[test]
            fn neighbors_of_zero_normal_boundary_and_maximum_have_exact_domains() {
                for bits in [0_u64, 1, 1_u64 << 52, f64::MAX.to_bits()] {
                    for neighbor in [bits.saturating_sub(1), bits, bits + 1] {
                        let magnitude = f64::from_bits(neighbor);
                        for direction in [SIGN, -SIGN] {
                            let expected = if !magnitude.is_finite() {
                                Err(UtilitiesErrors::NonFiniteF64)
                            } else if magnitude == 0.0 || direction == SIGN {
                                Ok(())
                            } else {
                                Err(UtilitiesErrors::$wrong_sign)
                            };
                            assert_eq!($validate(direction * magnitude), expected);
                        }
                    }
                }
            }

            #[test]
            fn mixed_raw_bits_match_the_inclusive_domain_with_nonfinite_error_priority() {
                for raw in raw_bit_samples() {
                    let expected = if !raw.is_finite() {
                        Err(UtilitiesErrors::NonFiniteF64)
                    } else if raw * SIGN >= 0.0 {
                        Ok(())
                    } else {
                        Err(UtilitiesErrors::$wrong_sign)
                    };
                    assert_eq!($validate(raw), expected, "bits: {:#018x}", raw.to_bits());
                }
            }

            #[test]
            fn accepts_underflow_to_either_signed_zero() {
                for raw in [SMALLEST / 2.0, -SMALLEST / 2.0] {
                    assert_eq!(raw, 0.0);
                    assert_eq!($validate(raw), Ok(()));
                }
            }
        }
    };
}

signed_suite!(positive, validate_finite_positive, 1.0, NonPositiveF64);
signed_suite!(negative, validate_finite_negativo, -1.0, NonNegativeF64);

mod nonzero {
    use super::*;

    // Active regressions: a nonzero validator must reject either zero and admit
    // every finite nonzero input. Do not encode the inverted check as a contract.
    #[test]
    fn rejects_both_signed_zeros() {
        let actual = [0.0, -0.0].map(validate_non_zero_finite);
        assert_eq!(actual, [Err(UtilitiesErrors::NonZeroFiniteF64); 2]);
    }

    #[test]
    fn accepts_all_sampled_nonzero_finite_values_including_subnormals_and_extremes() {
        let mut rejected = 0;
        let mut first_rejection = None;
        for raw in finite_samples().filter(|raw| *raw != 0.0) {
            let actual = validate_non_zero_finite(raw);
            if actual != Ok(()) {
                rejected += 1;
                first_rejection.get_or_insert((raw, actual));
            }
        }
        assert_eq!(
            rejected, 0,
            "first rejected input and result: {first_rejection:?}"
        );
    }
}

mod nonzero_positive {
    use super::*;

    #[test]
    fn accepts_positive_boundaries_including_subnormals_and_maximum() {
        for raw in boundary_magnitudes().into_iter().filter(|raw| *raw > 0.0) {
            assert_eq!(
                validate_non_zero_finite_positive(raw),
                Ok(()),
                "input: {raw:?}"
            );
        }
    }

    #[test]
    fn rejects_finite_negatives_with_a_sign_error_instead_of_a_zero_error() {
        for magnitude in boundary_magnitudes().into_iter().filter(|raw| *raw > 0.0) {
            assert_eq!(
                validate_non_zero_finite_positive(-magnitude),
                Err(UtilitiesErrors::NonPositiveF64)
            );
        }
    }

    #[test]
    fn rejects_both_signed_zeros_with_the_strict_positive_error() {
        for zero in [0.0_f64, -0.0] {
            assert_eq!(validate_finite_positive(zero), Ok(()));
            assert_eq!(
                validate_non_zero_finite_positive(zero),
                Err(UtilitiesErrors::NonZeroFinitePositiveF64)
            );
        }
    }

    #[test]
    fn every_sampled_finite_exponent_has_the_correct_strict_positive_domain() {
        for raw in finite_samples() {
            let expected = if raw > 0.0 {
                Ok(())
            } else if raw < 0.0 {
                Err(UtilitiesErrors::NonPositiveF64)
            } else {
                Err(UtilitiesErrors::NonZeroFinitePositiveF64)
            };
            assert_eq!(
                validate_non_zero_finite_positive(raw),
                expected,
                "bits: {:#018x}",
                raw.to_bits()
            );
        }
    }

    #[test]
    fn neighbors_of_zero_normal_boundary_and_maximum_keep_exact_error_categories() {
        for bits in [0_u64, 1, 1_u64 << 52, f64::MAX.to_bits()] {
            for neighbor in [bits.saturating_sub(1), bits, bits + 1] {
                let magnitude = f64::from_bits(neighbor);
                for sign in [-1.0, 1.0] {
                    let expected = if !magnitude.is_finite() {
                        Err(UtilitiesErrors::NonFiniteF64)
                    } else if magnitude == 0.0 {
                        Err(UtilitiesErrors::NonZeroFinitePositiveF64)
                    } else if sign < 0.0 {
                        Err(UtilitiesErrors::NonPositiveF64)
                    } else {
                        Ok(())
                    };
                    assert_eq!(
                        validate_non_zero_finite_positive(sign * magnitude),
                        expected
                    );
                }
            }
        }
    }

    #[test]
    fn mixed_raw_bits_match_the_strict_positive_domain_and_error_priority() {
        for raw in raw_bit_samples() {
            let expected = if !raw.is_finite() {
                Err(UtilitiesErrors::NonFiniteF64)
            } else if raw > 0.0 {
                Ok(())
            } else if raw == 0.0 {
                Err(UtilitiesErrors::NonZeroFinitePositiveF64)
            } else {
                Err(UtilitiesErrors::NonPositiveF64)
            };
            assert_eq!(
                validate_non_zero_finite_positive(raw),
                expected,
                "bits: {:#018x}",
                raw.to_bits()
            );
        }
    }

    #[test]
    fn underflow_to_zero_is_rejected_while_a_positive_subnormal_remains_valid() {
        assert_eq!(
            validate_non_zero_finite_positive(f64::MIN_POSITIVE / 2.0),
            Ok(())
        );
        for zero in [SMALLEST * 0.5, -SMALLEST * 0.5] {
            assert_eq!(zero, 0.0);
            assert_eq!(
                validate_non_zero_finite_positive(zero),
                Err(UtilitiesErrors::NonZeroFinitePositiveF64)
            );
        }
    }

    fn checked_reciprocal(denominator: f64) -> Result<f64, UtilitiesErrors> {
        validate_non_zero_finite_positive(denominator)?;
        let result = 1.0 / denominator;
        validate_finite(result)?;
        Ok(result)
    }

    #[test]
    fn question_mark_composition_preserves_success_and_each_validation_error() {
        assert_eq!(checked_reciprocal(2.0), Ok(0.5));
        for zero in [0.0, -0.0] {
            assert_eq!(
                checked_reciprocal(zero),
                Err(UtilitiesErrors::NonZeroFinitePositiveF64)
            );
        }
        assert_eq!(
            checked_reciprocal(-1.0),
            Err(UtilitiesErrors::NonPositiveF64)
        );
        for raw in nan_samples().chain([f64::NEG_INFINITY, f64::INFINITY]) {
            assert_eq!(checked_reciprocal(raw), Err(UtilitiesErrors::NonFiniteF64));
        }
    }

    #[test]
    fn a_valid_positive_denominator_still_requires_result_validation() {
        assert_eq!(validate_non_zero_finite_positive(SMALLEST), Ok(()));
        assert_eq!(
            checked_reciprocal(SMALLEST),
            Err(UtilitiesErrors::NonFiniteF64)
        );
        let tiny_reciprocal = checked_reciprocal(f64::MAX).unwrap();
        assert!(tiny_reciprocal > 0.0 && tiny_reciprocal < f64::MIN_POSITIVE);
    }
}

mod composition {
    use super::*;

    fn positive_sum(a: f64, b: f64) -> Result<f64, UtilitiesErrors> {
        validate_finite_positive(a)?;
        validate_finite_positive(b)?;
        let sum = a + b;
        validate_finite_positive(sum)?;
        Ok(sum)
    }

    #[test]
    fn question_mark_composes_valid_input_and_result_checks() {
        assert_eq!(positive_sum(1.5, 2.25), Ok(3.75));
        assert_eq!(positive_sum(0.0, -0.0), Ok(0.0));
        assert_eq!(positive_sum(SMALLEST, SMALLEST), Ok(2.0 * SMALLEST));
    }

    #[test]
    fn input_validation_does_not_replace_result_validation() {
        assert_eq!(validate_finite_positive(f64::MAX), Ok(()));
        assert_eq!(
            positive_sum(f64::MAX, f64::MAX),
            Err(UtilitiesErrors::NonFiniteF64)
        );
    }

    #[test]
    fn question_mark_propagates_the_first_error_in_execution_order() {
        assert_eq!(
            positive_sum(-1.0, f64::NAN),
            Err(UtilitiesErrors::NonPositiveF64)
        );
        assert_eq!(
            positive_sum(f64::NAN, -1.0),
            Err(UtilitiesErrors::NonFiniteF64)
        );
        assert_eq!(
            positive_sum(1.0, -1.0),
            Err(UtilitiesErrors::NonPositiveF64)
        );
        assert_eq!(
            positive_sum(1.0, f64::INFINITY),
            Err(UtilitiesErrors::NonFiniteF64)
        );
    }

    #[test]
    fn opposite_inclusive_sign_domains_intersect_at_both_signed_zeros() {
        for raw in finite_samples() {
            let both =
                validate_finite_positive(raw).is_ok() && validate_finite_negativo(raw).is_ok();
            let either =
                validate_finite_positive(raw).is_ok() || validate_finite_negativo(raw).is_ok();
            assert_eq!(both, raw == 0.0);
            assert!(either);
        }
    }

    #[test]
    fn negative_inputs_can_produce_a_valid_nonnegative_product() {
        fn product(a: f64, b: f64) -> Result<f64, UtilitiesErrors> {
            validate_finite_negativo(a)?;
            validate_finite_negativo(b)?;
            let result = a * b;
            validate_finite_positive(result)?;
            Ok(result)
        }
        assert_eq!(product(-2.0, -4.0), Ok(8.0));
        assert_eq!(product(-SMALLEST, -0.5), Ok(0.0));
        assert_eq!(product(-f64::MAX, -2.0), Err(UtilitiesErrors::NonFiniteF64));
        assert_eq!(product(2.0, -4.0), Err(UtilitiesErrors::NonNegativeF64));
    }
}
