use core::cmp::Ordering;

use utilities::{FiniteF64, FinitePositiveF64, UtilitiesErrors};

const SMALLEST: f64 = f64::from_bits(1);
const LARGEST_SUBNORMAL: f64 = f64::from_bits(f64::MIN_POSITIVE.to_bits() - 1);

fn finite(value: f64) -> FiniteF64 {
    match FiniteF64::new(value) {
        Ok(value) => value,
        Err(_) => panic!("finite fixture rejected: {value:?}"),
    }
}

fn value(result: Result<FiniteF64, UtilitiesErrors>) -> FiniteF64 {
    match result {
        Ok(value) => value,
        Err(_) => panic!("expected a finite result"),
    }
}

fn positive(value: f64) -> FinitePositiveF64 {
    FinitePositiveF64::new(value).expect("tolerance fixture must be finite and nonnegative")
}

fn assert_nonfinite_error(result: Result<FiniteF64, UtilitiesErrors>) {
    assert!(matches!(result, Err(UtilitiesErrors::NonFiniteF64)));
}

fn assert_bits(actual: FiniteF64, expected: f64) {
    assert_eq!(actual.get_value().to_bits(), expected.to_bits());
}

fn boundaries() -> [f64; 28] {
    [
        -f64::MAX,
        -1e300,
        -1e200,
        -2.0,
        -1.0,
        -0.5,
        -1e-200,
        -1e-300,
        -f64::MIN_POSITIVE,
        -LARGEST_SUBNORMAL,
        -2.0 * SMALLEST,
        -SMALLEST,
        -0.0,
        0.0,
        SMALLEST,
        2.0 * SMALLEST,
        LARGEST_SUBNORMAL,
        f64::MIN_POSITIVE,
        1e-300,
        1e-200,
        0.5,
        1.0,
        f64::from_bits(1.0_f64.to_bits() + 1),
        2.0,
        1e200,
        1e300,
        f64::from_bits(f64::MAX.to_bits() - 1),
        f64::MAX,
    ]
}

// Every finite exponent field, both signs, and mantissas at zero, one,
// midpoint and the upper boundary. This samples all exponent classes;
// enumerating all 2^64 bit patterns is deliberately not claimed.
fn exponent_samples() -> impl Iterator<Item = f64> {
    (0_u64..0x7ff).flat_map(|exponent| {
        [0, 1, 1_u64 << 51, (1_u64 << 52) - 1]
            .into_iter()
            .flat_map(move |mantissa| {
                [0, 1_u64 << 63].map(|sign| f64::from_bits(sign | (exponent << 52) | mantissa))
            })
    })
}

mod construction_and_traits {
    use super::*;

    #[test]
    fn constructor_and_getter_preserve_finite_boundary_bits() {
        for raw in boundaries() {
            assert_bits(finite(raw), raw);
        }
    }

    #[test]
    fn constructor_preserves_all_sampled_exponent_sign_and_mantissa_classes() {
        for raw in exponent_samples() {
            assert!(raw.is_finite());
            assert_bits(finite(raw), raw);
        }
    }

    #[test]
    fn constructor_rejects_infinities_and_signed_quiet_and_signaling_nan_payloads() {
        for sign in [0, 1_u64 << 63] {
            for payload in [0, 1, 2, (1_u64 << 51) - 1, 1_u64 << 51, (1_u64 << 52) - 1] {
                let raw = f64::from_bits(sign | (0x7ff_u64 << 52) | payload);
                assert_nonfinite_error(FiniteF64::new(raw));
            }
        }
    }

    #[test]
    fn constructor_distinguishes_neighbors_of_zero_normal_boundary_and_maximum() {
        for bits in [0, 1, (1_u64 << 52) - 1, 1_u64 << 52, f64::MAX.to_bits()] {
            for neighbor in [bits.saturating_sub(1), bits, bits + 1] {
                for sign in [0, 1_u64 << 63] {
                    let raw = f64::from_bits(sign | neighbor);
                    if raw.is_finite() {
                        assert_bits(finite(raw), raw);
                    } else {
                        assert_nonfinite_error(FiniteF64::new(raw));
                    }
                }
            }
        }
    }

    #[test]
    fn repeated_getter_calls_do_not_mutate_the_value() {
        for raw in boundaries() {
            let number = finite(raw);
            for _ in 0..4 {
                assert_bits(number, raw);
            }
        }
    }

    #[test]
    fn clone_and_copy_preserve_bits_without_consuming_the_original() {
        fn requires_copy<T: Copy>(item: T) -> (T, T) {
            (item, item)
        }
        fn clone_generic<T: Clone>(item: &T) -> T {
            item.clone()
        }
        for raw in boundaries() {
            let original = finite(raw);
            let (first, second) = requires_copy(original);
            for copy in [original, first, second, clone_generic(&original)] {
                assert_bits(copy, raw);
            }
        }
    }

    #[test]
    fn equality_uses_numeric_equality_including_both_zero_signs() {
        for left in boundaries() {
            for right in boundaries() {
                assert_eq!(finite(left) == finite(right), left == right);
                assert_eq!(finite(left) != finite(right), left != right);
            }
        }
        assert_eq!(finite(-0.0), finite(0.0));
        assert_ne!(finite(1.0), finite(f64::from_bits(1.0_f64.to_bits() + 1)));
    }

    #[test]
    fn ordering_is_defined_for_every_finite_boundary_pair() {
        for left in boundaries() {
            for right in boundaries() {
                let a = finite(left);
                let b = finite(right);
                assert_eq!(a.partial_cmp(&b), left.partial_cmp(&right));
                assert_eq!(a < b, left < right);
                assert_eq!(a <= b, left <= right);
                assert_eq!(a > b, left > right);
                assert_eq!(a >= b, left >= right);
            }
        }
        assert_eq!(
            finite(-0.0).partial_cmp(&finite(0.0)),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn ordering_composes_with_collection_sorting() {
        let mut numbers: Vec<_> = boundaries().into_iter().rev().map(finite).collect();
        numbers.sort_by(|a, b| a.partial_cmp(b).expect("finite values are comparable"));
        for pair in numbers.windows(2) {
            assert!(pair[0] <= pair[1]);
        }
        assert_bits(numbers[0], -f64::MAX);
        assert_bits(*numbers.last().unwrap(), f64::MAX);
    }

    #[test]
    fn debug_formatting_identifies_the_type_and_value() {
        let formatted = format!("{:?}", finite(-1.25));
        assert!(formatted.contains("FiniteF64"));
        assert!(formatted.contains("-1.25"));
    }
}

mod arithmetic {
    use super::*;

    // Besides exact hand-computed cases below, the wrapper must agree bit for
    // bit with IEEE f64 arithmetic and reject exactly its nonfinite outcomes.
    fn assert_ieee_result(actual: Result<FiniteF64, UtilitiesErrors>, expected: f64) {
        if expected.is_finite() {
            assert_bits(value(actual), expected);
        } else {
            assert_nonfinite_error(actual);
        }
    }

    fn assert_all_operations(left: f64, right: f64) {
        let a = finite(left);
        let b = finite(right);
        assert_ieee_result(a + b, left + right);
        assert_ieee_result(a - b, left - right);
        assert_ieee_result(a * b, left * right);
        assert_ieee_result(a / b, left / right);
        assert_bits(a, left);
        assert_bits(b, right);
    }

    #[test]
    fn addition_has_exact_results_for_positive_negative_and_cancelling_operands() {
        for (a, b, expected) in [(1.5, 2.25, 3.75), (-7.0, 3.0, -4.0), (8.0, -8.0, 0.0)] {
            assert_bits(value(finite(a) + finite(b)), expected);
            assert_bits(value(finite(b) + finite(a)), expected);
        }
    }

    #[test]
    fn subtraction_respects_operand_order() {
        for (a, b, expected) in [(7.5, 2.25, 5.25), (-7.0, -3.0, -4.0), (0.0, 8.0, -8.0)] {
            assert_bits(value(finite(a) - finite(b)), expected);
            assert_bits(value(finite(b) - finite(a)), -expected);
        }
    }

    #[test]
    fn multiplication_has_exact_results_in_all_sign_quadrants() {
        for (a, b, expected) in [(1.5, 2.0, 3.0), (-1.5, 2.0, -3.0), (-1.5, -2.0, 3.0)] {
            assert_bits(value(finite(a) * finite(b)), expected);
            assert_bits(value(finite(b) * finite(a)), expected);
        }
    }

    #[test]
    fn division_has_exact_results_in_all_sign_quadrants() {
        for (a, b, expected) in [(7.5, 2.5, 3.0), (-7.5, 2.5, -3.0), (-7.5, -2.5, 3.0)] {
            assert_bits(value(finite(a) / finite(b)), expected);
        }
        assert_bits(value(finite(2.0) / finite(8.0)), 0.25);
        assert_bits(value(finite(8.0) / finite(2.0)), 4.0);
    }

    #[test]
    fn negation_flips_only_the_sign_bit_and_is_an_involution_for_all_exponent_samples() {
        for raw in exponent_samples() {
            let number = finite(raw);
            assert_eq!(
                (-number).get_value().to_bits(),
                raw.to_bits() ^ (1_u64 << 63)
            );
            assert_bits(-(-number), raw);
            assert_bits(number, raw);
        }
    }

    #[test]
    fn arithmetic_preserves_ieee_signed_zero_rules() {
        assert_bits(value(finite(-0.0) + finite(-0.0)), -0.0);
        assert_bits(value(finite(-0.0) + finite(0.0)), 0.0);
        assert_bits(value(finite(-0.0) - finite(0.0)), -0.0);
        assert_bits(value(finite(0.0) - finite(-0.0)), 0.0);
        for zero in [-0.0, 0.0] {
            for operand in [-2.0, 2.0] {
                assert_bits(value(finite(zero) * finite(operand)), zero * operand);
                assert_bits(value(finite(zero) / finite(operand)), zero / operand);
            }
        }
    }

    #[test]
    fn identities_hold_across_every_sampled_finite_exponent() {
        for raw in exponent_samples() {
            let number = finite(raw);
            assert_bits(value(number - finite(0.0)), raw);
            assert_bits(value(number * finite(1.0)), raw);
            assert_bits(value(number / finite(1.0)), raw);
            assert_bits(value(number * finite(-1.0)), -raw);
            assert_bits(value(number / finite(-1.0)), -raw);
            assert_bits(value(number - number), 0.0);
            assert_bits(value(number + -number), 0.0);
            if raw != 0.0 {
                assert_bits(value(number / number), 1.0);
            }
        }
    }

    #[test]
    fn addition_rejects_overflow_of_either_sign() {
        for sign in [-1.0, 1.0] {
            let maximum = finite(sign * f64::MAX);
            assert_nonfinite_error(maximum + maximum);
            assert_nonfinite_error(maximum + finite(sign * f64::MAX / 2.0));
        }
    }

    #[test]
    fn subtraction_rejects_overflow_of_either_sign() {
        for sign in [-1.0, 1.0] {
            assert_nonfinite_error(finite(sign * f64::MAX) - finite(-sign * f64::MAX));
        }
    }

    #[test]
    fn multiplication_rejects_overflow_in_all_sign_quadrants() {
        for left_sign in [-1.0, 1.0] {
            for right_sign in [-1.0, 1.0] {
                assert_nonfinite_error(finite(left_sign * f64::MAX) * finite(right_sign * 2.0));
            }
        }
    }

    #[test]
    fn division_rejects_overflow_in_all_sign_quadrants() {
        for left_sign in [-1.0, 1.0] {
            for right_sign in [-1.0, 1.0] {
                assert_nonfinite_error(finite(left_sign * f64::MAX) / finite(right_sign * 0.5));
                assert_nonfinite_error(finite(left_sign) / finite(right_sign * SMALLEST));
            }
        }
    }

    #[test]
    fn division_by_either_signed_zero_rejects_infinities_and_nan() {
        for numerator in boundaries() {
            for zero in [-0.0, 0.0] {
                assert_nonfinite_error(finite(numerator) / finite(zero));
            }
        }
    }

    #[test]
    fn opposite_extremes_cancel_and_nearby_extremes_have_finite_differences() {
        let previous_max = f64::from_bits(f64::MAX.to_bits() - 1);
        let ulp = 2.0_f64.powi(971);
        assert_bits(value(finite(f64::MAX) + finite(-f64::MAX)), 0.0);
        assert_bits(value(finite(f64::MAX) - finite(previous_max)), ulp);
        assert_bits(value(finite(previous_max) - finite(f64::MAX)), -ulp);
        assert_bits(value(finite(f64::MAX) / finite(-f64::MAX)), -1.0);
    }

    #[test]
    fn subnormal_arithmetic_crosses_the_normal_boundary_without_rejection() {
        assert_bits(
            value(finite(LARGEST_SUBNORMAL) + finite(SMALLEST)),
            f64::MIN_POSITIVE,
        );
        assert_bits(
            value(finite(f64::MIN_POSITIVE) - finite(SMALLEST)),
            LARGEST_SUBNORMAL,
        );
        assert_bits(
            value(finite(f64::MIN_POSITIVE) * finite(0.5)),
            f64::MIN_POSITIVE / 2.0,
        );
        assert_bits(
            value(finite(f64::MIN_POSITIVE) / finite(2.0)),
            f64::MIN_POSITIVE / 2.0,
        );
        assert_bits(value(finite(SMALLEST) + finite(SMALLEST)), 2.0 * SMALLEST);
    }

    #[test]
    fn underflow_to_signed_zero_is_a_valid_finite_result() {
        for sign in [-1.0, 1.0] {
            assert_bits(value(finite(sign * SMALLEST) * finite(0.5)), sign * 0.0);
            assert_bits(value(finite(sign * SMALLEST) / finite(2.0)), sign * 0.0);
            assert_bits(
                value(finite(sign * f64::MIN_POSITIVE) / finite(f64::MAX)),
                sign * 0.0,
            );
        }
    }

    #[test]
    fn underflow_ties_round_to_even_for_multiplication_and_division() {
        for sign in [-1.0, 1.0] {
            for (units, expected_units) in [(1.0, 0.0), (2.0, 1.0), (3.0, 2.0), (5.0, 2.0)] {
                let raw = sign * units * SMALLEST;
                let expected = sign * expected_units * SMALLEST;
                assert_bits(value(finite(raw) * finite(0.5)), expected);
                assert_bits(value(finite(raw) / finite(2.0)), expected);
            }
        }
    }

    #[test]
    fn rounding_at_one_preserves_ties_and_adjacent_representable_values() {
        let half_ulp = f64::EPSILON / 2.0;
        assert_bits(value(finite(1.0) + finite(half_ulp)), 1.0);
        assert_bits(
            value(finite(1.0) + finite(f64::EPSILON)),
            f64::from_bits(1.0_f64.to_bits() + 1),
        );
        assert_bits(
            value(finite(1.0) - finite(half_ulp)),
            f64::from_bits(1.0_f64.to_bits() - 1),
        );
        assert_bits(value(finite(1.0) + finite(SMALLEST)), 1.0);
        assert_bits(value(finite(1.0) / finite(10.0)), 0.1);
    }

    #[test]
    fn overflow_threshold_neighbors_distinguish_finite_rounding_from_infinity() {
        let below_one = f64::from_bits(1.0_f64.to_bits() - 1);
        let above_one = f64::from_bits(1.0_f64.to_bits() + 1);
        let half_ulp_at_max = 2.0_f64.powi(970);
        let below_half_ulp = f64::from_bits(half_ulp_at_max.to_bits() - 1);
        for sign in [-1.0, 1.0] {
            let maximum = finite(sign * f64::MAX);
            assert_bits(
                value(maximum + finite(sign * below_half_ulp)),
                sign * f64::MAX,
            );
            assert_nonfinite_error(maximum + finite(sign * half_ulp_at_max));
            assert_bits(value(maximum * finite(1.0)), sign * f64::MAX);
            assert_nonfinite_error(maximum * finite(above_one));
            assert_nonfinite_error(maximum / finite(below_one));
            assert!(value(maximum * finite(below_one)).get_value().is_finite());
            assert!(value(maximum / finite(above_one)).get_value().is_finite());
        }
    }

    #[test]
    fn exact_integer_grid_obeys_arithmetic_and_reversible_composition() {
        for a in -32..=32 {
            for b in -32..=32 {
                let left = finite(f64::from(a));
                let right = finite(f64::from(b));
                assert_bits(value(left + right), f64::from(a + b));
                assert_bits(value(left - right), f64::from(a - b));
                // Integer zero has no sign; IEEE multiplication can yield -0.
                // Bit-level signed-zero behavior is checked separately.
                assert_eq!(value(left * right).get_value(), f64::from(a * b));
                assert_eq!(value(value(left + right) - right).get_value(), f64::from(a));
                if b != 0 {
                    assert_eq!(value(value(left * right) / right).get_value(), f64::from(a));
                }
            }
        }
    }

    #[test]
    fn every_boundary_pair_obeys_ieee_results_and_the_finite_invariant() {
        for left in boundaries() {
            for right in boundaries() {
                assert_all_operations(left, right);
            }
        }
    }

    #[test]
    fn every_exponent_class_supports_operations_with_zero_unit_and_extreme_operands() {
        for left in exponent_samples() {
            for right in [0.0, -0.0, 1.0, -1.0, 2.0, SMALLEST, f64::MAX] {
                assert_all_operations(left, right);
            }
        }
    }

    #[test]
    fn reproducible_bit_pattern_pairs_cover_mixed_exponents_and_mantissas() {
        // Fixed-seed xorshift: no external dependency or nondeterministic seed.
        let mut state = 0x6a09_e667_f3bc_c909_u64;
        let mut next = || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            f64::from_bits(state)
        };
        let mut checked = 0;
        for _ in 0..20_000 {
            let (left, right) = (next(), next());
            if left.is_finite() && right.is_finite() {
                assert_all_operations(left, right);
                checked += 1;
            }
        }
        assert!(checked > 19_900);
    }

    fn composed(a: FiniteF64, b: FiniteF64, c: FiniteF64) -> Result<FiniteF64, UtilitiesErrors> {
        let sum = (a + b)?;
        let product = (sum * c)?;
        let difference = (product - b)?;
        difference / c
    }

    #[test]
    fn result_question_mark_composes_all_four_arithmetic_operators() {
        assert_bits(value(composed(finite(2.0), finite(4.0), finite(2.0))), 4.0);
    }

    #[test]
    fn result_composition_propagates_errors_at_the_first_middle_and_final_steps() {
        assert_nonfinite_error(composed(finite(f64::MAX), finite(f64::MAX), finite(1.0)));
        assert_nonfinite_error(composed(finite(f64::MAX / 2.0), finite(0.0), finite(4.0)));
        assert_nonfinite_error(composed(finite(1.0), finite(0.0), finite(0.0)));
    }

    #[test]
    fn successful_and_failed_operations_leave_copied_operands_reusable() {
        let maximum = finite(f64::MAX);
        let zero = finite(-0.0);
        assert_nonfinite_error(maximum + maximum);
        assert_nonfinite_error(maximum / zero);
        assert_bits(value(maximum - maximum), 0.0);
        assert_bits(maximum, f64::MAX);
        assert_bits(zero, -0.0);
    }

    #[test]
    fn rounded_arithmetic_does_not_assume_real_number_associativity() {
        let a = finite(1e16);
        let b = finite(-1e16);
        let c = finite(1.0);
        assert_bits(value(value(a + b) + c), 1.0);
        assert_bits(value(a + value(b + c)), 0.0);
    }
}

mod approximation {
    use super::*;

    // User-specified contract: absolute distance with an inclusive bound.
    // Negative tolerances are rejected when constructing FinitePositiveF64.
    // A subtraction that overflows is farther apart than any finite tolerance.
    fn close(left: f64, right: f64, tolerance: f64) -> bool {
        finite(left).is_aproximated_equal_to(finite(right), positive(tolerance))
    }

    fn assert_both_directions(left: f64, right: f64, tolerance: f64, expected: bool) {
        for (a, b) in [(left, right), (right, left)] {
            assert_eq!(
                close(a, b, tolerance),
                expected,
                "a={a:?}, b={b:?}, tolerance={tolerance:?}"
            );
        }
    }

    #[test]
    fn identical_values_are_close_with_zero_tolerance_at_every_scale() {
        for raw in boundaries() {
            for tolerance in [-0.0, 0.0] {
                assert_both_directions(raw, raw, tolerance, true);
            }
        }
    }

    #[test]
    fn both_signed_zeros_are_close_with_either_signed_zero_tolerance() {
        for a in [-0.0, 0.0] {
            for b in [-0.0, 0.0] {
                for tolerance in [-0.0, 0.0, SMALLEST] {
                    assert_both_directions(a, b, tolerance, true);
                }
            }
        }
    }

    #[test]
    fn positive_tolerance_accepts_strictly_inside_in_both_directions() {
        for (a, b, tolerance) in [(1.0, 1.25, 0.5), (-4.0, -3.0, 2.0), (-0.5, 0.5, 2.0)] {
            assert_both_directions(a, b, tolerance, true);
        }
    }

    #[test]
    fn tolerance_boundary_is_inclusive_in_both_directions() {
        for (a, b, tolerance) in [(0.0, 1.0, 1.0), (-2.0, -1.5, 0.5), (-1.0, 1.0, 2.0)] {
            assert_both_directions(a, b, tolerance, true);
        }
    }

    #[test]
    fn separated_values_are_not_close_even_when_the_signed_difference_is_negative() {
        for (a, b, tolerance) in [(0.0, 100.0, 1.0), (-100.0, -1.0, 1.0), (-10.0, 10.0, 0.5)] {
            assert_both_directions(a, b, tolerance, false);
        }
    }

    #[test]
    fn distinct_values_are_not_close_at_zero_tolerance() {
        for (a, b) in [(0.0, SMALLEST), (-SMALLEST, 0.0), (1.0, 2.0), (-1.0, 1.0)] {
            for tolerance in [-0.0, 0.0] {
                assert_both_directions(a, b, tolerance, false);
            }
        }
    }

    #[test]
    fn closeness_is_symmetric_for_boundary_values_and_nonnegative_tolerances() {
        for a in boundaries() {
            for b in boundaries() {
                for tolerance in [0.0, SMALLEST, 0.5, 1.0, f64::MAX] {
                    assert_eq!(
                        close(a, b, tolerance),
                        close(b, a, tolerance),
                        "a={a:?}, b={b:?}, tolerance={tolerance:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn subnormal_tolerances_distinguish_equal_boundary_and_outside_distances() {
        assert_both_directions(0.0, SMALLEST, SMALLEST, true);
        assert_both_directions(0.0, 2.0 * SMALLEST, SMALLEST, false);
        assert_both_directions(-SMALLEST, SMALLEST, 2.0 * SMALLEST, true);
        assert_both_directions(LARGEST_SUBNORMAL, f64::MIN_POSITIVE, SMALLEST, true);
    }

    #[test]
    fn adjacent_tolerances_resolve_the_inclusive_boundary() {
        let below_one = f64::from_bits(1.0_f64.to_bits() - 1);
        let above_one = f64::from_bits(1.0_f64.to_bits() + 1);
        assert_both_directions(0.0, 1.0, below_one, false);
        assert_both_directions(0.0, 1.0, 1.0, true);
        assert_both_directions(0.0, 1.0, above_one, true);
    }

    #[test]
    fn adjacent_values_near_one_require_their_representable_spacing() {
        let next = f64::from_bits(1.0_f64.to_bits() + 1);
        assert_both_directions(1.0, next, f64::EPSILON / 2.0, false);
        assert_both_directions(1.0, next, f64::EPSILON, true);
    }

    #[test]
    fn largest_tolerance_accepts_the_largest_representable_distance() {
        assert_both_directions(0.0, f64::MAX, f64::MAX, true);
        assert_both_directions(-f64::MAX, 0.0, f64::MAX, true);
    }

    #[test]
    fn subtraction_overflow_returns_false_in_both_directions_without_error() {
        for (a, b) in [(f64::MAX, -f64::MAX), (f64::MAX, -f64::MAX / 2.0)] {
            for tolerance in [0.0, SMALLEST, 1.0, f64::MAX] {
                assert_both_directions(a, b, tolerance, false);
            }
        }
    }

    #[test]
    fn negative_tolerance_is_rejected_before_comparison() {
        for tolerance in [-SMALLEST, -1.0, -f64::MAX] {
            assert_eq!(
                FinitePositiveF64::new(tolerance),
                Err(UtilitiesErrors::NonPositiveF64)
            );
        }
    }

    #[test]
    fn inclusive_distance_matches_an_exact_integer_grid() {
        for a in -8_i32..=8 {
            for b in -8_i32..=8 {
                for tolerance in 0..=17 {
                    let expected = a.abs_diff(b) <= tolerance as u32;
                    assert_both_directions(
                        f64::from(a),
                        f64::from(b),
                        f64::from(tolerance),
                        expected,
                    );
                }
            }
        }
    }

    #[test]
    fn boundary_pair_grid_matches_absolute_inclusive_ieee_distance() {
        let mut mismatches = 0;
        let mut first_mismatch = None;
        for a in boundaries() {
            for b in boundaries() {
                for tolerance in [-0.0, 0.0, SMALLEST, f64::MIN_POSITIVE, 1.0, f64::MAX] {
                    let expected = (a - b).abs() <= tolerance;
                    let actual = close(a, b, tolerance);
                    if actual != expected {
                        mismatches += 1;
                        first_mismatch.get_or_insert((a, b, tolerance, expected, actual));
                    }
                }
            }
        }
        assert_eq!(
            mismatches, 0,
            "first (a, b, tolerance, expected, actual): {first_mismatch:?}"
        );
    }

    #[test]
    fn comparisons_compose_with_arithmetic_without_consuming_inputs() {
        let start = finite(2.0);
        let offset = finite(0.25);
        let end = value(start + offset);
        assert!(start.is_aproximated_equal_to(end, positive(0.5)));
        assert!(end.is_aproximated_equal_to(start, positive(0.5)));
        assert!(!start.is_aproximated_equal_to(end, positive(0.125)));
        assert_bits(start, 2.0);
        assert_bits(offset, 0.25);
        assert_bits(end, 2.25);
    }

    #[test]
    fn approximation_accepts_a_tolerance_produced_by_typed_arithmetic() {
        let tolerance: FinitePositiveF64 = (positive(0.25) + positive(0.25)).unwrap();
        assert!(finite(-0.25).is_aproximated_equal_to(finite(0.25), tolerance));
        assert!(!finite(-0.25).is_aproximated_equal_to(finite(0.5), tolerance));
    }

    #[test]
    fn equality_and_zero_tolerance_agree_across_all_exponent_samples() {
        for raw in exponent_samples() {
            assert_both_directions(raw, raw, 0.0, true);
            if raw != 0.0 {
                assert_both_directions(raw, -raw, 0.0, false);
            }
        }
    }

    #[test]
    fn increasing_tolerance_cannot_turn_a_match_into_a_miss() {
        for a in boundaries() {
            for b in boundaries() {
                let mut matched = false;
                for tolerance in [0.0, SMALLEST, f64::MIN_POSITIVE, 0.5, 1.0, 1e300, f64::MAX] {
                    let current = close(a, b, tolerance);
                    assert!(
                        !matched || current,
                        "a={a:?}, b={b:?}, tolerance={tolerance:?}"
                    );
                    matched = current;
                }
            }
        }
    }

    #[test]
    fn exact_translation_and_sign_reversal_preserve_closeness() {
        for offset in [-1024.0, 0.0, 1024.0] {
            for tolerance in [0.0, 0.125, 0.25, 1.0] {
                let expected = tolerance >= 0.25;
                assert_both_directions(offset + 1.0, offset + 1.25, tolerance, expected);
                assert_both_directions(-offset - 1.0, -offset - 1.25, tolerance, expected);
            }
        }
    }
}
