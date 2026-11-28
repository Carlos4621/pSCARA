use super::*;

fn power_of_two(exponent: i32) -> f64 {
    libm::scalbn(1.0, exponent)
}

fn assert_product_matches_reference(numerators: &[f64], denominators: &[f64]) {
    assert_eq!(
        product_ratio(numerators, denominators).to_bits(),
        product_ratio_reference(numerators, denominators).to_bits(),
        "numerators={numerators:?}, denominators={denominators:?}"
    );
}

#[test]
fn fast_factor_accepts_signed_zeros_and_the_intended_exponents() {
    let minimum = power_of_two(-200);
    let upper_bound = power_of_two(201);
    for sign in [-1.0, 1.0] {
        for value in [0.0, minimum, minimum.next_up(), upper_bound.next_down()] {
            assert!(is_fast_factor(sign * value), "factor={}", sign * value);
        }
        for value in [
            f64::from_bits(1),
            f64::MIN_POSITIVE,
            minimum.next_down(),
            upper_bound,
            f64::MAX,
            f64::INFINITY,
            f64::NAN,
        ] {
            assert!(!is_fast_factor(sign * value), "factor={}", sign * value);
        }
    }
}

#[test]
fn fast_products_match_reference() {
    let values = [
        -f64::MAX,
        -100.0,
        -2.0,
        -1.0,
        -0.0,
        0.0,
        f64::from_bits(1),
        f64::MIN_POSITIVE,
        power_of_two(-201),
        power_of_two(-200),
        0.001,
        0.1,
        0.5,
        1.0,
        2.0,
        3.0,
        10.0,
        100.0,
        power_of_two(200),
        power_of_two(201),
        f64::MAX,
    ];

    for a in values {
        for b in values {
            assert_product_matches_reference(&[a, b], &[]);
            for c in values {
                assert_product_matches_reference(&[a, b, c], &[2.0]);
            }
        }
    }
}

#[test]
fn fast_products_match_reference_for_pseudorandom_factors() {
    let mut state = 0xbb67_ae85_84ca_a73b_u64;
    let mut next = || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state
    };
    let mut factor = |min_exponent: u64, max_exponent: u64| {
        let exponent = min_exponent + next() % (max_exponent - min_exponent + 1);
        let sign_and_fraction = next() & !(0x7ff_u64 << 52);
        f64::from_bits(sign_and_fraction | (exponent << 52))
    };

    for _ in 0..8192 {
        let a = factor(823, 1223);
        let b = factor(823, 1223);
        let c = factor(823, 1223);
        assert_product_matches_reference(&[a, b], &[]);
        assert_product_matches_reference(&[a, b, c], &[2.0]);

        let wide = factor(0, 2046);
        assert_product_matches_reference(&[wide, b], &[]);
        assert_product_matches_reference(&[a, b, wide], &[2.0]);
    }
}

fn assert_ratio_matches_reference(numerators: &[f64], denominators: &[f64]) {
    let (expected_fraction, expected_exponent) = ratio_parts_reference(numerators, denominators);
    let (fraction, exponent) = ratio_parts(numerators, denominators);
    assert_eq!(
        (fraction.to_bits(), exponent),
        (expected_fraction.to_bits(), expected_exponent),
        "numerators={numerators:?}, denominators={denominators:?}"
    );
}

#[test]
fn optimized_ratio_matches_reference_at_finite_boundaries() {
    let values = [
        -f64::MAX,
        -2.0,
        -1.0,
        -0.0,
        0.0,
        f64::from_bits(1),
        f64::MIN_POSITIVE,
        0.5,
        1.0,
        2.0,
        f64::MAX,
    ];
    let divisors = [
        f64::from_bits(1),
        f64::MIN_POSITIVE,
        0.5,
        1.0,
        2.0,
        f64::MAX,
    ];

    for a in values {
        for b in divisors {
            for c in divisors {
                assert_ratio_matches_reference(&[a, b, c, b], &[b, c, b]);
            }
        }
    }
}

#[test]
fn optimized_ratio_preserves_uncancelled_factors_with_unequal_multiplicities() {
    assert_ratio_matches_reference(&[], &[]);
    assert_ratio_matches_reference(&[], &[2.3, 2.9, 3.1]);
    assert_ratio_matches_reference(&[1.1, 1.3, 1.7], &[]);
    assert_ratio_matches_reference(&[1.1, 1.3, 1.7], &[2.3, 2.9, 3.1]);
    assert_ratio_matches_reference(&[1.1, 7.0, 1.3, 7.0, 1.7], &[2.3, 7.0, 2.9, 3.1]);
    assert_ratio_matches_reference(&[1.1, 7.0, 1.3, 1.7], &[2.3, 7.0, 2.9, 7.0, 3.1]);
}

#[test]
fn optimized_ratio_matches_reference_at_and_above_the_mask_capacity() {
    let mut denominators = [2.0; 33];
    denominators[31] = 3.0;
    denominators[32] = 5.0;
    assert_ratio_matches_reference(&[3.0], &denominators[..32]);
    assert_ratio_matches_reference(&[2.0; 32], &denominators[..32]);
    assert_ratio_matches_reference(&[5.0, 2.0], &denominators);
}

#[test]
fn products_preserve_representable_results_despite_extreme_intermediate_scales() {
    for sign in [-1.0, 1.0] {
        assert_eq!(
            product_ratio(
                &[sign * power_of_two(1000), power_of_two(900)],
                &[power_of_two(950)]
            ),
            sign * power_of_two(950)
        );
        assert_eq!(
            product_ratio(
                &[sign * power_of_two(-1000), power_of_two(-900)],
                &[power_of_two(-950)]
            ),
            sign * power_of_two(-950)
        );
    }
}

#[test]
fn square_roots_are_taken_before_rounding_extreme_products_and_quotients() {
    for exponent in [-1000, -800, 0, 800, 1000] {
        let scale = power_of_two(exponent);
        assert_eq!(sqrt_product_ratio(&[scale, scale], &[]), scale);
    }
    let smallest = f64::from_bits(1);
    assert_eq!(sqrt_product_ratio(&[smallest, smallest], &[]), smallest);
    assert_eq!(
        sqrt_product_ratio(&[power_of_two(-800)], &[power_of_two(800)]),
        power_of_two(-800)
    );
    assert_eq!(
        sqrt_product_ratio(&[power_of_two(800)], &[power_of_two(-800)]),
        power_of_two(800)
    );
}

#[test]
fn identity_rescaling_preserves_bits_at_zero_subnormal_and_maximum_boundaries() {
    for value in [
        -0.0_f64,
        0.0,
        f64::from_bits(1),
        f64::MIN_POSITIVE,
        f64::MAX,
    ] {
        for factor in [f64::from_bits(1), 0.5, 2.0, f64::MAX] {
            assert_eq!(
                product_ratio(&[value, factor], &[factor]).to_bits(),
                value.to_bits()
            );
            assert_eq!(
                product_ratio(&[value, factor, factor], &[factor, factor]).to_bits(),
                value.to_bits()
            );
        }
    }
}

#[test]
fn only_the_final_result_can_overflow_or_round_to_zero() {
    let smallest = f64::from_bits(1);
    assert!(product_ratio(&[f64::MAX, 2.0], &[]).is_infinite());
    assert_eq!(product_ratio(&[smallest, 0.5], &[]), 0.0);
    assert_eq!(
        product_ratio(&[-smallest, 0.5], &[]).to_bits(),
        (-0.0_f64).to_bits()
    );
    assert_eq!(product_ratio(&[smallest, 1.5], &[]), 2.0 * smallest);
}

#[test]
fn product_equality_compares_exact_values_beyond_the_f64_exponent_range() {
    let tiny = f64::from_bits(1);
    for (left, right) in [
        ([tiny, 4.0 * tiny], [2.0 * tiny, 2.0 * tiny]),
        ([f64::MAX, 2.0], [2.0, f64::MAX]),
        (
            [power_of_two(1000), power_of_two(900)],
            [power_of_two(950); 2],
        ),
        (
            [power_of_two(-1000), power_of_two(-900)],
            [power_of_two(-950); 2],
        ),
        (
            [tiny, power_of_two(1000)],
            [power_of_two(-1000), power_of_two(926)],
        ),
    ] {
        assert!(products_equal(left, right));
    }
    assert!(!products_equal([tiny, tiny], [tiny, 2.0 * tiny]));
    assert!(!products_equal([f64::MAX, 2.0], [f64::MAX, 4.0]));
}

#[test]
fn product_equality_distinguishes_products_that_round_to_the_same_value() {
    let n = power_of_two(27);
    assert_eq!(n * n, (n - 1.0) * (n + 1.0));
    assert!(!products_equal([n, n], [n - 1.0, n + 1.0]));
}

#[test]
fn product_equality_handles_signs_and_both_signed_zeros() {
    for zero in [0.0, -0.0] {
        assert!(products_equal([zero, f64::MAX], [1.0, -zero]));
        assert!(!products_equal([zero, 1.0], [f64::from_bits(1), 1.0]));
    }
    assert!(products_equal([-2.0, -3.0], [1.0, 6.0]));
    assert!(products_equal([-2.0, 3.0], [2.0, -3.0]));
    assert!(!products_equal([-2.0, 3.0], [2.0, 3.0]));
}
