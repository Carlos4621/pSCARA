use super::*;

fn power_of_two(exponent: i32) -> f64 {
    libm::scalbn(1.0, exponent)
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
