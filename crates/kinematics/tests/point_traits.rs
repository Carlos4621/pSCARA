use kinematics::{KinematicsError, Point};
use utilities::UtilitiesErrors;

fn point(x: f64, y: f64) -> Point {
    Point::new(x, y).expect("test point should be finite")
}

fn assert_point_coordinates(actual: Point, expected_x: f64, expected_y: f64) {
    assert_eq!(actual.x(), expected_x);
    assert_eq!(actual.y(), expected_y);
}

#[test]
fn add_sums_each_coordinate() {
    for (left, right, expected) in [
        ((1.5, -2.0), (3.25, 4.5), (4.75, 2.5)),
        ((-8.0, 5.0), (8.0, -5.0), (0.0, 0.0)),
        (
            (f64::MIN_POSITIVE, 0.0),
            (f64::MIN_POSITIVE, 0.0),
            (2.0 * f64::MIN_POSITIVE, 0.0),
        ),
    ] {
        let result = (point(left.0, left.1) + point(right.0, right.1)).unwrap();
        assert_point_coordinates(result, expected.0, expected.1);
    }
}

#[test]
fn add_is_commutative_for_exactly_representable_values() {
    let first = point(12.5, -7.25);
    let second = point(-2.0, 3.5);

    let first_plus_second = (first + second).unwrap();
    let second_plus_first = (second + first).unwrap();

    assert_point_coordinates(
        first_plus_second,
        second_plus_first.x(),
        second_plus_first.y(),
    );
}

#[test]
fn sub_subtracts_each_coordinate_in_operand_order() {
    let first_minus_second = (point(7.5, -2.0) - point(3.0, 4.5)).unwrap();
    let second_minus_first = (point(3.0, 4.5) - point(7.5, -2.0)).unwrap();

    assert_point_coordinates(first_minus_second, 4.5, -6.5);
    assert_point_coordinates(second_minus_first, -4.5, 6.5);
}

#[test]
fn add_and_sub_compose_to_recover_the_original_point() {
    let original = point(12.5, -7.25);
    let displacement = point(-2.0, 3.5);

    let translated = (original + displacement).unwrap();
    let recovered = (translated - displacement).unwrap();

    assert_point_coordinates(recovered, original.x(), original.y());
}

#[test]
fn add_and_sub_reject_non_finite_results() {
    for result in [
        point(f64::MAX, 0.0) + point(f64::MAX, 0.0),
        point(0.0, f64::MAX) + point(0.0, f64::MAX),
        point(f64::MAX, 0.0) - point(-f64::MAX, 0.0),
        point(0.0, -f64::MAX) - point(0.0, f64::MAX),
    ] {
        assert_eq!(
            result,
            Err(KinematicsError::Utilities(UtilitiesErrors::NonFiniteF64))
        );
    }
}

#[test]
fn mul_calculates_the_dot_product() {
    for (left, right, expected) in [
        ((3.0, 4.0), (5.0, 6.0), 39.0),
        ((1.0, 0.0), (0.0, 1.0), 0.0),
        ((-2.0, 3.0), (4.0, -5.0), -23.0),
        ((3.0, 4.0), (3.0, 4.0), 25.0),
    ] {
        assert_eq!(
            point(left.0, left.1) * point(right.0, right.1),
            Ok(expected)
        );
    }
}

#[test]
fn dot_product_is_commutative_for_exactly_representable_values() {
    let first = point(1.5, -2.0);
    let second = point(3.0, 4.5);

    assert_eq!(first * second, second * first);
}

#[test]
fn dot_product_accepts_finite_underflow() {
    assert_eq!(
        point(f64::MIN_POSITIVE, f64::MIN_POSITIVE) * point(f64::MIN_POSITIVE, -f64::MIN_POSITIVE),
        Ok(0.0)
    );
}

#[test]
fn dot_product_reports_non_finite_intermediate_or_final_results() {
    for result in [
        point(f64::MAX, 0.0) * point(2.0, 0.0),
        point(f64::MAX, f64::MAX) * point(1.0, 1.0),
        point(f64::MAX, f64::MAX) * point(f64::MAX, -f64::MAX),
    ] {
        assert_eq!(
            result,
            Err(KinematicsError::Utilities(UtilitiesErrors::NonFiniteF64))
        );
    }
}

#[test]
fn negation_flips_only_coordinate_sign_bits_and_is_an_involution() {
    for exponent in 0_u64..0x7ff {
        for mantissa in [0, 1, 1_u64 << 51, (1_u64 << 52) - 1] {
            for sign in [0, 1_u64 << 63] {
                let raw = f64::from_bits(sign | (exponent << 52) | mantissa);
                let original = point(raw, -raw);
                let negated = -original;
                assert_eq!(negated.x().to_bits(), raw.to_bits() ^ (1_u64 << 63));
                assert_eq!(negated.y().to_bits(), raw.to_bits());
                assert_eq!((-negated).x().to_bits(), original.x().to_bits());
                assert_eq!((-negated).y().to_bits(), original.y().to_bits());
            }
        }
    }
}

#[test]
fn scale_supports_positive_negative_and_zero_factors() {
    let original = point(3.0, -4.0);
    for (factor, expected) in [(2.0, (6.0, -8.0)), (-0.5, (-1.5, 2.0)), (1.0, (3.0, -4.0))] {
        assert_point_coordinates(original.scale(factor).unwrap(), expected.0, expected.1);
    }
    for zero in [0.0_f64, -0.0] {
        let scaled = original.scale(zero).unwrap();
        assert_eq!(scaled.x().to_bits(), (3.0 * zero).to_bits());
        assert_eq!(scaled.y().to_bits(), (-4.0 * zero).to_bits());
    }
    assert_point_coordinates(original, 3.0, -4.0);
}

#[test]
fn scale_rejects_nonfinite_factors_before_multiplying_even_zero_coordinates() {
    for factor in [
        f64::NAN,
        -f64::NAN,
        f64::from_bits(0x7ff0_0000_0000_0001),
        f64::INFINITY,
        f64::NEG_INFINITY,
    ] {
        for original in [point(0.0, 0.0), point(f64::MAX, -f64::MAX)] {
            assert_eq!(original.scale(factor), Err(KinematicsError::InvalidScale));
        }
    }
}

#[test]
fn scale_maps_overflow_in_either_coordinate_to_numerical_failure() {
    for original in [
        point(f64::MAX, 0.0),
        point(0.0, f64::MAX),
        point(-f64::MAX, -f64::MAX),
    ] {
        for factor in [2.0, -2.0] {
            assert_eq!(
                original.scale(factor),
                Err(KinematicsError::NumericalFailure)
            );
        }
    }
}

#[test]
fn scale_accepts_subnormal_results_and_underflow_to_signed_zero() {
    let normal = point(f64::MIN_POSITIVE, -f64::MIN_POSITIVE);
    let subnormal = normal.scale(0.5).unwrap();
    assert_eq!(subnormal.x(), f64::MIN_POSITIVE / 2.0);
    assert_eq!(subnormal.y(), -f64::MIN_POSITIVE / 2.0);

    let smallest = f64::from_bits(1);
    for factor in [0.5, -0.5] {
        let zero = point(smallest, -smallest).scale(factor).unwrap();
        assert_eq!(zero.x().to_bits(), (smallest * factor).to_bits());
        assert_eq!(zero.y().to_bits(), (-smallest * factor).to_bits());
    }
}

#[test]
fn scaling_composes_with_translation_and_dot_product() {
    let a = point(3.0, -4.0);
    let b = point(-2.0, 1.0);
    let scaled_sum = (a + b).unwrap().scale(2.0).unwrap();
    let sum_of_scaled = (a.scale(2.0).unwrap() + b.scale(2.0).unwrap()).unwrap();
    assert_eq!(scaled_sum, sum_of_scaled);
    assert_eq!(a.scale(2.0).unwrap() * b, Ok(2.0 * (a * b).unwrap()));
}
