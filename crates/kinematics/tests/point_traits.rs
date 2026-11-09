use kinematics::{KinematicsError, Point};

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
        assert!(matches!(result, Err(KinematicsError::InvalidPoint)));
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
        assert_eq!(result, Err(KinematicsError::NumericalFailure));
    }
}
