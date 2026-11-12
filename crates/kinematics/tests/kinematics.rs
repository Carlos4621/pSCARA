use kinematics::{ElbowConfiguration, JointAngles, KinematicsError, Point, Scara};
use utilities::UtilitiesErrors;

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::{FRAC_PI_2, PI, TAU};

    const ABSOLUTE_TOLERANCE: f64 = 1e-10;

    fn point(x: f64, y: f64) -> Point {
        Point::new(x, y).expect("test point should be finite")
    }

    fn angles(first_theta: f64, second_theta: f64) -> JointAngles {
        JointAngles::new(first_theta, second_theta).expect("test angles should be finite")
    }

    fn forward(robot: &Scara, angles: JointAngles) -> Point {
        robot
            .forward(angles)
            .expect("finite test geometry should produce a finite point")
    }

    #[test]
    fn point_new_preserves_finite_coordinates_and_accessors() {
        for (x, y) in [
            (-f64::MAX, f64::MAX),
            (-f64::MIN_POSITIVE, f64::MIN_POSITIVE),
            (-f64::from_bits(1), f64::from_bits(1)),
            (-0.0, 0.0),
        ] {
            let point = Point::new(x, y).unwrap();
            assert_eq!(point.x().to_bits(), x.to_bits());
            assert_eq!(point.y().to_bits(), y.to_bits());
        }
    }

    #[test]
    fn point_new_rejects_every_non_finite_coordinate() {
        for invalid in [f64::NEG_INFINITY, f64::INFINITY, f64::NAN, -f64::NAN] {
            for (x, y) in [(invalid, 0.0), (0.0, invalid), (invalid, invalid)] {
                assert!(matches!(
                    Point::new(x, y),
                    Err(KinematicsError::Utilities(UtilitiesErrors::NonFiniteF64))
                ));
            }
        }
    }

    #[test]
    fn joint_angles_new_preserves_finite_values_and_accessors() {
        for (first_theta, second_theta) in [
            (-f64::MAX, f64::MAX),
            (-f64::MIN_POSITIVE, f64::MIN_POSITIVE),
            (-f64::from_bits(1), f64::from_bits(1)),
            (-0.0, 0.0),
        ] {
            let angles = JointAngles::new(first_theta, second_theta).unwrap();
            assert_eq!(angles.first_theta().to_bits(), first_theta.to_bits());
            assert_eq!(angles.second_theta().to_bits(), second_theta.to_bits());
        }
    }

    #[test]
    fn joint_angles_new_rejects_every_non_finite_component() {
        for invalid in [f64::NEG_INFINITY, f64::INFINITY, f64::NAN, -f64::NAN] {
            for (first_theta, second_theta) in [(invalid, 0.0), (0.0, invalid), (invalid, invalid)]
            {
                assert!(matches!(
                    JointAngles::new(first_theta, second_theta),
                    Err(KinematicsError::Utilities(UtilitiesErrors::NonFiniteF64))
                ));
            }
        }
    }

    #[test]
    fn joint_angles_closeness_accepts_identical_values_with_zero_tolerance() {
        for joint_angles in [
            angles(-f64::MAX, f64::MAX),
            angles(-f64::MIN_POSITIVE, f64::MIN_POSITIVE),
            angles(-f64::from_bits(1), f64::from_bits(1)),
            angles(-0.0, 0.0),
            angles(PI, -PI),
        ] {
            for zero in [0.0, -0.0] {
                assert_eq!(joint_angles.is_close_to_joint(joint_angles, zero), Ok(true));
            }
        }
    }

    #[test]
    fn joint_angles_closeness_includes_each_joint_tolerance_boundary() {
        let origin = angles(0.0, 0.0);

        for other in [
            angles(-1.0, -1.0),
            angles(-1.0, 1.0),
            angles(1.0, -1.0),
            angles(1.0, 1.0),
        ] {
            assert_eq!(origin.is_close_to_joint(other, 1.0), Ok(true));
            assert_eq!(other.is_close_to_joint(origin, 1.0), Ok(true));
            let just_below = f64::from_bits(1.0_f64.to_bits() - 1);
            assert_eq!(origin.is_close_to_joint(other, just_below), Ok(false));
        }
    }

    #[test]
    fn joint_angles_closeness_rejects_when_either_component_exceeds_tolerance() {
        let origin = angles(0.0, 0.0);
        let just_above_one = f64::from_bits(1.0_f64.to_bits() + 1);

        for outside in [
            angles(-just_above_one, 0.0),
            angles(just_above_one, 0.0),
            angles(0.0, -just_above_one),
            angles(0.0, just_above_one),
            angles(just_above_one, just_above_one),
        ] {
            assert_eq!(origin.is_close_to_joint(outside, 1.0), Ok(false));
            assert_eq!(outside.is_close_to_joint(origin, 1.0), Ok(false));
        }
    }

    #[test]
    fn joint_angles_closeness_is_component_wise_and_translation_invariant() {
        let origin = angles(0.0, 0.0);
        let diagonal = angles(0.75, -0.75);
        let translated_origin = angles(8.0, -16.0);
        let translated_diagonal = angles(8.75, -16.75);

        assert_eq!(origin.is_close_to_joint(diagonal, 1.0), Ok(true));
        assert!(libm::hypot(0.75, 0.75) > 1.0);
        assert_eq!(
            origin.is_close_to_joint(diagonal, 1.0),
            translated_origin.is_close_to_joint(translated_diagonal, 1.0)
        );
    }

    #[test]
    fn joint_angles_closeness_does_not_reduce_complete_turns() {
        let canonical = angles(0.25, -0.5);
        let another_turn = angles(0.25 + TAU, -0.5 - TAU);

        assert_eq!(
            canonical.is_close_to_joint(another_turn, ABSOLUTE_TOLERANCE),
            Ok(false)
        );
        assert_eq!(canonical.is_close_to_joint(another_turn, TAU), Ok(true));
        assert_eq!(
            canonical.is_close_to_joint(another_turn, 2.0 * TAU),
            Ok(true)
        );
    }

    #[test]
    fn joint_angles_closeness_returns_false_for_unrepresentable_finite_differences() {
        assert_eq!(
            angles(f64::MAX, 0.0).is_close_to_joint(angles(0.0, 0.0), f64::MAX),
            Ok(true)
        );
        for (a, b) in [
            (angles(f64::MAX, 0.0), angles(-f64::MAX, 0.0)),
            (angles(0.0, f64::MAX), angles(0.0, -f64::MAX)),
            (angles(f64::MAX, f64::MAX), angles(-f64::MAX, -f64::MAX)),
        ] {
            assert_eq!(a.is_close_to_joint(b, f64::MAX), Ok(false));
            assert_eq!(b.is_close_to_joint(a, f64::MAX), Ok(false));
        }
    }

    #[test]
    fn joint_angles_closeness_rejects_negative_or_non_finite_tolerances_first() {
        let identical = angles(0.0, 0.0);
        let extreme = angles(f64::MAX, -f64::MAX);

        for invalid in [
            f64::NEG_INFINITY,
            -f64::MAX,
            -1.0,
            -f64::from_bits(1),
            f64::NAN,
            f64::INFINITY,
        ] {
            assert_eq!(
                identical.is_close_to_joint(identical, invalid),
                Err(KinematicsError::InvalidTolerance)
            );
            assert_eq!(
                extreme.is_close_to_joint(angles(-f64::MAX, f64::MAX), invalid),
                Err(KinematicsError::InvalidTolerance)
            );
        }
    }

    #[test]
    fn new_accepts_strictly_positive_finite_arm_lengths_at_all_scales() {
        for lengths in [
            (150.0, 100.0),
            (f64::from_bits(1), f64::from_bits(1)),
            (f64::MAX, f64::MAX),
            (f64::MAX, f64::from_bits(1)),
            (f64::from_bits(1), f64::MAX),
        ] {
            assert!(Scara::new(lengths.0, lengths.1).is_ok());
        }
    }

    #[test]
    fn new_rejects_negative_or_non_finite_arm_lengths() {
        for lengths in [
            (-1.0, 100.0),
            (150.0, -1.0),
            (f64::INFINITY, 100.0),
            (150.0, f64::NEG_INFINITY),
            (f64::NAN, 100.0),
            (150.0, f64::NAN),
        ] {
            assert!(matches!(
                Scara::new(lengths.0, lengths.1),
                Err(KinematicsError::InvalidArmLength)
            ));
        }
    }

    #[test]
    fn forward_with_extended_arms() {
        let robot = Scara::new(150.0, 150.0).unwrap();
        let result = forward(&robot, angles(0.0, 0.0));

        assert!((result.x() - 300.0).abs() < 1e-6);
        assert!(result.y().abs() < 1e-6);
    }

    #[test]
    fn forward_with_perpendicular_arms() {
        let robot = Scara::new(100.0, 50.0).unwrap();

        let result = forward(&robot, angles(FRAC_PI_2, -FRAC_PI_2));

        assert!((result.x() - 50.0).abs() < 1e-6);
        assert!((result.y() - 100.0).abs() < 1e-6);
    }

    #[test]
    fn forward_uses_second_angle_relative_to_first() {
        let robot = Scara::new(100.0, 50.0).unwrap();

        let result = forward(&robot, angles(FRAC_PI_2, FRAC_PI_2));

        assert!((result.x() + 50.0).abs() < 1e-6);
        assert!((result.y() - 100.0).abs() < 1e-6);
    }

    #[test]
    fn point_distance_from_origin_is_correct() {
        assert_eq!(point(3.0, 4.0).distance_from_origin(), Ok(5.0));
    }

    #[test]
    fn point_distance_from_origin_handles_signed_zero_subnormals_and_finite_extremes() {
        for (x, y, expected) in [
            (-0.0, 0.0, 0.0),
            (f64::from_bits(1), 0.0, f64::from_bits(1)),
            (0.0, -f64::MAX, f64::MAX),
        ] {
            assert_eq!(point(x, y).distance_from_origin(), Ok(expected));
        }
        assert_eq!(
            point(f64::MAX, f64::MAX).distance_from_origin(),
            Err(KinematicsError::NumericalFailure)
        );
    }

    #[test]
    fn joint_angles_closeness_matches_independent_integer_joint_limits() {
        for a1 in -2_i32..=2 {
            for a2 in -2_i32..=2 {
                for b1 in -2_i32..=2 {
                    for b2 in -2_i32..=2 {
                        for tolerance in 0_u32..=4 {
                            let expected =
                                a1.abs_diff(b1) <= tolerance && a2.abs_diff(b2) <= tolerance;
                            assert_eq!(
                                angles(f64::from(a1), f64::from(a2)).is_close_to_joint(
                                    angles(f64::from(b1), f64::from(b2)),
                                    f64::from(tolerance),
                                ),
                                Ok(expected)
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn utilities_errors_convert_without_losing_the_original_variant() {
        fn propagate(error: UtilitiesErrors) -> Result<(), KinematicsError> {
            Err(error)?;
            Ok(())
        }
        for error in [
            UtilitiesErrors::NonFiniteF64,
            UtilitiesErrors::NonPositiveF64,
            UtilitiesErrors::NonNegativeF64,
            UtilitiesErrors::NonZeroFiniteF64,
            UtilitiesErrors::NonZeroFinitePositiveF64,
        ] {
            assert_eq!(
                KinematicsError::from(error),
                KinematicsError::Utilities(error)
            );
            assert_eq!(propagate(error), Err(KinematicsError::Utilities(error)));
        }
    }

    #[test]
    fn point_and_joint_angle_constructors_reject_signed_nan_payloads() {
        for sign in [0, 1_u64 << 63] {
            for payload in [1, 2, 1_u64 << 51, (1_u64 << 52) - 1] {
                let nan = f64::from_bits(sign | (0x7ff_u64 << 52) | payload);
                for (x, y) in [(nan, 0.0), (0.0, nan)] {
                    assert_eq!(
                        Point::new(x, y),
                        Err(KinematicsError::Utilities(UtilitiesErrors::NonFiniteF64))
                    );
                    assert!(matches!(
                        JointAngles::new(x, y),
                        Err(KinematicsError::Utilities(UtilitiesErrors::NonFiniteF64))
                    ));
                }
            }
        }
    }

    #[test]
    fn new_rejects_both_signed_zeros_in_either_arm() {
        for zero in [-0.0, 0.0] {
            for lengths in [(zero, 100.0), (100.0, zero), (zero, zero)] {
                assert!(matches!(
                    Scara::new(lengths.0, lengths.1),
                    Err(KinematicsError::InvalidArmLength)
                ));
            }
        }
    }

    #[test]
    fn forward_reports_numerical_failure_when_absolute_angle_sum_overflows() {
        let robot = Scara::new(100.0, 60.0).unwrap();
        assert_eq!(
            robot.forward(angles(f64::MAX, f64::MAX)),
            Err(KinematicsError::NumericalFailure)
        );
    }

    #[test]
    fn inverse_reaches_a_known_target_when_the_second_arm_is_longer() {
        let robot = Scara::new(60.0, 100.0).unwrap();
        assert_point_round_trip(&robot, point(60.0, 100.0));
    }

    #[test]
    fn point_distance_to_point_is_symmetric_and_matches_known_distances() {
        for (first, second, expected) in [
            (point(0.0, 0.0), point(0.0, 0.0), 0.0),
            (point(1.0, 2.0), point(4.0, 6.0), 5.0),
            (point(-3.0, -4.0), point(0.0, 0.0), 5.0),
            (point(-1.0, 2.0), point(2.0, -2.0), 5.0),
        ] {
            assert_eq!(first.distance_to_point(second), Ok(expected));
            assert_eq!(second.distance_to_point(first), Ok(expected));
        }
    }

    #[test]
    fn point_distance_to_point_handles_representable_extreme_scales() {
        let smallest = f64::from_bits(1);

        assert_eq!(
            point(0.0, 0.0).distance_to_point(point(f64::MAX, 0.0)),
            Ok(f64::MAX)
        );
        let subnormal_distance = point(0.0, 0.0)
            .distance_to_point(point(smallest, smallest))
            .unwrap();
        assert!(subnormal_distance.is_finite());
        assert!(subnormal_distance > 0.0);
    }

    #[test]
    fn point_distance_to_point_reports_unrepresentable_differences() {
        for (first, second) in [
            (point(-f64::MAX, 0.0), point(f64::MAX, 0.0)),
            (point(0.0, -f64::MAX), point(0.0, f64::MAX)),
            (point(-f64::MAX, -f64::MAX), point(f64::MAX, f64::MAX)),
        ] {
            assert_eq!(
                first.distance_to_point(second),
                Err(KinematicsError::NumericalFailure)
            );
            assert_eq!(
                second.distance_to_point(first),
                Err(KinematicsError::NumericalFailure)
            );
        }
    }

    #[test]
    fn point_closeness_uses_inclusive_euclidean_distance() {
        let origin = point(0.0, 0.0);
        let three_four_five = point(3.0, 4.0);

        for zero in [0.0, -0.0] {
            assert_eq!(origin.is_close_to_point(origin, zero), Ok(true));
        }
        assert_eq!(
            origin.is_close_to_point(origin, f64::from_bits(1)),
            Ok(true)
        );
        assert_eq!(origin.is_close_to_point(three_four_five, 5.0), Ok(true));
        assert_eq!(
            origin.is_close_to_point(three_four_five, f64::from_bits(5.0_f64.to_bits() - 1)),
            Ok(false)
        );
        assert_eq!(
            origin.is_close_to_point(three_four_five, 5.000_000_000_1),
            Ok(true)
        );
        assert_eq!(
            three_four_five.is_close_to_point(origin, 5.000_000_000_1),
            Ok(true)
        );
    }

    #[test]
    fn point_closeness_rejects_negative_or_non_finite_tolerances() {
        let origin = point(0.0, 0.0);

        for invalid in [
            f64::NEG_INFINITY,
            -f64::MAX,
            -1.0,
            -f64::from_bits(1),
            f64::NAN,
            f64::INFINITY,
        ] {
            assert_eq!(
                origin.is_close_to_point(origin, invalid),
                Err(KinematicsError::InvalidTolerance)
            );
            assert_eq!(
                point(-f64::MAX, 0.0).is_close_to_point(point(f64::MAX, 0.0), invalid),
                Err(KinematicsError::InvalidTolerance)
            );
        }
    }

    #[test]
    fn point_closeness_propagates_unrepresentable_distance_errors() {
        assert_eq!(
            point(-f64::MAX, 0.0).is_close_to_point(point(f64::MAX, 0.0), f64::MAX),
            Err(KinematicsError::NumericalFailure)
        );
    }

    #[test]
    fn forward_reports_numerical_failure_when_the_result_overflows() {
        let robot = Scara::new(f64::MAX, f64::MAX).unwrap();

        assert!(matches!(
            robot.forward(angles(0.0, 0.0)),
            Err(KinematicsError::NumericalFailure)
        ));
    }

    fn configurations() -> [ElbowConfiguration; 2] {
        [ElbowConfiguration::Positive, ElbowConfiguration::Negative]
    }

    fn assert_point_close(actual: Point, expected: Point, tolerance: f64) {
        assert!(
            (actual.x() - expected.x()).abs() <= tolerance
                && (actual.y() - expected.y()).abs() <= tolerance,
            "expected {expected:?}, got {actual:?}, tolerance {tolerance}"
        );
    }

    fn assert_angle_close(actual: f64, expected: f64) {
        let difference = actual - expected;
        let wrapped_difference = libm::atan2(libm::sin(difference), libm::cos(difference));
        assert!(
            wrapped_difference.abs() <= ABSOLUTE_TOLERANCE,
            "expected angle {expected} modulo 2*pi, got {actual}"
        );
    }

    fn assert_inverse(
        robot: &Scara,
        point: Point,
        configuration: ElbowConfiguration,
    ) -> JointAngles {
        let positive = matches!(configuration, ElbowConfiguration::Positive);
        let angles = robot.inverse(point, configuration).unwrap_or_else(|error| {
            panic!(
                "reachable point {point:?}, robot {robot:?}, branch {configuration:?}: {error:?}"
            )
        });
        assert!(angles.first_theta().is_finite(), "{point:?}: {angles:?}");
        assert!(angles.second_theta().is_finite(), "{point:?}: {angles:?}");
        if positive {
            assert!((0.0..=PI).contains(&angles.second_theta()), "{angles:?}");
            assert!(angles.second_theta().is_sign_positive(), "{angles:?}");
        } else {
            assert!((-PI..=0.0).contains(&angles.second_theta()), "{angles:?}");
            assert!(angles.second_theta().is_sign_negative(), "{angles:?}");
        }
        angles
    }

    fn assert_point_round_trip(robot: &Scara, point: Point) {
        for configuration in configurations() {
            let angles = assert_inverse(robot, point, configuration);
            assert_point_close(forward(robot, angles), point, ABSOLUTE_TOLERANCE);
        }
    }

    #[test]
    fn inverse_returns_both_expected_solutions_for_a_known_point() {
        let robot = Scara::new(100.0, 50.0).unwrap();
        let point = point(100.0, 50.0);

        let positive = assert_inverse(&robot, point, ElbowConfiguration::Positive);
        assert_angle_close(positive.first_theta(), 0.0);
        assert_angle_close(positive.second_theta(), FRAC_PI_2);

        let negative = assert_inverse(&robot, point, ElbowConfiguration::Negative);
        assert_angle_close(negative.first_theta(), 2.0 * libm::atan(0.5));
        assert_angle_close(negative.second_theta(), -FRAC_PI_2);
        assert_point_round_trip(&robot, point);
    }

    #[test]
    fn inverse_forward_round_trip_covers_both_elbow_configurations() {
        let robot = Scara::new(100.0, 60.0).unwrap();
        for (first_theta, second_theta) in [
            (-2.8, -2.2),
            (-1.3, 0.7),
            (0.0, 2.4),
            (1.7, -1.1),
            (3.0, 3.0),
        ] {
            assert_point_round_trip(&robot, forward(&robot, angles(first_theta, second_theta)));
        }
    }

    #[test]
    fn forward_inverse_round_trip_covers_a_grid_of_angles_and_arm_lengths() {
        let angle_values = [-PI, -2.25, -FRAC_PI_2, -0.1, 0.0, 0.1, FRAC_PI_2, 2.25, PI];
        for (first_length, second_length) in [
            (1.0, 1.0),
            (100.0, 60.0),
            (60.0, 100.0),
            (0.001, 500.0),
            (500.0, 0.001),
        ] {
            let robot = Scara::new(first_length, second_length).unwrap();
            for first_theta in angle_values {
                for second_theta in angle_values {
                    assert_point_round_trip(
                        &robot,
                        forward(&robot, angles(first_theta, second_theta)),
                    );
                }
            }
        }
    }

    #[test]
    fn inverse_recovers_angles_modulo_full_turns_for_the_matching_branch() {
        // Singular configurations are covered by position tests: the original
        // first angle is not uniquely recoverable at the origin with equal arms.
        for (first_length, second_length) in [(1.0, 1.0), (100.0, 60.0), (60.0, 100.0)] {
            let robot = Scara::new(first_length, second_length).unwrap();
            for first_theta in [-3.5, -FRAC_PI_2, 0.0, 2.5, 3.5] {
                for second_theta in [-3.0_f64, -FRAC_PI_2, -0.1, 0.1, FRAC_PI_2, 3.0] {
                    for turns in [-2.0, 0.0, 2.0] {
                        let original =
                            angles(first_theta + turns * TAU, second_theta + turns * TAU);
                        let configuration = if second_theta > 0.0 {
                            ElbowConfiguration::Positive
                        } else {
                            ElbowConfiguration::Negative
                        };
                        let recovered =
                            assert_inverse(&robot, forward(&robot, original), configuration);
                        assert_angle_close(recovered.first_theta(), original.first_theta());
                        assert_angle_close(recovered.second_theta(), original.second_theta());
                    }
                }
            }
        }
    }

    #[test]
    fn inverse_forward_round_trip_covers_independently_chosen_workspace_points() {
        for (first_length, second_length) in [(1.0_f64, 1.0_f64), (100.0, 60.0), (60.0, 100.0)] {
            let robot = Scara::new(first_length, second_length).unwrap();
            let inner = (first_length - second_length).abs();
            let outer = first_length + second_length;
            for fraction in [0.0, 0.1, 0.5, 0.9, 1.0] {
                let radius = inner + (outer - inner) * fraction;
                for direction in [-PI, -2.4, -FRAC_PI_2, -0.3, 0.0, 0.7, FRAC_PI_2, 2.4, PI] {
                    assert_point_round_trip(
                        &robot,
                        point(radius * libm::cos(direction), radius * libm::sin(direction)),
                    );
                }
            }
        }
    }

    #[test]
    fn inverse_accepts_outer_and_inner_workspace_boundaries() {
        for (first_length, second_length) in [(100.0, 60.0), (60.0, 100.0)] {
            let robot = Scara::new(first_length, second_length).unwrap();
            for (x, y) in [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)] {
                for (radius, expected_second) in [(160.0, 0.0), (40.0, PI)] {
                    let point = point(x * radius, y * radius);
                    for configuration in configurations() {
                        let angles = assert_inverse(&robot, point, configuration);
                        assert!(
                            (angles.second_theta().abs() - expected_second).abs()
                                <= ABSOLUTE_TOLERANCE
                        );
                        assert_point_close(forward(&robot, angles), point, ABSOLUTE_TOLERANCE);
                    }
                }
            }
        }
    }

    #[test]
    fn inverse_at_origin_with_equal_arms_returns_a_folded_solution_for_each_branch() {
        let robot = Scara::new(75.0, 75.0).unwrap();
        // Signed zeros can change atan2's branch without changing the target.
        for x in [-0.0, 0.0] {
            for y in [-0.0, 0.0] {
                for configuration in configurations() {
                    let point = point(x, y);
                    let angles = assert_inverse(&robot, point, configuration);
                    assert_eq!(angles.second_theta().abs(), PI);
                    assert_point_close(forward(&robot, angles), point, ABSOLUTE_TOLERANCE);
                }
            }
        }
    }

    #[test]
    fn inverse_clamps_points_just_outside_both_boundaries_within_roundoff_tolerance() {
        let robot = Scara::new(100.0, 60.0).unwrap();
        // These offsets are smaller than the roundoff allowance at this scale.
        // The nearest boundary point, rather than the out-of-workspace input,
        // is the geometric solution after clamping.
        for (radius, boundary) in [(160.0 + 1e-12, 160.0), (40.0 - 1e-12, 40.0)] {
            for (x, y) in [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)] {
                for configuration in configurations() {
                    let angles =
                        assert_inverse(&robot, point(x * radius, y * radius), configuration);
                    assert_point_close(
                        forward(&robot, angles),
                        point(x * boundary, y * boundary),
                        1e-12,
                    );
                }
            }
        }
    }

    #[test]
    fn inverse_rejects_points_outside_workspace_beyond_roundoff_tolerance() {
        for (first_length, second_length) in [(100.0, 60.0), (60.0, 100.0)] {
            let robot = Scara::new(first_length, second_length).unwrap();
            for radius in [0.0, 39.0, 40.0 - 1e-10, 160.0 + 1e-10, 161.0] {
                for (x, y) in [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)] {
                    for configuration in configurations() {
                        assert!(
                            matches!(
                                robot.inverse(point(x * radius, y * radius), configuration),
                                Err(KinematicsError::UnreachablePoint)
                            ),
                            "radius {radius} should be unreachable"
                        );
                    }
                }
            }
        }
    }

    fn assert_scale_invariance(scale: f64) {
        let robot = Scara::new(scale, scale).unwrap();
        for configuration in configurations() {
            let angles = assert_inverse(&robot, point(scale, 0.0), configuration);
            assert!((angles.first_theta().abs() - PI / 3.0).abs() <= ABSOLUTE_TOLERANCE);
            assert!((angles.second_theta().abs() - 2.0 * PI / 3.0).abs() <= ABSOLUTE_TOLERANCE);
            let recovered = forward(&robot, angles);
            assert_point_close(
                point(recovered.x() / scale, recovered.y() / scale),
                point(1.0, 0.0),
                ABSOLUTE_TOLERANCE,
            );
        }
    }

    #[test]
    fn inverse_is_scale_invariant_for_moderate_arm_lengths() {
        for scale in [1e-100, 1e-6, 1.0, 1e6, 1e100] {
            assert_scale_invariance(scale);
        }
    }

    #[test]
    fn inverse_reports_numerical_failure_when_normalization_erases_an_arm() {
        let smallest_length = f64::from_bits(1);
        let largest_length = f64::MAX;

        for (first_arm_length, second_arm_length) in [
            (smallest_length, largest_length),
            (largest_length, smallest_length),
        ] {
            let robot = Scara::new(first_arm_length, second_arm_length).unwrap();

            for point in [
                point(largest_length, 0.0),
                point(-largest_length, 0.0),
                point(0.0, largest_length),
                point(0.0, -largest_length),
            ] {
                for configuration in configurations() {
                    let actual = robot.inverse(point, configuration);
                    assert!(
                        matches!(actual, Err(KinematicsError::NumericalFailure)),
                        "robot {robot:?}, point {point:?}, branch {configuration:?}: {actual:?}"
                    );
                }
            }
        }
    }

    // Normalization keeps uniformly scaled geometries away from overflow and underflow.
    #[test]
    fn inverse_returns_finite_angles_when_squared_lengths_overflow() {
        assert_scale_invariance(1e200);
    }

    #[test]
    fn inverse_returns_finite_angles_when_squared_lengths_underflow() {
        assert_scale_invariance(1e-200);
    }

    #[test]
    fn inverse_normalizes_before_testing_an_overflowing_arm_sum_and_target_radius() {
        let robot = Scara::new(f64::MAX, f64::MAX).unwrap();
        let target = point(f64::MAX, f64::MAX);
        assert_eq!(
            target.distance_from_origin(),
            Err(KinematicsError::NumericalFailure)
        );

        let positive = assert_inverse(&robot, target, ElbowConfiguration::Positive);
        assert_angle_close(positive.first_theta(), 0.0);
        assert_angle_close(positive.second_theta(), FRAC_PI_2);
        let negative = assert_inverse(&robot, target, ElbowConfiguration::Negative);
        assert_angle_close(negative.first_theta(), FRAC_PI_2);
        assert_angle_close(negative.second_theta(), -FRAC_PI_2);
    }

    #[test]
    fn inverse_classifies_unreachable_extreme_targets_in_every_quadrant() {
        let robot = Scara::new(f64::MAX / 4.0, f64::MAX / 4.0).unwrap();
        for x in [-f64::MAX, f64::MAX] {
            for y in [-f64::MAX, f64::MAX] {
                for configuration in configurations() {
                    assert!(matches!(
                        robot.inverse(point(x, y), configuration),
                        Err(KinematicsError::UnreachablePoint)
                    ));
                }
            }
        }
    }

    #[test]
    fn inverse_roundoff_tolerance_stays_relative_near_the_maximum_finite_scale() {
        let arm = f64::MAX / 4.0;
        let outer = arm + arm;
        let inside_tolerance = f64::from_bits(outer.to_bits() + 1);
        let outside_tolerance = outer * (1.0 + 1024.0 * f64::EPSILON);
        let robot = Scara::new(arm, arm).unwrap();
        for sign in [-1.0, 1.0] {
            for configuration in configurations() {
                let angles =
                    assert_inverse(&robot, point(sign * inside_tolerance, 0.0), configuration);
                assert_eq!(angles.second_theta().abs(), 0.0);
                assert!(matches!(
                    robot.inverse(point(sign * outside_tolerance, 0.0), configuration),
                    Err(KinematicsError::UnreachablePoint)
                ));
            }
        }
    }
}
