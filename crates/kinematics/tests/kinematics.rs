use kinematics::{ElbowConfiguration, JointAngles, KinematicsError, Point, Scara};

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
                    Err(KinematicsError::InvalidPoint)
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
                    Err(KinematicsError::InvalidAngle)
                ));
            }
        }
    }

    #[test]
    fn new_accepts_positive_finite_arm_lengths() {
        assert!(Scara::new(150.0, 100.0).is_ok());
    }

    #[test]
    fn new_rejects_non_positive_or_non_finite_arm_lengths() {
        for lengths in [
            (0.0, 100.0),
            (-1.0, 100.0),
            (150.0, 0.0),
            (150.0, -1.0),
            (f64::INFINITY, 100.0),
            (150.0, f64::NEG_INFINITY),
            (f64::NAN, 100.0),
            (150.0, f64::NAN),
        ] {
            assert!(matches!(
                Scara::new(lengths.0, lengths.1),
                Err(KinematicsError::ArmWithInvalidLength)
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
        assert!((point(3.0, 4.0).distance_from_origin() - 5.0).abs() < ABSOLUTE_TOLERANCE);
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
    fn forward_reports_invalid_point_when_the_result_overflows() {
        let robot = Scara::new(f64::MAX, f64::MAX).unwrap();

        assert!(matches!(
            robot.forward(angles(0.0, 0.0)),
            Err(KinematicsError::InvalidPoint)
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
        let angles = robot
            .inverse(point, configuration)
            .expect("point should be reachable");
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
                    assert!(matches!(
                        robot.inverse(point, configuration),
                        Err(KinematicsError::NumericalFailure)
                    ));
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
}
