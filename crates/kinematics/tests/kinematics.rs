use kinematics::{ElbowConfiguration, JointAngles, KinematicsError, Point, Scara};

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::{FRAC_PI_2, PI, TAU};

    const ABSOLUTE_TOLERANCE: f64 = 1e-10;

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

        let angles = JointAngles {
            first_theta: 0.0,
            second_theta: 0.0,
        };

        let result = robot.forward(angles);

        assert!((result.x - 300.0).abs() < 1e-6);
        assert!(result.y.abs() < 1e-6);
    }

    #[test]
    fn forward_with_perpendicular_arms() {
        let robot = Scara::new(100.0, 50.0).unwrap();

        let result = robot.forward(JointAngles {
            first_theta: core::f64::consts::FRAC_PI_2,
            second_theta: -core::f64::consts::FRAC_PI_2,
        });

        assert!((result.x - 50.0).abs() < 1e-6);
        assert!((result.y - 100.0).abs() < 1e-6);
    }

    #[test]
    fn forward_uses_second_angle_relative_to_first() {
        let robot = Scara::new(100.0, 50.0).unwrap();

        let result = robot.forward(JointAngles {
            first_theta: core::f64::consts::FRAC_PI_2,
            second_theta: core::f64::consts::FRAC_PI_2,
        });

        assert!((result.x + 50.0).abs() < 1e-6);
        assert!((result.y - 100.0).abs() < 1e-6);
    }

    #[test]
    fn point_distance_from_origin_is_correct() {
        assert!((Point { x: 3.0, y: 4.0 }.distance_from_origin() - 5.0).abs() < ABSOLUTE_TOLERANCE);
    }

    fn configurations() -> [ElbowConfiguration; 2] {
        [ElbowConfiguration::Positive, ElbowConfiguration::Negative]
    }

    fn assert_point_close(actual: Point, expected: Point, tolerance: f64) {
        assert!(
            (actual.x - expected.x).abs() <= tolerance
                && (actual.y - expected.y).abs() <= tolerance,
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
        assert!(angles.first_theta.is_finite(), "{point:?}: {angles:?}");
        assert!(angles.second_theta.is_finite(), "{point:?}: {angles:?}");
        if positive {
            assert!((0.0..=PI).contains(&angles.second_theta), "{angles:?}");
            assert!(angles.second_theta.is_sign_positive(), "{angles:?}");
        } else {
            assert!((-PI..=0.0).contains(&angles.second_theta), "{angles:?}");
            assert!(angles.second_theta.is_sign_negative(), "{angles:?}");
        }
        angles
    }

    fn assert_point_round_trip(robot: &Scara, point: Point) {
        for configuration in configurations() {
            let angles = assert_inverse(robot, point, configuration);
            assert_point_close(robot.forward(angles), point, ABSOLUTE_TOLERANCE);
        }
    }

    #[test]
    fn inverse_returns_both_expected_solutions_for_a_known_point() {
        let robot = Scara::new(100.0, 50.0).unwrap();
        let point = Point { x: 100.0, y: 50.0 };

        let positive = assert_inverse(&robot, point, ElbowConfiguration::Positive);
        assert_angle_close(positive.first_theta, 0.0);
        assert_angle_close(positive.second_theta, FRAC_PI_2);

        let negative = assert_inverse(&robot, point, ElbowConfiguration::Negative);
        assert_angle_close(negative.first_theta, 2.0 * libm::atan(0.5));
        assert_angle_close(negative.second_theta, -FRAC_PI_2);
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
            assert_point_round_trip(
                &robot,
                robot.forward(JointAngles {
                    first_theta,
                    second_theta,
                }),
            );
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
                        robot.forward(JointAngles {
                            first_theta,
                            second_theta,
                        }),
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
                        let original = JointAngles {
                            first_theta: first_theta + turns * TAU,
                            second_theta: second_theta + turns * TAU,
                        };
                        let configuration = if second_theta > 0.0 {
                            ElbowConfiguration::Positive
                        } else {
                            ElbowConfiguration::Negative
                        };
                        let recovered =
                            assert_inverse(&robot, robot.forward(original), configuration);
                        assert_angle_close(recovered.first_theta, original.first_theta);
                        assert_angle_close(recovered.second_theta, original.second_theta);
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
                        Point {
                            x: radius * libm::cos(direction),
                            y: radius * libm::sin(direction),
                        },
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
                    let point = Point {
                        x: x * radius,
                        y: y * radius,
                    };
                    for configuration in configurations() {
                        let angles = assert_inverse(&robot, point, configuration);
                        assert!(
                            (angles.second_theta.abs() - expected_second).abs()
                                <= ABSOLUTE_TOLERANCE
                        );
                        assert_point_close(robot.forward(angles), point, ABSOLUTE_TOLERANCE);
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
                    let angles = assert_inverse(&robot, Point { x, y }, configuration);
                    assert_eq!(angles.second_theta.abs(), PI);
                    assert_point_close(robot.forward(angles), Point { x, y }, ABSOLUTE_TOLERANCE);
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
                    let angles = assert_inverse(
                        &robot,
                        Point {
                            x: x * radius,
                            y: y * radius,
                        },
                        configuration,
                    );
                    assert_point_close(
                        robot.forward(angles),
                        Point {
                            x: x * boundary,
                            y: y * boundary,
                        },
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
                                robot.inverse(
                                    Point {
                                        x: x * radius,
                                        y: y * radius
                                    },
                                    configuration
                                ),
                                Err(KinematicsError::UnreachablePoint)
                            ),
                            "radius {radius} should be unreachable"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn inverse_rejects_non_finite_coordinates_for_both_branches() {
        let robot = Scara::new(100.0, 60.0).unwrap();
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            for point in [
                Point { x: invalid, y: 0.0 },
                Point { x: 0.0, y: invalid },
                Point {
                    x: invalid,
                    y: invalid,
                },
            ] {
                for configuration in configurations() {
                    assert!(matches!(
                        robot.inverse(point, configuration),
                        Err(KinematicsError::UnreachablePoint)
                    ));
                }
            }
        }
    }

    fn assert_scale_invariance(scale: f64) {
        let robot = Scara::new(scale, scale).unwrap();
        for configuration in configurations() {
            let angles = assert_inverse(&robot, Point { x: scale, y: 0.0 }, configuration);
            assert!((angles.first_theta.abs() - PI / 3.0).abs() <= ABSOLUTE_TOLERANCE);
            assert!((angles.second_theta.abs() - 2.0 * PI / 3.0).abs() <= ABSOLUTE_TOLERANCE);
            let recovered = robot.forward(angles);
            assert_point_close(
                Point {
                    x: recovered.x / scale,
                    y: recovered.y / scale,
                },
                Point { x: 1.0, y: 0.0 },
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

    // Known production limitations: new() accepts these finite positive lengths,
    // but squaring/multiplying them in inverse() yields Ok with NaN angles.
    // Keep the desired behavior as explicit regressions without changing production.
    #[test]
    fn inverse_returns_finite_angles_when_squared_lengths_overflow() {
        assert_scale_invariance(1e200);
    }

    #[test]
    fn inverse_returns_finite_angles_when_squared_lengths_underflow() {
        assert_scale_invariance(1e-200);
    }
}
