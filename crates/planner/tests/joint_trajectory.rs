use core::f64::consts::{FRAC_PI_2, PI};

use kinematics::{ElbowConfiguration, JointAngles, KinematicsError, Point, Scara};
use planner::{JointTrajectory, PlannerError};
use trajectory::{LineSegment, TrajectoryError};

const ANGLE_TOLERANCE: f64 = 1e-10;

fn line(start: (f64, f64), end: (f64, f64)) -> LineSegment {
    LineSegment {
        start: Point {
            x: start.0,
            y: start.1,
        },
        end: Point { x: end.0, y: end.1 },
    }
}

fn configurations() -> [ElbowConfiguration; 2] {
    [ElbowConfiguration::Positive, ElbowConfiguration::Negative]
}

fn assert_point_close(actual: Point, expected: Point, scale: f64) {
    let tolerance = scale * 1e-10;
    assert!(
        (actual.x - expected.x).abs() <= tolerance && (actual.y - expected.y).abs() <= tolerance,
        "expected {expected:?}, got {actual:?}, tolerance {tolerance}"
    );
}

fn assert_angle_close(actual: f64, expected: f64) {
    let difference = actual - expected;
    let wrapped_difference = difference.sin().atan2(difference.cos());
    assert!(
        wrapped_difference.abs() <= ANGLE_TOLERANCE,
        "expected angle {expected} modulo 2*pi, got {actual}"
    );
}

fn assert_next_error(result: Option<Result<JointAngles, PlannerError>>, expected: PlannerError) {
    match result {
        Some(Err(error)) => assert_eq!(error, expected),
        Some(Ok(angles)) => panic!("expected {expected:?}, got angles {angles:?}"),
        None => panic!("expected {expected:?}, but the joint trajectory ended"),
    }
}

fn unwrap_angles(result: Option<Result<JointAngles, PlannerError>>) -> JointAngles {
    match result {
        Some(Ok(angles)) => angles,
        Some(Err(error)) => panic!("unexpected planner error: {error:?}"),
        None => panic!("joint trajectory ended early"),
    }
}

#[test]
fn new_propagates_invalid_maximum_distance_errors_exactly() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((40.0, 0.0), (160.0, 0.0));

    for maximum in [
        f64::NEG_INFINITY,
        -f64::MAX,
        -1.0,
        -f64::from_bits(1),
        -0.0,
        0.0,
        f64::NAN,
        f64::INFINITY,
    ] {
        assert_eq!(
            JointTrajectory::new(&robot, &path, maximum, ElbowConfiguration::Positive).unwrap_err(),
            PlannerError::Trajectory(TrajectoryError::InvalidMaxPointDitance)
        );
    }
}

#[test]
fn new_propagates_invalid_segment_and_excessive_resolution_errors_exactly() {
    let robot = Scara::new(100.0, 60.0).unwrap();

    for path in [
        line((f64::NAN, 0.0), (0.0, 0.0)),
        line((0.0, f64::INFINITY), (0.0, 0.0)),
        line((-f64::MAX, 0.0), (f64::MAX, 0.0)),
        line((0.0, 0.0), (f64::MAX, f64::MAX)),
    ] {
        assert_eq!(
            JointTrajectory::new(&robot, &path, 1.0, ElbowConfiguration::Positive).unwrap_err(),
            PlannerError::Trajectory(TrajectoryError::SegmentNonFinite)
        );
    }

    let path = line((0.0, 0.0), (1.0, 0.0));
    for maximum in [f64::from_bits(1), 1.0 / ((usize::MAX as f64) * 2.0)] {
        assert_eq!(
            JointTrajectory::new(&robot, &path, maximum, ElbowConfiguration::Negative).unwrap_err(),
            PlannerError::Trajectory(TrajectoryError::TooManySegments)
        );
    }
}

#[test]
fn reachable_path_matches_every_cartesian_sample_for_both_configurations() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((40.0, 0.0), (160.0, 0.0));
    let maximum = 7.0;
    let expected_points: Vec<_> = path
        .points(maximum)
        .unwrap()
        .map(|point| point.unwrap())
        .collect();

    assert_eq!(expected_points.len(), 19);

    for configuration in configurations() {
        let positive = configuration == ElbowConfiguration::Positive;
        let actual: Vec<_> = JointTrajectory::new(&robot, &path, maximum, configuration)
            .unwrap()
            .collect();

        assert_eq!(actual.len(), expected_points.len());
        for (result, expected_point) in actual.into_iter().zip(expected_points.iter().copied()) {
            let angles = result.unwrap();
            assert!(angles.first_theta.is_finite());
            assert!(angles.second_theta.is_finite());
            if positive {
                assert!((0.0..=PI).contains(&angles.second_theta));
            } else {
                assert!((-PI..=0.0).contains(&angles.second_theta));
            }
            assert_point_close(robot.forward(angles), expected_point, 160.0);
        }
    }
}

#[test]
fn forward_joint_trajectory_forward_round_trip_covers_a_joint_angle_grid() {
    let robot = Scara::new(75.0, 75.0).unwrap();
    let angle_pairs = [
        (-2.4, -1.2),
        (-1.0, 0.7),
        (0.0, 0.0),
        (0.8, -2.0),
        (2.2, 1.4),
    ];

    for (start_first, start_second) in angle_pairs {
        for (end_first, end_second) in angle_pairs {
            let path = LineSegment {
                start: robot.forward(JointAngles {
                    first_theta: start_first,
                    second_theta: start_second,
                }),
                end: robot.forward(JointAngles {
                    first_theta: end_first,
                    second_theta: end_second,
                }),
            };
            let expected_points: Vec<_> = path.points(5.0).unwrap().map(Result::unwrap).collect();

            for configuration in configurations() {
                let actual: Vec<_> = JointTrajectory::new(&robot, &path, 5.0, configuration)
                    .unwrap()
                    .collect();

                assert_eq!(actual.len(), expected_points.len());
                for (result, expected_point) in
                    actual.into_iter().zip(expected_points.iter().copied())
                {
                    let angles = result.unwrap();
                    assert_point_close(robot.forward(angles), expected_point, 150.0);
                }
            }
        }
    }
}

#[test]
fn iterator_unwraps_the_pi_branch_cut_between_consecutive_samples() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((-100.0, 1.0), (-100.0, -1.0));
    let expected_points: Vec<_> = path.points(0.25).unwrap().map(Result::unwrap).collect();

    for configuration in configurations() {
        let raw_angles: Vec<_> = expected_points
            .iter()
            .copied()
            .map(|point| robot.inverse(point, configuration).unwrap())
            .collect();
        assert!(
            raw_angles
                .windows(2)
                .any(|pair| (pair[1].first_theta - pair[0].first_theta).abs() > PI)
        );

        let unwrapped_angles: Vec<_> = JointTrajectory::new(&robot, &path, 0.25, configuration)
            .unwrap()
            .map(Result::unwrap)
            .collect();

        assert_eq!(unwrapped_angles.len(), expected_points.len());
        assert_eq!(unwrapped_angles[0].first_theta, raw_angles[0].first_theta);
        assert_eq!(unwrapped_angles[0].second_theta, raw_angles[0].second_theta);
        assert!(unwrapped_angles.windows(2).all(|pair| {
            (pair[1].first_theta - pair[0].first_theta).abs() <= PI
                && (pair[1].second_theta - pair[0].second_theta).abs() <= PI
        }));
        assert!(
            unwrapped_angles
                .iter()
                .zip(raw_angles.iter())
                .any(|(unwrapped, raw)| (unwrapped.first_theta - raw.first_theta).abs() > PI)
        );

        for (angles, expected_point) in unwrapped_angles
            .into_iter()
            .zip(expected_points.iter().copied())
        {
            assert_point_close(robot.forward(angles), expected_point, 160.0);
        }
    }
}

#[test]
fn configuration_selects_the_expected_inverse_branch() {
    let robot = Scara::new(100.0, 50.0).unwrap();
    let path = line((100.0, 50.0), (100.0, 50.0));

    let positive: Vec<_> =
        JointTrajectory::new(&robot, &path, f64::MAX, ElbowConfiguration::Positive)
            .unwrap()
            .map(Result::unwrap)
            .collect();
    let negative: Vec<_> =
        JointTrajectory::new(&robot, &path, f64::MAX, ElbowConfiguration::Negative)
            .unwrap()
            .map(Result::unwrap)
            .collect();

    assert_eq!(positive.len(), 2);
    assert_eq!(negative.len(), 2);
    for angles in positive {
        assert_angle_close(angles.first_theta, 0.0);
        assert_angle_close(angles.second_theta, FRAC_PI_2);
    }
    for angles in negative {
        assert_angle_close(angles.first_theta, 2.0 * 0.5_f64.atan());
        assert_angle_close(angles.second_theta, -FRAC_PI_2);
    }
}

#[test]
fn degenerate_reachable_path_emits_both_endpoint_samples_then_stays_finished() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let point = robot.forward(JointAngles {
        first_theta: 0.7,
        second_theta: -1.2,
    });
    let path = LineSegment {
        start: point,
        end: point,
    };
    let mut trajectory =
        JointTrajectory::new(&robot, &path, f64::MAX, ElbowConfiguration::Negative).unwrap();

    for _ in 0..2 {
        let angles = unwrap_angles(trajectory.next());
        assert_point_close(robot.forward(angles), point, 160.0);
    }
    assert!(trajectory.next().is_none());
    assert!(trajectory.next().is_none());
}

#[test]
fn unreachable_start_is_returned_once_then_the_iterator_stays_finished() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((0.0, 0.0), (160.0, 0.0));

    for configuration in configurations() {
        let mut trajectory = JointTrajectory::new(&robot, &path, 20.0, configuration).unwrap();
        assert_next_error(
            trajectory.next(),
            PlannerError::Kinematics(KinematicsError::UnreachablePoint),
        );
        assert!(trajectory.next().is_none());
        assert!(trajectory.next().is_none());
    }
}

#[test]
fn iterator_stops_at_the_first_unreachable_interior_sample() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((-160.0, 0.0), (160.0, 0.0));

    for configuration in configurations() {
        let mut trajectory = JointTrajectory::new(&robot, &path, 20.0, configuration).unwrap();

        for expected_x in [-160.0, -140.0, -120.0, -100.0, -80.0, -60.0, -40.0] {
            let angles = unwrap_angles(trajectory.next());
            assert_point_close(
                robot.forward(angles),
                Point {
                    x: expected_x,
                    y: 0.0,
                },
                160.0,
            );
        }

        assert_next_error(
            trajectory.next(),
            PlannerError::Kinematics(KinematicsError::UnreachablePoint),
        );
        assert!(trajectory.next().is_none());
        assert!(trajectory.next().is_none());
    }
}

#[test]
fn numerical_kinematics_failure_is_preserved_and_finishes_the_iterator() {
    let smallest_length = f64::from_bits(1);
    let largest_length = f64::MAX;

    for (first_arm_length, second_arm_length) in [
        (smallest_length, largest_length),
        (largest_length, smallest_length),
    ] {
        let robot = Scara::new(first_arm_length, second_arm_length).unwrap();
        let path = line((largest_length, 0.0), (largest_length, 0.0));

        for configuration in configurations() {
            let mut trajectory =
                JointTrajectory::new(&robot, &path, f64::MAX, configuration).unwrap();
            assert_next_error(
                trajectory.next(),
                PlannerError::Kinematics(KinematicsError::NumericalFailure),
            );
            assert!(trajectory.next().is_none());
        }
    }
}

#[test]
fn cartesian_to_joint_composition_is_stable_across_finite_scales() {
    for scale in [1e-200, 1e-100, 1e-6, 1.0, 1e6, 1e100, 1e200] {
        let robot = Scara::new(scale, scale).unwrap();
        let path = line((scale, 0.0), (0.0, scale));
        let expected_points: Vec<_> = path
            .points(scale / 4.0)
            .unwrap()
            .map(Result::unwrap)
            .collect();

        for configuration in configurations() {
            let actual: Vec<_> = JointTrajectory::new(&robot, &path, scale / 4.0, configuration)
                .unwrap()
                .collect();

            assert_eq!(actual.len(), expected_points.len());
            assert_eq!(actual.len(), 7);
            for (result, expected_point) in actual.into_iter().zip(expected_points.iter().copied())
            {
                let angles = result.unwrap();
                assert!(angles.first_theta.is_finite());
                assert!(angles.second_theta.is_finite());
                let recovered = robot.forward(angles);
                assert!(recovered.x.is_finite());
                assert!(recovered.y.is_finite());
                assert_point_close(recovered, expected_point, scale);
            }
        }
    }
}
