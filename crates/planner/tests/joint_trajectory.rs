use core::f64::consts::{FRAC_PI_2, PI};

use kinematics::{ElbowConfiguration, JointAngles, KinematicsError, Point, Scara};
use planner::{JointLimits, JointRange, JointTrajectory, PlannerError, ValidatedTrajectory};
use trajectory::{LineSegment, TrajectoryError};

const ANGLE_TOLERANCE: f64 = 1e-10;
const MIN_POSITIVE_STEP: f64 = f64::from_bits(1);

fn line(start: (f64, f64), end: (f64, f64)) -> LineSegment {
    LineSegment::new(point(start.0, start.1), point(end.0, end.1))
        .expect("test segment must have a finite length")
}

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

fn configurations() -> [ElbowConfiguration; 2] {
    [ElbowConfiguration::Positive, ElbowConfiguration::Negative]
}

fn limits(first_min: f64, first_max: f64, second_min: f64, second_max: f64) -> JointLimits {
    JointLimits::new(
        JointRange::new(first_min, first_max).expect("first joint range should be valid"),
        JointRange::new(second_min, second_max).expect("second joint range should be valid"),
    )
}

fn assert_angles_exact(actual: JointAngles, expected: JointAngles) {
    assert_eq!(
        actual.first_theta().to_bits(),
        expected.first_theta().to_bits()
    );
    assert_eq!(
        actual.second_theta().to_bits(),
        expected.second_theta().to_bits()
    );
}

fn assert_point_close(actual: Point, expected: Point, scale: f64) {
    let tolerance = scale * 1e-10;
    assert!(
        (actual.x() - expected.x()).abs() <= tolerance
            && (actual.y() - expected.y()).abs() <= tolerance,
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

fn assert_validation_error(trajectory: &JointTrajectory<'_>, expected: PlannerError) {
    assert_eq!(trajectory.clone().validate(), Err(expected));
    assert_eq!(trajectory.clone().into_validated().unwrap_err(), expected);
    for budget in [0, 1, 4, usize::MAX] {
        assert_eq!(
            trajectory
                .clone()
                .into_validated_with_refinement(budget)
                .unwrap_err(),
            expected,
            "refinement budget {budget}, trajectory {trajectory:?}"
        );
    }
}

#[test]
fn new_propagates_invalid_maximum_distance_errors_before_checking_reachability() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    for path in [
        line((40.0, 0.0), (160.0, 0.0)),
        line((0.0, 0.0), (160.0, 0.0)),
        line((-160.0, 0.0), (160.0, 0.0)),
    ] {
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
            for configuration in configurations() {
                assert_eq!(
                    JointTrajectory::new(&robot, &path, maximum, configuration).unwrap_err(),
                    PlannerError::Trajectory(TrajectoryError::InvalidMaxPointDistance)
                );
            }
        }
    }
}

#[test]
fn invalid_segments_are_rejected_before_planning_and_resolution_errors_are_preserved() {
    let robot = Scara::new(100.0, 60.0).unwrap();

    for (start, end) in [
        (point(-f64::MAX, 0.0), point(f64::MAX, 0.0)),
        (point(0.0, 0.0), point(f64::MAX, f64::MAX)),
    ] {
        assert!(matches!(
            LineSegment::new(start, end),
            Err(TrajectoryError::SegmentNonFinite)
        ));
    }

    let path = line((0.0, 0.0), (1.0, 0.0));
    for (maximum, error) in [
        (f64::from_bits(1), TrajectoryError::InvalidSegmentNumber),
        (
            1.0 / ((usize::MAX as f64) * 2.0),
            TrajectoryError::TooManySegments,
        ),
    ] {
        assert_eq!(
            JointTrajectory::new(&robot, &path, maximum, ElbowConfiguration::Negative).unwrap_err(),
            PlannerError::Trajectory(error)
        );
    }
}

#[test]
fn cartesian_precision_loss_propagates_once_and_prevents_validated_wrappers() {
    let robot = Scara::new(1e16, 1e16).unwrap();
    let path = line((1e16, 0.0), (1e16 + 4.0, 0.0));
    let expected = PlannerError::Trajectory(TrajectoryError::PrecissionLost);
    for configuration in configurations() {
        let mut trajectory = JointTrajectory::new(&robot, &path, 1.0, configuration).unwrap();
        assert_eq!(trajectory.clone().validate(), Err(expected));
        assert_eq!(trajectory.clone().into_validated().unwrap_err(), expected);
        // Refinement retries excessive joint steps, not Cartesian precision failures.
        assert_eq!(
            trajectory
                .clone()
                .into_validated_with_refinement(8)
                .unwrap_err(),
            expected
        );
        let first = trajectory.next().unwrap().unwrap();
        let repeated = trajectory.next().unwrap().unwrap();
        assert_angles_exact(first, repeated);
        assert_next_error(trajectory.next(), expected);
        assert!(trajectory.next().is_none());
        assert!(trajectory.next().is_none());
        let mut finished_clone = trajectory.clone();
        assert!(finished_clone.next().is_none());
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
            assert!(angles.first_theta().is_finite());
            assert!(angles.second_theta().is_finite());
            if positive {
                assert!((0.0..=PI).contains(&angles.second_theta()));
            } else {
                assert!((-PI..=0.0).contains(&angles.second_theta()));
            }
            assert_point_close(forward(&robot, angles), expected_point, 160.0);
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
            let path = LineSegment::new(
                forward(&robot, angles(start_first, start_second)),
                forward(&robot, angles(end_first, end_second)),
            )
            .expect("test segment must have a finite length");
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
                    assert_point_close(forward(&robot, angles), expected_point, 150.0);
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
                .any(|pair| (pair[1].first_theta() - pair[0].first_theta()).abs() > PI)
        );

        let unwrapped_angles: Vec<_> = JointTrajectory::new(&robot, &path, 0.25, configuration)
            .unwrap()
            .map(Result::unwrap)
            .collect();

        assert_eq!(unwrapped_angles.len(), expected_points.len());
        assert_eq!(
            unwrapped_angles[0].first_theta(),
            raw_angles[0].first_theta()
        );
        assert_eq!(
            unwrapped_angles[0].second_theta(),
            raw_angles[0].second_theta()
        );
        assert!(unwrapped_angles.windows(2).all(|pair| {
            (pair[1].first_theta() - pair[0].first_theta()).abs() <= PI
                && (pair[1].second_theta() - pair[0].second_theta()).abs() <= PI
        }));
        assert!(
            unwrapped_angles
                .iter()
                .zip(raw_angles.iter())
                .any(|(unwrapped, raw)| {
                    (unwrapped.first_theta() - raw.first_theta()).abs() > PI
                })
        );

        for (angles, expected_point) in unwrapped_angles
            .into_iter()
            .zip(expected_points.iter().copied())
        {
            assert_point_close(forward(&robot, angles), expected_point, 160.0);
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
        assert_angle_close(angles.first_theta(), 0.0);
        assert_angle_close(angles.second_theta(), FRAC_PI_2);
    }
    for angles in negative {
        assert_angle_close(angles.first_theta(), 2.0 * 0.5_f64.atan());
        assert_angle_close(angles.second_theta(), -FRAC_PI_2);
    }
}

#[test]
fn degenerate_reachable_path_emits_both_endpoint_samples_then_stays_finished() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let point = forward(&robot, angles(0.7, -1.2));
    let path = LineSegment::new(point, point).expect("test segment must have a finite length");
    let mut trajectory =
        JointTrajectory::new(&robot, &path, f64::MAX, ElbowConfiguration::Negative).unwrap();

    for _ in 0..2 {
        let angles = unwrap_angles(trajectory.next());
        assert_point_close(forward(&robot, angles), point, 160.0);
    }
    assert!(trajectory.next().is_none());
    assert!(trajectory.next().is_none());
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
                assert!(angles.first_theta().is_finite());
                assert!(angles.second_theta().is_finite());
                let recovered = forward(&robot, angles);
                assert!(recovered.x().is_finite());
                assert!(recovered.y().is_finite());
                assert_point_close(recovered, expected_point, scale);
            }
        }
    }
}

#[test]
fn with_max_joint_step_rejects_non_finite_and_non_positive_thresholds() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (160.0, 0.0));

    for invalid in [
        f64::NEG_INFINITY,
        -f64::MAX,
        -1.0,
        -MIN_POSITIVE_STEP,
        -0.0,
        0.0,
        f64::NAN,
        f64::INFINITY,
    ] {
        assert_eq!(
            JointTrajectory::new(&robot, &path, f64::MAX, ElbowConfiguration::Positive)
                .unwrap()
                .with_max_joint_step(invalid)
                .unwrap_err(),
            PlannerError::InvalidMaxJointStep
        );
    }

    for valid in [MIN_POSITIVE_STEP, f64::MIN_POSITIVE, 1.0, f64::MAX] {
        assert!(
            JointTrajectory::new(&robot, &path, f64::MAX, ElbowConfiguration::Positive)
                .unwrap()
                .with_max_joint_step(valid)
                .is_ok()
        );
    }
}

#[test]
fn smallest_positive_max_joint_step_accepts_unchanged_consecutive_samples() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (160.0, 0.0));

    for configuration in configurations() {
        let samples: Vec<_> = JointTrajectory::new(&robot, &path, f64::MAX, configuration)
            .unwrap()
            .with_max_joint_step(MIN_POSITIVE_STEP)
            .unwrap()
            .map(Result::unwrap)
            .collect();

        assert_eq!(samples.len(), 2);
        assert_eq!(samples[0].first_theta(), samples[1].first_theta());
        assert_eq!(samples[0].second_theta(), samples[1].second_theta());
    }
}

#[test]
fn max_joint_step_starts_comparing_after_the_first_sample_without_an_initial_position() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (140.0, 0.0));

    for configuration in configurations() {
        let expected_first = robot.inverse(path.start(), configuration).unwrap();
        let mut trajectory = JointTrajectory::new(&robot, &path, f64::MAX, configuration)
            .unwrap()
            .with_max_joint_step(MIN_POSITIVE_STEP)
            .unwrap();

        assert_angles_exact(unwrap_angles(trajectory.next()), expected_first);
        assert_next_error(trajectory.next(), PlannerError::MaxJointStepExceeded);
        assert!(trajectory.next().is_none());
        assert!(trajectory.next().is_none());
    }
}

#[test]
fn max_joint_step_compares_the_initial_position_with_the_first_sample() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let initial = angles(0.1, 0.0);
    let path = line((160.0, 0.0), (160.0, 0.0));

    let mut max_angle_last =
        JointTrajectory::new(&robot, &path, f64::MAX, ElbowConfiguration::Positive)
            .unwrap()
            .with_initial_position(initial, f64::MAX, f64::MAX)
            .unwrap()
            .with_max_joint_step(MIN_POSITIVE_STEP)
            .unwrap();
    assert_next_error(max_angle_last.next(), PlannerError::MaxJointStepExceeded);
    assert!(max_angle_last.next().is_none());

    let mut initial_position_last =
        JointTrajectory::new(&robot, &path, f64::MAX, ElbowConfiguration::Positive)
            .unwrap()
            .with_max_joint_step(MIN_POSITIVE_STEP)
            .unwrap()
            .with_initial_position(initial, f64::MAX, f64::MAX)
            .unwrap();
    assert_next_error(
        initial_position_last.next(),
        PlannerError::MaxJointStepExceeded,
    );
    assert!(initial_position_last.next().is_none());
}

#[test]
fn max_joint_step_is_inclusive_and_checks_each_joint_independently() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let cases = [
        (angles(0.0, 1.0), angles(0.25, 1.0), true),
        (angles(0.0, 0.0), angles(0.0, 0.25), false),
    ];

    for (start_angles, end_angles, first_joint_dominates) in cases {
        let path = LineSegment::new(forward(&robot, start_angles), forward(&robot, end_angles))
            .expect("test segment must have a finite length");
        let expected: Vec<_> =
            JointTrajectory::new(&robot, &path, f64::MAX, ElbowConfiguration::Positive)
                .unwrap()
                .map(Result::unwrap)
                .collect();
        assert_eq!(expected.len(), 2);

        let first_delta = (expected[1].first_theta() - expected[0].first_theta()).abs();
        let second_delta = (expected[1].second_theta() - expected[0].second_theta()).abs();
        if first_joint_dominates {
            assert!(first_delta > second_delta);
        } else {
            assert!(second_delta > first_delta);
        }
        let required_threshold = first_delta.max(second_delta);
        assert!(required_threshold > 0.0);

        let accepted: Vec<_> =
            JointTrajectory::new(&robot, &path, f64::MAX, ElbowConfiguration::Positive)
                .unwrap()
                .with_max_joint_step(required_threshold)
                .unwrap()
                .map(Result::unwrap)
                .collect();
        assert_eq!(accepted.len(), expected.len());
        for (actual, expected) in accepted.into_iter().zip(expected.iter().copied()) {
            assert_angles_exact(actual, expected);
        }

        let immediately_smaller = f64::from_bits(required_threshold.to_bits() - 1);
        let mut rejected =
            JointTrajectory::new(&robot, &path, f64::MAX, ElbowConfiguration::Positive)
                .unwrap()
                .with_max_joint_step(immediately_smaller)
                .unwrap();
        assert_angles_exact(unwrap_angles(rejected.next()), expected[0]);
        assert_next_error(rejected.next(), PlannerError::MaxJointStepExceeded);
        assert!(rejected.next().is_none());
    }
}

#[test]
fn the_last_with_max_joint_step_call_replaces_the_previous_threshold() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (140.0, 0.0));

    let accepted: Vec<_> =
        JointTrajectory::new(&robot, &path, f64::MAX, ElbowConfiguration::Positive)
            .unwrap()
            .with_max_joint_step(MIN_POSITIVE_STEP)
            .unwrap()
            .with_max_joint_step(f64::MAX)
            .unwrap()
            .map(Result::unwrap)
            .collect();
    assert_eq!(accepted.len(), 2);

    let mut rejected = JointTrajectory::new(&robot, &path, f64::MAX, ElbowConfiguration::Positive)
        .unwrap()
        .with_max_joint_step(f64::MAX)
        .unwrap()
        .with_max_joint_step(MIN_POSITIVE_STEP)
        .unwrap();
    assert!(rejected.next().unwrap().is_ok());
    assert_next_error(rejected.next(), PlannerError::MaxJointStepExceeded);
    assert!(rejected.next().is_none());
}

#[test]
fn max_joint_step_checks_unwrapped_steps_across_the_pi_branch_cut() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((-100.0, 1.0), (-100.0, -1.0));
    let maximum = 0.25;

    for configuration in configurations() {
        let points: Vec<_> = path.points(maximum).unwrap().map(Result::unwrap).collect();
        let raw: Vec<_> = points
            .iter()
            .copied()
            .map(|point| robot.inverse(point, configuration).unwrap())
            .collect();
        assert!(
            raw.windows(2)
                .any(|pair| { (pair[1].first_theta() - pair[0].first_theta()).abs() > PI })
        );

        let expected: Vec<_> = JointTrajectory::new(&robot, &path, maximum, configuration)
            .unwrap()
            .map(Result::unwrap)
            .collect();
        let required_threshold = expected
            .windows(2)
            .map(|pair| {
                (pair[1].first_theta() - pair[0].first_theta())
                    .abs()
                    .max((pair[1].second_theta() - pair[0].second_theta()).abs())
            })
            .fold(0.0, f64::max);
        assert!(required_threshold < PI);

        let actual: Vec<_> = JointTrajectory::new(&robot, &path, maximum, configuration)
            .unwrap()
            .with_max_joint_step(required_threshold)
            .unwrap()
            .map(Result::unwrap)
            .collect();
        assert_eq!(actual.len(), expected.len());
        for (actual, expected) in actual.into_iter().zip(expected) {
            assert_angles_exact(actual, expected);
        }
    }
}

#[test]
fn validate_and_into_validated_preserve_max_joint_step_violations() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (140.0, 0.0));

    for configuration in configurations() {
        assert_eq!(
            JointTrajectory::new(&robot, &path, f64::MAX, configuration)
                .unwrap()
                .with_max_joint_step(MIN_POSITIVE_STEP)
                .unwrap()
                .validate(),
            Err(PlannerError::MaxJointStepExceeded)
        );
        assert_eq!(
            JointTrajectory::new(&robot, &path, f64::MAX, configuration)
                .unwrap()
                .with_max_joint_step(MIN_POSITIVE_STEP)
                .unwrap()
                .into_validated()
                .unwrap_err(),
            PlannerError::MaxJointStepExceeded
        );
    }
}

#[test]
fn numerical_kinematics_errors_take_precedence_over_max_joint_step_validation() {
    let path = line((f64::MAX, 0.0), (f64::MAX, 0.0));
    for (l1, l2) in [(f64::MAX, MIN_POSITIVE_STEP), (MIN_POSITIVE_STEP, f64::MAX)] {
        let robot = Scara::new(l1, l2).unwrap();
        for configuration in configurations() {
            let mut trajectory = JointTrajectory::new(&robot, &path, f64::MAX, configuration)
                .unwrap()
                .with_max_joint_step(MIN_POSITIVE_STEP)
                .unwrap();
            assert_next_error(
                trajectory.next(),
                PlannerError::Kinematics(KinematicsError::NumericalFailure),
            );
            assert!(trajectory.next().is_none());
            assert!(trajectory.next().is_none());
        }
    }
}

#[test]
fn with_limits_accepts_the_same_trajectory_when_every_sample_is_in_range() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((40.0, 0.0), (160.0, 0.0));
    let maximum = 7.0;

    for configuration in configurations() {
        let expected: Vec<_> = JointTrajectory::new(&robot, &path, maximum, configuration)
            .unwrap()
            .map(Result::unwrap)
            .collect();
        let first_min = expected
            .iter()
            .map(JointAngles::first_theta)
            .fold(f64::INFINITY, f64::min);
        let first_max = expected
            .iter()
            .map(JointAngles::first_theta)
            .fold(f64::NEG_INFINITY, f64::max);
        let second_min = expected
            .iter()
            .map(JointAngles::second_theta)
            .fold(f64::INFINITY, f64::min);
        let second_max = expected
            .iter()
            .map(JointAngles::second_theta)
            .fold(f64::NEG_INFINITY, f64::max);
        let joint_limits = limits(first_min, first_max, second_min, second_max);

        let actual: Vec<_> = JointTrajectory::new(&robot, &path, maximum, configuration)
            .unwrap()
            .with_limits(&joint_limits)
            .unwrap()
            .map(Result::unwrap)
            .collect();

        assert_eq!(actual.len(), expected.len());
        for (actual, expected) in actual.into_iter().zip(expected) {
            assert_angles_exact(actual, expected);
        }
    }
}

#[test]
fn with_limits_rejects_either_joint_on_the_first_sample_and_then_stays_finished() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let configuration = ElbowConfiguration::Positive;
    let expected = angles(0.4, 0.8);
    let start = forward(&robot, expected);
    let path = LineSegment::new(start, start).expect("test segment must have a finite length");
    let solved = robot.inverse(start, configuration).unwrap();
    let rejected_limits = [
        limits(
            solved.first_theta() + 0.25,
            solved.first_theta() + 0.5,
            -f64::MAX,
            f64::MAX,
        ),
        limits(
            -f64::MAX,
            f64::MAX,
            solved.second_theta() + 0.25,
            solved.second_theta() + 0.5,
        ),
    ];

    for joint_limits in &rejected_limits {
        let mut trajectory = JointTrajectory::new(&robot, &path, f64::MAX, configuration)
            .unwrap()
            .with_limits(joint_limits)
            .unwrap();

        assert_next_error(trajectory.next(), PlannerError::AngleOutOfLimits);
        assert!(trajectory.next().is_none());
        assert!(trajectory.next().is_none());
    }
}

#[test]
fn with_limits_returns_the_accepted_prefix_then_stops_at_the_first_violation() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (40.0, 0.0));
    let joint_limits = limits(0.0, 0.0, 0.0, 0.0);

    for configuration in configurations() {
        let mut trajectory = JointTrajectory::new(&robot, &path, 20.0, configuration)
            .unwrap()
            .with_limits(&joint_limits)
            .unwrap();
        let expected = robot.inverse(path.start(), configuration).unwrap();

        assert_angles_exact(unwrap_angles(trajectory.next()), expected);
        assert_next_error(trajectory.next(), PlannerError::AngleOutOfLimits);
        assert!(trajectory.next().is_none());
        assert!(trajectory.next().is_none());
    }
}

#[test]
fn the_last_with_limits_call_replaces_the_previous_limits() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (160.0, 0.0));
    let allowed = limits(0.0, 0.0, 0.0, 0.0);
    let rejected = limits(1.0, 2.0, 1.0, 2.0);

    let mut allowed_last =
        JointTrajectory::new(&robot, &path, f64::MAX, ElbowConfiguration::Positive)
            .unwrap()
            .with_limits(&rejected)
            .unwrap()
            .with_limits(&allowed)
            .unwrap();
    assert_angles_exact(unwrap_angles(allowed_last.next()), angles(0.0, 0.0));

    let mut rejected_last =
        JointTrajectory::new(&robot, &path, f64::MAX, ElbowConfiguration::Positive)
            .unwrap()
            .with_limits(&allowed)
            .unwrap()
            .with_limits(&rejected)
            .unwrap();
    assert_next_error(rejected_last.next(), PlannerError::AngleOutOfLimits);
}

#[test]
fn with_limits_validates_unwrapped_angles_across_the_pi_branch_cut() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((-100.0, 1.0), (-100.0, -1.0));
    let maximum = 0.25;

    for configuration in configurations() {
        let expected: Vec<_> = JointTrajectory::new(&robot, &path, maximum, configuration)
            .unwrap()
            .map(Result::unwrap)
            .collect();
        let raw: Vec<_> = path
            .points(maximum)
            .unwrap()
            .map(|point| robot.inverse(point.unwrap(), configuration).unwrap())
            .collect();
        let first_min = expected
            .iter()
            .map(JointAngles::first_theta)
            .fold(f64::INFINITY, f64::min);
        let first_max = expected
            .iter()
            .map(JointAngles::first_theta)
            .fold(f64::NEG_INFINITY, f64::max);
        let second_min = expected
            .iter()
            .map(JointAngles::second_theta)
            .fold(f64::INFINITY, f64::min);
        let second_max = expected
            .iter()
            .map(JointAngles::second_theta)
            .fold(f64::NEG_INFINITY, f64::max);
        let first_range = JointRange::new(first_min, first_max).unwrap();
        assert!(
            raw.iter()
                .any(|angles| first_range.contains(angles.first_theta()) == Ok(false))
        );
        let joint_limits = JointLimits::new(
            first_range,
            JointRange::new(second_min, second_max).unwrap(),
        );

        let actual: Vec<_> = JointTrajectory::new(&robot, &path, maximum, configuration)
            .unwrap()
            .with_limits(&joint_limits)
            .unwrap()
            .map(Result::unwrap)
            .collect();

        assert_eq!(actual.len(), expected.len());
        for (actual, expected) in actual.into_iter().zip(expected) {
            assert_angles_exact(actual, expected);
        }
    }
}

#[test]
fn with_limits_preserves_numerical_kinematics_error_precedence_and_finishes_the_iterator() {
    let path = line((f64::MAX, 0.0), (f64::MAX, 0.0));
    let restrictive_limits = limits(0.0, 0.0, 0.0, 0.0);
    for (l1, l2) in [(f64::MAX, MIN_POSITIVE_STEP), (MIN_POSITIVE_STEP, f64::MAX)] {
        let robot = Scara::new(l1, l2).unwrap();
        for configuration in configurations() {
            let mut trajectory = JointTrajectory::new(&robot, &path, f64::MAX, configuration)
                .unwrap()
                .with_limits(&restrictive_limits)
                .unwrap();
            assert_next_error(
                trajectory.next(),
                PlannerError::Kinematics(KinematicsError::NumericalFailure),
            );
            assert!(trajectory.next().is_none());
            assert!(trajectory.next().is_none());
        }
    }
}

#[test]
fn with_initial_position_preserves_the_requested_equivalent_turn() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((-100.0, 0.0), (-100.0, -1.0));

    for configuration in configurations() {
        let canonical = robot.inverse(path.start(), configuration).unwrap();
        let initial = angles(
            canonical.first_theta() + core::f64::consts::TAU,
            canonical.second_theta() - core::f64::consts::TAU,
        );
        let mut trajectory = JointTrajectory::new(&robot, &path, 0.25, configuration)
            .unwrap()
            .with_initial_position(initial, 1e-10, 1e-10)
            .unwrap();

        let first = unwrap_angles(trajectory.next());
        assert!((first.first_theta() - initial.first_theta()).abs() <= ANGLE_TOLERANCE);
        assert!((first.second_theta() - initial.second_theta()).abs() <= ANGLE_TOLERANCE);

        let remaining: Vec<_> = trajectory.map(Result::unwrap).collect();
        assert!(!remaining.is_empty());
        assert!(remaining.iter().all(|next| {
            (next.first_theta() - initial.first_theta()).abs() <= PI
                && (next.second_theta() - initial.second_theta()).abs() <= PI
        }));
    }
}

#[test]
fn with_initial_position_uses_an_inclusive_cartesian_tolerance() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((159.0, 0.0), (159.0, 0.0));
    let initial = angles(0.0, 0.0);

    assert!(
        JointTrajectory::new(&robot, &path, f64::MAX, ElbowConfiguration::Positive)
            .unwrap()
            .with_initial_position(initial, 1.0, f64::MAX)
            .is_ok()
    );

    assert_eq!(
        JointTrajectory::new(&robot, &path, f64::MAX, ElbowConfiguration::Positive)
            .unwrap()
            .with_initial_position(initial, f64::from_bits(1.0_f64.to_bits() - 1), f64::MAX,)
            .unwrap_err(),
        PlannerError::InitialPositionNotInStart
    );

    let exact_path = line((160.0, 0.0), (160.0, 0.0));
    for zero in [0.0, -0.0] {
        assert!(
            JointTrajectory::new(&robot, &exact_path, f64::MAX, ElbowConfiguration::Positive)
                .unwrap()
                .with_initial_position(initial, zero, 0.0)
                .is_ok()
        );
    }
}

#[test]
fn with_initial_position_maps_invalid_cartesian_tolerances_exactly() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (160.0, 0.0));
    let initial = angles(0.0, 0.0);

    for invalid in [
        f64::NEG_INFINITY,
        -f64::MAX,
        -1.0,
        -f64::from_bits(1),
        f64::NAN,
        f64::INFINITY,
    ] {
        assert_eq!(
            JointTrajectory::new(&robot, &path, f64::MAX, ElbowConfiguration::Positive)
                .unwrap()
                .with_initial_position(initial, invalid, 0.0)
                .unwrap_err(),
            PlannerError::Kinematics(KinematicsError::InvalidTolerance)
        );
    }
}

#[test]
fn with_initial_position_rejects_a_finite_position_outside_the_start_tolerance() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((100.0, 0.0), (100.0, 1.0));
    let initial = angles(0.0, 0.0);

    for configuration in configurations() {
        assert_eq!(
            JointTrajectory::new(&robot, &path, 0.25, configuration)
                .unwrap()
                .with_initial_position(initial, 0.5, f64::MAX)
                .unwrap_err(),
            PlannerError::InitialPositionNotInStart
        );
    }
}

#[test]
fn with_initial_position_maps_forward_numerical_failure_exactly() {
    let robot = Scara::new(f64::MAX, f64::MAX).unwrap();
    let path = line((1.0, 0.0), (1.0, 0.0));

    assert_eq!(
        JointTrajectory::new(&robot, &path, f64::MAX, ElbowConfiguration::Positive)
            .unwrap()
            .with_initial_position(angles(0.0, 0.0), f64::MAX, f64::MAX)
            .unwrap_err(),
        PlannerError::Kinematics(KinematicsError::NumericalFailure)
    );
}

#[test]
fn with_initial_position_validates_existing_limits_before_cartesian_position() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((100.0, 0.0), (100.0, 0.0));
    let restrictive_limits = limits(1.0, 2.0, 1.0, 2.0);

    assert_eq!(
        JointTrajectory::new(&robot, &path, f64::MAX, ElbowConfiguration::Positive)
            .unwrap()
            .with_limits(&restrictive_limits)
            .unwrap()
            .with_initial_position(angles(0.0, 0.0), 0.5, f64::MAX)
            .unwrap_err(),
        PlannerError::AngleOutOfLimits
    );
}

#[test]
fn initial_position_outside_limits_is_rejected_in_either_builder_order() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (160.0, 0.0));
    let initial = angles(0.0, 0.0);
    let rejected_limits = [
        limits(1.0, 2.0, -f64::MAX, f64::MAX),
        limits(-f64::MAX, f64::MAX, 1.0, 2.0),
    ];

    for configuration in configurations() {
        for joint_limits in &rejected_limits {
            assert_eq!(
                JointTrajectory::new(&robot, &path, f64::MAX, configuration)
                    .unwrap()
                    .with_limits(joint_limits)
                    .unwrap()
                    .with_initial_position(initial, 0.0, 0.0)
                    .unwrap_err(),
                PlannerError::AngleOutOfLimits
            );

            assert_eq!(
                JointTrajectory::new(&robot, &path, f64::MAX, configuration)
                    .unwrap()
                    .with_initial_position(initial, 0.0, 0.0)
                    .unwrap()
                    .with_limits(joint_limits)
                    .unwrap_err(),
                PlannerError::AngleOutOfLimits
            );
        }
    }
}

#[test]
fn limits_including_initial_position_are_accepted_in_either_builder_order() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (160.0, 0.0));
    let initial = angles(0.0, 0.0);
    let inclusive_limits = limits(0.0, 0.0, 0.0, 0.0);

    for configuration in configurations() {
        let limits_first: Vec<_> = JointTrajectory::new(&robot, &path, f64::MAX, configuration)
            .unwrap()
            .with_limits(&inclusive_limits)
            .unwrap()
            .with_initial_position(initial, 0.0, 0.0)
            .unwrap()
            .map(Result::unwrap)
            .collect();
        let initial_first: Vec<_> = JointTrajectory::new(&robot, &path, f64::MAX, configuration)
            .unwrap()
            .with_initial_position(initial, 0.0, 0.0)
            .unwrap()
            .with_limits(&inclusive_limits)
            .unwrap()
            .map(Result::unwrap)
            .collect();

        assert_eq!(limits_first.len(), 2);
        assert_eq!(initial_first.len(), limits_first.len());
        for (limits_first, initial_first) in limits_first.into_iter().zip(initial_first) {
            assert_angles_exact(limits_first, initial_first);
        }
    }
}

#[test]
fn limits_after_initial_position_validate_the_stored_noncanonical_turn() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((-100.0, 0.0), (-100.0, 0.0));

    for configuration in configurations() {
        let canonical = robot.inverse(path.start(), configuration).unwrap();
        let initial = angles(
            canonical.first_theta() + core::f64::consts::TAU,
            canonical.second_theta(),
        );
        let initial_turn_limits = limits(
            initial.first_theta() - 0.1,
            initial.first_theta() + 0.1,
            -f64::MAX,
            f64::MAX,
        );
        let canonical_turn_limits = limits(
            canonical.first_theta() - 0.1,
            canonical.first_theta() + 0.1,
            -f64::MAX,
            f64::MAX,
        );

        let trajectory = JointTrajectory::new(&robot, &path, f64::MAX, configuration)
            .unwrap()
            .with_initial_position(initial, 1e-10, 1e-10)
            .unwrap()
            .with_limits(&initial_turn_limits)
            .unwrap();

        assert_eq!(
            trajectory.with_limits(&canonical_turn_limits).unwrap_err(),
            PlannerError::AngleOutOfLimits
        );
    }
}

#[test]
fn initial_position_and_limits_can_select_a_noncanonical_turn() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((-100.0, 0.0), (-100.0, -1.0));
    let configuration = ElbowConfiguration::Positive;
    let canonical = robot.inverse(path.start(), configuration).unwrap();
    let initial = angles(
        canonical.first_theta() + core::f64::consts::TAU,
        canonical.second_theta(),
    );
    let joint_limits = limits(
        initial.first_theta() - 0.1,
        initial.first_theta() + 0.1,
        -f64::MAX,
        f64::MAX,
    );

    let mut without_initial = JointTrajectory::new(&robot, &path, 0.25, configuration)
        .unwrap()
        .with_limits(&joint_limits)
        .unwrap();
    assert_next_error(without_initial.next(), PlannerError::AngleOutOfLimits);

    let actual: Vec<_> = JointTrajectory::new(&robot, &path, 0.25, configuration)
        .unwrap()
        .with_limits(&joint_limits)
        .unwrap()
        .with_initial_position(initial, 1e-10, 1e-10)
        .unwrap()
        .map(Result::unwrap)
        .collect();

    assert!(!actual.is_empty());
    assert!(
        actual
            .iter()
            .all(|angles| joint_limits.contains(*angles) == Ok(true))
    );
}

#[test]
fn with_initial_position_accepts_the_configured_elbow_branch() {
    let robot = Scara::new(100.0, 50.0).unwrap();
    let path = line((100.0, 50.0), (100.0, 50.0));

    for configuration in configurations() {
        let initial = robot.inverse(path.start(), configuration).unwrap();

        assert!(
            JointTrajectory::new(&robot, &path, f64::MAX, configuration)
                .unwrap()
                .with_initial_position(initial, 1e-10, 1e-10)
                .is_ok()
        );
    }
}

#[test]
fn with_initial_position_rejects_the_opposite_elbow_branch() {
    let robot = Scara::new(100.0, 50.0).unwrap();
    let path = line((100.0, 50.0), (100.0, 50.0));

    for (configured, opposite) in [
        (ElbowConfiguration::Positive, ElbowConfiguration::Negative),
        (ElbowConfiguration::Negative, ElbowConfiguration::Positive),
    ] {
        let initial = robot.inverse(path.start(), opposite).unwrap();

        assert_eq!(
            JointTrajectory::new(&robot, &path, f64::MAX, configured)
                .unwrap()
                .with_initial_position(initial, 1e-10, 1e-10)
                .unwrap_err(),
            PlannerError::InitialConfigurationMismatch
        );
    }
}

#[test]
fn with_initial_position_maps_invalid_angular_tolerances_exactly() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (160.0, 0.0));
    let initial = angles(0.0, 0.0);

    for invalid in [
        f64::NEG_INFINITY,
        -f64::MAX,
        -1.0,
        -f64::from_bits(1),
        f64::NAN,
        f64::INFINITY,
    ] {
        assert_eq!(
            JointTrajectory::new(&robot, &path, f64::MAX, ElbowConfiguration::Positive)
                .unwrap()
                .with_initial_position(initial, 0.0, invalid)
                .unwrap_err(),
            PlannerError::Kinematics(KinematicsError::InvalidTolerance)
        );
    }
}

#[test]
fn validate_accepts_reachable_trajectories_for_both_configurations_and_scales() {
    for scale in [1e-200, 1.0, 1e200] {
        let robot = Scara::new(scale, scale).unwrap();
        let path = line((scale, 0.0), (0.0, scale));

        for configuration in configurations() {
            assert_eq!(
                JointTrajectory::new(&robot, &path, scale / 4.0, configuration)
                    .unwrap()
                    .validate(),
                Ok(())
            );
        }
    }
}

#[test]
fn new_rejects_unreachable_endpoints_and_degenerate_segments() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    for path in [
        line((0.0, 0.0), (160.0, 0.0)),
        line((160.0, 0.0), (0.0, 0.0)),
        line((160.0, 0.0), (200.0, 0.0)),
        line((200.0, 0.0), (160.0, 0.0)),
        line((0.0, 0.0), (0.0, 0.0)),
        line((200.0, 0.0), (200.0, 0.0)),
    ] {
        for configuration in configurations() {
            assert_eq!(
                JointTrajectory::new(&robot, &path, 20.0, configuration).unwrap_err(),
                PlannerError::SegmentUnreachable
            );
        }
    }
}

#[test]
fn new_rejects_a_hole_between_reachable_samples_regardless_of_resolution_and_scale() {
    for scale in [1e-300, 1.0, 1e300] {
        let robot = Scara::new(3.0 * scale, scale).unwrap();
        for path in [
            line((0.0, -3.0 * scale), (0.0, 3.0 * scale)),
            line((0.0, 3.0 * scale), (0.0, -3.0 * scale)),
        ] {
            let samples: Vec<_> = path.points(f64::MAX).unwrap().map(Result::unwrap).collect();
            assert_eq!(samples.len(), 2);
            for configuration in configurations() {
                assert!(
                    samples
                        .iter()
                        .all(|point| robot.inverse(*point, configuration).is_ok())
                );
                assert!(matches!(
                    robot.inverse(path.point_at(0.5).unwrap(), configuration),
                    Err(KinematicsError::UnreachablePoint)
                ));
                // The coarsest sampling omits the hole entirely. Construction
                // must still reject the full segment before yielding an iterator.
                for spacing in [f64::MAX, 6.0 * scale, 3.0 * scale, scale] {
                    assert_eq!(
                        JointTrajectory::new(&robot, &path, spacing, configuration).unwrap_err(),
                        PlannerError::SegmentUnreachable,
                        "scale {scale}, spacing {spacing}, configuration {configuration:?}"
                    );
                }
            }
        }
    }
}

#[test]
fn new_rejects_an_unreachable_segment_before_attempting_inverse_kinematics() {
    let path = line((f64::MAX, 0.0), (f64::MAX / 2.0, 0.0));
    for (l1, l2) in [(f64::MAX, MIN_POSITIVE_STEP), (MIN_POSITIVE_STEP, f64::MAX)] {
        let robot = Scara::new(l1, l2).unwrap();
        for configuration in configurations() {
            // Inverse kinematics at the start loses the shorter arm during
            // normalization. The end is outside the reachable annulus.
            assert!(matches!(
                robot.inverse(path.start(), configuration),
                Err(KinematicsError::NumericalFailure)
            ));
            assert_eq!(
                JointTrajectory::new(&robot, &path, f64::MAX, configuration).unwrap_err(),
                PlannerError::SegmentUnreachable
            );
        }
    }
}

#[test]
fn new_propagates_segment_numerical_errors_even_when_each_endpoint_is_reachable() {
    let robot = Scara::new(f64::MAX / 2.0, f64::MAX / 4.0).unwrap();
    for path in [
        line((f64::MAX / 2.0, 0.0), (f64::MAX / 2.0, MIN_POSITIVE_STEP)),
        line((f64::MAX / 2.0, MIN_POSITIVE_STEP), (f64::MAX / 2.0, 0.0)),
    ] {
        assert_ne!(path.start(), path.end());
        for configuration in configurations() {
            assert!(robot.inverse(path.start(), configuration).is_ok());
            assert!(robot.inverse(path.end(), configuration).is_ok());
            // Segment normalization erases the difference between endpoints;
            // the constructor propagates this error rather than accepting them.
            assert_eq!(
                JointTrajectory::new(&robot, &path, f64::MAX, configuration).unwrap_err(),
                PlannerError::Kinematics(KinematicsError::NumericalFailure)
            );
            // Sampling validation runs before segment normalization.
            assert_eq!(
                JointTrajectory::new(&robot, &path, 0.0, configuration).unwrap_err(),
                PlannerError::Trajectory(TrajectoryError::InvalidMaxPointDistance)
            );
        }
    }
}

#[test]
fn validation_accepts_complete_boundary_segments_without_consuming_the_wrapper() {
    for scale in [1e-300, 1.0, 1e300] {
        let robot = Scara::new(3.0 * scale, scale).unwrap();
        for path in [
            line((2.0 * scale, -scale), (2.0 * scale, scale)), // Inner tangent.
            line((4.0 * scale, 0.0), (0.0, 4.0 * scale)),      // Outer endpoints.
            line((2.0 * scale, 0.0), (4.0 * scale, 0.0)),      // Both boundaries.
            line((2.0 * scale, 0.0), (2.0 * scale, 0.0)),      // Degenerate inner.
            line((4.0 * scale, 0.0), (4.0 * scale, 0.0)),      // Degenerate outer.
        ] {
            for configuration in configurations() {
                let trajectory =
                    // Leave spacing margin: this fixture tests wrapper validation;
                    // strict Cartesian precision errors have separate regressions.
                    JointTrajectory::new(&robot, &path, 0.6 * scale, configuration).unwrap();
                let expected: Vec<_> = trajectory.clone().map(Result::unwrap).collect();
                assert_eq!(trajectory.clone().validate(), Ok(()));
                let validated = trajectory.clone().into_validated().unwrap();
                let refined = trajectory.into_validated_with_refinement(0).unwrap();
                for wrapper in [validated, refined] {
                    let actual: Vec<_> = wrapper.map(Result::unwrap).collect();
                    assert_eq!(actual.len(), expected.len());
                    for (actual, expected) in actual.into_iter().zip(expected.iter().copied()) {
                        assert_angles_exact(actual, expected);
                    }
                }
            }
        }
    }
}

#[test]
fn validate_rejects_a_trajectory_after_a_successful_next_call() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (40.0, 0.0));

    for configuration in configurations() {
        let mut trajectory = JointTrajectory::new(&robot, &path, 20.0, configuration).unwrap();
        assert!(matches!(trajectory.next(), Some(Ok(_))));

        assert_eq!(
            trajectory.validate(),
            Err(PlannerError::TrajectoryAlreadyStarted)
        );
    }
}

#[test]
fn validate_rejects_a_trajectory_after_an_error_was_observed() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (140.0, 0.0));
    let restrictive_limits = limits(0.0, 0.0, 0.0, 0.0);
    for configuration in configurations() {
        let mut trajectory = JointTrajectory::new(&robot, &path, 20.0, configuration)
            .unwrap()
            .with_limits(&restrictive_limits)
            .unwrap();
        assert!(trajectory.next().unwrap().is_ok());
        assert_next_error(trajectory.next(), PlannerError::AngleOutOfLimits);
        assert_validation_error(&trajectory, PlannerError::TrajectoryAlreadyStarted);
    }
}

#[test]
fn validate_rejects_a_fully_consumed_trajectory() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (160.0, 0.0));

    for configuration in configurations() {
        let mut trajectory = JointTrajectory::new(&robot, &path, f64::MAX, configuration).unwrap();
        assert_eq!(trajectory.by_ref().count(), 2);
        assert!(trajectory.next().is_none());

        assert_eq!(
            trajectory.validate(),
            Err(PlannerError::TrajectoryAlreadyStarted)
        );
    }
}

#[test]
fn validate_preserves_numerical_kinematics_failures() {
    let smallest_length = f64::from_bits(1);
    let largest_length = f64::MAX;

    for (first_arm_length, second_arm_length) in [
        (smallest_length, largest_length),
        (largest_length, smallest_length),
    ] {
        let robot = Scara::new(first_arm_length, second_arm_length).unwrap();
        let path = line((largest_length, 0.0), (largest_length, 0.0));

        for configuration in configurations() {
            assert_eq!(
                JointTrajectory::new(&robot, &path, f64::MAX, configuration)
                    .unwrap()
                    .validate(),
                Err(PlannerError::Kinematics(KinematicsError::NumericalFailure))
            );
        }
    }
}

#[test]
fn validate_detects_a_limit_violation_after_an_accepted_start() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (40.0, 0.0));
    let restrictive_limits = limits(0.0, 0.0, 0.0, 0.0);

    for configuration in configurations() {
        assert_eq!(
            JointTrajectory::new(&robot, &path, 20.0, configuration)
                .unwrap()
                .with_limits(&restrictive_limits)
                .unwrap()
                .validate(),
            Err(PlannerError::AngleOutOfLimits)
        );
    }
}

#[test]
fn validate_accepts_noncanonical_initial_turns_with_matching_limits() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((-100.0, 0.0), (-100.0, 0.0));

    for configuration in configurations() {
        let canonical = robot.inverse(path.start(), configuration).unwrap();
        let initial = angles(
            canonical.first_theta() + core::f64::consts::TAU,
            canonical.second_theta() - core::f64::consts::TAU,
        );
        let matching_limits = limits(
            initial.first_theta() - 1e-10,
            initial.first_theta() + 1e-10,
            initial.second_theta() - 1e-10,
            initial.second_theta() + 1e-10,
        );

        assert_eq!(
            JointTrajectory::new(&robot, &path, f64::MAX, configuration)
                .unwrap()
                .with_initial_position(initial, 1e-10, 1e-10)
                .unwrap()
                .with_limits(&matching_limits)
                .unwrap()
                .validate(),
            Ok(())
        );
    }
}

#[test]
fn into_validated_returns_the_complete_original_trajectory_for_both_configurations() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((40.0, 0.0), (160.0, 0.0));
    let maximum = 7.0;

    for configuration in configurations() {
        let expected: Vec<_> = JointTrajectory::new(&robot, &path, maximum, configuration)
            .unwrap()
            .map(Result::unwrap)
            .collect();
        let mut validated = JointTrajectory::new(&robot, &path, maximum, configuration)
            .unwrap()
            .into_validated()
            .unwrap();

        let _: &ValidatedTrajectory<'_> = &validated;
        let actual: Vec<_> = validated.by_ref().map(Result::unwrap).collect();

        assert_eq!(actual.len(), expected.len());
        for (actual, expected) in actual.into_iter().zip(expected) {
            assert_angles_exact(actual, expected);
        }
        assert!(validated.next().is_none());
        assert!(validated.next().is_none());
    }
}

#[test]
fn into_validated_preserves_initial_position_unwrapping_and_limits() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((-100.0, 0.0), (-100.0, -1.0));

    for configuration in configurations() {
        let canonical = robot.inverse(path.start(), configuration).unwrap();
        let initial = angles(
            canonical.first_theta() + core::f64::consts::TAU,
            canonical.second_theta() - core::f64::consts::TAU,
        );
        let matching_limits = limits(
            initial.first_theta() - 0.1,
            initial.first_theta() + 0.1,
            initial.second_theta() - 0.1,
            initial.second_theta() + 0.1,
        );

        let actual: Vec<_> = JointTrajectory::new(&robot, &path, 0.25, configuration)
            .unwrap()
            .with_initial_position(initial, 1e-10, 1e-10)
            .unwrap()
            .with_limits(&matching_limits)
            .unwrap()
            .into_validated()
            .unwrap()
            .map(Result::unwrap)
            .collect();

        assert!(!actual.is_empty());
        assert!(
            actual
                .iter()
                .all(|angles| matching_limits.contains(*angles) == Ok(true))
        );
        assert!(actual.iter().all(|angles| {
            angles.first_theta() > core::f64::consts::PI
                && angles.second_theta().abs() > core::f64::consts::PI
        }));
    }
}

#[test]
fn into_validated_preserves_numerical_and_limit_errors() {
    let smallest_length = f64::from_bits(1);
    let largest_length = f64::MAX;

    for (first_arm_length, second_arm_length) in [
        (smallest_length, largest_length),
        (largest_length, smallest_length),
    ] {
        let robot = Scara::new(first_arm_length, second_arm_length).unwrap();
        let path = line((largest_length, 0.0), (largest_length, 0.0));

        for configuration in configurations() {
            assert_eq!(
                JointTrajectory::new(&robot, &path, f64::MAX, configuration)
                    .unwrap()
                    .into_validated()
                    .unwrap_err(),
                PlannerError::Kinematics(KinematicsError::NumericalFailure)
            );
        }
    }

    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (40.0, 0.0));
    let restrictive_limits = limits(0.0, 0.0, 0.0, 0.0);

    for configuration in configurations() {
        assert_eq!(
            JointTrajectory::new(&robot, &path, 20.0, configuration)
                .unwrap()
                .with_limits(&restrictive_limits)
                .unwrap()
                .into_validated()
                .unwrap_err(),
            PlannerError::AngleOutOfLimits
        );
    }
}

#[test]
fn into_validated_rejects_any_trajectory_that_has_already_started() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (40.0, 0.0));

    for configuration in configurations() {
        let mut partially_consumed =
            JointTrajectory::new(&robot, &path, 20.0, configuration).unwrap();
        assert!(matches!(partially_consumed.next(), Some(Ok(_))));
        assert_eq!(
            partially_consumed.into_validated().unwrap_err(),
            PlannerError::TrajectoryAlreadyStarted
        );

        let mut fully_consumed = JointTrajectory::new(&robot, &path, 20.0, configuration).unwrap();
        assert!(fully_consumed.by_ref().all(|result| result.is_ok()));
        assert_eq!(
            fully_consumed.into_validated().unwrap_err(),
            PlannerError::TrajectoryAlreadyStarted
        );

        let mut failed = JointTrajectory::new(&robot, &path, 20.0, configuration)
            .unwrap()
            .with_max_joint_step(MIN_POSITIVE_STEP)
            .unwrap();
        assert!(failed.next().unwrap().is_ok());
        assert_next_error(failed.next(), PlannerError::MaxJointStepExceeded);
        assert_eq!(
            failed.into_validated().unwrap_err(),
            PlannerError::TrajectoryAlreadyStarted
        );
    }
}

#[test]
fn refinement_preserves_the_complete_valid_trajectory_without_adding_samples() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((40.0, 0.0), (160.0, 0.0));

    for configuration in configurations() {
        let trajectory = JointTrajectory::new(&robot, &path, 7.0, configuration).unwrap();
        let expected: Vec<_> = trajectory.clone().map(Result::unwrap).collect();

        for budget in [0, 1, 2, 8, usize::MAX] {
            let mut validated = trajectory
                .clone()
                .into_validated_with_refinement(budget)
                .unwrap();
            let _: &ValidatedTrajectory<'_> = &validated;
            let actual: Vec<_> = validated.by_ref().map(Result::unwrap).collect();
            assert_eq!(actual.len(), expected.len());
            for (actual, expected) in actual.into_iter().zip(expected.iter().copied()) {
                assert_angles_exact(actual, expected);
            }
            assert!(validated.next().is_none());
            assert!(validated.next().is_none());
        }
    }
}

#[test]
fn refinement_with_zero_attempts_accepts_a_valid_path_without_consuming_it() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (140.0, 0.0));

    for configuration in configurations() {
        let trajectory = JointTrajectory::new(&robot, &path, 20.0, configuration).unwrap();
        assert_eq!(trajectory.clone().validate(), Ok(()));
        let expected: Vec<_> = trajectory.clone().map(Result::unwrap).collect();
        let actual: Vec<_> = trajectory
            .into_validated_with_refinement(0)
            .expect("a valid trajectory should not require refinement attempts")
            .map(Result::unwrap)
            .collect();
        assert_eq!(actual.len(), expected.len());
        for (actual, expected) in actual.into_iter().zip(expected) {
            assert_angles_exact(actual, expected);
        }
    }
}

#[test]
fn refinement_with_zero_attempts_rejects_a_path_that_requires_refinement() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (140.0, 0.0));

    for configuration in configurations() {
        let trajectory = JointTrajectory::new(&robot, &path, 20.0, configuration)
            .unwrap()
            .with_max_joint_step(0.01)
            .unwrap();
        assert_eq!(
            trajectory.clone().validate(),
            Err(PlannerError::MaxJointStepExceeded)
        );
        assert_eq!(
            trajectory.into_validated_with_refinement(0).unwrap_err(),
            PlannerError::MaxRefinementAttemptsReached
        );
    }
}

#[test]
fn refinement_preserves_initial_turns_limits_and_branch_cut_unwrapping() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((-100.0, 1.0), (-100.0, -1.0));

    for configuration in configurations() {
        let canonical = robot.inverse(path.start(), configuration).unwrap();
        let initial = angles(
            canonical.first_theta() + core::f64::consts::TAU,
            canonical.second_theta() - core::f64::consts::TAU,
        );
        let matching_limits = limits(
            initial.first_theta() - 0.1,
            initial.first_theta() + 0.1,
            initial.second_theta() - 0.1,
            initial.second_theta() + 0.1,
        );
        let trajectory = JointTrajectory::new(&robot, &path, 0.25, configuration)
            .unwrap()
            .with_initial_position(initial, 1e-10, 1e-10)
            .unwrap()
            .with_limits(&matching_limits)
            .unwrap()
            .with_max_joint_step(0.1)
            .unwrap();
        let expected: Vec<_> = trajectory.clone().map(Result::unwrap).collect();
        let actual: Vec<_> = trajectory
            .into_validated_with_refinement(4)
            .unwrap()
            .map(Result::unwrap)
            .collect();

        assert_eq!(actual.len(), expected.len());
        for (actual, expected) in actual.into_iter().zip(expected) {
            assert_angles_exact(actual, expected);
            assert_eq!(matching_limits.contains(actual), Ok(true));
            assert!(actual.first_theta() > PI);
            assert!(actual.second_theta().abs() > PI);
        }
    }
}

#[test]
fn refinement_accepts_degenerate_paths_and_subnormal_spacing() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (160.0, 0.0));

    for configuration in configurations() {
        for spacing in [MIN_POSITIVE_STEP, 1.0, f64::MAX] {
            let mut validated = JointTrajectory::new(&robot, &path, spacing, configuration)
                .unwrap()
                .with_max_joint_step(MIN_POSITIVE_STEP)
                .unwrap()
                .into_validated_with_refinement(2)
                .unwrap();
            let first = unwrap_angles(validated.next());
            let second = unwrap_angles(validated.next());
            assert_eq!(second.first_theta(), first.first_theta());
            assert_eq!(second.second_theta(), first.second_theta());
            assert_point_close(forward(&robot, first), path.start(), 160.0);
            assert!(validated.next().is_none());
            assert!(validated.next().is_none());
        }
    }
}

#[test]
fn refinement_rejects_started_exhausted_and_failed_trajectories_before_checking_budget() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (140.0, 0.0));

    for configuration in configurations() {
        let mut partial = JointTrajectory::new(&robot, &path, 5.0, configuration).unwrap();
        assert!(partial.next().unwrap().is_ok());
        let mut exhausted = JointTrajectory::new(&robot, &path, 5.0, configuration).unwrap();
        assert!(exhausted.by_ref().all(|item| item.is_ok()));
        assert!(exhausted.next().is_none());
        let mut failed = JointTrajectory::new(&robot, &path, 5.0, configuration)
            .unwrap()
            .with_max_joint_step(MIN_POSITIVE_STEP)
            .unwrap();
        assert!(failed.next().unwrap().is_ok());
        assert_next_error(failed.next(), PlannerError::MaxJointStepExceeded);
        let mut failed_and_exhausted = failed.clone();
        assert!(failed_and_exhausted.next().is_none());

        for trajectory in [partial, exhausted, failed, failed_and_exhausted] {
            for budget in [0, 1, 8] {
                assert_eq!(
                    trajectory
                        .clone()
                        .into_validated_with_refinement(budget)
                        .unwrap_err(),
                    PlannerError::TrajectoryAlreadyStarted
                );
            }
        }
    }
}

#[test]
fn refinement_preserves_joint_limit_errors_without_retrying() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (140.0, 0.0));
    let restrictive_limits = limits(0.0, 0.0, 0.0, 0.0);
    for configuration in configurations() {
        let trajectory = JointTrajectory::new(&robot, &path, 20.0, configuration)
            .unwrap()
            .with_limits(&restrictive_limits)
            .unwrap();
        assert_validation_error(&trajectory, PlannerError::AngleOutOfLimits);
    }
}

#[test]
fn refinement_preserves_numerical_failures_without_retrying() {
    let path = line((f64::MAX, 0.0), (f64::MAX, 0.0));

    for lengths in [(MIN_POSITIVE_STEP, f64::MAX), (f64::MAX, MIN_POSITIVE_STEP)] {
        let robot = Scara::new(lengths.0, lengths.1).unwrap();
        for configuration in configurations() {
            let trajectory = JointTrajectory::new(&robot, &path, f64::MAX, configuration).unwrap();
            assert_eq!(
                trajectory.clone().validate(),
                Err(PlannerError::Kinematics(KinematicsError::NumericalFailure))
            );
            for budget in [0, 1, 4, usize::MAX] {
                assert_eq!(
                    trajectory
                        .clone()
                        .into_validated_with_refinement(budget)
                        .unwrap_err(),
                    PlannerError::Kinematics(KinematicsError::NumericalFailure)
                );
            }
        }
    }
}

#[test]
fn refinement_does_not_relax_an_initial_position_step_violation() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (160.0, 0.0));

    for configuration in configurations() {
        let trajectory = JointTrajectory::new(&robot, &path, 1.0, configuration)
            .unwrap()
            .with_initial_position(angles(0.1, 0.0), f64::MAX, f64::MAX)
            .unwrap()
            .with_max_joint_step(0.01)
            .unwrap();
        assert_eq!(
            trajectory.clone().validate(),
            Err(PlannerError::MaxJointStepExceeded)
        );
        assert_eq!(
            trajectory.into_validated_with_refinement(8).unwrap_err(),
            PlannerError::MaxRefinementAttemptsReached
        );
    }
}

fn largest_joint_step(samples: &[JointAngles]) -> f64 {
    samples
        .windows(2)
        .map(|pair| {
            (pair[1].first_theta() - pair[0].first_theta())
                .abs()
                .max((pair[1].second_theta() - pair[0].second_theta()).abs())
        })
        .fold(0.0, f64::max)
}

// These regressions require actual resampling: reducing a stored distance alone
// cannot repair a MaxJointStepExceeded error in the existing point iterator.
#[test]
fn refinement_recovers_a_joint_step_violation_after_one_halving() {
    let mut failures = Vec::new();
    for scale in [1e-120, 1.0, 1e120] {
        let robot = Scara::new(100.0 * scale, 60.0 * scale).unwrap();
        let path = line((160.0 * scale, 0.0), (140.0 * scale, 0.0));
        for configuration in configurations() {
            let coarse = JointTrajectory::new(&robot, &path, 20.0 * scale, configuration).unwrap();
            let fine = JointTrajectory::new(&robot, &path, 10.0 * scale, configuration).unwrap();
            let coarse_samples: Vec<_> = coarse.clone().map(Result::unwrap).collect();
            let fine_samples: Vec<_> = fine.clone().map(Result::unwrap).collect();
            let coarse_step = largest_joint_step(&coarse_samples);
            let fine_step = largest_joint_step(&fine_samples);
            assert!(fine_step < coarse_step);
            let threshold = (coarse_step + fine_step) / 2.0;
            let trajectory = coarse.with_max_joint_step(threshold).unwrap();

            assert_eq!(
                trajectory.clone().validate(),
                Err(PlannerError::MaxJointStepExceeded)
            );
            assert_eq!(
                fine.with_max_joint_step(threshold).unwrap().validate(),
                Ok(())
            );
            assert_eq!(
                trajectory
                    .clone()
                    .into_validated_with_refinement(0)
                    .unwrap_err(),
                PlannerError::MaxRefinementAttemptsReached
            );

            let validated = match trajectory.into_validated_with_refinement(1) {
                Ok(validated) => validated,
                Err(error) => {
                    failures.push((scale, configuration, error));
                    continue;
                }
            };
            let actual: Vec<_> = validated.map(Result::unwrap).collect();
            assert!(actual.len() > coarse_samples.len());
            assert_eq!(actual.len(), fine_samples.len());
            assert!(largest_joint_step(&actual) <= threshold);
            for (actual, expected) in actual.into_iter().zip(fine_samples) {
                assert_angles_exact(actual, expected);
            }
        }
    }
    assert!(
        failures.is_empty(),
        "one halving should repair the excessive joint step: {failures:?}"
    );
}

#[test]
fn refinement_recovers_after_two_halvings_preserving_initial_turns_and_limits() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (140.0, 0.0));
    let mut failures = Vec::new();

    for configuration in configurations() {
        let initial = angles(core::f64::consts::TAU, -core::f64::consts::TAU);
        let matching_limits = limits(PI, 3.0 * PI, -3.0 * PI, -PI);
        let build = |spacing| {
            JointTrajectory::new(&robot, &path, spacing, configuration)
                .unwrap()
                .with_initial_position(initial, 1e-10, 1e-10)
                .unwrap()
                .with_limits(&matching_limits)
                .unwrap()
        };
        let middle: Vec<_> = build(10.0).map(Result::unwrap).collect();
        let expected: Vec<_> = build(5.0).map(Result::unwrap).collect();
        let middle_step = largest_joint_step(&middle);
        let fine_step = largest_joint_step(&expected);
        assert!(fine_step < middle_step);
        let threshold = (middle_step + fine_step) / 2.0;

        for spacing in [20.0, 10.0] {
            assert_eq!(
                build(spacing)
                    .with_max_joint_step(threshold)
                    .unwrap()
                    .validate(),
                Err(PlannerError::MaxJointStepExceeded)
            );
        }
        assert_eq!(
            build(5.0)
                .with_max_joint_step(threshold)
                .unwrap()
                .validate(),
            Ok(())
        );
        let trajectory = build(20.0).with_max_joint_step(threshold).unwrap();
        assert_eq!(
            trajectory
                .clone()
                .into_validated_with_refinement(1)
                .unwrap_err(),
            PlannerError::MaxRefinementAttemptsReached
        );

        let validated = match trajectory.into_validated_with_refinement(2) {
            Ok(validated) => validated,
            Err(error) => {
                failures.push((configuration, error));
                continue;
            }
        };
        let actual: Vec<_> = validated.map(Result::unwrap).collect();
        assert_eq!(actual.len(), expected.len());
        assert!(largest_joint_step(&actual) <= threshold);
        for (actual, expected) in actual.into_iter().zip(expected) {
            assert_angles_exact(actual, expected);
            assert_eq!(matching_limits.contains(actual), Ok(true));
        }
    }
    assert!(
        failures.is_empty(),
        "two halvings should repair the excessive joint step: {failures:?}"
    );
}

#[test]
fn refinement_rejects_spacing_that_underflows_to_zero() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (160.0, 0.0));
    assert_eq!(MIN_POSITIVE_STEP / 2.0, 0.0);

    for configuration in configurations() {
        let trajectory = JointTrajectory::new(&robot, &path, MIN_POSITIVE_STEP, configuration)
            .unwrap()
            .with_initial_position(angles(0.1, 0.0), f64::MAX, f64::MAX)
            .unwrap()
            .with_max_joint_step(0.01)
            .unwrap();
        assert_eq!(
            trajectory.clone().validate(),
            Err(PlannerError::MaxJointStepExceeded)
        );
        // No refinement budget means no attempt to halve the spacing.
        assert_eq!(
            trajectory
                .clone()
                .into_validated_with_refinement(0)
                .unwrap_err(),
            PlannerError::MaxRefinementAttemptsReached
        );
        assert_eq!(
            trajectory.into_validated_with_refinement(1).unwrap_err(),
            PlannerError::NumericalFailure
        );
    }
}

#[test]
fn refinement_propagates_too_many_segments_when_resampling() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (159.0, 0.0));
    let spacing = 2.0_f64.powi(1 - usize::BITS as i32);
    assert!(path.points(spacing).is_ok());
    assert!(matches!(
        path.points(spacing / 2.0),
        Err(TrajectoryError::TooManySegments)
    ));

    for configuration in configurations() {
        let trajectory = JointTrajectory::new(&robot, &path, spacing, configuration)
            .unwrap()
            .with_initial_position(angles(0.1, 0.0), f64::MAX, f64::MAX)
            .unwrap()
            .with_max_joint_step(0.01)
            .unwrap();
        // Fail at the first point rather than iterating a huge sample count.
        assert_eq!(
            trajectory.clone().validate(),
            Err(PlannerError::MaxJointStepExceeded)
        );
        assert_eq!(
            trajectory
                .clone()
                .into_validated_with_refinement(0)
                .unwrap_err(),
            PlannerError::MaxRefinementAttemptsReached
        );
        assert_eq!(
            trajectory.into_validated_with_refinement(1).unwrap_err(),
            PlannerError::Trajectory(TrajectoryError::TooManySegments)
        );
    }
}

#[test]
fn refinement_propagates_joint_limit_violations_discovered_after_resampling() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (140.0, 0.0));
    let restrictive_limits = limits(0.0, 0.0, 0.0, 0.0);

    for configuration in configurations() {
        let coarse = JointTrajectory::new(&robot, &path, 20.0, configuration).unwrap();
        let fine = JointTrajectory::new(&robot, &path, 10.0, configuration).unwrap();
        let coarse_samples: Vec<_> = coarse.clone().map(Result::unwrap).collect();
        let fine_samples: Vec<_> = fine.clone().map(Result::unwrap).collect();
        let threshold =
            (largest_joint_step(&coarse_samples) + largest_joint_step(&fine_samples)) / 2.0;
        let trajectory = coarse
            .with_limits(&restrictive_limits)
            .unwrap()
            .with_max_joint_step(threshold)
            .unwrap();
        assert_eq!(
            trajectory.clone().validate(),
            Err(PlannerError::MaxJointStepExceeded)
        );
        assert_eq!(
            fine.with_limits(&restrictive_limits)
                .unwrap()
                .with_max_joint_step(threshold)
                .unwrap()
                .validate(),
            Err(PlannerError::AngleOutOfLimits)
        );
        assert_eq!(
            trajectory.into_validated_with_refinement(1).unwrap_err(),
            PlannerError::AngleOutOfLimits
        );
    }
}
