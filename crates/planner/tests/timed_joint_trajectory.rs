use core::f64::consts::{FRAC_PI_2, PI, TAU};

use kinematics::{ElbowConfiguration, JointAngles, Point, Scara};
use planner::{
    JointLimits, JointRange, JointTrajectory, JointVelocityLimits, PlannerError, TimedJointSample,
    TimedJointTrajectory, ValidatedTrajectory,
};
use trajectory::LineSegment;

const SMALLEST: f64 = f64::from_bits(1);

fn configurations() -> [ElbowConfiguration; 2] {
    [ElbowConfiguration::Positive, ElbowConfiguration::Negative]
}

fn line(start: (f64, f64), end: (f64, f64)) -> LineSegment {
    LineSegment::new(
        Point::new(start.0, start.1).unwrap(),
        Point::new(end.0, end.1).unwrap(),
    )
    .expect("test segment must have a finite length")
}

fn validated<'a>(
    robot: &'a Scara,
    path: &'a LineSegment,
    spacing: f64,
    configuration: ElbowConfiguration,
) -> ValidatedTrajectory<'a> {
    JointTrajectory::new(robot, path, spacing, configuration)
        .unwrap()
        .into_validated()
        .unwrap()
}

fn velocities(first: f64, second: f64) -> JointVelocityLimits {
    JointVelocityLimits::new(first, second).unwrap()
}

fn sample(item: Option<Result<TimedJointSample, PlannerError>>) -> TimedJointSample {
    item.expect("a timed sample must exist")
        .expect("the timed sample must be valid")
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

fn assert_close(actual: f64, expected: f64) {
    assert!(actual.is_finite());
    assert!(
        (actual - expected).abs() <= 1e-12 * expected.abs().max(1.0),
        "expected time {expected}, got {actual}"
    );
}

fn assert_error_then_finished(trajectory: &mut TimedJointTrajectory<'_>, expected: PlannerError) {
    match trajectory.next() {
        Some(Err(error)) => assert_eq!(error, expected),
        Some(Ok(sample)) => panic!(
            "expected {expected:?}, got angle {:?} at time {}",
            sample.angle, sample.time_elapsed
        ),
        None => panic!("expected {expected:?}, but the iterator ended silently"),
    }
    for _ in 0..3 {
        assert!(trajectory.next().is_none());
    }
}

#[test]
fn first_sample_is_at_zero_and_a_known_single_move_has_the_expected_duration() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (100.0, 60.0));
    for configuration in configurations() {
        for (first_speed, second_speed, expected) in [(10.0, 1.0, FRAC_PI_2), (20.0, 2.0, PI / 4.0)]
        {
            let mut timed = TimedJointTrajectory::new(
                validated(&robot, &path, f64::MAX, configuration),
                velocities(first_speed, second_speed),
            );
            let start = sample(timed.next());
            let end = sample(timed.next());
            assert_eq!(start.time_elapsed, 0.0);
            assert_angles_exact(
                start.angle,
                robot.inverse(path.start(), configuration).unwrap(),
            );
            assert_angles_exact(end.angle, robot.inverse(path.end(), configuration).unwrap());
            // The second joint moves pi/2 radians in either elbow branch.
            assert_close(end.time_elapsed, expected);
            assert!(timed.next().is_none());
            assert!(timed.next().is_none());
        }
    }
}

#[test]
fn every_angle_is_preserved_and_times_accumulate_the_independent_velocity_limits() {
    for scale in [1e-200, 1.0, 1e200] {
        let robot = Scara::new(100.0 * scale, 60.0 * scale).unwrap();
        let path = line((160.0 * scale, 0.0), (140.0 * scale, 0.0));
        for configuration in configurations() {
            for (first_speed, second_speed) in [(0.25, 10.0), (10.0, 0.25), (1.0, 2.0)] {
                let trajectory =
                    // Four intervals with margin for Cartesian rounding; the
                    // independent timing oracle below is unchanged.
                    JointTrajectory::new(&robot, &path, 6.0 * scale, configuration).unwrap();
                let expected: Vec<_> = trajectory.clone().map(Result::unwrap).collect();
                assert!(expected.len() > 2);
                let actual: Vec<_> = TimedJointTrajectory::new(
                    trajectory.into_validated().unwrap(),
                    velocities(first_speed, second_speed),
                )
                .map(Result::unwrap)
                .collect();
                assert_eq!(actual.len(), expected.len());
                let reference_limits = velocities(first_speed, second_speed);
                let mut reference_time = 0.0;
                let mut previous_time = 0.0;
                for (index, (actual, expected_angle)) in actual.iter().zip(&expected).enumerate() {
                    assert_angles_exact(actual.angle, *expected_angle);
                    if index > 0 {
                        reference_time += reference_limits
                            .min_duration_between(expected[index - 1], *expected_angle)
                            .unwrap();
                        assert!(actual.time_elapsed > previous_time);
                    }
                    assert_close(actual.time_elapsed, reference_time);
                    previous_time = actual.time_elapsed;
                }
            }
        }
    }
}

#[test]
fn scaling_both_velocity_limits_scales_every_elapsed_time() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (140.0, 0.0));
    for configuration in configurations() {
        let build = |factor| {
            TimedJointTrajectory::new(
                validated(&robot, &path, 5.0, configuration),
                velocities(factor, 2.0 * factor),
            )
            .map(Result::unwrap)
            .collect::<Vec<_>>()
        };
        let baseline = build(1.0);
        for factor in [0.5, 2.0, 4.0] {
            let actual = build(factor);
            assert_eq!(actual.len(), baseline.len());
            for (actual, baseline) in actual.into_iter().zip(&baseline) {
                assert_angles_exact(actual.angle, baseline.angle);
                assert_close(actual.time_elapsed, baseline.time_elapsed / factor);
            }
        }
    }
}

#[test]
fn degenerate_paths_keep_both_samples_at_zero_even_with_extreme_speeds_and_spacing() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (160.0, 0.0));
    for configuration in configurations() {
        for spacing in [SMALLEST, 1.0, f64::MAX] {
            for speed in [SMALLEST, 1.0, f64::MAX] {
                let mut timed = TimedJointTrajectory::new(
                    validated(&robot, &path, spacing, configuration),
                    velocities(speed, speed),
                );
                let first = sample(timed.next());
                let second = sample(timed.next());
                // Unwrapping may turn -0.0 into +0.0 without movement.
                assert_eq!(first.angle.first_theta(), second.angle.first_theta());
                assert_eq!(first.angle.second_theta(), second.angle.second_theta());
                assert_eq!(first.time_elapsed, 0.0);
                assert_eq!(second.time_elapsed, 0.0);
                assert!(timed.next().is_none());
                assert!(timed.next().is_none());
            }
        }
    }
}

#[test]
fn an_exhausted_validated_trajectory_stays_empty_when_timed() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (140.0, 0.0));
    for configuration in configurations() {
        let mut source = validated(&robot, &path, 5.0, configuration);
        assert!(source.by_ref().count() > 2);
        let mut timed = TimedJointTrajectory::new(source, velocities(1.0, 1.0));
        for _ in 0..3 {
            assert!(timed.next().is_none());
        }
    }
}

#[test]
fn partially_consumed_input_starts_the_remaining_sequence_at_time_zero() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (140.0, 0.0));
    for configuration in configurations() {
        let mut source = validated(&robot, &path, 5.0, configuration);
        assert!(source.next().unwrap().is_ok());
        assert!(source.next().unwrap().is_ok());
        let expected: Vec<_> = validated(&robot, &path, 5.0, configuration)
            .skip(2)
            .map(Result::unwrap)
            .collect();
        let actual: Vec<_> = TimedJointTrajectory::new(source, velocities(1.0, 1.0))
            .map(Result::unwrap)
            .collect();
        assert_eq!(actual.len(), expected.len());
        assert_eq!(actual[0].time_elapsed, 0.0);
        for (actual, expected) in actual.iter().zip(&expected) {
            assert_angles_exact(actual.angle, *expected);
        }
        for pair in actual.windows(2) {
            assert!(pair[1].time_elapsed > pair[0].time_elapsed);
        }
    }
}

#[test]
fn timing_preserves_initial_turns_limits_and_branch_cut_unwrapping() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((-100.0, 1.0), (-100.0, -1.0));
    for configuration in configurations() {
        let canonical = robot.inverse(path.start(), configuration).unwrap();
        let initial = JointAngles::new(
            canonical.first_theta() + TAU,
            canonical.second_theta() - TAU,
        )
        .unwrap();
        let limits = JointLimits::new(
            JointRange::new(initial.first_theta() - 0.1, initial.first_theta() + 0.1).unwrap(),
            JointRange::new(initial.second_theta() - 0.1, initial.second_theta() + 0.1).unwrap(),
        );
        let source = JointTrajectory::new(&robot, &path, 0.25, configuration)
            .unwrap()
            .with_initial_position(initial, 1e-10, 1e-10)
            .unwrap()
            .with_limits(&limits)
            .unwrap();
        let expected: Vec<_> = source.clone().map(Result::unwrap).collect();
        let actual: Vec<_> =
            TimedJointTrajectory::new(source.into_validated().unwrap(), velocities(4.0, 2.0))
                .map(Result::unwrap)
                .collect();
        assert_eq!(actual.len(), expected.len());
        assert_eq!(actual[0].time_elapsed, 0.0);
        assert!(actual.last().unwrap().time_elapsed < 1.0);
        for (sample, expected) in actual.into_iter().zip(expected) {
            assert_angles_exact(sample.angle, expected);
            assert_eq!(limits.contains(sample.angle), Ok(true));
            assert!(sample.angle.first_theta() > PI);
            assert!(sample.angle.second_theta().abs() > PI);
        }
    }
}

#[test]
fn a_refined_trajectory_keeps_every_added_sample_when_timed() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (140.0, 0.0));
    for configuration in configurations() {
        let coarse = JointTrajectory::new(&robot, &path, 20.0, configuration).unwrap();
        let expected: Vec<_> = JointTrajectory::new(&robot, &path, 10.0, configuration)
            .unwrap()
            .map(Result::unwrap)
            .collect();
        let largest_step = |angles: &[JointAngles]| {
            angles
                .windows(2)
                .map(|pair| {
                    (pair[1].first_theta() - pair[0].first_theta())
                        .abs()
                        .max((pair[1].second_theta() - pair[0].second_theta()).abs())
                })
                .fold(0.0, f64::max)
        };
        let coarse_samples: Vec<_> = coarse.clone().map(Result::unwrap).collect();
        let threshold = (largest_step(&coarse_samples) + largest_step(&expected)) / 2.0;
        let source = coarse.with_max_joint_step(threshold).unwrap();
        assert_eq!(
            source.clone().validate(),
            Err(PlannerError::MaxJointStepExceeded)
        );
        let refined = source.into_validated_with_refinement(1).unwrap();
        let actual: Vec<_> = TimedJointTrajectory::new(refined, velocities(1.0, 1.0))
            .map(Result::unwrap)
            .collect();
        assert!(actual.len() > coarse_samples.len());
        assert_eq!(actual.len(), expected.len());
        for (actual, expected) in actual.into_iter().zip(expected) {
            assert_angles_exact(actual.angle, expected);
            assert!(actual.time_elapsed.is_finite());
        }
    }
}

#[test]
fn step_time_overflow_is_returned_once_and_stops_before_later_source_samples() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (140.0, 0.0));
    for configuration in configurations() {
        let source = validated(&robot, &path, 5.0, configuration);
        let mut timed = TimedJointTrajectory::new(source, velocities(SMALLEST, SMALLEST));
        assert_eq!(sample(timed.next()).time_elapsed, 0.0);
        assert_error_then_finished(&mut timed, PlannerError::NumericalFailure);
    }
}

#[test]
fn step_time_underflow_is_returned_once_after_a_valid_initial_sample() {
    let robot = Scara::new(0.5, 0.5).unwrap();
    let path = line((1.0, 0.0), (1.0, SMALLEST));
    for configuration in configurations() {
        let expected: Vec<_> = validated(&robot, &path, f64::MAX, configuration)
            .map(Result::unwrap)
            .collect();
        assert_eq!(
            expected[1].first_theta() - expected[0].first_theta(),
            SMALLEST
        );
        let mut timed = TimedJointTrajectory::new(
            validated(&robot, &path, f64::MAX, configuration),
            velocities(f64::MAX, f64::MAX),
        );
        assert_eq!(sample(timed.next()).time_elapsed, 0.0);
        assert_error_then_finished(&mut timed, PlannerError::NumericalFailure);
    }
}

#[test]
fn the_smallest_positive_elapsed_time_is_accepted() {
    let robot = Scara::new(0.5, 0.5).unwrap();
    let path = line((1.0, 0.0), (1.0, SMALLEST));
    for configuration in configurations() {
        let mut timed = TimedJointTrajectory::new(
            validated(&robot, &path, f64::MAX, configuration),
            velocities(1.0, 1.0),
        );
        assert_eq!(sample(timed.next()).time_elapsed, 0.0);
        assert_eq!(sample(timed.next()).time_elapsed, SMALLEST);
        assert!(timed.next().is_none());
    }
}

#[test]
fn a_large_finite_elapsed_time_is_accepted() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (100.0, 60.0));
    for configuration in configurations() {
        let mut timed = TimedJointTrajectory::new(
            validated(&robot, &path, f64::MAX, configuration),
            velocities(1.0, 1e-308),
        );
        assert_eq!(sample(timed.next()).time_elapsed, 0.0);
        let last = sample(timed.next());
        assert!(last.time_elapsed > f64::MAX / 2.0);
        assert_close(last.time_elapsed, FRAC_PI_2 / 1e-308);
        assert!(timed.next().is_none());
    }
}

#[test]
fn regression_accumulated_time_overflow_is_reported_once_and_finishes_iteration() {
    let robot = Scara::new(1.0, 1.0).unwrap();
    let path = line((1.0, 0.0), (-1.0, 0.0));
    for configuration in configurations() {
        let expected: Vec<_> = validated(&robot, &path, 1.0, configuration)
            .map(Result::unwrap)
            .collect();
        assert_eq!(expected.len(), 3);
        let reference = velocities(2e-308, 2e-308);
        let durations: Vec<_> = expected
            .windows(2)
            .map(|pair| reference.min_duration_between(pair[0], pair[1]).unwrap())
            .collect();
        assert!(durations.iter().all(|duration| duration.is_finite()));
        assert!(durations.iter().sum::<f64>().is_infinite());
        let mut timed = TimedJointTrajectory::new(
            validated(&robot, &path, 1.0, configuration),
            velocities(2e-308, 2e-308),
        );
        assert_eq!(sample(timed.next()).time_elapsed, 0.0);
        assert!(sample(timed.next()).time_elapsed.is_finite());
        assert_error_then_finished(&mut timed, PlannerError::TimeElapsedOverflow);
    }
}

#[test]
fn a_positive_step_lost_in_a_large_accumulated_time_returns_precision_lost() {
    let robot = Scara::new(6.0, 4.0).unwrap();
    let path = line((3.0, 0.0), (5.0, 0.0));
    for configuration in configurations() {
        let angles: Vec<_> = validated(&robot, &path, 1.0, configuration)
            .map(Result::unwrap)
            .collect();
        assert_eq!(angles.len(), 3);
        let limits = velocities(1e-300, 1.0);
        let first_delta = limits.min_duration_between(angles[0], angles[1]).unwrap();
        let last_delta = limits.min_duration_between(angles[1], angles[2]).unwrap();
        assert!(first_delta.is_finite() && first_delta > 0.0);
        assert!(last_delta.is_finite() && last_delta > 0.0);
        assert_eq!(first_delta + last_delta, first_delta);
        let mut timed = TimedJointTrajectory::new(
            validated(&robot, &path, 1.0, configuration),
            velocities(1e-300, 1.0),
        );
        assert_eq!(sample(timed.next()).time_elapsed, 0.0);
        assert_eq!(sample(timed.next()).time_elapsed, first_delta);
        assert_error_then_finished(&mut timed, PlannerError::TimeElapsedPrecisionLost);
    }
}
