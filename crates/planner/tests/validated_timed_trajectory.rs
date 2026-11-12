use core::f64::consts::{FRAC_PI_2, PI, TAU};

use kinematics::{ElbowConfiguration, JointAngles, Point, Scara};
use planner::{
    JointLimits, JointRange, JointTrajectory, JointVelocityLimits, PlannerError, TimedJointSample,
    TimedJointTrajectory, ValidatedTimedTrajectory,
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

fn timed<'a>(
    robot: &'a Scara,
    path: &'a LineSegment,
    spacing: f64,
    configuration: ElbowConfiguration,
    first_speed: f64,
    second_speed: f64,
) -> TimedJointTrajectory<'a> {
    TimedJointTrajectory::new(
        JointTrajectory::new(robot, path, spacing, configuration)
            .unwrap()
            .into_validated()
            .unwrap(),
        JointVelocityLimits::new(first_speed, second_speed).unwrap(),
    )
}

fn assert_same_samples(actual: &[TimedJointSample], expected: &[TimedJointSample]) {
    assert_eq!(actual.len(), expected.len());
    for (actual, expected) in actual.iter().zip(expected) {
        assert_eq!(
            actual.angle.first_theta().to_bits(),
            expected.angle.first_theta().to_bits()
        );
        assert_eq!(
            actual.angle.second_theta().to_bits(),
            expected.angle.second_theta().to_bits()
        );
        assert_eq!(
            actual.time_elapsed.to_bits(),
            expected.time_elapsed.to_bits()
        );
    }
}

fn assert_validation_error(source: &TimedJointTrajectory<'_>, expected: PlannerError) {
    assert_eq!(source.clone().validate(), Err(expected));
    assert_eq!(source.clone().into_validated().unwrap_err(), expected);
}

#[test]
fn validation_preserves_the_full_timed_sequence_for_both_branches_and_extreme_scales() {
    for scale in [1e-200, 1.0, 1e200] {
        let robot = Scara::new(100.0 * scale, 60.0 * scale).unwrap();
        let path = line((160.0 * scale, 0.0), (140.0 * scale, 0.0));
        for configuration in configurations() {
            for spacing in [7.0 * scale, f64::MAX] {
                let source = timed(&robot, &path, spacing, configuration, 1.0, 2.0);
                let expected: Vec<_> = source.clone().map(Result::unwrap).collect();
                assert_eq!(source.clone().validate(), Ok(()));
                let mut wrapped: ValidatedTimedTrajectory<'_> = source.into_validated().unwrap();
                // This annotation checks the public Iterator::Item contract.
                let results: Vec<Result<TimedJointSample, PlannerError>> =
                    wrapped.by_ref().collect();
                let actual: Vec<_> = results.into_iter().map(Result::unwrap).collect();
                assert_same_samples(&actual, &expected);
                assert_eq!(actual[0].time_elapsed, 0.0);
                assert!(wrapped.next().is_none());
                assert!(wrapped.next().is_none());
            }
        }
    }
}

#[test]
fn a_known_single_move_keeps_its_analytical_duration_after_validation() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (100.0, 60.0));
    for configuration in configurations() {
        let mut wrapped = timed(&robot, &path, f64::MAX, configuration, 10.0, 1.0)
            .into_validated()
            .unwrap();
        assert_eq!(wrapped.next().unwrap().unwrap().time_elapsed, 0.0);
        let end = wrapped.next().unwrap().unwrap();
        assert!((end.time_elapsed - FRAC_PI_2).abs() <= 1e-12);
        assert!(wrapped.next().is_none());
    }
}

#[test]
fn stationary_trajectories_keep_both_zero_time_samples_with_extreme_spacing_and_speeds() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (160.0, 0.0));
    for configuration in configurations() {
        for spacing in [SMALLEST, 1.0, f64::MAX] {
            for speed in [SMALLEST, 1.0, f64::MAX] {
                let source = timed(&robot, &path, spacing, configuration, speed, speed);
                assert_eq!(source.clone().validate(), Ok(()));
                let actual: Vec<_> = source
                    .into_validated()
                    .unwrap()
                    .map(Result::unwrap)
                    .collect();
                assert_eq!(actual.len(), 2);
                assert!(actual.iter().all(|sample| sample.time_elapsed == 0.0));
                assert_eq!(actual[0].angle.first_theta(), actual[1].angle.first_theta());
                assert_eq!(
                    actual[0].angle.second_theta(),
                    actual[1].angle.second_theta()
                );
            }
        }
    }
}

#[test]
fn validation_accepts_subnormal_and_near_maximum_finite_elapsed_times() {
    let small_robot = Scara::new(0.5, 0.5).unwrap();
    let small_path = line((1.0, 0.0), (1.0, SMALLEST));
    let large_robot = Scara::new(100.0, 60.0).unwrap();
    let large_path = line((160.0, 0.0), (100.0, 60.0));
    for configuration in configurations() {
        for (robot, path, first_speed, second_speed, expected) in [
            (&small_robot, &small_path, 1.0, 1.0, SMALLEST),
            (&large_robot, &large_path, 1.0, 1e-308, FRAC_PI_2 / 1e-308),
        ] {
            let source = timed(
                robot,
                path,
                f64::MAX,
                configuration,
                first_speed,
                second_speed,
            );
            assert_eq!(source.clone().validate(), Ok(()));
            let samples: Vec<_> = source
                .into_validated()
                .unwrap()
                .map(Result::unwrap)
                .collect();
            assert_eq!(samples.len(), 2);
            assert_eq!(samples[0].time_elapsed, 0.0);
            let time = samples[1].time_elapsed;
            assert!(time.is_finite() && time > 0.0);
            if expected == SMALLEST {
                assert_eq!(time, SMALLEST);
            } else {
                assert!((time - expected).abs() <= 1e-12 * expected);
            }
        }
    }
}

#[test]
fn validation_propagates_step_duration_overflow_and_underflow_before_creating_a_wrapper() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (140.0, 0.0));
    let tiny_robot = Scara::new(0.5, 0.5).unwrap();
    let tiny_path = line((1.0, 0.0), (1.0, SMALLEST));
    for configuration in configurations() {
        for source in [
            timed(&robot, &path, 5.0, configuration, SMALLEST, SMALLEST),
            timed(
                &tiny_robot,
                &tiny_path,
                f64::MAX,
                configuration,
                f64::MAX,
                f64::MAX,
            ),
        ] {
            let mut direct = source.clone();
            assert_eq!(direct.next().unwrap().unwrap().time_elapsed, 0.0);
            assert_eq!(
                direct.next().unwrap().unwrap_err(),
                PlannerError::NumericalFailure
            );
            assert_validation_error(&source, PlannerError::NumericalFailure);
        }
    }
}

#[test]
fn validation_propagates_accumulation_overflow_after_a_valid_prefix() {
    let robot = Scara::new(1.0, 1.0).unwrap();
    let path = line((1.0, 0.0), (-1.0, 0.0));
    for configuration in configurations() {
        let source = timed(&robot, &path, 1.0, configuration, 2e-308, 2e-308);
        let mut direct = source.clone();
        assert_eq!(direct.next().unwrap().unwrap().time_elapsed, 0.0);
        assert!(direct.next().unwrap().unwrap().time_elapsed.is_finite());
        assert_eq!(
            direct.next().unwrap().unwrap_err(),
            PlannerError::TimeElapsedOverflow
        );
        assert_validation_error(&source, PlannerError::TimeElapsedOverflow);
    }
}

#[test]
fn validation_propagates_precision_loss_after_a_large_finite_elapsed_time() {
    let robot = Scara::new(6.0, 4.0).unwrap();
    let path = line((3.0, 0.0), (5.0, 0.0));
    for configuration in configurations() {
        let source = timed(&robot, &path, 1.0, configuration, 1e-300, 1.0);
        let mut direct = source.clone();
        assert_eq!(direct.next().unwrap().unwrap().time_elapsed, 0.0);
        assert!(direct.next().unwrap().unwrap().time_elapsed > 0.0);
        assert_eq!(
            direct.next().unwrap().unwrap_err(),
            PlannerError::TimeElapsedPrecisionLost
        );
        assert_validation_error(&source, PlannerError::TimeElapsedPrecisionLost);
    }
}

#[test]
fn the_wrapper_preserves_initial_turns_limits_and_branch_cut_unwrapping() {
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
        let source = TimedJointTrajectory::new(
            JointTrajectory::new(&robot, &path, 0.25, configuration)
                .unwrap()
                .with_initial_position(initial, 1e-10, 1e-10)
                .unwrap()
                .with_limits(&limits)
                .unwrap()
                .into_validated()
                .unwrap(),
            JointVelocityLimits::new(4.0, 2.0).unwrap(),
        );
        let expected: Vec<_> = source.clone().map(Result::unwrap).collect();
        let actual: Vec<_> = source
            .into_validated()
            .unwrap()
            .map(Result::unwrap)
            .collect();
        assert_same_samples(&actual, &expected);
        assert!(actual.last().unwrap().time_elapsed < 1.0);
        for sample in actual {
            assert_eq!(limits.contains(sample.angle), Ok(true));
            assert!(sample.angle.first_theta() > PI);
            assert!(sample.angle.second_theta().abs() > PI);
        }
    }
}

#[test]
fn validating_a_refined_timed_trajectory_does_not_drop_or_recreate_samples() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (140.0, 0.0));
    for configuration in configurations() {
        let coarse = JointTrajectory::new(&robot, &path, 20.0, configuration).unwrap();
        let fine = JointTrajectory::new(&robot, &path, 10.0, configuration).unwrap();
        let fine_samples: Vec<_> = fine.clone().map(Result::unwrap).collect();
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
        let threshold = (largest_step(&coarse_samples) + largest_step(&fine_samples)) / 2.0;
        let refined = coarse
            .with_max_joint_step(threshold)
            .unwrap()
            .into_validated_with_refinement(1)
            .unwrap();
        let actual: Vec<_> =
            TimedJointTrajectory::new(refined, JointVelocityLimits::new(1.0, 1.0).unwrap())
                .into_validated()
                .unwrap()
                .map(Result::unwrap)
                .collect();
        let expected: Vec<_> = TimedJointTrajectory::new(
            fine.into_validated().unwrap(),
            JointVelocityLimits::new(1.0, 1.0).unwrap(),
        )
        .map(Result::unwrap)
        .collect();
        assert!(actual.len() > coarse_samples.len());
        assert_same_samples(&actual, &expected);
    }
}

#[test]
fn validation_rejects_partially_consumed_timed_inputs_without_resetting_their_clock() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (140.0, 0.0));
    for configuration in configurations() {
        for prefix_length in [1, 2] {
            let mut source = timed(&robot, &path, 5.0, configuration, 1.0, 2.0);
            let expected: Vec<_> = source.clone().map(Result::unwrap).collect();
            for expected in &expected[..prefix_length] {
                let actual = source.next().unwrap().unwrap();
                assert_same_samples(&[actual], &[*expected]);
            }
            assert_validation_error(&source, PlannerError::TrajectoryAlreadyStarted);
            // Checking clones must neither consume nor reset the source.
            let remaining: Vec<_> = source.map(Result::unwrap).collect();
            assert_same_samples(&remaining, &expected[prefix_length..]);
            assert!(remaining[0].time_elapsed > expected[prefix_length - 1].time_elapsed);
        }
    }
}

#[test]
fn exhausted_or_already_failed_iterators_cannot_be_converted_to_a_validated_wrapper() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (140.0, 0.0));
    for configuration in configurations() {
        let mut exhausted = timed(&robot, &path, 5.0, configuration, 1.0, 2.0);
        let count = exhausted.clone().count();
        for _ in 0..count {
            assert!(exhausted.next().unwrap().is_ok());
        }
        // Consuming the last sample is enough; observing None is not required.
        let exhausted_before_none = exhausted.clone();
        assert!(exhausted.next().is_none());
        let mut failed = timed(&robot, &path, 5.0, configuration, SMALLEST, SMALLEST);
        assert!(failed.next().unwrap().is_ok());
        assert_eq!(
            failed.next().unwrap().unwrap_err(),
            PlannerError::NumericalFailure
        );
        let mut failed_after_none = failed.clone();
        assert!(failed_after_none.next().is_none());
        for source in [exhausted_before_none, exhausted, failed, failed_after_none] {
            assert_validation_error(&source, PlannerError::TrajectoryAlreadyStarted);
        }
    }
}

#[test]
fn observing_none_on_an_initially_empty_iterator_also_marks_it_started() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (140.0, 0.0));
    for configuration in configurations() {
        let mut joints = JointTrajectory::new(&robot, &path, 5.0, configuration)
            .unwrap()
            .into_validated()
            .unwrap();
        assert!(joints.by_ref().all(|sample| sample.is_ok()));
        let mut source =
            TimedJointTrajectory::new(joints, JointVelocityLimits::new(1.0, 1.0).unwrap());
        // A fresh empty timed iterator is valid until next() is actually called.
        assert_eq!(source.clone().validate(), Ok(()));
        let mut wrapper = source.clone().into_validated().unwrap();
        assert!(wrapper.next().is_none());
        assert!(wrapper.next().is_none());
        assert!(source.next().is_none());
        assert_validation_error(&source, PlannerError::TrajectoryAlreadyStarted);
        assert!(source.next().is_none());
        assert_validation_error(&source, PlannerError::TrajectoryAlreadyStarted);
    }
}

#[test]
fn already_started_takes_precedence_over_pending_and_observed_timing_errors() {
    let normal_robot = Scara::new(100.0, 60.0).unwrap();
    let normal_path = line((160.0, 0.0), (140.0, 0.0));
    let tiny_robot = Scara::new(0.5, 0.5).unwrap();
    let tiny_path = line((1.0, 0.0), (1.0, SMALLEST));
    let diameter_robot = Scara::new(1.0, 1.0).unwrap();
    let diameter = line((1.0, 0.0), (-1.0, 0.0));
    let precision_robot = Scara::new(6.0, 4.0).unwrap();
    let precision_path = line((3.0, 0.0), (5.0, 0.0));
    for configuration in configurations() {
        for (mut source, prefix_length, error) in [
            (
                timed(
                    &normal_robot,
                    &normal_path,
                    5.0,
                    configuration,
                    SMALLEST,
                    SMALLEST,
                ),
                1,
                PlannerError::NumericalFailure,
            ),
            (
                timed(
                    &tiny_robot,
                    &tiny_path,
                    f64::MAX,
                    configuration,
                    f64::MAX,
                    f64::MAX,
                ),
                1,
                PlannerError::NumericalFailure,
            ),
            (
                timed(
                    &diameter_robot,
                    &diameter,
                    1.0,
                    configuration,
                    2e-308,
                    2e-308,
                ),
                2,
                PlannerError::TimeElapsedOverflow,
            ),
            (
                timed(
                    &precision_robot,
                    &precision_path,
                    1.0,
                    configuration,
                    1e-300,
                    1.0,
                ),
                2,
                PlannerError::TimeElapsedPrecisionLost,
            ),
        ] {
            assert_validation_error(&source, error);
            for _ in 0..prefix_length {
                assert!(source.next().unwrap().is_ok());
            }
            assert_validation_error(&source, PlannerError::TrajectoryAlreadyStarted);
            // Failed validation of a clone must not consume the pending error.
            assert_eq!(source.next().unwrap().unwrap_err(), error);
            assert_validation_error(&source, PlannerError::TrajectoryAlreadyStarted);
            assert!(source.next().is_none());
        }
    }
}

#[test]
fn clones_preserve_started_state_without_starting_the_original_iterator() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = line((160.0, 0.0), (140.0, 0.0));
    for configuration in configurations() {
        let original = timed(&robot, &path, 5.0, configuration, 1.0, 2.0);
        let expected: Vec<_> = original.clone().map(Result::unwrap).collect();
        let mut started = original.clone();
        assert!(started.next().unwrap().is_ok());
        assert_validation_error(&started, PlannerError::TrajectoryAlreadyStarted);
        assert_validation_error(&started.clone(), PlannerError::TrajectoryAlreadyStarted);
        assert_eq!(original.clone().validate(), Ok(()));
        let actual: Vec<_> = original
            .into_validated()
            .unwrap()
            .map(Result::unwrap)
            .collect();
        assert_same_samples(&actual, &expected);
    }
}
