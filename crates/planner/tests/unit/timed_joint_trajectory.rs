use super::*;
use crate::{JointLimits, JointRange, JointTrajectory};
use kinematics::{ElbowConfiguration, Point, Scara};
use trajectory::LineSegment;

fn configurations() -> [ElbowConfiguration; 2] {
    [ElbowConfiguration::Positive, ElbowConfiguration::Negative]
}

fn timed<'a>(
    robot: &'a Scara,
    path: &'a LineSegment,
    speed: f64,
    configuration: ElbowConfiguration,
) -> TimedJointTrajectory<'a> {
    let source = JointTrajectory::new(robot, path, f64::MAX, configuration)
        .unwrap()
        .into_validated()
        .unwrap();
    TimedJointTrajectory::new(source, JointVelocityLimits::new(speed, speed).unwrap())
}

fn assert_same_angles(actual: JointAngles, expected: JointAngles) {
    assert_eq!(
        actual.first_theta().to_bits(),
        expected.first_theta().to_bits()
    );
    assert_eq!(
        actual.second_theta().to_bits(),
        expected.second_theta().to_bits()
    );
}

fn assert_error_preserves_state_and_finishes(
    timed: &mut TimedJointTrajectory<'_>,
    expected: PlannerError,
) {
    let previous = timed.previous_angle.expect("a valid prefix must exist");
    let time = timed.time_elapsed;
    assert_eq!(timed.next().unwrap().unwrap_err(), expected);
    assert_same_angles(timed.previous_angle.unwrap(), previous);
    assert_eq!(timed.time_elapsed.to_bits(), time.to_bits());
    for _ in 0..3 {
        assert!(timed.next().is_none());
    }
}

#[test]
fn regression_an_upstream_error_is_returned_once_after_a_valid_prefix() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = LineSegment::new(
        Point::new(160.0, 0.0).unwrap(),
        Point::new(140.0, 0.0).unwrap(),
    )
    .expect("test segment must have a finite length");
    let limits = JointLimits::new(
        JointRange::new(0.0, 0.0).unwrap(),
        JointRange::new(0.0, 0.0).unwrap(),
    );
    for configuration in [ElbowConfiguration::Positive, ElbowConfiguration::Negative] {
        let source = JointTrajectory::new(&robot, &path, 5.0, configuration)
            .unwrap()
            .with_limits(&limits)
            .unwrap();
        assert_eq!(
            source.clone().validate(),
            Err(PlannerError::AngleOutOfLimits)
        );
        // Fault injection: bypass the validating public factory to exercise
        // the fallible source iterator. This fixture cannot be built externally.
        let injected = ValidatedTrajectory::new(source);
        let mut timed =
            TimedJointTrajectory::new(injected, JointVelocityLimits::new(1.0, 1.0).unwrap());
        assert_eq!(
            timed.clone().validate(),
            Err(PlannerError::AngleOutOfLimits)
        );
        assert_eq!(
            timed.clone().into_validated().unwrap_err(),
            PlannerError::AngleOutOfLimits
        );
        assert!(matches!(timed.next(), Some(Ok(_))));
        assert_eq!(
            timed.clone().validate(),
            Err(PlannerError::TrajectoryAlreadyStarted)
        );
        assert_eq!(
            timed.clone().into_validated().unwrap_err(),
            PlannerError::TrajectoryAlreadyStarted
        );
        match timed.next() {
            Some(Err(error)) => assert_eq!(error, PlannerError::AngleOutOfLimits),
            Some(Ok(sample)) => panic!(
                "expected AngleOutOfLimits, got time {}",
                sample.time_elapsed
            ),
            None => panic!("the upstream AngleOutOfLimits was silently discarded"),
        }
        // The source iterator is terminal after its error, so verify the
        // observable termination without requiring a particular private flag.
        assert!(timed.next().is_none());
        assert!(timed.next().is_none());
    }
}

#[test]
fn cartesian_precision_failure_preserves_the_last_committed_angle_and_time() {
    let robot = Scara::new(1e16, 1e16).unwrap();
    let path = LineSegment::new(
        Point::new(1e16, 0.0).unwrap(),
        Point::new(1e16 + 4.0, 0.0).unwrap(),
    )
    .unwrap();
    let expected = PlannerError::Trajectory(trajectory::TrajectoryError::PrecissionLost);
    for configuration in configurations() {
        let source = JointTrajectory::new(&robot, &path, 1.0, configuration).unwrap();
        assert_eq!(source.clone().validate(), Err(expected));
        // Only this internal fixture can bypass the rejecting public validation.
        let injected = ValidatedTrajectory::new(source);
        let mut timed =
            TimedJointTrajectory::new(injected, JointVelocityLimits::new(1.0, 1.0).unwrap());
        assert_eq!(timed.clone().validate(), Err(expected));
        assert_eq!(timed.clone().into_validated().unwrap_err(), expected);
        for _ in 0..2 {
            let sample = timed.next().unwrap().unwrap();
            assert_eq!(sample.time_elapsed, 0.0);
        }
        assert_error_preserves_state_and_finishes(&mut timed, expected);
    }
}

#[test]
fn a_positive_duration_absorbed_by_the_clock_returns_precision_lost() {
    let robot = Scara::new(0.5, 0.5).unwrap();
    let path = LineSegment::new(
        Point::new(1.0, 0.0).unwrap(),
        Point::new(1.0, f64::from_bits(1)).unwrap(),
    )
    .expect("test segment must have a finite length");
    for configuration in configurations() {
        let mut timed = timed(&robot, &path, 1.0, configuration);
        let first = timed.next().unwrap().unwrap();
        let end = robot.inverse(path.end(), configuration).unwrap();
        let delta = timed
            .velocity_limits
            .min_duration_between(first.angle, end)
            .unwrap();
        assert_eq!(delta, f64::from_bits(1));
        // Seed a prior elapsed time to isolate loss during accumulation from
        // division underflow. No enormous prefix needs to be iterated.
        timed.time_elapsed = 1.0;
        assert_eq!(1.0 + delta, 1.0);
        assert_error_preserves_state_and_finishes(
            &mut timed,
            PlannerError::TimeElapsedPrecisionLost,
        );
    }
}

#[test]
fn neighbouring_durations_around_half_an_ulp_are_classified_correctly() {
    let robot = Scara::new(0.5, 0.5).unwrap();
    let path = LineSegment::new(
        Point::new(1.0, 0.0).unwrap(),
        Point::new(1.0, f64::EPSILON).unwrap(),
    )
    .expect("test segment must have a finite length");
    let below_two = f64::from_bits(2.0_f64.to_bits() - 1);
    let above_two = f64::from_bits(2.0_f64.to_bits() + 1);
    for configuration in configurations() {
        for (speed, accepted) in [
            (above_two, false),
            (2.0, false),
            (below_two, true),
            (1.0, true),
        ] {
            let mut timed = timed(&robot, &path, speed, configuration);
            let first = timed.next().unwrap().unwrap();
            let end = robot.inverse(path.end(), configuration).unwrap();
            assert_eq!(end.first_theta() - first.angle.first_theta(), f64::EPSILON);
            let delta = timed
                .velocity_limits
                .min_duration_between(first.angle, end)
                .unwrap();
            assert!(delta.is_finite() && delta > 0.0);
            timed.time_elapsed = 1.0;
            if accepted {
                let next = timed.next().unwrap().unwrap();
                assert_eq!(next.time_elapsed, 1.0 + f64::EPSILON);
                assert_eq!(timed.time_elapsed, next.time_elapsed);
                assert_same_angles(timed.previous_angle.unwrap(), next.angle);
                assert!(timed.next().is_none());
            } else {
                assert!(delta <= f64::EPSILON / 2.0);
                assert_error_preserves_state_and_finishes(
                    &mut timed,
                    PlannerError::TimeElapsedPrecisionLost,
                );
            }
        }
    }
}

#[test]
fn exact_half_ulp_ties_can_advance_an_odd_elapsed_clock() {
    let robot = Scara::new(0.5, 0.5).unwrap();
    let path = LineSegment::new(
        Point::new(1.0, 0.0).unwrap(),
        Point::new(1.0, f64::EPSILON).unwrap(),
    )
    .expect("test segment must have a finite length");
    for configuration in configurations() {
        let mut timed = timed(&robot, &path, 2.0, configuration);
        let first = timed.next().unwrap().unwrap();
        let end = robot.inverse(path.end(), configuration).unwrap();
        assert_eq!(
            timed.velocity_limits.min_duration_between(first.angle, end),
            Ok(f64::EPSILON / 2.0)
        );
        // Round-to-even rejects the same delta at 1.0 (previous test), but
        // rounds upward when the clock's least significant mantissa bit is 1.
        timed.time_elapsed = f64::from_bits(1.0_f64.to_bits() + 1);
        let next = timed.next().unwrap().unwrap();
        assert_eq!(next.time_elapsed, f64::from_bits(1.0_f64.to_bits() + 2));
        assert!(timed.next().is_none());
    }
}

#[test]
fn zero_motion_keeps_a_nonzero_clock_and_is_not_precision_loss() {
    let robot = Scara::new(0.5, 0.5).unwrap();
    let path = LineSegment::new(Point::new(1.0, 0.0).unwrap(), Point::new(1.0, 0.0).unwrap())
        .expect("test segment must have a finite length");
    for configuration in configurations() {
        for elapsed in [f64::from_bits(1), f64::MIN_POSITIVE, 1.0, f64::MAX] {
            let mut timed = timed(&robot, &path, 1.0, configuration);
            let first = timed.next().unwrap().unwrap();
            assert_eq!(first.time_elapsed, 0.0);
            timed.time_elapsed = elapsed;
            let next = timed.next().unwrap().unwrap();
            assert_eq!(next.angle.first_theta(), first.angle.first_theta());
            assert_eq!(next.angle.second_theta(), first.angle.second_theta());
            assert_eq!(next.time_elapsed, elapsed);
            assert!(timed.next().is_none());
        }
    }
}

#[test]
fn timing_failures_preserve_the_committed_state_and_stop_before_later_samples() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = LineSegment::new(
        Point::new(160.0, 0.0).unwrap(),
        Point::new(140.0, 0.0).unwrap(),
    )
    .expect("test segment must have a finite length");
    for configuration in configurations() {
        for (speed, expected) in [
            (1.0, PlannerError::TimeElapsedPrecisionLost),
            (1e-308, PlannerError::TimeElapsedOverflow),
            (f64::from_bits(1), PlannerError::NumericalFailure),
        ] {
            let source = JointTrajectory::new(&robot, &path, 5.0, configuration)
                .unwrap()
                .into_validated()
                .unwrap();
            let mut timed =
                TimedJointTrajectory::new(source, JointVelocityLimits::new(speed, speed).unwrap());
            assert_eq!(timed.next().unwrap().unwrap().time_elapsed, 0.0);
            timed.time_elapsed = f64::MAX;
            assert_error_preserves_state_and_finishes(&mut timed, expected);
        }
    }
}
