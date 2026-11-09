use super::*;
use crate::{JointLimits, JointRange, JointTrajectory};
use kinematics::{ElbowConfiguration, Point, Scara};
use trajectory::LineSegment;

#[test]
fn regression_an_upstream_error_is_returned_once_after_a_valid_prefix() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = LineSegment {
        start: Point::new(160.0, 0.0).unwrap(),
        end: Point::new(140.0, 0.0).unwrap(),
    };
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
        assert!(matches!(timed.next(), Some(Ok(_))));
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
