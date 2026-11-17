use super::*;

fn angles(first: f64) -> JointAngles {
    JointAngles::new(first, 0.0).unwrap()
}

fn assert_remaining<I: ExactSizeIterator + FusedIterator>(iterator: &I, remaining: usize) {
    assert_eq!(Iterator::size_hint(iterator), (remaining, Some(remaining)));
    assert_eq!(ExactSizeIterator::len(iterator), remaining);
}

// Public construction validates every segment up front. These deliberately
// invalid private fixtures exercise next's defensive Result path without
// unsafe mutation or implying that new accepts the invalid states.
#[test]
fn a_failed_segment_is_reported_once_and_the_next_segment_is_still_attempted() {
    for (coordinates, speeds, expected) in [
        (
            [0.0, 0.0, 1.0],
            [0.0; 3],
            PlannerError::ZeroDisplacementMotion,
        ),
        (
            [0.0, 1.0, 2.0],
            [3.0, 0.0, 0.0],
            PlannerError::InfeasibleMotion,
        ),
        (
            [0.0, 2.0, 3.0],
            [f64::from_bits(1), 0.0, 0.0],
            PlannerError::NumericalFailure,
        ),
    ] {
        let points = coordinates.map(angles);
        let vl = JointVelocityLimits::new(10.0, 10.0).unwrap();
        let al = JointAccelerationLimits::new(2.0, 2.0).unwrap();
        let mut chain = CoordinatedJointPathProfiles {
            points: &points,
            speeds: &speeds,
            velocity_limits: &vl,
            acceleration_limits: &al,
            current_segment: 0,
        };
        assert_remaining(&chain, 2);
        assert_eq!(
            chain.next().expect("error was hidden as None").unwrap_err(),
            expected
        );
        assert_remaining(&chain, 1);
        let next = chain.next().expect("later segment was skipped").unwrap();
        assert_remaining(&chain, 0);
        assert_eq!(next.position_at(0.0).unwrap().first_theta(), coordinates[1]);
        assert_eq!(
            next.position_at(next.total_time()).unwrap().first_theta(),
            coordinates[2]
        );
        assert!(chain.next().is_none());
        assert!(chain.next().is_none());
        assert_remaining(&chain, 0);
    }
}

#[test]
fn an_error_on_the_final_segment_is_followed_by_permanent_exhaustion() {
    let points = [0.0, 1.0, 1.0].map(angles);
    let speeds = [0.0; 3];
    let vl = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let al = JointAccelerationLimits::new(2.0, 2.0).unwrap();
    let mut chain = CoordinatedJointPathProfiles {
        points: &points,
        speeds: &speeds,
        velocity_limits: &vl,
        acceleration_limits: &al,
        current_segment: 0,
    };
    assert_remaining(&chain, 2);
    assert!(chain.next().unwrap().is_ok());
    assert_remaining(&chain, 1);
    assert_eq!(
        chain.next().unwrap().unwrap_err(),
        PlannerError::ZeroDisplacementMotion
    );
    assert_remaining(&chain, 0);
    for _ in 0..8 {
        assert!(chain.next().is_none());
        assert_remaining(&chain, 0);
    }
}
