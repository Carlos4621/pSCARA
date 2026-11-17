use kinematics::JointAngles;
use planner::{
    CoordinatedJointMotionProfile, CoordinatedJointPathProfiles, JointAccelerationLimits,
    JointVelocityLimits, PlannerError, TimedCoordinatedJointProfile,
};

type Pair = [f64; 2];

fn angles(q: Pair) -> JointAngles {
    JointAngles::new(q[0], q[1]).unwrap()
}

fn profile() -> CoordinatedJointMotionProfile {
    // Rest-to-rest displacement (4,-2), acceleration (4,2): duration 2.
    CoordinatedJointMotionProfile::new(
        angles([2.0, -1.0]),
        angles([6.0, -3.0]),
        &JointVelocityLimits::new(20.0, 10.0).unwrap(),
        &JointAccelerationLimits::new(4.0, 2.0).unwrap(),
        0.0,
        0.0,
    )
    .unwrap()
}

fn position(timed: &TimedCoordinatedJointProfile, global_time: f64) -> Result<Pair, PlannerError> {
    timed
        .position_at(global_time)
        .map(|q| [q.first_theta(), q.second_theta()])
}

fn slow_profile(velocity: f64) -> CoordinatedJointMotionProfile {
    CoordinatedJointMotionProfile::new(
        angles([0.0, 0.0]),
        angles([1.0, 0.0]),
        &JointVelocityLimits::new(velocity, 1.0).unwrap(),
        &JointAccelerationLimits::new(1.0, 1.0).unwrap(),
        0.0,
        0.0,
    )
    .unwrap()
}

#[test]
fn constructor_preserves_finite_nonnegative_start_times_including_signed_zero() {
    for start in [
        0.0,
        -0.0,
        f64::from_bits(1),
        f64::MIN_POSITIVE,
        0.5,
        10.0,
        1e300,
        f64::MAX,
    ] {
        let timed = TimedCoordinatedJointProfile::new(start, profile());
        assert_eq!(timed.start_time().to_bits(), start.to_bits());
    }
}

#[test]
fn end_time_adds_the_profile_duration_to_the_start_time() {
    assert_eq!(profile().total_time(), 2.0);
    for start in [0.0, -0.0, 0.25, 0.5, 1.0, 10.0] {
        assert_eq!(
            TimedCoordinatedJointProfile::new(start, profile()).end_time(),
            Ok(start + 2.0)
        );
    }
}

#[test]
fn end_time_reports_nonfinite_start_times_as_numerical_failure() {
    // new is currently infallible; validation happens when end_time is queried.
    for start in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let timed = TimedCoordinatedJointProfile::new(start, profile());
        assert_eq!(timed.end_time(), Err(PlannerError::NumericalFailure));
    }
}

#[test]
fn a_genuine_end_time_overflow_is_reported_as_numerical_failure() {
    let profile = slow_profile(f64::MIN_POSITIVE / 2.0);
    let duration = profile.total_time();
    assert_eq!(duration, libm::scalbn(1.0, 1023));
    assert!((f64::MAX + duration).is_infinite());
    let timed = TimedCoordinatedJointProfile::new(f64::MAX, profile);
    assert_eq!(timed.end_time(), Err(PlannerError::NumericalFailure));
}

#[test]
fn a_large_representable_end_time_is_accepted() {
    let profile = slow_profile(f64::MIN_POSITIVE);
    let duration = profile.total_time();
    assert_eq!(duration, libm::scalbn(1.0, 1022));
    let timed = TimedCoordinatedJointProfile::new(duration, profile);
    assert_eq!(timed.end_time(), Ok(libm::scalbn(1.0, 1023)));
}

#[test]
fn a_positive_duration_absorbed_by_start_time_returns_precision_lost() {
    let start = libm::scalbn(1.0, 54);
    let profile = profile();
    assert!(profile.total_time() > 0.0);
    assert_eq!(start + profile.total_time(), start);
    let timed = TimedCoordinatedJointProfile::new(start, profile);
    assert_eq!(
        timed.end_time(),
        Err(PlannerError::TimeElapsedPrecisionLost)
    );
}

#[test]
fn round_to_even_can_advance_an_odd_start_time_by_half_an_ulp() {
    let start = libm::scalbn(1.0, 54).next_up();
    assert_eq!(start + profile().total_time(), start.next_up());
    let timed = TimedCoordinatedJointProfile::new(start, profile());
    assert_eq!(timed.end_time(), Ok(start.next_up()));
}

#[test]
fn a_representable_subnormal_duration_is_valid_at_zero_offset() {
    let profile = CoordinatedJointMotionProfile::new(
        angles([0.0, 0.0]),
        angles([1.0, 0.0]),
        &JointVelocityLimits::new(f64::MAX, 1.0).unwrap(),
        &JointAccelerationLimits::new(1.0, 1.0).unwrap(),
        f64::MAX,
        f64::MAX,
    )
    .unwrap();
    let duration = profile.total_time();
    assert!(duration > 0.0 && duration < f64::MIN_POSITIVE);
    let timed = TimedCoordinatedJointProfile::new(0.0, profile);
    assert_eq!(timed.end_time(), Ok(duration));
    assert_eq!(position(&timed, 0.0), Ok([0.0, 0.0]));
    assert_eq!(position(&timed, duration), Ok([1.0, 0.0]));
}

#[test]
fn zero_offset_preserves_every_sample_of_the_original_profile() {
    let reference = profile();
    let timed = TimedCoordinatedJointProfile::new(0.0, profile());
    for step in 0..=128 {
        let time = reference.total_time() * (f64::from(step) / 128.0);
        let expected = reference.position_at(time).unwrap();
        assert_eq!(
            position(&timed, time),
            Ok([expected.first_theta(), expected.second_theta()])
        );
    }
}

#[test]
fn negative_zero_offset_and_global_time_are_valid_at_the_start() {
    let timed = TimedCoordinatedJointProfile::new(-0.0, profile());
    assert_eq!(position(&timed, -0.0), Ok([2.0, -1.0]));
    assert_eq!(position(&timed, 0.0), Ok([2.0, -1.0]));
}

#[test]
fn nonzero_offset_accepts_the_exact_global_start_time() {
    let timed = TimedCoordinatedJointProfile::new(10.0, profile());
    assert_eq!(position(&timed, timed.start_time()), Ok([2.0, -1.0]));
}

#[test]
fn nonzero_offset_maps_a_global_interior_time_to_the_local_profile_time() {
    let timed = TimedCoordinatedJointProfile::new(10.0, profile());
    assert_eq!(position(&timed, 11.0), Ok([4.0, -2.0]));
}

#[test]
fn nonzero_offset_accepts_the_exact_global_end_time() {
    let timed = TimedCoordinatedJointProfile::new(10.0, profile());
    assert_eq!(timed.end_time(), Ok(12.0));
    assert_eq!(position(&timed, 12.0), Ok([6.0, -3.0]));
}

#[test]
fn translating_the_time_origin_preserves_the_position_curve() {
    let reference = profile();
    for start in [0.25, 0.5, 1.0, 4.0] {
        let timed = TimedCoordinatedJointProfile::new(start, profile());
        for step in 0..=32 {
            // Binary fractions make global/local conversion exact here.
            let local = 2.0 * (f64::from(step) / 32.0);
            let expected = reference.position_at(local).unwrap();
            assert_eq!(
                position(&timed, start + local),
                Ok([expected.first_theta(), expected.second_theta()]),
                "start={start}, local={local}"
            );
        }
    }
}

#[test]
fn global_times_outside_the_scheduled_interval_are_rejected() {
    for start in [0.0_f64, 0.5, 10.0] {
        let timed = TimedCoordinatedJointProfile::new(start, profile());
        let end = timed.end_time().unwrap();
        assert_eq!(
            position(&timed, start.next_down()),
            Err(PlannerError::InvalidTime)
        );
        assert_eq!(
            position(&timed, end.next_up()),
            Err(PlannerError::InvalidTime)
        );
    }
}

#[test]
fn nonfinite_or_negative_global_times_are_rejected() {
    for start in [0.0, 10.0] {
        let timed = TimedCoordinatedJointProfile::new(start, profile());
        for time in [
            f64::NAN,
            f64::INFINITY,
            f64::NEG_INFINITY,
            -f64::from_bits(1),
            -1.0,
        ] {
            assert_eq!(position(&timed, time), Err(PlannerError::InvalidTime));
        }
    }
}

#[test]
fn position_queries_propagate_an_unrepresentable_end_time() {
    let timed = TimedCoordinatedJointProfile::new(f64::MAX, slow_profile(f64::MIN_POSITIVE / 2.0));
    assert_eq!(
        position(&timed, f64::MAX),
        Err(PlannerError::NumericalFailure)
    );
}

#[test]
fn invalid_global_time_is_rejected_before_an_invalid_schedule_is_evaluated() {
    let timed = TimedCoordinatedJointProfile::new(f64::NAN, profile());
    assert_eq!(position(&timed, -1.0), Err(PlannerError::InvalidTime));
    assert_eq!(
        position(&timed, f64::INFINITY),
        Err(PlannerError::InvalidTime)
    );
    assert_eq!(position(&timed, 0.0), Err(PlannerError::NumericalFailure));
}

#[test]
fn position_queries_are_repeatable_and_do_not_require_monotonic_time() {
    let timed = TimedCoordinatedJointProfile::new(0.0, profile());
    for time in [2.0, 0.0, 1.0, 0.5, 2.0, 1.0, 0.0] {
        let expected = profile().position_at(time).unwrap();
        assert_eq!(
            position(&timed, time),
            Ok([expected.first_theta(), expected.second_theta()])
        );
    }
}

#[test]
fn profiles_with_nonzero_boundary_speeds_are_preserved_at_zero_offset() {
    let vl = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let al = JointAccelerationLimits::new(2.0, 2.0).unwrap();
    for (from, to) in [([0.0, 0.0], [2.0, -1.0]), ([2.0, -1.0], [0.0, 0.0])] {
        let make_profile = || {
            CoordinatedJointMotionProfile::new(angles(from), angles(to), &vl, &al, 0.5, 0.25)
                .unwrap()
        };
        let reference = make_profile();
        let timed = TimedCoordinatedJointProfile::new(0.0, make_profile());
        assert_eq!(timed.end_time(), Ok(reference.total_time()));
        for step in 0..=32 {
            let time = reference.total_time() * (f64::from(step) / 32.0);
            let expected = reference.position_at(time).unwrap();
            assert_eq!(
                position(&timed, time),
                Ok([expected.first_theta(), expected.second_theta()])
            );
        }
    }
}

#[test]
fn zero_offset_delegation_survives_extreme_common_geometric_scales() {
    for scale in [
        libm::scalbn(1.0, -500),
        1e-100,
        1.0,
        1e100,
        libm::scalbn(1.0, 500),
    ] {
        let make_profile = || {
            CoordinatedJointMotionProfile::new(
                angles([0.0, 0.0]),
                angles([4.0 * scale, -2.0 * scale]),
                &JointVelocityLimits::new(20.0 * scale, 10.0 * scale).unwrap(),
                &JointAccelerationLimits::new(4.0 * scale, 2.0 * scale).unwrap(),
                0.0,
                0.0,
            )
            .unwrap()
        };
        let reference = make_profile();
        let timed = TimedCoordinatedJointProfile::new(0.0, make_profile());
        for step in 0..=32 {
            let time = reference.total_time() * (f64::from(step) / 32.0);
            let expected = reference.position_at(time).unwrap();
            assert_eq!(
                position(&timed, time),
                Ok([expected.first_theta(), expected.second_theta()])
            );
        }
    }
}

#[test]
fn scheduled_profiles_from_a_complete_path_agree_at_their_shared_global_boundary() {
    let points = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]].map(angles);
    let speeds = [0.0; 3];
    let vl = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let al = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let mut profiles = CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).unwrap();
    let first = TimedCoordinatedJointProfile::new(0.0, profiles.next().unwrap().unwrap());
    let boundary = first.end_time().unwrap();
    let second = TimedCoordinatedJointProfile::new(boundary, profiles.next().unwrap().unwrap());
    assert!(profiles.next().is_none());
    assert_eq!(position(&first, boundary), Ok([1.0, 0.0]));
    assert_eq!(position(&second, boundary), Ok([1.0, 0.0]));
    assert_eq!(
        position(&second, second.end_time().unwrap()),
        Ok([2.0, 0.0])
    );
}
