use kinematics::JointAngles;
use planner::{
    CoordinatedJointMotionProfile, CoordinatedJointPathProfiles, JointAccelerationLimits,
    JointVelocityLimits, PlannerError, TimedCoordinatedJointPathProfiles,
    TimedCoordinatedJointProfile, plan_joint_path_speeds,
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
fn constructor_preserves_start_time_bits_including_signed_zero() {
    for start in [
        0.0,
        -0.0,
        2.0 * f64::EPSILON,
        0.5,
        10.0,
        libm::scalbn(1.0, 53),
    ] {
        let timed = TimedCoordinatedJointProfile::new(start, profile()).unwrap();
        assert_eq!(timed.start_time().to_bits(), start.to_bits());
    }
}

#[test]
fn end_time_adds_the_profile_duration_to_the_start_time() {
    assert_eq!(profile().total_time(), 2.0);
    for start in [0.0, -0.0, 0.25, 0.5, 1.0, 10.0] {
        assert_eq!(
            TimedCoordinatedJointProfile::new(start, profile())
                .unwrap()
                .end_time(),
            Ok(start + 2.0)
        );
    }
}

#[test]
fn constructor_rejects_nonfinite_and_negative_start_times() {
    for start in [
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
        -f64::from_bits(1),
        -f64::MIN_POSITIVE,
        -1.0,
        -f64::MAX,
    ] {
        assert!(
            matches!(
                TimedCoordinatedJointProfile::new(start, profile()),
                Err(PlannerError::InvalidTime)
            ),
            "start={start}"
        );
    }
}

#[test]
fn constructor_rejects_end_time_overflow() {
    let profile = slow_profile(f64::MIN_POSITIVE / 2.0);
    let duration = profile.total_time();
    assert_eq!(duration, libm::scalbn(1.0, 1023));
    assert!((f64::MAX + duration).is_infinite());
    assert!(matches!(
        TimedCoordinatedJointProfile::new(f64::MAX, profile),
        Err(PlannerError::TimeElapsedOverflow)
    ));
}

#[test]
fn a_large_representable_end_time_is_accepted() {
    let profile = slow_profile(f64::MIN_POSITIVE);
    let duration = profile.total_time();
    assert_eq!(duration, libm::scalbn(1.0, 1022));
    let timed = TimedCoordinatedJointProfile::new(duration, profile).unwrap();
    let end = libm::scalbn(1.0, 1023);
    assert_eq!(timed.end_time(), Ok(end));
    for (time, velocity, acceleration) in [
        (duration, (0.0, 0.0), (1.0, 0.0)),
        (
            f64::midpoint(duration, end),
            (f64::MIN_POSITIVE, 0.0),
            (0.0, 0.0),
        ),
        (end, (0.0, 0.0), (-1.0, 0.0)),
    ] {
        assert_eq!(timed.velocity_at(time), Ok(velocity));
        assert_eq!(timed.acceleration_at(time), Ok(acceleration));
    }
}

#[test]
fn constructor_rejects_a_duration_absorbed_by_start_time() {
    for start in [libm::scalbn(1.0, 54), 1e300, f64::MAX] {
        let profile = profile();
        assert_eq!(profile.total_time(), 2.0);
        assert_eq!(start + profile.total_time(), start);
        assert!(matches!(
            TimedCoordinatedJointProfile::new(start, profile),
            Err(PlannerError::TimeElapsedPrecisionLost)
        ));
    }
}

#[test]
fn constructor_rejects_a_positive_start_time_absorbed_by_duration() {
    for start in [f64::from_bits(1), f64::MIN_POSITIVE, f64::EPSILON / 2.0] {
        let profile = profile();
        assert_eq!(start + profile.total_time(), profile.total_time());
        assert!(matches!(
            TimedCoordinatedJointProfile::new(start, profile),
            Err(PlannerError::TimeElapsedPrecisionLost)
        ));
    }
}

#[test]
fn round_to_even_can_advance_an_odd_start_time_by_half_an_ulp() {
    let start = libm::scalbn(1.0, 54).next_up();
    assert_eq!(start + profile().total_time(), start.next_up());
    let timed = TimedCoordinatedJointProfile::new(start, profile()).unwrap();
    assert_eq!(timed.end_time(), Ok(start.next_up()));
    // No representable time lies strictly inside this global interval.
    assert_eq!(position(&timed, start), Ok([2.0, -1.0]));
    assert_eq!(position(&timed, start.next_up()), Ok([6.0, -3.0]));
    assert_eq!(timed.velocity_at(start), Ok((0.0, 0.0)));
    assert_eq!(timed.acceleration_at(start), Ok((4.0, -2.0)));
    assert_eq!(timed.velocity_at(start.next_up()), Ok((0.0, 0.0)));
    assert_eq!(timed.acceleration_at(start.next_up()), Ok((-4.0, 2.0)));
    assert_eq!(
        position(&timed, start.next_down()),
        Err(PlannerError::InvalidTime)
    );
    assert_eq!(
        position(&timed, start.next_up().next_up()),
        Err(PlannerError::InvalidTime)
    );
}

#[test]
fn a_representable_subnormal_duration_is_valid_at_zero_and_subnormal_offsets() {
    for start in [0.0, f64::from_bits(1)] {
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
        let timed = TimedCoordinatedJointProfile::new(start, profile).unwrap();
        assert_eq!(timed.start_time().to_bits(), start.to_bits());
        assert_eq!(timed.end_time(), Ok(start + duration));
        assert_eq!(position(&timed, start), Ok([0.0, 0.0]));
        assert_eq!(position(&timed, start + duration), Ok([1.0, 0.0]));
        for time in [start, start + duration / 2.0, start + duration] {
            assert_eq!(timed.velocity_at(time), Ok((f64::MAX, 0.0)));
            assert_eq!(timed.acceleration_at(time), Ok((0.0, 0.0)));
        }
    }
}

#[test]
fn zero_offset_preserves_every_sample_of_the_original_profile() {
    let reference = profile();
    let timed = TimedCoordinatedJointProfile::new(0.0, profile()).unwrap();
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
    let timed = TimedCoordinatedJointProfile::new(-0.0, profile()).unwrap();
    for time in [-0.0, 0.0] {
        assert_eq!(position(&timed, time), Ok([2.0, -1.0]));
        assert_eq!(timed.velocity_at(time), Ok((0.0, 0.0)));
        assert_eq!(timed.acceleration_at(time), Ok((4.0, -2.0)));
    }
}

#[test]
fn nonzero_offset_accepts_the_exact_global_start_time() {
    let timed = TimedCoordinatedJointProfile::new(10.0, profile()).unwrap();
    assert_eq!(position(&timed, timed.start_time()), Ok([2.0, -1.0]));
}

#[test]
fn nonzero_offset_maps_a_global_interior_time_to_the_local_profile_time() {
    let timed = TimedCoordinatedJointProfile::new(10.0, profile()).unwrap();
    assert_eq!(position(&timed, 11.0), Ok([4.0, -2.0]));
}

#[test]
fn nonzero_offset_accepts_the_exact_global_end_time() {
    let timed = TimedCoordinatedJointProfile::new(10.0, profile()).unwrap();
    assert_eq!(timed.end_time(), Ok(12.0));
    assert_eq!(position(&timed, 12.0), Ok([6.0, -3.0]));
}

#[test]
fn shifted_triangular_profiles_preserve_signed_velocity_and_acceleration() {
    let timed = TimedCoordinatedJointProfile::new(10.0, profile()).unwrap();
    for (global, velocity, acceleration) in [
        (10.0, (0.0, 0.0), (4.0, -2.0)),
        (10.5, (2.0, -1.0), (4.0, -2.0)),
        (11.0, (4.0, -2.0), (4.0, -2.0)),
        (11.5, (2.0, -1.0), (-4.0, 2.0)),
        (12.0, (0.0, 0.0), (-4.0, 2.0)),
    ] {
        assert_eq!(timed.velocity_at(global), Ok(velocity), "t={global}");
        assert_eq!(
            timed.acceleration_at(global),
            Ok(acceleration),
            "t={global}"
        );
    }
    assert_eq!(timed.acceleration_at(11.0_f64.next_up()), Ok((-4.0, 2.0)));
}

#[test]
fn shifted_trapezoidal_profiles_preserve_cruise_and_phase_boundaries() {
    // Joint displacements (8,-4) with peak velocities (4,-2) give one
    // time unit for each ramp and one for cruise.
    let motion = CoordinatedJointMotionProfile::new(
        angles([1.0, 2.0]),
        angles([9.0, -2.0]),
        &JointVelocityLimits::new(4.0, 2.0).unwrap(),
        &JointAccelerationLimits::new(4.0, 2.0).unwrap(),
        0.0,
        0.0,
    )
    .unwrap();
    let timed = TimedCoordinatedJointProfile::new(8.0, motion).unwrap();
    assert_eq!(timed.end_time(), Ok(11.0));
    for (global, velocity, acceleration) in [
        (8.5, (2.0, -1.0), (4.0, -2.0)),
        (9.0, (4.0, -2.0), (4.0, -2.0)),
        (9.0_f64.next_up(), (4.0, -2.0), (0.0, 0.0)),
        (9.5, (4.0, -2.0), (0.0, 0.0)),
        (10.0, (4.0, -2.0), (0.0, 0.0)),
        (10.5, (2.0, -1.0), (-4.0, 2.0)),
        (11.0, (0.0, 0.0), (-4.0, 2.0)),
    ] {
        assert_eq!(timed.velocity_at(global), Ok(velocity), "t={global}");
        assert_eq!(
            timed.acceleration_at(global),
            Ok(acceleration),
            "t={global}"
        );
    }
    assert_eq!(timed.acceleration_at(10.0_f64.next_up()), Ok((-4.0, 2.0)));
}

#[test]
fn global_end_returns_the_exact_nonzero_final_velocity_despite_rounding() {
    let make_profile = || {
        // Constant braking from progress speed 3/4 to 1/4 takes two time units.
        CoordinatedJointMotionProfile::new(
            angles([0.0, 0.0]),
            angles([4.0, -2.0]),
            &JointVelocityLimits::new(3.0, 1.5).unwrap(),
            &JointAccelerationLimits::new(1.0, 0.5).unwrap(),
            0.75,
            0.25,
        )
        .unwrap()
    };
    for (start, rounds_up) in [(0.01, false), (2.06, true)] {
        let reference = make_profile();
        let duration = reference.total_time();
        assert_eq!(duration, 2.0);
        let end = start + duration;
        let local = end - start;
        if rounds_up {
            assert!(local > duration);
            assert_eq!(reference.velocity_at(local), Err(PlannerError::InvalidTime));
            assert_eq!(
                reference.acceleration_at(local),
                Err(PlannerError::InvalidTime)
            );
        } else {
            assert!(local < duration);
            assert!(reference.velocity_at(local).unwrap().0 > 1.0);
        }
        let timed = TimedCoordinatedJointProfile::new(start, make_profile()).unwrap();
        assert_eq!(timed.end_time(), Ok(end));
        assert_eq!(timed.velocity_at(start), Ok((3.0, -1.5)));
        assert_eq!(timed.acceleration_at(start), Ok((-1.0, 0.5)));
        assert_eq!(timed.velocity_at(end), Ok((1.0, -0.5)));
        assert_eq!(timed.acceleration_at(end), Ok((-1.0, 0.5)));
        assert_eq!(
            timed.velocity_at(end.next_up()),
            Err(PlannerError::InvalidTime)
        );
        assert_eq!(
            timed.acceleration_at(end.next_up()),
            Err(PlannerError::InvalidTime)
        );
    }
}

#[test]
fn the_global_end_time_returns_the_exact_endpoint_despite_local_time_rounding() {
    // Subtracting the offset can land on either side of the local duration.
    for (start, rounds_up) in [(0.01, false), (2.06, true)] {
        let reference = profile();
        let duration = reference.total_time();
        let end = start + duration;
        let local = end - start;
        if rounds_up {
            assert!(local > duration);
            assert!(matches!(
                reference.position_at(local),
                Err(PlannerError::InvalidTime)
            ));
        } else {
            assert!(local < duration);
        }
        let timed = TimedCoordinatedJointProfile::new(start, profile()).unwrap();
        assert_eq!(timed.end_time(), Ok(end));
        assert_eq!(position(&timed, end), Ok([6.0, -3.0]));
        assert_eq!(
            position(&timed, end.next_up()),
            Err(PlannerError::InvalidTime)
        );
    }
}

#[test]
fn decimal_offsets_preserve_samples_strictly_inside_the_global_interval() {
    for start in [0.01, 0.3, 2.06, 10.1] {
        let reference = profile();
        let timed = TimedCoordinatedJointProfile::new(start, profile()).unwrap();
        let end = timed.end_time().unwrap();
        for global in [start.next_up(), start + 1.0, end.next_down()] {
            assert!(global > start && global < end);
            let expected = reference.position_at(global - start).unwrap();
            assert_eq!(
                position(&timed, global),
                Ok([expected.first_theta(), expected.second_theta()])
            );
            assert_eq!(
                timed.velocity_at(global),
                reference.velocity_at(global - start)
            );
            assert_eq!(
                timed.acceleration_at(global),
                reference.acceleration_at(global - start)
            );
        }
    }
}

#[test]
fn translating_the_time_origin_preserves_the_position_curve() {
    let reference = profile();
    for start in [0.25, 0.5, 1.0, 4.0] {
        let timed = TimedCoordinatedJointProfile::new(start, profile()).unwrap();
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
        let timed = TimedCoordinatedJointProfile::new(start, profile()).unwrap();
        let end = timed.end_time().unwrap();
        for time in [start.next_down(), end.next_up(), f64::MAX] {
            assert_eq!(position(&timed, time), Err(PlannerError::InvalidTime));
            assert_eq!(timed.velocity_at(time), Err(PlannerError::InvalidTime));
            assert_eq!(timed.acceleration_at(time), Err(PlannerError::InvalidTime));
        }
    }
}

#[test]
fn nonfinite_or_negative_global_times_are_rejected() {
    for start in [0.0, 10.0] {
        let timed = TimedCoordinatedJointProfile::new(start, profile()).unwrap();
        for time in [
            f64::NAN,
            f64::INFINITY,
            f64::NEG_INFINITY,
            -f64::from_bits(1),
            -1.0,
        ] {
            assert_eq!(position(&timed, time), Err(PlannerError::InvalidTime));
            assert_eq!(timed.velocity_at(time), Err(PlannerError::InvalidTime));
            assert_eq!(timed.acceleration_at(time), Err(PlannerError::InvalidTime));
        }
    }
}

#[test]
fn position_queries_are_repeatable_and_do_not_require_monotonic_time() {
    let timed = TimedCoordinatedJointProfile::new(0.0, profile()).unwrap();
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
        let timed = TimedCoordinatedJointProfile::new(0.0, make_profile()).unwrap();
        assert_eq!(timed.end_time(), Ok(reference.total_time()));
        for step in 0..=32 {
            let time = reference.total_time() * (f64::from(step) / 32.0);
            let expected = reference.position_at(time).unwrap();
            assert_eq!(
                position(&timed, time),
                Ok([expected.first_theta(), expected.second_theta()])
            );
            assert_eq!(timed.velocity_at(time), reference.velocity_at(time));
            assert_eq!(timed.acceleration_at(time), reference.acceleration_at(time));
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
        let timed = TimedCoordinatedJointProfile::new(0.0, make_profile()).unwrap();
        for step in 0..=32 {
            let time = reference.total_time() * (f64::from(step) / 32.0);
            let expected = reference.position_at(time).unwrap();
            assert_eq!(
                position(&timed, time),
                Ok([expected.first_theta(), expected.second_theta()])
            );
            assert_eq!(timed.velocity_at(time), reference.velocity_at(time));
            assert_eq!(timed.acceleration_at(time), reference.acceleration_at(time));
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
    let first = TimedCoordinatedJointProfile::new(0.0, profiles.next().unwrap().unwrap()).unwrap();
    let boundary = first.end_time().unwrap();
    let second =
        TimedCoordinatedJointProfile::new(boundary, profiles.next().unwrap().unwrap()).unwrap();
    assert!(profiles.next().is_none());
    assert_eq!(position(&first, boundary), Ok([1.0, 0.0]));
    assert_eq!(position(&second, boundary), Ok([1.0, 0.0]));
    assert_eq!(
        position(&second, second.end_time().unwrap()),
        Ok([2.0, 0.0])
    );
}

#[test]
fn timed_path_preserves_joint_velocity_and_one_sided_acceleration_at_junctions() {
    let points = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [2.0, 1.0], [2.0, 2.0]].map(angles);
    let vl = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let al = JointAccelerationLimits::new(2.0, 2.0).unwrap();
    let mut speeds = [f64::NAN; 5];
    plan_joint_path_speeds(&points, &vl, &al, &mut speeds).unwrap();
    assert_eq!(speeds, [0.0, 2.0, 0.0, 2.0, 0.0]);
    let source = CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).unwrap();
    let timed = TimedCoordinatedJointPathProfiles::new(source).unwrap();
    let segments = timed.collect::<Result<Vec<_>, _>>().unwrap();
    assert_eq!(segments.len(), 4);

    for (index, (time, velocity, left_acceleration, right_acceleration)) in [
        (1.0, (2.0, 0.0), (2.0, 0.0), (-2.0, 0.0)),
        (2.0, (0.0, 0.0), (-2.0, 0.0), (0.0, 2.0)),
        (3.0, (0.0, 2.0), (0.0, 2.0), (0.0, -2.0)),
    ]
    .into_iter()
    .enumerate()
    {
        let left = &segments[index];
        let right = &segments[index + 1];
        assert_eq!(left.end_time(), Ok(time));
        assert_eq!(right.start_time(), time);
        assert_eq!(left.velocity_at(time), Ok(velocity));
        assert_eq!(right.velocity_at(time), Ok(velocity));
        assert_eq!(left.acceleration_at(time), Ok(left_acceleration));
        assert_eq!(right.acceleration_at(time), Ok(right_acceleration));
    }
}
