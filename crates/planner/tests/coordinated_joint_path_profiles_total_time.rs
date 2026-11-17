use kinematics::JointAngles;
use planner::{
    CoordinatedJointPathProfiles, JointAccelerationLimits, JointVelocityLimits, PlannerError,
    plan_joint_path_speeds,
};

type Pair = [f64; 2];

fn angles(q: Pair) -> JointAngles {
    JointAngles::new(q[0], q[1]).unwrap()
}

fn total<const N: usize>(
    coordinates: [Pair; N],
    speeds: [f64; N],
    velocity: Pair,
    acceleration: Pair,
) -> Result<f64, PlannerError> {
    let points = coordinates.map(angles);
    let vl = JointVelocityLimits::new(velocity[0], velocity[1]).unwrap();
    let al = JointAccelerationLimits::new(acceleration[0], acceleration[1]).unwrap();
    let profiles = CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).unwrap();
    CoordinatedJointPathProfiles::total_path_time(profiles)
}

fn close(actual: f64, expected: f64) {
    assert!(actual.is_finite() && actual > 0.0);
    assert!(
        actual == expected || (actual - expected).abs() <= 32.0 * f64::EPSILON * expected,
        "actual={actual:.17e}, expected={expected:.17e}"
    );
}

#[test]
fn one_triangular_segment_has_the_analytic_rest_to_rest_duration() {
    // T = 2*sqrt(distance/acceleration) = 4.
    assert_eq!(
        total([[0.0, 0.0], [4.0, 0.0]], [0.0; 2], [10.0; 2], [1.0; 2]),
        Ok(4.0)
    );
}

#[test]
fn one_trapezoidal_segment_includes_both_ramps_and_cruise() {
    // T = distance/velocity + velocity/acceleration = 10/2 + 2/2.
    assert_eq!(
        total([[0.0, 0.0], [10.0, 0.0]], [0.0; 2], [2.0; 2], [2.0; 2]),
        Ok(6.0)
    );
}

#[test]
fn stopped_segments_add_their_individual_durations() {
    // Segment distances 1, 4 and 9 take 2, 4 and 6 time units.
    assert_eq!(
        total(
            [0.0, 1.0, 5.0, 14.0].map(|x| [x, 0.0]),
            [0.0; 4],
            [100.0; 2],
            [1.0; 2]
        ),
        Ok(12.0)
    );
}

#[test]
fn nonzero_junction_speed_is_used_instead_of_stopping_at_each_point() {
    let points = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]];
    assert_eq!(total(points, [0.0, 2.0, 0.0], [10.0; 2], [2.0; 2]), Ok(2.0));
    assert!(total(points, [0.0; 3], [10.0; 2], [2.0; 2]).unwrap() > 2.0);
}

#[test]
fn corners_and_signed_zero_boundary_speeds_are_valid() {
    assert_eq!(
        total(
            [[0.0, 0.0], [1.0, 0.0], [1.0, 4.0]],
            [-0.0, 0.0, -0.0],
            [100.0; 2],
            [1.0; 2]
        ),
        Ok(6.0)
    );
}

#[test]
fn partially_consumed_iterator_sums_only_the_remaining_profiles() {
    let points = [0.0, 1.0, 5.0, 14.0].map(|x| angles([x, 0.0]));
    let speeds = [0.0; 4];
    let vl = JointVelocityLimits::new(100.0, 100.0).unwrap();
    let al = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let mut profiles = CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).unwrap();
    assert_eq!(profiles.next().unwrap().unwrap().total_time(), 2.0);
    assert_eq!(
        CoordinatedJointPathProfiles::total_path_time(profiles),
        Ok(10.0)
    );
}

#[test]
fn skipped_profiles_are_not_included_in_the_total() {
    let points = [0.0, 1.0, 5.0, 14.0].map(|x| angles([x, 0.0]));
    let speeds = [0.0; 4];
    let vl = JointVelocityLimits::new(100.0, 100.0).unwrap();
    let al = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let mut profiles = CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).unwrap();
    assert_eq!(profiles.nth(1).unwrap().unwrap().total_time(), 4.0);
    assert_eq!(
        CoordinatedJointPathProfiles::total_path_time(profiles),
        Ok(6.0)
    );
}

#[test]
fn an_exhausted_iterator_has_a_zero_total_with_no_error() {
    let points = [[0.0, 0.0], [1.0, 0.0]].map(angles);
    let speeds = [0.0; 2];
    let vl = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let al = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let mut profiles = CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).unwrap();
    assert!(profiles.next().unwrap().is_ok());
    assert!(profiles.next().is_none());
    assert!(profiles.next().is_none());
    assert_eq!(
        CoordinatedJointPathProfiles::total_path_time(profiles)
            .unwrap()
            .to_bits(),
        0.0_f64.to_bits()
    );
}

#[test]
fn total_matches_the_actual_durations_from_the_public_iterator() {
    let points = [[0.0, 0.0], [2.0, -1.0], [6.0, -3.0], [12.0, -6.0]].map(angles);
    let vl = JointVelocityLimits::new(20.0, 10.0).unwrap();
    let al = JointAccelerationLimits::new(4.0, 2.0).unwrap();
    let mut speeds = [f64::NAN; 4];
    plan_joint_path_speeds(&points, &vl, &al, &mut speeds).unwrap();
    let reference = CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).unwrap();
    let expected: f64 = reference.map(|profile| profile.unwrap().total_time()).sum();
    let profiles = CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).unwrap();
    assert_eq!(
        CoordinatedJointPathProfiles::total_path_time(profiles),
        Ok(expected)
    );
}

#[test]
fn reversing_reflecting_and_exchanging_joints_preserves_total_time() {
    let points = [[0.0, 0.0], [2.0, -1.0], [6.0, -3.0], [12.0, -6.0]];
    let speeds = [0.0, 4.0, 4.0, 0.0];
    let expected = total(points, speeds, [20.0, 10.0], [4.0, 2.0]).unwrap();
    let mut reversed = points;
    reversed.reverse();
    close(
        total(reversed, speeds, [20.0, 10.0], [4.0, 2.0]).unwrap(),
        expected,
    );
    close(
        total(
            points.map(|q| [32.0 - q[0], -64.0 + q[1]]),
            speeds,
            [20.0, 10.0],
            [4.0, 2.0],
        )
        .unwrap(),
        expected,
    );
    close(
        total(
            points.map(|q| [q[1], q[0]]),
            speeds,
            [10.0, 20.0],
            [2.0, 4.0],
        )
        .unwrap(),
        expected,
    );
}

#[test]
fn scaling_geometry_and_joint_limits_together_preserves_total_time() {
    for scale in [
        libm::scalbn(1.0, -500),
        1e-100,
        1.0,
        1e100,
        libm::scalbn(1.0, 500),
    ] {
        close(
            total(
                [0.0, 1.0, 5.0, 14.0].map(|x| [x * scale, 0.0]),
                [0.0; 4],
                [100.0 * scale; 2],
                [scale; 2],
            )
            .unwrap(),
            12.0,
        );
    }
}

#[test]
fn triangular_times_follow_the_square_root_distance_law_at_extreme_scales() {
    for exponent in [-500, -100, 0, 100, 500] {
        let scale = libm::scalbn(1.0, exponent);
        let root = libm::scalbn(1.0, exponent / 2);
        close(
            total(
                [0.0, 1.0, 5.0, 14.0].map(|x| [x * scale, 0.0]),
                [0.0; 4],
                [10.0 * root; 2],
                [1.0; 2],
            )
            .unwrap(),
            12.0 * root,
        );
    }
}

#[test]
fn finite_individual_durations_can_overflow_when_added() {
    let points = [0.0, 1.0, 2.0, 3.0, 4.0].map(|x| angles([x, 0.0]));
    let speeds = [0.0; 5];
    let vl = JointVelocityLimits::new(f64::MIN_POSITIVE, 1.0).unwrap();
    let al = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let profiles = CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).unwrap();
    for profile in profiles {
        assert_eq!(profile.unwrap().total_time(), libm::scalbn(1.0, 1022));
    }
    let profiles = CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).unwrap();
    assert_eq!(
        CoordinatedJointPathProfiles::total_path_time(profiles),
        Err(PlannerError::TimeElapsedOverflow)
    );
}

#[test]
fn a_large_but_representable_accumulated_time_is_accepted() {
    assert_eq!(
        total(
            [0.0, 1.0, 2.0, 3.0].map(|x| [x, 0.0]),
            [0.0; 4],
            [f64::MIN_POSITIVE, 1.0],
            [1.0; 2]
        ),
        Ok(3.0 * libm::scalbn(1.0, 1022))
    );
}

#[test]
fn a_positive_duration_absorbed_by_a_large_total_returns_precision_lost() {
    let points = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]].map(angles);
    let speeds = [0.0; 3];
    let vl = JointVelocityLimits::new(f64::MIN_POSITIVE, 1.0).unwrap();
    let al = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let mut reference = CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).unwrap();
    let first = reference.next().unwrap().unwrap().total_time();
    let second = reference.next().unwrap().unwrap().total_time();
    assert!(first.is_finite() && second > 0.0 && second.is_finite());
    assert_eq!(first + second, first);
    let profiles = CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).unwrap();
    assert_eq!(
        CoordinatedJointPathProfiles::total_path_time(profiles),
        Err(PlannerError::TimeElapsedPrecisionLost)
    );
}

#[test]
fn a_small_duration_before_a_large_one_still_advances_the_clock() {
    let expected = libm::scalbn(1.0, 1022);
    assert_eq!(
        total(
            [[1.0, 1.0], [1.0, 0.0], [0.0, 0.0]],
            [0.0; 3],
            [f64::MIN_POSITIVE, 1.0],
            [1.0; 2]
        ),
        Ok(expected)
    );
}

#[test]
fn durations_below_at_and_above_half_an_ulp_are_classified_by_actual_rounding() {
    let points = [[0.0, 0.0], [0.25, 0.0], [0.25, 1.0]].map(angles);
    let speeds = [0.0; 3];
    for (exponent, accepted) in [(110, false), (108, false), (106, true)] {
        let vl = JointVelocityLimits::new(10.0, libm::scalbn(1.0, 56)).unwrap();
        let al = JointAccelerationLimits::new(1.0, libm::scalbn(1.0, exponent)).unwrap();
        let mut reference = CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).unwrap();
        let first = reference.next().unwrap().unwrap().total_time();
        let second = reference.next().unwrap().unwrap().total_time();
        assert_eq!(first, 1.0);
        assert_eq!(second, libm::scalbn(1.0, 1 - exponent / 2));
        let profiles = CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).unwrap();
        let result = CoordinatedJointPathProfiles::total_path_time(profiles);
        if accepted {
            assert_eq!(result, Ok(1.0 + f64::EPSILON));
        } else {
            assert_eq!(result, Err(PlannerError::TimeElapsedPrecisionLost));
        }
    }
}

#[test]
fn an_exact_half_ulp_can_advance_an_odd_total_under_round_to_even() {
    let distance = 0.25_f64.next_up().next_up();
    let points = [[0.0, 0.0], [distance, 0.0], [distance, 1.0]].map(angles);
    let speeds = [0.0; 3];
    let vl = JointVelocityLimits::new(10.0, libm::scalbn(1.0, 56)).unwrap();
    let al = JointAccelerationLimits::new(1.0, libm::scalbn(1.0, 108)).unwrap();
    let mut reference = CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).unwrap();
    let first = reference.next().unwrap().unwrap().total_time();
    let second = reference.next().unwrap().unwrap().total_time();
    assert_eq!(first, 1.0_f64.next_up());
    assert_eq!(second, f64::EPSILON / 2.0);
    let profiles = CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).unwrap();
    assert_eq!(
        CoordinatedJointPathProfiles::total_path_time(profiles),
        Ok(first.next_up())
    );
}

#[test]
fn the_first_stalled_increment_is_reported_before_a_later_possible_overflow() {
    assert_eq!(
        total(
            [
                [0.0, 0.0],
                [1.0, 0.0],
                [1.0, 1.0],
                [2.0, 1.0],
                [3.0, 1.0],
                [4.0, 1.0]
            ],
            [0.0; 6],
            [f64::MIN_POSITIVE, 1.0],
            [1.0; 2]
        ),
        Err(PlannerError::TimeElapsedPrecisionLost)
    );
}

#[test]
fn overflow_is_reported_before_a_later_possible_stalled_increment() {
    assert_eq!(
        total(
            [
                [0.0, 0.0],
                [1.0, 0.0],
                [2.0, 0.0],
                [3.0, 0.0],
                [4.0, 0.0],
                [4.0, 1.0]
            ],
            [0.0; 6],
            [f64::MIN_POSITIVE, 1.0],
            [1.0; 2]
        ),
        Err(PlannerError::TimeElapsedOverflow)
    );
}
