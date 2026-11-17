use kinematics::JointAngles;
use planner::{
    CoordinatedJointMotionProfile, CoordinatedJointPathProfiles, JointAccelerationLimits,
    JointVelocityLimits, PlannerError,
};

type Pair = [f64; 2];

fn assert_remaining<I: ExactSizeIterator + core::iter::FusedIterator>(
    iterator: &I,
    remaining: usize,
) {
    // UFCS deliberately calls the traits rather than an inherent method with
    // the same name. The bounds also check that both traits are implemented.
    assert_eq!(Iterator::size_hint(iterator), (remaining, Some(remaining)));
    assert_eq!(ExactSizeIterator::len(iterator), remaining);
}

fn angles(q: Pair) -> JointAngles {
    JointAngles::new(q[0], q[1]).unwrap()
}

fn position(profile: &CoordinatedJointMotionProfile, time: f64) -> Pair {
    let q = profile.position_at(time).unwrap();
    [q.first_theta(), q.second_theta()]
}

fn assert_same_profile(
    actual: &CoordinatedJointMotionProfile,
    expected: &CoordinatedJointMotionProfile,
) {
    assert_eq!(actual.total_time(), expected.total_time());
    for step in 0..=32 {
        let time = actual.total_time() * (f64::from(step) / 32.0);
        assert_eq!(position(actual, time), position(expected, time));
        assert_eq!(actual.velocity_at(time), expected.velocity_at(time));
        assert_eq!(actual.acceleration_at(time), expected.acceleration_at(time));
    }
}

fn check_iteration<const N: usize>(
    coordinates: [Pair; N],
    speeds: [f64; N],
    velocity: Pair,
    acceleration: Pair,
) -> Vec<CoordinatedJointMotionProfile> {
    let points = coordinates.map(angles);
    let vl = JointVelocityLimits::new(velocity[0], velocity[1]).unwrap();
    let al = JointAccelerationLimits::new(acceleration[0], acceleration[1]).unwrap();
    let mut chain = CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).unwrap();
    let mut profiles = Vec::new();
    for i in 0..N - 1 {
        assert_remaining(&chain, N - 1 - i);
        let profile = chain
            .next()
            .expect("iterator ended before the last segment")
            .unwrap();
        assert_eq!(position(&profile, 0.0), coordinates[i]);
        assert_eq!(position(&profile, profile.total_time()), coordinates[i + 1]);
        let length = (coordinates[i + 1][0] - coordinates[i][0])
            .abs()
            .max((coordinates[i + 1][1] - coordinates[i][1]).abs());
        let expected = CoordinatedJointMotionProfile::new(
            points[i],
            points[i + 1],
            &vl,
            &al,
            speeds[i] / length,
            speeds[i + 1] / length,
        )
        .unwrap();
        assert_same_profile(&profile, &expected);
        profiles.push(profile);
    }
    assert_eq!(profiles.len(), N - 1);
    for _ in 0..8 {
        assert_remaining(&chain, 0);
        assert!(
            chain.next().is_none(),
            "exhausted iterator yielded another profile"
        );
    }
    profiles
}

#[test]
fn exact_remaining_length_decreases_after_each_next_and_stays_zero_at_exhaustion() {
    let points = [0.0, 1.0, 2.0, 3.0, 4.0].map(|x| angles([x, 0.0]));
    let speeds = [0.0; 5];
    let vl = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let al = JointAccelerationLimits::new(2.0, 2.0).unwrap();
    let mut chain = CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).unwrap();
    for remaining in (1..=4).rev() {
        assert_remaining(&chain, remaining);
        assert!(chain.next().unwrap().is_ok());
        assert_remaining(&chain, remaining - 1);
    }
    for _ in 0..8 {
        assert!(chain.next().is_none());
        assert_remaining(&chain, 0);
    }
}

#[test]
fn exact_remaining_length_tracks_skipping_partial_consumption_and_overrun() {
    let points = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0].map(|x| angles([x, 0.0]));
    let speeds = [0.0; 6];
    let vl = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let al = JointAccelerationLimits::new(2.0, 2.0).unwrap();
    let mut chain = CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).unwrap();
    assert_remaining(&chain, 5);
    assert!(chain.next().unwrap().is_ok());
    assert_remaining(&chain, 4);
    assert!(chain.nth(1).unwrap().is_ok());
    assert_remaining(&chain, 2);
    {
        let mut prefix = chain.by_ref().take(1);
        assert_remaining(&prefix, 1);
        assert!(prefix.next().unwrap().is_ok());
        assert_remaining(&prefix, 0);
        assert!(prefix.next().is_none());
    }
    assert_remaining(&chain, 1);
    assert!(chain.nth(10).is_none());
    assert_remaining(&chain, 0);
    assert_eq!(chain.by_ref().count(), 0);
    assert_remaining(&chain, 0);
}

#[test]
fn exact_size_hint_is_available_through_dynamic_trait_dispatch() {
    let points = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]].map(angles);
    let speeds = [0.0; 3];
    let vl = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let al = JointAccelerationLimits::new(2.0, 2.0).unwrap();
    let mut chain = CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).unwrap();
    let iterator: &mut dyn ExactSizeIterator<
        Item = Result<CoordinatedJointMotionProfile, PlannerError>,
    > = &mut chain;
    for remaining in (1..=2).rev() {
        assert_eq!(iterator.size_hint(), (remaining, Some(remaining)));
        assert_eq!(iterator.len(), remaining);
        assert!(iterator.next().unwrap().is_ok());
    }
    assert_eq!(iterator.size_hint(), (0, Some(0)));
    assert_eq!(iterator.len(), 0);
    assert!(iterator.next().is_none());
}

#[test]
fn a_single_segment_is_yielded_once_and_then_exhausted_permanently() {
    let profiles = check_iteration([[3.0, -2.0], [4.0, 1.0]], [0.0; 2], [10.0; 2], [2.0; 2]);
    assert_eq!(profiles[0].velocity_at(0.0), Ok((0.0, 0.0)));
    assert_eq!(
        profiles[0].velocity_at(profiles[0].total_time()),
        Ok((0.0, 0.0))
    );
}

#[test]
fn every_segment_is_yielded_in_order_including_corners_and_reversals() {
    check_iteration(
        [
            [0.0, 0.0],
            [2.0, -1.0],
            [5.0, 1.0],
            [2.0, -1.0],
            [-1.0, -1.0],
        ],
        [0.0; 5],
        [10.0; 2],
        [2.0; 2],
    );
}

#[test]
fn unequal_segment_lengths_normalize_each_pair_of_speeds_independently() {
    let profiles = check_iteration(
        [[0.0, 0.0], [2.0, -1.0], [6.0, -3.0], [12.0, -6.0]],
        [0.0, 4.0, 4.0, 0.0],
        [20.0, 10.0],
        [4.0, 2.0],
    );
    for adjacent in profiles.windows(2) {
        assert_eq!(
            adjacent[0].velocity_at(adjacent[0].total_time()),
            adjacent[1].velocity_at(0.0)
        );
    }
}

#[test]
fn nth_skips_exactly_the_requested_number_of_segments() {
    let points = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0].map(|x| angles([x, 0.0]));
    let speeds = [0.0; 6];
    let vl = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let al = JointAccelerationLimits::new(2.0, 2.0).unwrap();
    let mut chain = CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).unwrap();
    let second = chain.nth(1).unwrap().unwrap();
    assert_eq!(position(&second, 0.0), [1.0, 0.0]);
    let third = chain.next().unwrap().unwrap();
    assert_eq!(position(&third, 0.0), [2.0, 0.0]);
    let fifth = chain.nth(1).unwrap().unwrap();
    assert_eq!(position(&fifth, 0.0), [4.0, 0.0]);
    assert_eq!(position(&fifth, fifth.total_time()), [5.0, 0.0]);
    assert!(chain.next().is_none());
}

#[test]
fn nth_beyond_the_end_leaves_the_iterator_exhausted() {
    let points = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]].map(angles);
    let vl = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let al = JointAccelerationLimits::new(2.0, 2.0).unwrap();
    let speeds = [0.0; 3];
    let mut chain = CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).unwrap();
    assert!(chain.nth(10).is_none());
    for _ in 0..8 {
        assert!(chain.next().is_none());
    }
}

#[test]
fn partial_consumption_with_take_can_be_resumed_without_repeating_segments() {
    let points = [0.0, 1.0, 2.0, 3.0, 4.0].map(|x| angles([x, 0.0]));
    let speeds = [0.0; 5];
    let vl = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let al = JointAccelerationLimits::new(2.0, 2.0).unwrap();
    let mut chain = CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).unwrap();
    let prefix = chain
        .by_ref()
        .take(2)
        .collect::<Result<Vec<_>, PlannerError>>()
        .unwrap();
    assert_eq!(prefix.len(), 2);
    assert_eq!(position(&prefix[0], 0.0), [0.0, 0.0]);
    assert_eq!(position(&prefix[1], 0.0), [1.0, 0.0]);
    let suffix = chain
        .by_ref()
        .collect::<Result<Vec<_>, PlannerError>>()
        .unwrap();
    assert_eq!(suffix.len(), 2);
    assert_eq!(position(&suffix[0], 0.0), [2.0, 0.0]);
    assert_eq!(position(&suffix[1], 0.0), [3.0, 0.0]);
    assert!(chain.next().is_none());
    // Previously yielded owned profiles remain usable after iteration advances.
    assert_eq!(position(&prefix[0], prefix[0].total_time()), [1.0, 0.0]);
}

#[test]
fn iterators_sharing_inputs_keep_independent_cursors() {
    let points = [0.0, 1.0, 2.0, 3.0].map(|x| angles([x, 0.0]));
    let speeds = [0.0; 4];
    let vl = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let al = JointAccelerationLimits::new(2.0, 2.0).unwrap();
    let mut first = CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).unwrap();
    let mut second = CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).unwrap();
    assert_eq!(position(&first.next().unwrap().unwrap(), 0.0), [0.0, 0.0]);
    assert_eq!(position(&first.next().unwrap().unwrap(), 0.0), [1.0, 0.0]);
    let all = second
        .by_ref()
        .collect::<Result<Vec<_>, PlannerError>>()
        .unwrap();
    assert_eq!(all.len(), 3);
    assert_eq!(position(&all[0], 0.0), [0.0, 0.0]);
    assert_eq!(position(&first.next().unwrap().unwrap(), 0.0), [2.0, 0.0]);
    assert!(first.next().is_none() && second.next().is_none());
}

#[test]
fn either_joint_can_be_stationary_with_signed_zero() {
    for zero in [0.0, -0.0] {
        for points in [
            [[zero, 0.0], [-zero, 1.0], [zero, 3.0]],
            [[0.0, zero], [1.0, -zero], [3.0, zero]],
        ] {
            check_iteration(points, [0.0, 2.0, 0.0], [10.0; 2], [2.0; 2]);
        }
    }
}

#[test]
fn representable_subnormal_boundary_speeds_survive_next() {
    let tiny = f64::from_bits(1);
    let profiles = check_iteration(
        [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]],
        [0.0, tiny, 0.0],
        [10.0; 2],
        [2.0; 2],
    );
    assert_eq!(
        profiles[0].velocity_at(profiles[0].total_time()),
        Ok((tiny, 0.0))
    );
    assert_eq!(profiles[1].velocity_at(0.0), Ok((tiny, 0.0)));
    let profiles = check_iteration(
        [[0.0, 0.0], [2.0, 0.0], [4.0, 0.0]],
        [0.0, 2.0 * tiny, 0.0],
        [10.0; 2],
        [2.0; 2],
    );
    assert_eq!(profiles[1].velocity_at(0.0), Ok((2.0 * tiny, 0.0)));
}

#[test]
fn subnormal_geometry_with_scaled_limits_is_iterable() {
    let tiny = f64::from_bits(1);
    check_iteration(
        [[0.0, 0.0], [tiny, 0.0], [2.0 * tiny, 0.0]],
        [0.0, tiny, 0.0],
        [tiny; 2],
        [tiny; 2],
    );
}

#[test]
fn largest_finite_geometry_and_boundary_speed_are_iterable() {
    check_iteration(
        [[-f64::MAX, 0.0], [0.0, 0.0], [f64::MAX, 0.0]],
        [0.0, f64::MAX, 0.0],
        [f64::MAX; 2],
        [f64::MAX; 2],
    );
}

#[test]
fn iteration_matches_direct_profiles_across_extreme_common_scales() {
    for scale in [
        libm::scalbn(1.0, -500),
        1e-100,
        1.0,
        1e100,
        libm::scalbn(1.0, 500),
    ] {
        check_iteration(
            [
                [0.0, 0.0],
                [2.0 * scale, -scale],
                [6.0 * scale, -3.0 * scale],
                [12.0 * scale, -6.0 * scale],
            ],
            [0.0, 4.0 * scale, 4.0 * scale, 0.0],
            [20.0 * scale, 10.0 * scale],
            [4.0 * scale, 2.0 * scale],
        );
    }
}
