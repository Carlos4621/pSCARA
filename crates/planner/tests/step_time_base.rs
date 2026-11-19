use kinematics::JointAngles;
use planner::{
    CoordinatedJointPathProfiles, JointAccelerationLimits, JointVelocityLimits, PlannerError,
    StepTimeBase, TimedCoordinatedJointPathProfiles,
};

const U64_EXCLUSIVE_UPPER_BOUND: f64 = (1_u128 << 64) as f64;
const EXACT_INTEGER_LIMIT: u64 = 1_u64 << 53;
const SMALLEST: f64 = f64::from_bits(1);

#[test]
fn ordinary_frequencies_convert_seconds_and_fractional_seconds() {
    for (frequency, seconds, expected) in [
        (1, 2.0, 2),
        (10, 0.25, 3),
        (1_000, 0.125, 125),
        (48_000, 1.25, 60_000),
        (1_000_000, 0.001, 1_000),
    ] {
        let base = StepTimeBase::new(frequency).unwrap();
        assert_eq!(
            base.seconds_to_ticks(seconds),
            Ok(expected),
            "frequency={frequency}, seconds={seconds}"
        );
    }
}

#[test]
fn signed_zeros_and_positive_subnormal_times_quantize_to_zero() {
    for frequency in [1, 1_000, u64::MAX] {
        let base = StepTimeBase::new(frequency).unwrap();
        for seconds in [0.0, -0.0, SMALLEST, f64::MIN_POSITIVE.next_down()] {
            assert_eq!(
                base.seconds_to_ticks(seconds),
                Ok(0),
                "frequency={frequency}, seconds={seconds}"
            );
        }
    }
}

#[test]
fn the_constructor_rejects_zero_frequency() {
    assert_eq!(
        StepTimeBase::new(0).unwrap_err(),
        PlannerError::InvalidTicks
    );
}

#[test]
fn negative_or_nonfinite_seconds_are_invalid_at_any_frequency() {
    for frequency in [1, 1_000, u64::MAX] {
        let base = StepTimeBase::new(frequency).unwrap();
        for seconds in [
            -SMALLEST,
            -1.0,
            -f64::MAX,
            f64::NAN,
            -f64::NAN,
            f64::INFINITY,
            f64::NEG_INFINITY,
        ] {
            assert_eq!(
                base.seconds_to_ticks(seconds),
                Err(PlannerError::InvalidTime),
                "frequency={frequency}, seconds={seconds}"
            );
        }
    }
}

#[test]
fn half_ticks_round_up_and_their_adjacent_times_round_to_the_nearest_tick() {
    let base = StepTimeBase::new(8).unwrap();
    for (ticks, floor) in [(0.5, 0), (1.5, 1), (7.5, 7), (511.5, 511)] {
        let middle: f64 = ticks / 8.0;
        assert_eq!(base.seconds_to_ticks(middle.next_down()), Ok(floor));
        assert_eq!(base.seconds_to_ticks(middle), Ok(floor + 1));
        assert_eq!(base.seconds_to_ticks(middle.next_up()), Ok(floor + 1));
    }
}

#[test]
fn a_grid_of_binary_fractions_matches_an_integer_rounding_reference() {
    const DENOMINATOR: u128 = 1024;
    for frequency in [
        1,
        3,
        7,
        64,
        1_000,
        48_000,
        EXACT_INTEGER_LIMIT + 1,
        u64::MAX - 1,
        u64::MAX,
    ] {
        let base = StepTimeBase::new(frequency).unwrap();
        for numerator in 0..=2048_u32 {
            let seconds = f64::from(numerator) / DENOMINATOR as f64;
            let rounded =
                (u128::from(frequency) * u128::from(numerator) + DENOMINATOR / 2) / DENOMINATOR;
            let expected = u64::try_from(rounded).map_err(|_| PlannerError::NumericalFailure);
            assert_eq!(
                base.seconds_to_ticks(seconds),
                expected,
                "frequency={frequency}, numerator={numerator}"
            );
        }
    }
}

#[test]
fn the_largest_f64_below_the_u64_limit_converts_without_saturating() {
    let last = U64_EXCLUSIVE_UPPER_BOUND.next_down();
    let expected = u64::MAX - 2047;
    assert_eq!(
        StepTimeBase::new(1).unwrap().seconds_to_ticks(last),
        Ok(expected)
    );
    assert_eq!(
        StepTimeBase::new(8).unwrap().seconds_to_ticks(last / 8.0),
        Ok(expected)
    );
}

#[test]
fn exactly_two_to_the_power_of_64_ticks_is_out_of_range() {
    let cases = [
        (1, U64_EXCLUSIVE_UPPER_BOUND),
        (8, U64_EXCLUSIVE_UPPER_BOUND / 8.0),
        (1_u64 << 63, 2.0),
    ];
    assert_eq!(
        cases.map(|(frequency, seconds)| {
            StepTimeBase::new(frequency)
                .unwrap()
                .seconds_to_ticks(seconds)
        }),
        [Err(PlannerError::NumericalFailure); 3]
    );
}

#[test]
fn both_finite_excess_and_product_overflow_return_numerical_failure() {
    for (frequency, seconds) in [
        (1, U64_EXCLUSIVE_UPPER_BOUND.next_up()),
        (1, f64::MAX),
        (2, f64::MAX),
        (u64::MAX, f64::MAX),
    ] {
        let base = StepTimeBase::new(frequency).unwrap();
        assert_eq!(
            base.seconds_to_ticks(seconds),
            Err(PlannerError::NumericalFailure)
        );
        assert_eq!(base.seconds_to_ticks(0.0), Ok(0));
    }
}

#[test]
fn the_constructor_accepts_and_preserves_positive_frequency_limits() {
    let frequencies = [
        1,
        2,
        EXACT_INTEGER_LIMIT - 1,
        EXACT_INTEGER_LIMIT,
        EXACT_INTEGER_LIMIT + 1,
        EXACT_INTEGER_LIMIT + 3,
        u64::MAX - 1,
        u64::MAX,
    ];
    for frequency in frequencies {
        let base = StepTimeBase::new(frequency).unwrap();
        assert_eq!(
            base.seconds_to_ticks(1.0),
            Ok(frequency),
            "frequency={frequency}"
        );
    }
}

#[test]
fn half_a_second_with_a_large_odd_frequency_retains_the_half_tick() {
    let frequency = EXACT_INTEGER_LIMIT + 1;
    let base = StepTimeBase::new(frequency).unwrap();
    assert_eq!(base.seconds_to_ticks(0.5), Ok(frequency / 2 + 1));
}

#[test]
fn a_product_just_below_half_a_tick_does_not_round_twice() {
    let base = StepTimeBase::new(3).unwrap();
    // This f64 is strictly below 1/6, but a floating-point product rounds to 0.5.
    let below = 1.0_f64 / 6.0;
    assert_eq!(3.0 * below, 0.5);
    assert_eq!(base.seconds_to_ticks(below), Ok(0));
    assert_eq!(base.seconds_to_ticks(below.next_up()), Ok(1));
}

#[test]
fn the_first_tick_at_maximum_frequency_respects_the_exact_half_tick_boundary() {
    let base = StepTimeBase::new(u64::MAX).unwrap();
    let below = 1.0 / (1_u128 << 65) as f64;
    // The exact product is (2^64 - 1) / 2^65, just under half a tick.
    assert_eq!(base.seconds_to_ticks(below), Ok(0));
    assert_eq!(base.seconds_to_ticks(below.next_up()), Ok(1));
}

#[test]
fn rounding_near_the_u64_limit_checks_the_final_integer() {
    let twice_the_limit = 1_u128 << 65;
    let rounds_to_max = u64::try_from((twice_the_limit - 3) / 47).unwrap();
    let rounds_past_max = u64::try_from((twice_the_limit - 1) / 31).unwrap();
    // Products are exactly 2^64 - 1.5 and 2^64 - 0.5 respectively.
    assert_eq!(
        StepTimeBase::new(rounds_to_max)
            .unwrap()
            .seconds_to_ticks(23.5),
        Ok(u64::MAX)
    );
    assert_eq!(
        StepTimeBase::new(rounds_past_max)
            .unwrap()
            .seconds_to_ticks(15.5),
        Err(PlannerError::NumericalFailure)
    );
}

#[test]
fn coordinated_profile_boundaries_quantize_to_shared_ticks() {
    let points = [0.0, 1.0, 5.0, 14.0].map(|first| JointAngles::new(first, 0.0).unwrap());
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let source =
        CoordinatedJointPathProfiles::new(&points, &[0.0; 4], &velocity, &acceleration).unwrap();
    let mut timed = TimedCoordinatedJointPathProfiles::new(source).unwrap();
    let base = StepTimeBase::new(2_000).unwrap();

    // Distances 1, 4 and 9 with unit acceleration take 2, 4 and 6 seconds.
    for (start, end) in [(0, 4_000), (4_000, 12_000), (12_000, 24_000)] {
        let profile = timed.next().unwrap().unwrap();
        assert_eq!(base.seconds_to_ticks(profile.start_time()), Ok(start));
        assert_eq!(
            profile
                .end_time()
                .and_then(|time| base.seconds_to_ticks(time)),
            Ok(end)
        );
    }
    assert!(timed.next().is_none());
}
