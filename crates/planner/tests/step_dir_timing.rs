use planner::{PlannerError, StepDirTiming, StepTimeBase};

#[test]
fn accepts_positive_times_and_exposes_them_in_nanoseconds() {
    let timing = StepDirTiming::new(1, 23, 47, 89).unwrap();

    assert_eq!(timing.step_high_ns(), 1);
    assert_eq!(timing.step_low_ns(), 23);
    assert_eq!(timing.dir_setup_ns(), 47);
    assert_eq!(timing.dir_hold_ns(), 89);
}

#[test]
fn rejects_zero_in_each_timing_parameter() {
    for times @ [step_high_ns, step_low_ns, dir_setup_ns, dir_hold_ns] in
        [[0, 1, 1, 1], [1, 0, 1, 1], [1, 1, 0, 1], [1, 1, 1, 0]]
    {
        assert_eq!(
            StepDirTiming::new(step_high_ns, step_low_ns, dir_setup_ns, dir_hold_ns),
            Err(PlannerError::InvalidStepDirTiming),
            "times={times:?}"
        );
    }
}

#[test]
fn accepts_large_times_up_to_u64_max() {
    let max = u64::MAX;
    for (high, low, setup, hold) in [(max - 3, max - 2, max - 1, max), (max, max, max, max)] {
        let timing = StepDirTiming::new(high, low, setup, hold).unwrap();

        assert_eq!(timing.step_high_ns(), high);
        assert_eq!(timing.step_low_ns(), low);
        assert_eq!(timing.dir_setup_ns(), setup);
        assert_eq!(timing.dir_hold_ns(), hold);
    }
}

#[test]
fn minimum_step_interval_rounds_up_the_sum_of_high_and_low_times() {
    for (high, low, frequency, expected_ticks) in [
        (2_000, 2_000, 1_000_000, 4),
        (2_000, 2_000, 3_000_000, 12),
        (2_000, 2_000, 100_000, 1),
        (1_100, 1_500, 1_000_000, 3),
        (1, 1, 1, 1),
    ] {
        let timing = StepDirTiming::new(high, low, u64::MAX, u64::MAX).unwrap();
        let timebase = StepTimeBase::new(frequency).unwrap();

        assert_eq!(
            timing.min_step_interval_ticks(timebase),
            Ok(expected_ticks),
            "high={high}, low={low}, frequency={frequency}"
        );
    }
}

#[test]
fn minimum_step_interval_handles_maximum_step_times_at_clock_extremes() {
    let timing = StepDirTiming::new(u64::MAX, u64::MAX, 1, 1).unwrap();

    assert_eq!(
        timing.min_step_interval_ticks(StepTimeBase::new(1).unwrap()),
        Ok(36_893_488_148)
    );
    assert_eq!(
        timing.min_step_interval_ticks(StepTimeBase::new(u64::MAX).unwrap()),
        Err(PlannerError::NumericalFailure)
    );
}

#[test]
fn minimum_step_interval_checks_the_u64_limit_after_rounding() {
    let max = u64::MAX;
    let timebase = StepTimeBase::new(750_000_000).unwrap();
    // A 750 MHz, (max + max / 3) nanosegundos equivalen exactamente a max ticks.
    let low_at_limit = max / 3;

    for (low, expected) in [
        (low_at_limit - 1, Ok(max)),
        (low_at_limit, Ok(max)),
        (low_at_limit + 1, Err(PlannerError::NumericalFailure)),
    ] {
        let timing = StepDirTiming::new(max, low, 1, 1).unwrap();
        assert_eq!(
            timing.min_step_interval_ticks(timebase),
            expected,
            "low={low}"
        );
    }
}
