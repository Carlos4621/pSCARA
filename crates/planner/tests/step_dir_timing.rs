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

#[test]
fn dir_setup_and_hold_times_round_up_to_whole_ticks() {
    for (nanoseconds, frequency, expected_ticks) in [
        (200, 1_000_000, 1),
        (1_000, 1_000_000, 1),
        (1_500, 1_000_000, 2),
        (200, 10_000_000, 2),
        (1, 1, 1),
    ] {
        let timing = StepDirTiming::new(2_000, 2_000, nanoseconds, nanoseconds).unwrap();
        let timebase = StepTimeBase::new(frequency).unwrap();

        assert_eq!(timing.min_dir_setup_ticks(timebase), Ok(expected_ticks));
        assert_eq!(timing.min_dir_hold_ticks(timebase), Ok(expected_ticks));
    }
}

#[test]
fn dir_setup_and_hold_use_their_own_durations() {
    let timing = StepDirTiming::new(2_000, 2_000, 1_500, 800).unwrap();
    let timebase = StepTimeBase::new(1_000_000).unwrap();

    assert_eq!(timing.min_dir_setup_ticks(timebase), Ok(2));
    assert_eq!(timing.min_dir_hold_ticks(timebase), Ok(1));
}

#[test]
fn dir_setup_and_hold_detect_results_above_u64_max() {
    let max = u64::MAX;
    let overflow = Err(PlannerError::NumericalFailure);
    for (setup, hold, frequency, expected_setup, expected_hold) in [
        (max - 1, max, 1_000_000_000, Ok(max - 1), Ok(max)),
        (1_000_000_000, 1_000_000_001, max, Ok(max), overflow),
        (max, max, max, overflow, overflow),
    ] {
        let timing = StepDirTiming::new(2_000, 2_000, setup, hold).unwrap();
        let timebase = StepTimeBase::new(frequency).unwrap();

        assert_eq!(timing.min_dir_setup_ticks(timebase), expected_setup);
        assert_eq!(timing.min_dir_hold_ticks(timebase), expected_hold);
    }
}

#[test]
fn dir_change_window_checks_setup_hold_and_tick_order() {
    let timing = StepDirTiming::new(2_000, 2_000, 3_000, 2_000).unwrap();
    let timebase = StepTimeBase::new(1_000_000).unwrap();
    let unschedulable = Err(PlannerError::DirChangeNotSchedulable);

    for (previous, next, expected) in [
        (None, 10, Ok((0, 7))),
        (None, 3, Ok((0, 0))),
        (Some(100), 110, Ok((102, 107))),
        (Some(100), 105, Ok((102, 102))),
        (Some(100), 104, unschedulable),
        (Some(100), 100, Err(PlannerError::InvalidTicks)),
        (Some(100), 99, Err(PlannerError::InvalidTicks)),
        (None, 2, unschedulable),
    ] {
        assert_eq!(
            timing.dir_change_window(previous, next, timebase),
            expected,
            "previous={previous:?}, next={next}"
        );
    }
}

#[test]
fn dir_change_window_checks_tick_addition_near_u64_max() {
    let timing = StepDirTiming::new(2_000, 2_000, 3_000, 2_000).unwrap();
    let timebase = StepTimeBase::new(1_000_000).unwrap();
    let max = u64::MAX;
    let unschedulable = Err(PlannerError::DirChangeNotSchedulable);

    for (previous, expected) in [
        (Some(max - 5), Ok((max - 3, max - 3))),
        (Some(max - 2), unschedulable),
        (Some(max - 1), unschedulable),
        (Some(max), Err(PlannerError::InvalidTicks)),
    ] {
        assert_eq!(
            timing.dir_change_window(previous, max, timebase),
            expected,
            "previous={previous:?}"
        );
    }
}

#[test]
fn dir_change_window_propagates_conversion_errors_for_applicable_constraints() {
    let timebase = StepTimeBase::new(2_000_000_000).unwrap();
    let long_hold = StepDirTiming::new(1, 1, 1, u64::MAX).unwrap();
    let long_setup = StepDirTiming::new(1, 1, u64::MAX, 1).unwrap();

    assert_eq!(long_hold.dir_change_window(None, 10, timebase), Ok((0, 8)));
    assert_eq!(
        long_hold.dir_change_window(Some(0), u64::MAX, timebase),
        Err(PlannerError::NumericalFailure)
    );
    assert_eq!(
        long_setup.dir_change_window(None, u64::MAX, timebase),
        Err(PlannerError::NumericalFailure)
    );
}
