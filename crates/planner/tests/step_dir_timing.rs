use planner::{PlannerError, StepDirTiming};

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
