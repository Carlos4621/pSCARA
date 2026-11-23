use planner::{PinOperationReleaseCursor, PlannerError};

#[test]
fn releases_only_ticks_at_or_before_the_inclusive_safe_limit() {
    let mut cursor = PinOperationReleaseCursor::new(10).unwrap();
    assert!(!cursor.can_release(0));

    cursor.observe(5).unwrap();
    assert!(!cursor.can_release(0));

    cursor.observe(10).unwrap();
    assert!(cursor.can_release(0));
    assert!(!cursor.can_release(1));

    cursor.observe(100).unwrap();
    assert!(cursor.can_release(89));
    assert!(cursor.can_release(90));
    assert!(!cursor.can_release(91));

    cursor.observe(101).unwrap();
    assert!(cursor.can_release(91));
    assert!(!cursor.can_release(92));
}

#[test]
fn rejects_zero_dir_setup() {
    assert_eq!(
        PinOperationReleaseCursor::new(0).unwrap_err(),
        PlannerError::InvalidStepDirTiming
    );
}

#[test]
fn rejects_repeated_and_decreasing_ticks_and_preserves_the_limit() {
    let mut cursor = PinOperationReleaseCursor::new(10).unwrap();
    cursor.observe(100).unwrap();

    for invalid_tick in [100, 99] {
        assert_eq!(
            cursor.observe(invalid_tick),
            Err(PlannerError::InvalidTicks)
        );
        assert!(cursor.can_release(90));
        assert!(!cursor.can_release(91));
    }

    cursor.observe(101).unwrap();
    assert!(cursor.can_release(91));
}

#[test]
fn finishing_allows_every_tick_and_rejects_further_observations() {
    for last_step in [None, Some(100)] {
        let mut cursor = PinOperationReleaseCursor::new(10).unwrap();
        if let Some(tick) = last_step {
            cursor.observe(tick).unwrap();
        }

        cursor.finish();
        assert!(cursor.can_release(u64::MAX));
        cursor.finish();
        assert!(cursor.can_release(0));
        assert!(cursor.can_release(u64::MAX));

        assert_eq!(cursor.observe(101), Err(PlannerError::InvalidTicks));
        assert!(cursor.can_release(u64::MAX));
    }
}

#[test]
fn computes_the_safe_limit_near_u64_max() {
    let max = u64::MAX;
    let mut cursor = PinOperationReleaseCursor::new(10).unwrap();

    cursor.observe(max - 1).unwrap();
    assert!(cursor.can_release(max - 11));
    assert!(!cursor.can_release(max - 10));

    cursor.observe(max).unwrap();
    assert!(cursor.can_release(max - 10));
    assert!(!cursor.can_release(max - 9));
    assert!(!cursor.can_release(max));
}

#[test]
fn accepts_a_dir_setup_covering_the_entire_tick_range() {
    let max = u64::MAX;
    let mut cursor = PinOperationReleaseCursor::new(max).unwrap();

    cursor.observe(max - 1).unwrap();
    assert!(!cursor.can_release(0));

    cursor.observe(max).unwrap();
    assert!(cursor.can_release(0));
    assert!(!cursor.can_release(1));
}
