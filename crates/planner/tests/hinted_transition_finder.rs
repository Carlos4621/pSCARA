#![cfg(feature = "transition-estimation")]

use planner::{
    MotorStepMove, MotorSteps, PlannerError, TransitionEstimationStatistics,
    find_first_transition_tick_exponential, find_first_transition_tick_hinted,
};

fn steps(first: i64, second: i64) -> MotorSteps {
    MotorSteps { first, second }
}

#[test]
fn exact_hints_confirm_both_motors_with_one_or_two_samples() {
    let start = 100;
    let end = 180;
    let initial = steps(7, -9);
    let target = steps(8, -10);
    for transition in [start + 1, start + 2, 147, end] {
        let mut probes = Vec::new();
        let actual =
            find_first_transition_tick_hinted(start, end, initial, Some(transition), |tick| {
                probes.push(tick);
                Ok(if tick < transition { initial } else { target })
            });
        assert_eq!(
            actual,
            Ok(Some((transition, MotorStepMove::new(initial, target))))
        );
        if transition == start + 1 {
            assert_eq!(probes, [transition]);
        } else {
            assert_eq!(probes, [transition, transition - 1]);
        }
    }
}

#[test]
fn wrong_and_unusable_hints_fall_back_to_the_exponential_probes() {
    let start = 10;
    let end = 30;
    let initial = steps(0, 0);
    let sample = |tick| Ok(if tick < 17 { initial } else { steps(0, -1) });
    let mut baseline = Vec::new();
    let expected = find_first_transition_tick_exponential(start, end, initial, |tick| {
        baseline.push(tick);
        sample(tick)
    });
    for (hint, prefix) in [
        (Some(16), &[16][..]),
        (Some(18), &[18, 17][..]),
        (None, &[][..]),
        (Some(start), &[][..]),
        (Some(start - 1), &[][..]),
        (Some(end + 1), &[][..]),
    ] {
        let mut probes = Vec::new();
        let actual = find_first_transition_tick_hinted(start, end, initial, hint, |tick| {
            probes.push(tick);
            sample(tick)
        });
        assert_eq!(actual, expected, "hint={hint:?}");
        assert_eq!(&probes[..prefix.len()], prefix);
        assert_eq!(&probes[prefix.len()..], baseline);
    }
}

#[test]
fn hinted_and_exponential_search_agree_for_all_hints_in_short_intervals() {
    let initial = steps(0, 0);
    for start in [0, u64::MAX - 32] {
        for end in start + 1..=start + 32 {
            for change_at in start + 1..=end {
                for next in [steps(1, 0), steps(0, -1), steps(1, -1), steps(2, -1)] {
                    let sample = |tick| Ok(if tick < change_at { initial } else { next });
                    let expected =
                        find_first_transition_tick_exponential(start, end, initial, sample);
                    for hint in (start..=end).map(Some).chain([None]) {
                        assert_eq!(
                            find_first_transition_tick_hinted(start, end, initial, hint, sample),
                            expected,
                            "start={start}, end={end}, transition={change_at}, hint={hint:?}"
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn preserves_empty_stationary_and_inverted_interval_results() {
    let initial = steps(1, -1);
    for hint in [None, Some(10), Some(11), Some(u64::MAX)] {
        assert_eq!(
            find_first_transition_tick_hinted(10, 10, initial, hint, |_| unreachable!()),
            Ok(None)
        );
        assert_eq!(
            find_first_transition_tick_hinted(10, 9, initial, hint, |_| unreachable!()),
            Err(PlannerError::EndBeforeStart)
        );
        assert_eq!(
            find_first_transition_tick_hinted(10, 20, initial, hint, |_| Ok(initial)),
            Ok(None)
        );
    }
}

#[test]
fn propagates_errors_in_verification_and_in_the_fallback() {
    let initial = steps(0, 0);
    for (hint, error_tick) in [(Some(7), 7), (Some(7), 6), (Some(2), 4), (None, 1)] {
        assert_eq!(
            find_first_transition_tick_hinted(0, 20, initial, hint, |tick| {
                if tick == error_tick {
                    Err(PlannerError::InvalidTime)
                } else if tick >= 7 {
                    Ok(steps(1, 0))
                } else {
                    Ok(initial)
                }
            }),
            Err(PlannerError::InvalidTime)
        );
    }
}

#[test]
fn estimation_statistics_count_missing_and_inclusive_tick_errors() {
    let mut stats = TransitionEstimationStatistics::default();
    for estimated_tick in [None, Some(100), Some(99), Some(102), Some(103), Some(94)] {
        stats.record(100, estimated_tick);
    }
    assert_eq!(stats.transitions, 6);
    assert_eq!(stats.estimates, 5);
    assert_eq!(stats.exact_estimates, 1);
    assert_eq!(stats.estimates_within_one_tick, 2);
    assert_eq!(stats.estimates_within_two_ticks, 3);
    assert_eq!(stats.max_error_ticks, 6);
}
