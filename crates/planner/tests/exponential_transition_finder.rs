use planner::{
    MotorStepMove, MotorSteps, PlannerError, find_first_transition_tick,
    find_first_transition_tick_exponential,
};

fn steps(first: i64, second: i64) -> MotorSteps {
    MotorSteps { first, second }
}

#[test]
fn exponential_and_binary_agree_at_each_tick_in_short_monotone_intervals() {
    let start = 41;
    let initial = steps(7, -5);
    for first_change in [steps(6, -5), steps(7, -4), steps(6, -4)] {
        for width in 1..=64 {
            let end = start + width;
            for transition in start + 1..=end {
                let sample = |tick: u64| {
                    assert!((start + 1..=end).contains(&tick));
                    Ok(match tick.cmp(&transition) {
                        core::cmp::Ordering::Less => initial,
                        core::cmp::Ordering::Equal => first_change,
                        core::cmp::Ordering::Greater => steps(4, -2),
                    })
                };
                let expected = Ok(Some((
                    transition,
                    MotorStepMove::new(initial, first_change),
                )));
                assert_eq!(
                    find_first_transition_tick(start, end, initial, sample),
                    expected
                );
                assert_eq!(
                    find_first_transition_tick_exponential(start, end, initial, sample),
                    expected,
                    "start={start}, end={end}, transition={transition}, first={first_change:?}"
                );
            }
        }
    }
}

#[test]
fn validates_empty_and_inverted_intervals_without_sampling() {
    let initial = steps(0, 0);
    for tick in [0, 40, u64::MAX] {
        assert_eq!(
            find_first_transition_tick_exponential(tick, tick, initial, |_| unreachable!()),
            Ok(None)
        );
    }
    assert_eq!(
        find_first_transition_tick_exponential(u64::MAX, 0, initial, |_| unreachable!()),
        Err(PlannerError::EndBeforeStart)
    );
}

#[test]
fn searches_the_full_range_and_near_the_maximum_tick_without_overflow() {
    let initial = steps(i64::MAX, i64::MIN);
    let target = steps(i64::MAX - 1, i64::MIN + 1);
    for start in [0, u64::MAX - 32, u64::MAX - 1] {
        let end = u64::MAX;
        let middle = start + ((end - start) / 2).max(1);
        for transition in [start + 1, middle, end] {
            let sample = |tick| Ok(if tick < transition { initial } else { target });
            let expected = find_first_transition_tick(start, end, initial, sample);
            let mut probes = Vec::new();
            let actual = find_first_transition_tick_exponential(start, end, initial, |tick| {
                assert!((start + 1..=end).contains(&tick));
                assert!(!probes.contains(&tick));
                probes.push(tick);
                sample(tick)
            });
            assert_eq!(
                expected,
                Ok(Some((transition, MotorStepMove::new(initial, target))))
            );
            assert_eq!(actual, expected);
            assert!(probes.len() <= 128);
        }
    }
}

#[test]
fn rejects_multiple_steps_at_first_change_even_if_later_positions_are_valid() {
    let initial = steps(0, 0);
    for jump in [
        steps(-2, 0),
        steps(0, 2),
        steps(-2, 1),
        steps(i64::MIN, i64::MAX),
    ] {
        let sample = |tick| {
            Ok(match tick {
                11..=13 => initial,
                14 => jump,
                15..=16 => steps(-1, 1),
                _ => unreachable!(),
            })
        };
        let expected = find_first_transition_tick(10, 16, initial, sample);
        assert_eq!(expected, Err(PlannerError::MultipleStepsOnTick));
        assert_eq!(
            find_first_transition_tick_exponential(10, 16, initial, sample),
            expected
        );
    }
}

#[test]
fn propagates_errors_in_the_exponential_and_binary_phases() {
    let initial = steps(0, 0);
    for (failed_tick, error) in [
        (116, PlannerError::InvalidTime),
        (124, PlannerError::NumericalFailure),
    ] {
        let actual = find_first_transition_tick_exponential(100, 180, initial, |tick| {
            if tick == failed_tick {
                return Err(error);
            }
            Ok(if tick < 119 { initial } else { steps(1, 0) })
        });
        assert_eq!(actual, Err(error));
    }
}

#[test]
fn a_nearby_transition_uses_fewer_samples_and_a_stationary_range_reaches_its_end() {
    let initial = steps(0, 0);
    for (transition, exponential_samples) in [(1, 1), (2, 2), (4, 4), (8, 6), (16, 8)] {
        let sample = |tick| {
            Ok(if tick < transition {
                initial
            } else {
                steps(0, -1)
            })
        };
        let mut binary_count = 0;
        let binary = find_first_transition_tick(0, 100_000, initial, |tick| {
            binary_count += 1;
            sample(tick)
        });
        let mut exponential_count = 0;
        let exponential = find_first_transition_tick_exponential(0, 100_000, initial, |tick| {
            exponential_count += 1;
            sample(tick)
        });
        assert_eq!(exponential, binary);
        assert_eq!(exponential_count, exponential_samples);
        assert!(exponential_count < binary_count);
    }

    let mut binary_probes = Vec::new();
    let binary = find_first_transition_tick(0, u64::MAX, initial, |tick| {
        binary_probes.push(tick);
        Ok(initial)
    });
    let mut exponential_probes = Vec::new();
    let exponential = find_first_transition_tick_exponential(0, u64::MAX, initial, |tick| {
        exponential_probes.push(tick);
        Ok(initial)
    });
    assert_eq!(binary, Ok(None));
    assert_eq!(exponential, binary);
    assert_eq!(binary_probes, [u64::MAX]);
    assert_eq!(exponential_probes.len(), 65);
    assert_eq!(exponential_probes.last(), Some(&u64::MAX));
    assert!(exponential_probes.windows(2).all(|pair| pair[0] < pair[1]));
}
