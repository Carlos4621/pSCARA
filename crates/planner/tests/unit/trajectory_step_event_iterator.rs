use crate::{MotorSteps, PlannerError, TrajectoryStepEventIterator};

const INITIAL_POSITION: MotorSteps = MotorSteps {
    first: i64::MIN,
    second: i64::MAX,
};

fn unexpected_sample(tick: u64) -> Result<MotorSteps, PlannerError> {
    panic!("constructor sampled tick {tick}");
}

#[test]
fn initializes_nonempty_intervals_at_the_given_tick_and_position() {
    for (start, end) in [(0, 1), (7, 12), (u64::MAX - 1, u64::MAX), (0, u64::MAX)] {
        let iterator =
            TrajectoryStepEventIterator::new(start, end, INITIAL_POSITION, unexpected_sample)
                .unwrap();

        assert_eq!(iterator.current_tick, start);
        assert_eq!(iterator.end_tick, end);
        assert_eq!(iterator.current_position, INITIAL_POSITION);
        assert!(!iterator.finished);
    }
}

#[test]
fn marks_equal_start_and_end_ticks_as_finished() {
    for tick in [0, 42, u64::MAX] {
        let iterator =
            TrajectoryStepEventIterator::new(tick, tick, INITIAL_POSITION, unexpected_sample)
                .unwrap();

        assert_eq!(iterator.current_tick, tick);
        assert_eq!(iterator.end_tick, tick);
        assert_eq!(iterator.current_position, INITIAL_POSITION);
        assert!(iterator.finished);
    }
}

#[test]
fn rejects_reversed_intervals_without_sampling() {
    for (start, end) in [(1, 0), (12, 7), (u64::MAX, 0)] {
        let result =
            TrajectoryStepEventIterator::new(start, end, INITIAL_POSITION, unexpected_sample);

        assert_eq!(result.err(), Some(PlannerError::EndBeforeStart));
    }
}

#[test]
fn retains_the_supplied_stateful_sampler_without_consuming_a_sample() {
    let target = MotorSteps {
        first: i64::MIN + 1,
        second: i64::MAX,
    };
    let mut samples = [Ok(target), Err(PlannerError::InvalidTime)].into_iter();
    let mut iterator = TrajectoryStepEventIterator::new(15, 16, INITIAL_POSITION, |tick| {
        assert_eq!(tick, 16);
        samples.next().expect("sampler called too many times")
    })
    .unwrap();

    assert_eq!((iterator.position_at_tick)(16), Ok(target));
    assert_eq!(
        (iterator.position_at_tick)(16),
        Err(PlannerError::InvalidTime)
    );
}
