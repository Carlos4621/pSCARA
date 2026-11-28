use super::SegmentedTrajectoryStepEventIterator;
use crate::{MotorSteps, PlannerError};

const INITIAL_POSITION: MotorSteps = MotorSteps {
    first: i64::MIN,
    second: i64::MAX,
};

fn unexpected_sample(tick: u64) -> Result<MotorSteps, PlannerError> {
    panic!("constructor sampled tick {tick}");
}

#[test]
fn initializes_at_the_first_boundary_of_the_borrowed_slice() {
    let ticks = [0, 7, 8, 42, u64::MAX, 0];
    let boundaries = &ticks[1..5];

    let iterator =
        SegmentedTrajectoryStepEventIterator::new(boundaries, INITIAL_POSITION, unexpected_sample)
            .unwrap();

    assert!(core::ptr::eq(iterator.boundaries, boundaries));
    assert_eq!(iterator.segment_index, 0);
    assert_eq!(iterator.current_tick, 7);
    assert_eq!(iterator.current_position, INITIAL_POSITION);
    assert!(!iterator.finished);
}

#[test]
fn accepts_single_segments_at_tick_range_limits() {
    for boundaries in [[0, 1], [0, u64::MAX], [u64::MAX - 1, u64::MAX]] {
        let iterator = SegmentedTrajectoryStepEventIterator::new(
            &boundaries,
            INITIAL_POSITION,
            unexpected_sample,
        )
        .unwrap();

        assert_eq!(iterator.boundaries, &boundaries);
        assert_eq!(iterator.current_tick, boundaries[0]);
        assert!(!iterator.finished);
    }
}

#[test]
fn rejects_slices_with_fewer_than_two_boundaries() {
    for boundaries in [&[][..], &[0][..], &[42][..], &[u64::MAX][..]] {
        let result = SegmentedTrajectoryStepEventIterator::new(
            boundaries,
            INITIAL_POSITION,
            unexpected_sample,
        );

        assert_eq!(
            result.err(),
            Some(PlannerError::BoundariesWithoutFrontiers),
            "{boundaries:?}"
        );
    }
}

#[test]
fn rejects_duplicate_boundaries_at_any_segment() {
    for boundaries in [
        &[0, 0][..],
        &[u64::MAX, u64::MAX][..],
        &[7, 7, 7][..],
        &[0, 0, 1][..],
        &[0, 1, 1, 2][..],
        &[0, u64::MAX, u64::MAX][..],
    ] {
        let result = SegmentedTrajectoryStepEventIterator::new(
            boundaries,
            INITIAL_POSITION,
            unexpected_sample,
        );

        assert_eq!(
            result.err(),
            Some(PlannerError::InvalidTicks),
            "{boundaries:?}"
        );
    }
}

#[test]
fn rejects_decreasing_boundaries_at_any_segment() {
    for boundaries in [
        &[u64::MAX, 0][..],
        &[3, 2, 5][..],
        &[0, 4, 3, 5][..],
        &[0, u64::MAX, u64::MAX - 1][..],
        &[2, 2, 1][..],
        &[2, 1, 1][..],
    ] {
        let result = SegmentedTrajectoryStepEventIterator::new(
            boundaries,
            INITIAL_POSITION,
            unexpected_sample,
        );

        assert_eq!(
            result.err(),
            Some(PlannerError::InvalidTicks),
            "{boundaries:?}"
        );
    }
}

#[test]
fn retains_the_sampler_without_consuming_its_state() {
    let first_sample = MotorSteps {
        first: i64::MIN + 1,
        second: i64::MAX - 1,
    };
    let mut samples = [(16, Ok(first_sample)), (17, Err(PlannerError::InvalidTime))].into_iter();
    let mut iterator =
        SegmentedTrajectoryStepEventIterator::new(&[15, 16, 17], INITIAL_POSITION, move |tick| {
            let (expected_tick, result) = samples.next().expect("sampler called too many times");
            assert_eq!(tick, expected_tick);
            result
        })
        .unwrap();

    assert_eq!((iterator.transition_search)(16), Ok(first_sample));
    assert_eq!(
        (iterator.transition_search)(17),
        Err(PlannerError::InvalidTime)
    );
}
