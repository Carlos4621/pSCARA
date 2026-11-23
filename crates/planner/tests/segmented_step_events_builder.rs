use core::cell::Cell;

use kinematics::JointAngles;
use planner::{
    CoordinatedJointPathProfiles, JointAccelerationLimits, JointVelocityLimits, MotorStepMove,
    MotorSteps, PlannerError, StepTimeBase, TimedCoordinatedJointPathProfiles,
    build_segmented_step_events,
};

fn steps(first: i64, second: i64) -> MotorSteps {
    MotorSteps { first, second }
}

fn points<const N: usize>(first_joint: [f64; N]) -> [JointAngles; N] {
    first_joint.map(|angle| JointAngles::new(angle, 0.0).unwrap())
}

fn limits() -> (JointVelocityLimits, JointAccelerationLimits) {
    (
        JointVelocityLimits::new(10.0, 10.0).unwrap(),
        JointAccelerationLimits::new(1.0, 1.0).unwrap(),
    )
}

fn timed_path<'a>(
    points: &'a [JointAngles],
    speeds: &'a [f64],
    (velocity, acceleration): &'a (JointVelocityLimits, JointAccelerationLimits),
) -> TimedCoordinatedJointPathProfiles<'a> {
    let path = CoordinatedJointPathProfiles::new(points, speeds, velocity, acceleration).unwrap();
    TimedCoordinatedJointPathProfiles::new(path).unwrap()
}

fn unexpected_sample(tick: u64) -> Result<MotorSteps, PlannerError> {
    panic!("builder sampled tick {tick}");
}

#[test]
fn uses_the_supplied_initial_position_and_samples_only_after_construction() {
    let initial = steps(i64::MIN, i64::MAX);
    let target = steps(i64::MIN + 1, i64::MAX - 1);

    for last_tick in [1, u64::MAX] {
        let mut boundaries = [17, 23];
        let calls = Cell::new(0);
        let mut events = {
            let points = points([0.0, 0.25]);
            let limits = limits();
            let timed = timed_path(&points, &[0.0; 2], &limits);
            let base = StepTimeBase::new(last_tick).unwrap();

            // The profile lasts one second. Only the buffer and sampler are
            // needed after construction, so the profile inputs can go away.
            build_segmented_step_events(timed, base, &mut boundaries, initial, |tick| {
                assert!((1..=last_tick).contains(&tick));
                calls.set(calls.get() + 1);
                Ok(if tick == last_tick { target } else { initial })
            })
            .unwrap()
        };

        assert_eq!(calls.get(), 0);
        assert_eq!(
            events.next(),
            Some(Ok((last_tick, MotorStepMove::new(initial, target))))
        );
        let calls_after_event = calls.get();
        for _ in 0..3 {
            assert_eq!(events.next(), None);
        }
        assert_eq!(calls.get(), calls_after_event);
        assert_eq!(boundaries, [0, last_tick]);
    }
}

#[test]
fn insufficient_boundary_storage_leaves_the_buffer_available_for_a_retry() {
    let points = points([0.0, 0.25, 0.5]);
    let limits = limits();
    let timed = timed_path(&points, &[0.0; 3], &limits);
    let base = StepTimeBase::new(1).unwrap();
    let initial = steps(3, -5);
    let mut boundaries = [19, 23, 29];

    for capacity in 0..boundaries.len() {
        let result = build_segmented_step_events(
            timed,
            base,
            &mut boundaries[..capacity],
            initial,
            unexpected_sample,
        );
        assert_eq!(result.err(), Some(PlannerError::InsufficientBoundaryBuffer));
        assert_eq!(boundaries, [19, 23, 29]);
    }

    let mut events =
        build_segmented_step_events(timed, base, &mut boundaries, initial, |_| Ok(initial))
            .unwrap();
    assert_eq!(events.next(), None);
    assert_eq!(events.next(), None);
    assert_eq!(boundaries, [0, 1, 2]);
}

#[test]
fn rejects_started_and_exhausted_timed_paths_without_sampling() {
    let points = points([0.0, 0.25, 0.5]);
    let limits = limits();
    let mut timed = timed_path(&points, &[0.0; 3], &limits);
    let base = StepTimeBase::new(1).unwrap();
    let initial = steps(3, -5);
    let mut boundaries = [19, 23, 29];

    assert_eq!(timed.next().unwrap().unwrap().end_time(), Ok(1.0));
    let result =
        build_segmented_step_events(timed, base, &mut boundaries, initial, unexpected_sample);
    assert_eq!(result.err(), Some(PlannerError::InvalidTime));
    assert_eq!(boundaries, [19, 23, 29]);

    assert_eq!(timed.next().unwrap().unwrap().end_time(), Ok(2.0));
    let result =
        build_segmented_step_events(timed, base, &mut boundaries, initial, unexpected_sample);
    assert_eq!(result.err(), Some(PlannerError::InsufficientPoints));
    assert_eq!(boundaries, [19, 23, 29]);
}

#[test]
fn builds_off_grid_final_boundaries_without_accessing_the_sampler() {
    let limits = limits();
    let base = StepTimeBase::new(1).unwrap();

    for (last_point, last_tick) in [(0.25 + 1.0 / 256.0, 2), (0.25 + 9.0 / 16.0, 3)] {
        let points = points([0.0, 0.25, last_point]);
        let timed = timed_path(&points, &[0.0; 3], &limits);
        let mut boundaries = [19, 23, 29];

        let result = build_segmented_step_events(
            timed,
            base,
            &mut boundaries,
            steps(3, -5),
            unexpected_sample,
        );
        assert_eq!(result.err(), None, "last point {last_point}");
        assert_eq!(boundaries, [0, 1, last_tick]);
    }
}

#[test]
fn rejects_tick_overflow_before_accessing_the_sampler() {
    let points = points([0.0, 0.25, 0.5]);
    let limits = limits();
    let timed = timed_path(&points, &[0.0; 3], &limits);
    let base = StepTimeBase::new(u64::MAX).unwrap();
    let mut boundaries = [19, 23, 29];
    let result = build_segmented_step_events(
        timed,
        base,
        &mut boundaries,
        steps(3, -5),
        unexpected_sample,
    );
    assert_eq!(result.err(), Some(PlannerError::NumericalFailure));
    assert_eq!(boundaries, [19, 23, 29]);
}

#[test]
fn returns_sampling_errors_and_multiple_steps_only_during_iteration() {
    let points = points([0.0, 0.25, 0.5]);
    let limits = limits();
    let timed = timed_path(&points, &[0.0; 3], &limits);
    let base = StepTimeBase::new(1).unwrap();
    let initial = steps(7, -4);
    let first = steps(8, -5);

    for (second, error) in [
        (
            Err(PlannerError::NumericalFailure),
            PlannerError::NumericalFailure,
        ),
        (Ok(steps(10, -5)), PlannerError::MultipleStepsOnTick),
    ] {
        let mut boundaries = [19, 23, 29];
        let mut samples = vec![(1, Ok(first)), (2, second)].into_iter();
        let mut events =
            build_segmented_step_events(timed, base, &mut boundaries, initial, move |tick| {
                let (expected_tick, result) = samples.next().expect("unexpected extra sample");
                assert_eq!(tick, expected_tick);
                result
            })
            .unwrap();

        assert_eq!(
            events.next(),
            Some(Ok((1, MotorStepMove::new(initial, first))))
        );
        assert_eq!(events.next(), Some(Err(error)));
        assert_eq!(events.next(), None);
        assert_eq!(events.next(), None);
        assert_eq!(boundaries, [0, 1, 2]);
    }
}
