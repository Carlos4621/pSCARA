use kinematics::JointAngles;
use planner::{
    CoordinatedJointPathProfiles, CoordinatedJointTrajectory, CoordinatedJointTrajectoryCursor,
    JointAccelerationLimits, JointToStepsConverter, JointVelocityLimits, MotorPositionSampler,
    MotorSteps, PlannerError, StepTimeBase, TimedCoordinatedJointPathProfiles,
};

fn angles([first, second]: [f64; 2]) -> JointAngles {
    JointAngles::new(first, second).unwrap()
}

fn steps(first: i64, second: i64) -> MotorSteps {
    MotorSteps { first, second }
}

fn timed_path<'a>(
    points: &'a [JointAngles],
    speeds: &'a [f64],
    velocity: &'a JointVelocityLimits,
    acceleration: &'a JointAccelerationLimits,
) -> TimedCoordinatedJointPathProfiles<'a> {
    let profiles =
        CoordinatedJointPathProfiles::new(points, speeds, velocity, acceleration).unwrap();
    TimedCoordinatedJointPathProfiles::new(profiles).unwrap()
}

fn sampler<'a>(
    timed: TimedCoordinatedJointPathProfiles<'a>,
    tick_rate: u64,
    calibration: [f64; 2],
) -> MotorPositionSampler<'a> {
    let cursor =
        CoordinatedJointTrajectoryCursor::new(CoordinatedJointTrajectory::new(timed).unwrap());
    let converter = JointToStepsConverter::new(calibration[0], calibration[1]).unwrap();
    MotorPositionSampler::new(cursor, converter, StepTimeBase::new(tick_rate).unwrap())
}

#[test]
fn samples_absolute_motor_positions_through_both_segments() {
    let points = [[0.0, 0.0], [1.0, -0.5], [1.0, -1.5]].map(angles);
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let mut sampler = sampler(
        timed_path(&points, &[0.0; 3], &velocity, &acceleration),
        8,
        [8.0, 16.0],
    );

    // Each segment lasts two seconds. The first moves both joints, the second only one.
    for (tick, expected) in [
        (0, steps(0, 0)),
        (4, steps(1, -1)),
        (8, steps(4, -4)),
        (12, steps(7, -7)),
        (16, steps(8, -8)),
        (16, steps(8, -8)),
        (20, steps(8, -10)),
        (24, steps(8, -16)),
        (28, steps(8, -22)),
        (32, steps(8, -24)),
    ] {
        assert_eq!(sampler.sample_at_tick(tick), Ok(expected), "tick={tick}");
    }
    assert_eq!(sampler.sample_at_tick(33), Err(PlannerError::InvalidTime));
}

#[test]
fn a_nonbinary_tick_period_still_samples_the_expected_motion() {
    let points = [[0.0, 0.0], [1.0, -0.5]].map(angles);
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let mut sampler = sampler(
        timed_path(&points, &[0.0; 2], &velocity, &acceleration),
        3,
        [72.0, 144.0],
    );

    // With unit acceleration and 72 steps per radian, t = 1/3 and 2/3
    // give 36*t^2 = 4 and 16 steps on the first axis.
    for (tick, expected) in [(1, 4), (2, 16), (3, 36), (4, 56), (5, 68), (6, 72)] {
        assert_eq!(sampler.sample_at_tick(tick), Ok(steps(expected, -expected)));
    }
}

#[test]
fn rounding_the_end_time_up_does_not_extend_the_trajectory() {
    let points = [[0.0, 0.0], [0.5625, 0.0]].map(angles);
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let timed = timed_path(&points, &[0.0; 2], &velocity, &acceleration);
    let mut reference = timed;
    let end = reference.next().unwrap().unwrap().end_time().unwrap();
    let rounded_end = StepTimeBase::new(1).unwrap().seconds_to_ticks(end).unwrap();
    assert_eq!(end, 1.5);
    assert_eq!(rounded_end, 2);

    let mut sampler = sampler(timed, 1, [16.0, 1.0]);
    assert_eq!(
        sampler.sample_at_tick(rounded_end),
        Err(PlannerError::InvalidTime)
    );
    assert_eq!(sampler.sample_at_tick(1), Ok(steps(7, 0)));
}

#[test]
fn rejected_cursor_times_do_not_advance_the_sampler() {
    let points = [[0.0, 0.0], [1.0, 0.0]].map(angles);
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let mut sampler = sampler(
        timed_path(&points, &[0.0; 2], &velocity, &acceleration),
        8,
        [8.0, 1.0],
    );

    assert_eq!(sampler.sample_at_tick(8), Ok(steps(4, 0)));
    assert_eq!(
        sampler.sample_at_tick(7),
        Err(PlannerError::TimeWentBackwards)
    );
    assert_eq!(
        sampler.sample_at_tick(u64::MAX),
        Err(PlannerError::InvalidTime)
    );
    assert_eq!(sampler.sample_at_tick(8), Ok(steps(4, 0)));
    assert_eq!(sampler.sample_at_tick(12), Ok(steps(7, 0)));
    assert_eq!(sampler.sample_at_tick(16), Ok(steps(8, 0)));
}

#[test]
fn partially_consumed_paths_preserve_the_original_tick_origin() {
    let points = [[0.0, 0.0], [1.0, -0.5], [1.0, -1.5]].map(angles);
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let mut timed = timed_path(&points, &[0.0; 3], &velocity, &acceleration);
    assert_eq!(timed.next().unwrap().unwrap().end_time(), Ok(2.0));
    let mut sampler = sampler(timed, 8, [8.0, 16.0]);

    for tick in [0, 15] {
        assert_eq!(
            sampler.sample_at_tick(tick),
            Err(PlannerError::ProfileInstanceNotFound)
        );
    }
    assert_eq!(sampler.sample_at_tick(16), Ok(steps(8, -8)));
    assert_eq!(
        sampler.sample_at_tick(15),
        Err(PlannerError::TimeWentBackwards)
    );
    assert_eq!(sampler.sample_at_tick(24), Ok(steps(8, -16)));
    assert_eq!(sampler.sample_at_tick(32), Ok(steps(8, -24)));
}

#[test]
fn an_empty_path_has_no_position_at_tick_zero() {
    let points = [[0.0, 0.0], [1.0, 0.0]].map(angles);
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let mut timed = timed_path(&points, &[0.0; 2], &velocity, &acceleration);
    assert!(timed.next().unwrap().is_ok());
    assert!(timed.next().is_none());
    let mut sampler = sampler(timed, 1, [8.0, 8.0]);

    assert_eq!(
        sampler.sample_at_tick(0),
        Err(PlannerError::ProfileInstanceNotFound)
    );
    assert_eq!(sampler.sample_at_tick(1), Err(PlannerError::InvalidTime));
    assert_eq!(
        sampler.sample_at_tick(0),
        Err(PlannerError::ProfileInstanceNotFound)
    );
}

#[test]
fn a_sampling_failure_in_the_cursor_is_forwarded_and_can_be_retried() {
    let points = [[0.0, 0.0], [1.04, 0.0]].map(angles);
    let velocity = JointVelocityLimits::new(1.04 * 8.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(f64::MAX, 1.0).unwrap();
    let mut timed = timed_path(&points, &[0.0; 2], &velocity, &acceleration);
    let mut sampler = sampler(timed, 1_000, [8.0, 1.0]);
    assert_eq!(timed.next().unwrap().unwrap().end_time(), Ok(0.125));

    // Joint acceleration overflows at both endpoints, while cruise is finite.
    assert_eq!(
        sampler.sample_at_tick(0),
        Err(PlannerError::NumericalFailure)
    );
    assert_eq!(
        sampler.sample_at_tick(0),
        Err(PlannerError::NumericalFailure)
    );
    assert_eq!(sampler.sample_at_tick(50), Ok(steps(3, 0)));
    assert_eq!(
        sampler.sample_at_tick(125),
        Err(PlannerError::NumericalFailure)
    );
    assert_eq!(sampler.sample_at_tick(50), Ok(steps(3, 0)));
    assert_eq!(sampler.sample_at_tick(100), Ok(steps(7, 0)));
}

#[test]
fn a_step_conversion_failure_still_records_the_tick_sampled_by_the_cursor() {
    let points = [[0.0, 0.0], [1.0, 0.0], [0.0, 0.0]].map(angles);
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(4.0, 4.0).unwrap();
    let calibration = (1_u64 << 63) as f64;
    let tick_rate = 1_u64 << 54;
    let failing_tick = tick_rate + 1;
    let timebase = StepTimeBase::new(tick_rate).unwrap();
    assert_eq!(timebase.ticks_to_seconds(failing_tick), 1.0);
    assert_eq!(timebase.ticks_to_seconds(tick_rate), 1.0);
    let mut sampler = sampler(
        timed_path(&points, &[0.0; 3], &velocity, &acceleration),
        tick_rate,
        [calibration, 1.0],
    );

    assert_eq!(sampler.sample_at_tick(0), Ok(steps(0, 0)));
    // The cursor samples angle 1, but the resulting 2^63 steps do not fit in i64.
    assert_eq!(
        sampler.sample_at_tick(failing_tick),
        Err(PlannerError::NumericalFailure)
    );
    assert_eq!(
        sampler.sample_at_tick(tick_rate),
        Err(PlannerError::TimeWentBackwards)
    );
    assert_eq!(
        sampler.sample_at_tick(failing_tick),
        Err(PlannerError::NumericalFailure)
    );
    assert_eq!(
        sampler.sample_at_tick(3 * tick_rate / 2),
        Ok(steps(1_i64 << 62, 0))
    );
    assert_eq!(sampler.sample_at_tick(2 * tick_rate), Ok(steps(0, 0)));
}

#[test]
fn the_signed_step_limit_is_checked_for_the_second_axis_as_well() {
    let points = [[0.0, -1.0], [0.0, 0.0], [0.0, 1.0]].map(angles);
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let calibration = (1_u64 << 63) as f64;
    let mut sampler = sampler(
        timed_path(&points, &[0.0; 3], &velocity, &acceleration),
        1,
        [1.0, calibration],
    );

    assert_eq!(sampler.sample_at_tick(0), Ok(steps(0, i64::MIN)));
    assert_eq!(sampler.sample_at_tick(2), Ok(steps(0, 0)));
    assert_eq!(
        sampler.sample_at_tick(4),
        Err(PlannerError::NumericalFailure)
    );
}

#[test]
fn adjacent_ticks_are_ordered_even_when_they_round_to_the_same_seconds() {
    let points = [[0.0, 0.0], [1.0, -0.5]].map(angles);
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let tick_rate = 1_u64 << 54;
    let timebase = StepTimeBase::new(tick_rate).unwrap();
    assert_eq!(timebase.ticks_to_seconds(tick_rate), 1.0);
    assert_eq!(timebase.ticks_to_seconds(tick_rate + 1), 1.0);
    let mut sampler = sampler(
        timed_path(&points, &[0.0; 2], &velocity, &acceleration),
        tick_rate,
        [8.0, 16.0],
    );

    assert_eq!(sampler.sample_at_tick(tick_rate), Ok(steps(4, -4)));
    assert_eq!(sampler.sample_at_tick(tick_rate + 1), Ok(steps(4, -4)));
    assert_eq!(
        sampler.sample_at_tick(tick_rate),
        Err(PlannerError::TimeWentBackwards)
    );
    assert_eq!(
        sampler.sample_at_tick(tick_rate - 4),
        Err(PlannerError::TimeWentBackwards)
    );
    assert_eq!(sampler.sample_at_tick(tick_rate + 1), Ok(steps(4, -4)));
    assert_eq!(sampler.sample_at_tick(2 * tick_rate), Ok(steps(8, -8)));
}

#[test]
fn the_tick_order_uses_the_full_unsigned_range() {
    let points = [[0.0, 0.0], [1.0, -0.5]].map(angles);
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let mut sampler = sampler(
        timed_path(&points, &[0.0; 2], &velocity, &acceleration),
        u64::MAX,
        [8.0, 16.0],
    );

    let signed_boundary = 1_u64 << 63;
    assert_eq!(
        sampler.sample_at_tick(signed_boundary - 1),
        Ok(steps(1, -1))
    );
    assert_eq!(sampler.sample_at_tick(signed_boundary), Ok(steps(1, -1)));
    assert_eq!(
        sampler.sample_at_tick(signed_boundary - 1),
        Err(PlannerError::TimeWentBackwards)
    );
    assert_eq!(sampler.sample_at_tick(u64::MAX), Ok(steps(4, -4)));
    assert_eq!(
        sampler.sample_at_tick(u64::MAX - 1),
        Err(PlannerError::TimeWentBackwards)
    );
    assert_eq!(sampler.sample_at_tick(u64::MAX), Ok(steps(4, -4)));
}
