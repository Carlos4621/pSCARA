use kinematics::JointAngles;

use super::{validate_subtick_boundaries, validate_subtick_boundary, validate_subtick_excursion};
use crate::{
    CoordinatedJointPathProfiles, CoordinatedJointTrajectory, JointAccelerationLimits,
    JointToStepsConverter, JointVelocityLimits, MotorSteps, PlannerError, StepTimeBase,
    TimedCoordinatedJointPathProfiles,
};

fn steps(first: i64, second: i64) -> MotorSteps {
    MotorSteps { first, second }
}

fn limits() -> Result<(JointVelocityLimits, JointAccelerationLimits), PlannerError> {
    Ok((
        JointVelocityLimits::new(10.0, 10.0)?,
        JointAccelerationLimits::new(1.0, 1.0)?,
    ))
}

fn timed_trajectory<'a>(
    points: &'a [JointAngles],
    speeds: &'a [f64],
    (velocity, acceleration): &'a (JointVelocityLimits, JointAccelerationLimits),
) -> Result<
    (
        TimedCoordinatedJointPathProfiles<'a>,
        CoordinatedJointTrajectory<'a>,
    ),
    PlannerError,
> {
    let profiles = CoordinatedJointPathProfiles::new(points, speeds, velocity, acceleration)?;
    let timed = TimedCoordinatedJointPathProfiles::new(profiles)?;
    Ok((timed, CoordinatedJointTrajectory::new(timed)?))
}

#[test]
fn validates_the_total_travel_on_each_motor_in_both_directions() {
    let invalid = Err(PlannerError::SubtickMotionNotRepresentable);
    for (before, at_boundary, after, expected) in [
        (0, 1, 0, invalid),
        (0, 1, 1, Ok(())),
        (0, 0, 1, Ok(())),
        (0, 2, 0, invalid),
        (0, 1, 2, invalid),
        (0, 0, 0, Ok(())),
    ] {
        for sign in [-1, 1] {
            let (before, at_boundary, after) = (sign * before, sign * at_boundary, sign * after);
            assert_eq!(
                validate_subtick_excursion(
                    steps(before, 0),
                    steps(at_boundary, 0),
                    steps(after, 0)
                ),
                expected,
                "motor 1: {before} -> {at_boundary} -> {after}"
            );
            assert_eq!(
                validate_subtick_excursion(
                    steps(0, before),
                    steps(0, at_boundary),
                    steps(0, after)
                ),
                expected,
                "motor 2: {before} -> {at_boundary} -> {after}"
            );
        }
    }
}

#[test]
fn allows_one_step_per_motor_even_when_both_motors_move() {
    assert_eq!(
        validate_subtick_excursion(steps(7, -4), steps(8, -4), steps(8, -5)),
        Ok(())
    );
    assert_eq!(
        validate_subtick_excursion(
            steps(i64::MIN, i64::MAX),
            steps(i64::MIN, i64::MAX),
            steps(i64::MIN + 1, i64::MAX - 1)
        ),
        Ok(())
    );
}

#[test]
fn rejects_travel_that_would_overflow_a_u64_sum() {
    let invalid = Err(PlannerError::SubtickMotionNotRepresentable);
    for (before, at_boundary, after) in [
        (i64::MIN, i64::MAX, i64::MIN),
        (i64::MIN, i64::MAX, i64::MAX - 1),
        (i64::MAX, i64::MIN, i64::MIN + 1),
    ] {
        assert_eq!(
            validate_subtick_excursion(steps(before, 0), steps(at_boundary, 0), steps(after, 0)),
            invalid
        );
        assert_eq!(
            validate_subtick_excursion(steps(0, before), steps(0, at_boundary), steps(0, after)),
            invalid
        );
    }
}

#[test]
fn rejects_a_forward_and_backward_step_inside_one_tick() -> Result<(), PlannerError> {
    let points = [0.0, 1.0 / 16.0, 0.0].map(|angle| JointAngles::new(angle, 0.0).unwrap());
    let limits = limits()?;
    let (timed, trajectory) = timed_trajectory(&points, &[0.0; 3], &limits)?;
    let mut profiles = timed;
    let boundary_time = profiles.next().expect("outbound profile")?.end_time()?;
    let converter = JointToStepsConverter::new(16.0, 16.0)?;
    let timebase = StepTimeBase::new(1)?;
    let position_at = |seconds| converter.convert(trajectory.position_at(seconds)?);

    assert_eq!(boundary_time, 0.5);
    assert_eq!(position_at(0.0)?, steps(0, 0));
    assert_eq!(position_at(boundary_time)?, steps(1, 0));
    assert_eq!(position_at(1.0)?, steps(0, 0));
    assert_eq!(
        validate_subtick_boundary(boundary_time, &trajectory, &converter, timebase),
        Err(PlannerError::SubtickMotionNotRepresentable)
    );
    assert_eq!(
        validate_subtick_boundaries(timed, &trajectory, &converter, timebase),
        Err(PlannerError::SubtickMotionNotRepresentable)
    );
    Ok(())
}

#[test]
fn accepts_a_reversal_with_only_one_quantized_step_inside_the_tick() -> Result<(), PlannerError> {
    let points = [0.0, 9.0 / 64.0, 1.0 / 8.0].map(|angle| JointAngles::new(angle, 0.0).unwrap());
    let limits = limits()?;
    let (timed, trajectory) = timed_trajectory(&points, &[0.0; 3], &limits)?;
    let mut profiles = timed;
    let boundary_time = profiles.next().expect("outbound profile")?.end_time()?;
    let converter = JointToStepsConverter::new(8.0, 8.0)?;
    let timebase = StepTimeBase::new(1)?;
    let position_at = |seconds| converter.convert(trajectory.position_at(seconds)?);

    assert_eq!(boundary_time, 0.75);
    assert_eq!(position_at(0.0)?, steps(0, 0));
    assert_eq!(position_at(boundary_time)?, steps(1, 0));
    assert_eq!(position_at(1.0)?, steps(1, 0));
    assert_eq!(
        validate_subtick_boundary(boundary_time, &trajectory, &converter, timebase),
        Ok(())
    );
    assert_eq!(
        validate_subtick_boundaries(timed, &trajectory, &converter, timebase),
        Ok(())
    );
    Ok(())
}

#[test]
fn counts_the_upper_tick_only_after_all_boundaries_in_the_interval() -> Result<(), PlannerError> {
    let points = [0.0, 1.0 / 64.0, 2.0 / 64.0, 6.0 / 64.0]
        .map(|angle| JointAngles::new(angle, -angle).unwrap());
    let limits = limits()?;
    let (timed, trajectory) = timed_trajectory(&points, &[0.0; 4], &limits)?;
    let converter = JointToStepsConverter::new(8.0, 8.0)?;
    let timebase = StepTimeBase::new(1)?;
    let position_at = |seconds| converter.convert(trajectory.position_at(seconds)?);

    for (time, expected) in [
        (0.0, steps(0, 0)),
        (0.25, steps(0, 0)),
        (0.5, steps(0, 0)),
        (1.0, steps(1, -1)),
    ] {
        assert_eq!(position_at(time)?, expected, "at {time} s");
    }
    assert_eq!(
        validate_subtick_boundaries(timed, &trajectory, &converter, timebase),
        Ok(())
    );
    Ok(())
}

#[test]
fn rejects_five_steps_in_one_tick_even_when_each_boundary_is_valid() -> Result<(), PlannerError> {
    let oscillating = [0.0, 1.0 / 256.0, 0.0, 1.0 / 256.0, 0.0, 1.0 / 256.0];
    // The other axis sets the last segment's duration to 0.5 s and stays at step zero.
    let other = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0 / 16.0];
    let limits = limits()?;
    let timebase = StepTimeBase::new(1)?;

    for (first, second, scales, hit) in [
        (oscillating, other, [256.0, 1.0], steps(1, 0)),
        (other, oscillating, [1.0, 256.0], steps(0, 1)),
    ] {
        let points: [JointAngles; 6] =
            core::array::from_fn(|index| JointAngles::new(first[index], second[index]).unwrap());
        let (timed, trajectory) = timed_trajectory(&points, &[0.0; 6], &limits)?;
        let converter = JointToStepsConverter::new(scales[0], scales[1])?;
        let position_at = |seconds| converter.convert(trajectory.position_at(seconds)?);
        let zero = steps(0, 0);

        assert_eq!(position_at(0.0)?, zero);
        assert_eq!(position_at(1.0)?, hit);
        for (profile, (time, expected)) in
            timed.zip([(0.125, hit), (0.25, zero), (0.375, hit), (0.5, zero)])
        {
            let boundary_time = profile?.end_time()?;
            assert_eq!(boundary_time, time);
            assert_eq!(position_at(boundary_time)?, expected);
            assert_eq!(
                validate_subtick_boundary(boundary_time, &trajectory, &converter, timebase),
                Ok(())
            );
        }
        assert_eq!(
            validate_subtick_boundaries(timed, &trajectory, &converter, timebase),
            Err(PlannerError::SubtickMotionNotRepresentable)
        );
    }
    Ok(())
}

#[test]
fn skips_aligned_internal_boundaries_and_the_final_endpoint() -> Result<(), PlannerError> {
    let limits = limits()?;
    let converter = JointToStepsConverter::new(4.0, 4.0)?;
    let timebase = StepTimeBase::new(1)?;

    for (last_point, end_time) in [(0.0, 2.0), (5.0 / 16.0, 1.5)] {
        let points = [0.0, 0.25, last_point].map(|angle| JointAngles::new(angle, 0.0).unwrap());
        let (timed, trajectory) = timed_trajectory(&points, &[0.0; 3], &limits)?;
        let mut iterator = timed;
        assert_eq!(iterator.next().expect("outbound profile")?.end_time()?, 1.0);
        assert_eq!(
            iterator.next().expect("last profile")?.end_time()?,
            end_time
        );
        assert_eq!(
            validate_subtick_boundaries(timed, &trajectory, &converter, timebase),
            Ok(())
        );
    }
    Ok(())
}

#[test]
fn resets_the_travel_when_the_next_clock_interval_begins() -> Result<(), PlannerError> {
    let points =
        [0.0, 1.0 / 16.0, 5.0 / 16.0, 3.0 / 8.0].map(|angle| JointAngles::new(angle, 0.0).unwrap());
    let limits = limits()?;
    let (timed, trajectory) = timed_trajectory(&points, &[0.0; 4], &limits)?;
    let converter = JointToStepsConverter::new(4.0, 4.0)?;
    let timebase = StepTimeBase::new(1)?;
    let position_at = |seconds| converter.convert(trajectory.position_at(seconds)?);

    for (time, expected) in [
        (0.0, steps(0, 0)),
        (0.5, steps(0, 0)),
        (1.0, steps(1, 0)),
        (1.5, steps(1, 0)),
        (2.0, steps(2, 0)),
    ] {
        assert_eq!(position_at(time)?, expected, "at {time} s");
    }
    assert_eq!(
        validate_subtick_boundaries(timed, &trajectory, &converter, timebase),
        Ok(())
    );
    Ok(())
}
