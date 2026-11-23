use kinematics::JointAngles;

use super::{validate_subtick_boundary, validate_subtick_excursion};
use crate::{
    CoordinatedJointPathProfiles, CoordinatedJointTrajectory, JointAccelerationLimits,
    JointToStepsConverter, JointVelocityLimits, MotorSteps, PlannerError, StepTimeBase,
    TimedCoordinatedJointPathProfiles,
};

fn steps(first: i64, second: i64) -> MotorSteps {
    MotorSteps { first, second }
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
    let velocity = JointVelocityLimits::new(10.0, 10.0)?;
    let acceleration = JointAccelerationLimits::new(1.0, 1.0)?;
    let profiles = CoordinatedJointPathProfiles::new(&points, &[0.0; 3], &velocity, &acceleration)?;
    let mut timed = TimedCoordinatedJointPathProfiles::new(profiles)?;
    let trajectory = CoordinatedJointTrajectory::new(timed)?;
    let boundary_time = timed.next().expect("outbound profile")?.end_time()?;
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
    Ok(())
}

#[test]
fn accepts_a_reversal_with_only_one_quantized_step_inside_the_tick() -> Result<(), PlannerError> {
    let points = [0.0, 9.0 / 64.0, 1.0 / 8.0].map(|angle| JointAngles::new(angle, 0.0).unwrap());
    let velocity = JointVelocityLimits::new(10.0, 10.0)?;
    let acceleration = JointAccelerationLimits::new(1.0, 1.0)?;
    let profiles = CoordinatedJointPathProfiles::new(&points, &[0.0; 3], &velocity, &acceleration)?;
    let mut timed = TimedCoordinatedJointPathProfiles::new(profiles)?;
    let trajectory = CoordinatedJointTrajectory::new(timed)?;
    let boundary_time = timed.next().expect("outbound profile")?.end_time()?;
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
    Ok(())
}
