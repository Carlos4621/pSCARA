use super::validate_subtick_excursion;
use crate::{MotorSteps, PlannerError};

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
