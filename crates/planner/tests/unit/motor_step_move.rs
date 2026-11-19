use kinematics::JointAngles;

use super::{AxisStepMove, MotorStepMove, MotorSteps, StepDirection};
use crate::JointToStepsConverter;

use StepDirection::{Negative, Positive};

fn steps(first: i64, second: i64) -> MotorSteps {
    MotorSteps { first, second }
}

fn axis(direction: StepDirection, pulses: u64) -> AxisStepMove {
    AxisStepMove { direction, pulses }
}

#[test]
fn each_motor_uses_the_target_for_its_own_axis() {
    let movement = MotorStepMove::new(steps(10, -10), steps(13, -4));

    assert_eq!(movement.second, axis(Positive, 6));
    assert_eq!(movement.first, axis(Positive, 3));
}

#[test]
fn moving_one_motor_does_not_emit_pulses_for_the_other() {
    let origin = steps(5, -7);
    for (target, first, second) in [
        (steps(8, -7), axis(Positive, 3), axis(Positive, 0)),
        (steps(5, -12), axis(Positive, 0), axis(Negative, 5)),
    ] {
        let movement = MotorStepMove::new(origin, target);
        assert_eq!(movement.first, first, "target={target:?}");
        assert_eq!(movement.second, second, "target={target:?}");
    }
}

#[test]
fn an_unchanged_position_emits_zero_pulses_on_both_axes() {
    let expected = MotorStepMove {
        first: axis(Positive, 0),
        second: axis(Positive, 0),
    };
    for position in [steps(0, 0), steps(4, -8), steps(i64::MIN, i64::MAX)] {
        assert_eq!(
            MotorStepMove::new(position, position),
            expected,
            "position={position:?}"
        );
    }
}

#[test]
fn a_common_target_handles_each_combination_of_axis_directions() {
    let target = steps(2, 2);
    let cases = [
        (1, axis(Positive, 1)),
        (2, axis(Positive, 0)),
        (5, axis(Negative, 3)),
    ];
    for (first, first_move) in cases {
        for (second, second_move) in cases {
            assert_eq!(
                MotorStepMove::new(steps(first, second), target),
                MotorStepMove {
                    first: first_move,
                    second: second_move
                }
            );
        }
    }
}

#[test]
fn the_full_i64_separation_fits_in_u64_in_both_directions_on_either_axis() {
    for (origin, target, direction) in [
        (i64::MIN, i64::MAX, Positive),
        (i64::MAX, i64::MIN, Negative),
    ] {
        let first_moves = MotorStepMove::new(steps(origin, target), steps(target, target));
        assert_eq!(first_moves.first, axis(direction, u64::MAX));
        assert_eq!(first_moves.second, axis(Positive, 0));

        let second_moves = MotorStepMove::new(steps(target, origin), steps(target, target));
        assert_eq!(second_moves.first, axis(Positive, 0));
        assert_eq!(second_moves.second, axis(direction, u64::MAX));
    }
}

#[test]
fn constructor_matches_a_wide_signed_delta_reference_on_both_axes() {
    fn expected(origin: i64, target: i64) -> AxisStepMove {
        let delta = i128::from(target) - i128::from(origin);
        let direction = if delta < 0 { Negative } else { Positive };
        axis(direction, u64::try_from(delta.unsigned_abs()).unwrap())
    }

    let positions = [i64::MIN, i64::MIN + 1, -1, 0, 1, i64::MAX - 1, i64::MAX];
    for from_first in positions {
        for from_second in positions {
            for to_first in positions {
                for to_second in positions {
                    let origin = steps(from_first, from_second);
                    let target = steps(to_first, to_second);
                    assert_eq!(
                        MotorStepMove::new(origin, target),
                        MotorStepMove {
                            first: expected(from_first, to_first),
                            second: expected(from_second, to_second),
                        },
                        "origin={origin:?}, target={target:?}"
                    );
                }
            }
        }
    }
}

#[test]
fn converted_joint_positions_produce_the_expected_motor_displacements() {
    let converter = JointToStepsConverter::new(4.0, 8.0).unwrap();
    let origin = converter
        .convert(JointAngles::new(1.0, -0.5).unwrap())
        .unwrap();
    let target = converter
        .convert(JointAngles::new(1.75, -2.0).unwrap())
        .unwrap();
    assert_eq!(origin, steps(4, -4));
    assert_eq!(target, steps(7, -16));

    let movement = MotorStepMove::new(origin, target);
    assert_eq!(
        movement,
        MotorStepMove {
            first: axis(Positive, 3),
            second: axis(Negative, 12)
        }
    );
}
