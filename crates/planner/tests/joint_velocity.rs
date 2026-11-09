use core::f64::consts::{FRAC_PI_2, PI, TAU};

use kinematics::{ElbowConfiguration, JointAngles, Point, Scara};
use planner::{JointTrajectory, JointVelocityLimits, PlannerError};
use trajectory::LineSegment;

const SMALLEST: f64 = f64::from_bits(1);

fn angles(first: f64, second: f64) -> JointAngles {
    JointAngles::new(first, second).expect("test angles must be finite")
}

fn velocity(first: f64, second: f64) -> JointVelocityLimits {
    JointVelocityLimits::new(first, second).expect("test velocities must be valid")
}

fn assert_duration_in_both_directions(
    velocity: &JointVelocityLimits,
    start: JointAngles,
    end: JointAngles,
    expected: f64,
) {
    for (a, b) in [(start, end), (end, start)] {
        assert_eq!(
            velocity.min_duration_between(a, b),
            Ok(expected),
            "velocity {velocity:?}, movement {a:?} -> {b:?}"
        );
    }
}

fn assert_numerical_failure_in_both_directions(
    velocity: &JointVelocityLimits,
    start: JointAngles,
    end: JointAngles,
) {
    for (a, b) in [(start, end), (end, start)] {
        assert_eq!(
            velocity.min_duration_between(a, b),
            Err(PlannerError::NumericalFailure),
            "velocity {velocity:?}, movement {a:?} -> {b:?}"
        );
    }
}

#[test]
fn new_accepts_every_combination_of_strictly_positive_finite_boundary_values() {
    let valid = [SMALLEST, f64::MIN_POSITIVE, 0.5, 1.0, f64::MAX];
    for first in valid {
        for second in valid {
            assert!(
                JointVelocityLimits::new(first, second).is_ok(),
                "velocities ({first}, {second}) should be accepted"
            );
        }
    }
}

#[test]
fn new_rejects_zero_negative_or_nonfinite_velocities_in_either_joint() {
    let invalid = [
        f64::NEG_INFINITY,
        -f64::MAX,
        -1.0,
        -f64::MIN_POSITIVE,
        -SMALLEST,
        -0.0,
        0.0,
        f64::INFINITY,
        f64::NAN,
        -f64::NAN,
    ];
    for bad in invalid {
        for valid in [SMALLEST, f64::MIN_POSITIVE, 1.0, f64::MAX] {
            for (first, second) in [(bad, valid), (valid, bad)] {
                assert_eq!(
                    JointVelocityLimits::new(first, second).unwrap_err(),
                    PlannerError::InvalidJointVelocity
                );
            }
        }
        for other_bad in invalid {
            assert_eq!(
                JointVelocityLimits::new(bad, other_bad).unwrap_err(),
                PlannerError::InvalidJointVelocity
            );
        }
    }
}

#[test]
fn duration_is_set_by_the_slower_joint_including_simultaneous_arrival() {
    // Travel is 4 radians for joint 1 and 6 radians for joint 2.
    let start = angles(-3.0, 7.0);
    let end = angles(1.0, 1.0);
    for (first, second, duration) in [(0.5, 3.0, 8.0), (4.0, 1.0, 6.0), (2.0, 3.0, 2.0)] {
        assert_duration_in_both_directions(&velocity(first, second), start, end, duration);
    }
}

#[test]
fn moving_one_joint_does_not_add_time_for_the_stationary_joint() {
    let velocity = velocity(0.5, 3.0);
    let start = angles(4.0, -8.0);
    assert_duration_in_both_directions(&velocity, start, angles(6.0, -8.0), 4.0);
    assert_duration_in_both_directions(&velocity, start, angles(4.0, 4.0), 4.0);
}

#[test]
fn identical_angles_take_zero_time_with_positive_velocities_even_at_extreme_angles() {
    for value in [
        -f64::MAX,
        -f64::MIN_POSITIVE,
        -SMALLEST,
        -0.0,
        0.0,
        SMALLEST,
        f64::MAX,
    ] {
        let position = angles(value, -value);
        for (first, second) in [(SMALLEST, SMALLEST), (1.0, 2.0), (f64::MAX, f64::MAX)] {
            assert_eq!(
                velocity(first, second).min_duration_between(position, position),
                Ok(0.0)
            );
        }
    }
}

#[test]
fn signed_zero_angles_do_not_change_the_duration() {
    let velocity = velocity(1.0, 2.0);
    for first in [-0.0, 0.0] {
        for second in [-0.0, 0.0] {
            assert_duration_in_both_directions(
                &velocity,
                angles(first, second),
                angles(0.0, -0.0),
                0.0,
            );
            assert_duration_in_both_directions(
                &velocity,
                angles(first, second),
                angles(3.0, -4.0),
                3.0,
            );
        }
    }
}

#[test]
fn duration_preserves_full_turns_and_does_not_choose_the_shortest_wrapped_angle() {
    assert_duration_in_both_directions(
        &velocity(TAU, TAU / 2.0),
        angles(0.0, 0.0),
        angles(3.0 * TAU, -2.0 * TAU),
        4.0,
    );
    assert_duration_in_both_directions(
        &velocity(1.0, 1.0),
        angles(PI - 0.125, 0.0),
        angles(-PI + 0.125, 0.0),
        TAU - 0.25,
    );
}

#[test]
fn duration_is_unchanged_by_joint_translation_or_swapping_joints_and_velocities() {
    for (first_offset, second_offset) in [(-1024.0, 256.0), (0.0, 0.0), (1024.0, -256.0)] {
        let start = angles(first_offset, second_offset);
        let end = angles(first_offset + 4.0, second_offset - 6.0);
        assert_duration_in_both_directions(&velocity(2.0, 1.0), start, end, 6.0);
        assert_duration_in_both_directions(
            &velocity(1.0, 2.0),
            angles(start.second_theta(), start.first_theta()),
            angles(end.second_theta(), end.first_theta()),
            6.0,
        );
    }
}

#[test]
fn doubling_both_velocities_halves_the_duration() {
    for (factor, expected) in [(0.25, 16.0), (0.5, 8.0), (1.0, 4.0), (2.0, 2.0), (4.0, 1.0)] {
        assert_duration_in_both_directions(
            &velocity(2.0 * factor, 4.0 * factor),
            angles(-1.0, 3.0),
            angles(7.0, -5.0),
            expected,
        );
    }
}

#[test]
fn duration_is_scale_invariant_from_subnormal_angles_and_velocities_to_near_maximum() {
    for scale in [
        SMALLEST,
        f64::from_bits(16),
        f64::MIN_POSITIVE,
        1e-300,
        1e-200,
        1.0,
        1e200,
        1e300,
        f64::MAX / 4.0,
    ] {
        assert_duration_in_both_directions(
            &velocity(scale, 2.0 * scale),
            angles(-scale, scale),
            angles(scale, -scale),
            2.0,
        );
    }
}

#[test]
fn duration_can_represent_the_smallest_subnormal_and_largest_finite_time() {
    for duration in [
        SMALLEST,
        2.0 * SMALLEST,
        f64::from_bits(f64::MIN_POSITIVE.to_bits() - 1),
        f64::MIN_POSITIVE,
        f64::from_bits(f64::MAX.to_bits() - 1),
        f64::MAX,
    ] {
        for end in [angles(duration, 0.0), angles(0.0, -duration)] {
            assert_duration_in_both_directions(
                &velocity(1.0, 1.0),
                angles(0.0, 0.0),
                end,
                duration,
            );
        }
    }
}

#[test]
fn positive_velocity_division_overflow_returns_numerical_failure() {
    for (first, second, end) in [
        (SMALLEST, 1.0, angles(1.0, 0.0)),
        (1.0, SMALLEST, angles(0.0, 1.0)),
        (SMALLEST, SMALLEST, angles(f64::MAX, f64::MAX)),
        (SMALLEST, 1.0, angles(1.0, 2.0)),
        (1.0, SMALLEST, angles(2.0, 1.0)),
        (SMALLEST, f64::MAX, angles(1.0, SMALLEST)),
        (f64::MAX, SMALLEST, angles(SMALLEST, 1.0)),
    ] {
        // An infinite joint time cannot be masked by another joint's finite,
        // zero, or underflowed time.
        assert_numerical_failure_in_both_directions(
            &velocity(first, second),
            angles(0.0, 0.0),
            end,
        );
    }
}

#[test]
fn nonzero_movements_whose_duration_rounds_to_zero_return_numerical_failure() {
    for speed in [2.0, f64::MAX] {
        for end in [
            angles(SMALLEST, 0.0),
            angles(0.0, -SMALLEST),
            angles(SMALLEST, -SMALLEST),
        ] {
            assert_numerical_failure_in_both_directions(
                &velocity(speed, speed),
                angles(0.0, 0.0),
                end,
            );
        }
    }
}

#[test]
fn adjacent_velocities_distinguish_the_smallest_positive_duration_from_zero_rounding() {
    let below_two = f64::from_bits(2.0_f64.to_bits() - 1);
    let above_two = f64::from_bits(2.0_f64.to_bits() + 1);
    for (first_moves, end) in [
        (true, angles(SMALLEST, 0.0)),
        (false, angles(0.0, SMALLEST)),
    ] {
        let make_velocity = |speed| {
            if first_moves {
                velocity(speed, 1.0)
            } else {
                velocity(1.0, speed)
            }
        };
        // SMALLEST / 2 is exactly halfway between zero and SMALLEST and
        // rounds to zero. The immediately smaller speed rounds to SMALLEST.
        assert_duration_in_both_directions(
            &make_velocity(below_two),
            angles(0.0, 0.0),
            end,
            SMALLEST,
        );
        for speed in [2.0, above_two] {
            assert_numerical_failure_in_both_directions(
                &make_velocity(speed),
                angles(0.0, 0.0),
                end,
            );
        }
    }
}

#[test]
fn adjacent_velocities_distinguish_the_largest_finite_duration_from_overflow() {
    let below_one = f64::from_bits(1.0_f64.to_bits() - 1);
    let above_one = f64::from_bits(1.0_f64.to_bits() + 1);
    for (first_moves, end) in [
        (true, angles(f64::MAX, 0.0)),
        (false, angles(0.0, f64::MAX)),
    ] {
        let make_velocity = |speed| {
            if first_moves {
                velocity(speed, 1.0)
            } else {
                velocity(1.0, speed)
            }
        };
        assert_numerical_failure_in_both_directions(
            &make_velocity(below_one),
            angles(0.0, 0.0),
            end,
        );
        assert_duration_in_both_directions(&make_velocity(1.0), angles(0.0, 0.0), end, f64::MAX);
        for (start, finish) in [(angles(0.0, 0.0), end), (end, angles(0.0, 0.0))] {
            let duration = make_velocity(above_one)
                .min_duration_between(start, finish)
                .unwrap();
            assert!(duration.is_finite() && duration > 0.0 && duration < f64::MAX);
        }
    }
}

#[test]
fn one_underflowed_joint_time_does_not_reject_another_joints_valid_duration() {
    for (first, second, end) in [
        (f64::MAX, 2.0, angles(SMALLEST, 4.0)),
        (2.0, f64::MAX, angles(4.0, SMALLEST)),
    ] {
        assert_duration_in_both_directions(&velocity(first, second), angles(0.0, 0.0), end, 2.0);
    }
    for (first, second) in [(f64::MAX, 1.0), (1.0, f64::MAX)] {
        assert_duration_in_both_directions(
            &velocity(first, second),
            angles(0.0, 0.0),
            angles(SMALLEST, SMALLEST),
            SMALLEST,
        );
    }
}

#[test]
fn successful_durations_are_finite_and_zero_only_for_stationary_movements() {
    let positions = [
        angles(0.0, -0.0),
        angles(-0.0, 0.0),
        angles(SMALLEST, 0.0),
        angles(0.0, SMALLEST),
        angles(SMALLEST, -SMALLEST),
        angles(1.0, -1.0),
        angles(f64::MAX, f64::MAX),
        angles(-f64::MAX, f64::MAX),
    ];
    for first in [SMALLEST, f64::MIN_POSITIVE, 1.0, f64::MAX] {
        for second in [SMALLEST, f64::MIN_POSITIVE, 1.0, f64::MAX] {
            let velocities = velocity(first, second);
            for start in positions {
                for end in positions {
                    let stationary = start.first_theta() == end.first_theta()
                        && start.second_theta() == end.second_theta();
                    match velocities.min_duration_between(start, end) {
                        Ok(duration) => {
                            assert!(duration.is_finite());
                            if stationary {
                                assert_eq!(duration, 0.0);
                            } else {
                                assert!(duration > 0.0);
                            }
                        }
                        Err(error) => {
                            assert!(!stationary);
                            assert!(matches!(
                                error,
                                PlannerError::AngleNonFinite | PlannerError::NumericalFailure
                            ));
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn overflowing_angle_differences_return_angle_nonfinite_for_either_joint() {
    for (start, end) in [
        (angles(-f64::MAX, 0.0), angles(f64::MAX, 0.0)),
        (angles(0.0, -f64::MAX), angles(0.0, f64::MAX)),
        (angles(-f64::MAX, f64::MAX), angles(f64::MAX, -f64::MAX)),
    ] {
        for velocities in [
            velocity(SMALLEST, SMALLEST),
            velocity(1.0, 1.0),
            velocity(f64::MAX, f64::MAX),
        ] {
            for (a, b) in [(start, end), (end, start)] {
                assert_eq!(
                    velocities.min_duration_between(a, b),
                    Err(PlannerError::AngleNonFinite)
                );
            }
        }
    }
}

#[test]
fn duration_satisfies_both_speed_limits_and_is_the_smallest_feasible_representable_time() {
    // Binary speeds and small integer displacements make these comparisons
    // exact. The preceding representable time must violate at least one limit.
    for (first, second) in [(0.5, 2.0), (2.0, 0.5), (4.0, 4.0)] {
        let velocities = velocity(first, second);
        for dx in -8..=8 {
            for dy in -8..=8 {
                let duration = velocities
                    .min_duration_between(
                        angles(3.0, -5.0),
                        angles(3.0 + dx as f64, -5.0 + dy as f64),
                    )
                    .unwrap();
                let first_distance = (dx as f64).abs();
                let second_distance = (dy as f64).abs();
                assert!(duration.is_finite() && duration >= 0.0);
                assert!(first_distance <= first * duration);
                assert!(second_distance <= second * duration);
                if duration > 0.0 {
                    let previous_time = f64::from_bits(duration.to_bits() - 1);
                    assert!(
                        first_distance > first * previous_time
                            || second_distance > second * previous_time,
                        "duration {duration} is not minimal for ({dx}, {dy}) at ({first}, {second})"
                    );
                }
            }
        }
    }
}

#[test]
fn routing_via_an_intermediate_position_cannot_be_faster_than_the_direct_move() {
    let velocities = velocity(2.0, 4.0);
    let positions: Vec<_> = [-4.0, 0.0, 4.0]
        .into_iter()
        .flat_map(|first| [-4.0, 0.0, 4.0].map(|second| angles(first, second)))
        .collect();
    for start in &positions {
        for middle in &positions {
            for end in &positions {
                let direct = velocities.min_duration_between(*start, *end).unwrap();
                let via = velocities.min_duration_between(*start, *middle).unwrap()
                    + velocities.min_duration_between(*middle, *end).unwrap();
                assert!(direct <= via);
            }
        }
    }
}

#[test]
fn duration_works_with_the_unwrapped_angles_of_a_validated_trajectory() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let path = LineSegment {
        start: Point::new(160.0, 0.0).unwrap(),
        end: Point::new(100.0, 60.0).unwrap(),
    };
    for configuration in [ElbowConfiguration::Positive, ElbowConfiguration::Negative] {
        let samples: Vec<_> = JointTrajectory::new(&robot, &path, f64::MAX, configuration)
            .unwrap()
            .into_validated()
            .unwrap()
            .map(Result::unwrap)
            .collect();
        assert_eq!(samples.len(), 2);
        // The second joint moves pi/2 in either elbow configuration and is
        // slower than the first joint with these velocity limits.
        for (velocities, expected) in [
            (velocity(10.0, 1.0), FRAC_PI_2),
            (velocity(20.0, 2.0), PI / 4.0),
        ] {
            let duration = velocities
                .min_duration_between(samples[0], samples[1])
                .unwrap();
            assert!((duration - expected).abs() <= 1e-12);
        }
    }
}
