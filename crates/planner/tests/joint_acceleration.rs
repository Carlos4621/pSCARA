use core::f64::consts::{FRAC_PI_2, TAU};

use kinematics::{ElbowConfiguration, JointAngles, Point, Scara};
use planner::{JointAccelerationLimits, JointTrajectory, JointVelocityLimits, PlannerError};
use trajectory::LineSegment;

const SMALLEST: f64 = f64::from_bits(1);

fn angles(first: f64, second: f64) -> JointAngles {
    JointAngles::new(first, second).unwrap()
}

fn acceleration(first: f64, second: f64) -> JointAccelerationLimits {
    JointAccelerationLimits::new(first, second).unwrap()
}

fn velocity(first: f64, second: f64) -> JointVelocityLimits {
    JointVelocityLimits::new(first, second).unwrap()
}

fn assert_close(actual: f64, expected: f64) {
    assert!(actual.is_finite());
    if expected == 0.0 {
        assert_eq!(actual, 0.0);
    } else {
        assert!(
            (actual - expected).abs() <= expected.abs() * 8.0 * f64::EPSILON,
            "expected {expected:e}, got {actual:e}"
        );
    }
}

// Exercise the public API in both directions and with the joint roles reversed.
fn assert_duration(
    start: JointAngles,
    end: JointAngles,
    speeds: [f64; 2],
    accelerations: [f64; 2],
    expected: f64,
) {
    for swap in [false, true] {
        let (start, end, speeds, accelerations) = if swap {
            (
                angles(start.second_theta(), start.first_theta()),
                angles(end.second_theta(), end.first_theta()),
                [speeds[1], speeds[0]],
                [accelerations[1], accelerations[0]],
            )
        } else {
            (start, end, speeds, accelerations)
        };
        let limits = acceleration(accelerations[0], accelerations[1]);
        let speeds = velocity(speeds[0], speeds[1]);
        for (from, to) in [(start, end), (end, start)] {
            let duration = limits.min_duration_between(from, to, &speeds).unwrap();
            assert_close(duration, expected);
        }
    }
}

fn power_of_two(exponent: i32) -> f64 {
    assert!((-1022..=1023).contains(&exponent));
    f64::from_bits(((exponent + 1023) as u64) << 52)
}

#[test]
fn new_accepts_all_positive_finite_boundaries_for_each_joint() {
    let valid = [SMALLEST, f64::MIN_POSITIVE, 0.5, 1.0, f64::MAX];
    for first in valid {
        for second in valid {
            assert!(JointAccelerationLimits::new(first, second).is_ok());
        }
    }
}

#[test]
fn new_rejects_nonpositive_and_nonfinite_acceleration_in_either_joint() {
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
        for good in [SMALLEST, f64::MIN_POSITIVE, 1.0, f64::MAX] {
            for (first, second) in [(bad, good), (good, bad)] {
                assert_eq!(
                    JointAccelerationLimits::new(first, second).unwrap_err(),
                    PlannerError::InvalidJointAcceleration
                );
            }
        }
        for other in invalid {
            assert_eq!(
                JointAccelerationLimits::new(bad, other).unwrap_err(),
                PlannerError::InvalidJointAcceleration
            );
        }
    }
}

#[test]
fn triangular_duration_includes_acceleration_and_deceleration_from_rest() {
    // With a=4 and ramp time=0.5, the peak speed is 2 and the
    // two triangular velocity areas sum to one radian. Total time is 1.
    for distance in [-1.0, 1.0] {
        assert_duration(
            angles(0.0, 0.0),
            angles(distance, 0.0),
            [8.0, 1.0],
            [4.0, 1.0],
            1.0,
        );
    }
}

#[test]
fn trapezoidal_duration_includes_both_ramps_and_the_cruise_phase() {
    // a=2, v=4: two 2-second ramps cover 8 radians together;
    // a 3-second cruise covers another 12. Total time is 7.
    assert_duration(
        angles(-3.0, 5.0),
        angles(17.0, 5.0),
        [4.0, 1.0],
        [2.0, 1.0],
        7.0,
    );
}

#[test]
fn constructed_velocity_profiles_recover_their_analytical_duration() {
    // Reference distances come from integrating velocity areas, rather than
    // duplicating the implementation's duration formula.
    for a in [0.25, 1.0, 4.0] {
        for ramp_time in [0.25, 1.0, 4.0] {
            let peak_speed = a * ramp_time;
            let ramp_distance = peak_speed * ramp_time;
            for cruise_time in [0.0, 0.5, 2.0, 8.0] {
                let distance = ramp_distance + peak_speed * cruise_time;
                let speed_limit = if cruise_time == 0.0 {
                    2.0 * peak_speed
                } else {
                    peak_speed
                };
                assert_duration(
                    angles(0.0, 0.0),
                    angles(distance, 0.0),
                    [speed_limit, 1.0],
                    [a, 1.0],
                    2.0 * ramp_time + cruise_time,
                );
            }
        }
    }
}

#[test]
fn critical_distance_and_its_adjacent_floats_are_continuous() {
    let critical: f64 = 4.0; // v=2, a=1
    for distance in [
        f64::from_bits(critical.to_bits() - 1),
        critical,
        f64::from_bits(critical.to_bits() + 1),
    ] {
        assert_duration(
            angles(0.0, 0.0),
            angles(distance, 0.0),
            [2.0, 1.0],
            [1.0, 1.0],
            4.0,
        );
    }
}

#[test]
fn slower_joint_controls_the_result_including_equal_and_mixed_profiles() {
    for (distance, speeds, accelerations, expected) in [
        ([1.0, 4.0], [8.0, 8.0], [1.0, 1.0], 4.0),
        ([4.0, 1.0], [8.0, 8.0], [1.0, 1.0], 4.0),
        ([1.0, 4.0], [8.0, 8.0], [1.0, 4.0], 2.0),
        ([1.0, 8.0], [8.0, 2.0], [1.0, 1.0], 6.0),
        ([16.0, 8.0], [8.0, 2.0], [1.0, 1.0], 8.0),
    ] {
        assert_duration(
            angles(0.0, 0.0),
            angles(distance[0], -distance[1]),
            speeds,
            accelerations,
            expected,
        );
    }
}

#[test]
fn stationary_positions_and_signed_zero_take_zero_time_at_extreme_angles() {
    for theta in [-f64::MAX, -SMALLEST, -0.0, 0.0, SMALLEST, f64::MAX] {
        assert_duration(
            angles(theta, -theta),
            angles(theta, -theta),
            [1.0, f64::MAX],
            [SMALLEST, f64::MAX],
            0.0,
        );
    }
    assert_duration(
        angles(-0.0, 0.0),
        angles(0.0, -0.0),
        [1.0, 2.0],
        [3.0, 4.0],
        0.0,
    );
}

#[test]
fn duration_preserves_full_turns_without_wrapping_angles() {
    assert_duration(
        angles(0.0, 0.0),
        angles(4.0 * TAU, -TAU),
        [TAU, TAU],
        [TAU, TAU],
        5.0,
    );
}

#[test]
fn duration_is_unchanged_by_translation() {
    for offset in [-1024.0, 0.0, 1024.0] {
        assert_duration(
            angles(offset, -offset),
            angles(offset + 20.0, -offset + 1.0),
            [4.0, 8.0],
            [2.0, 4.0],
            7.0,
        );
    }
}

#[test]
fn scaling_speed_by_k_and_acceleration_by_k_squared_divides_time_by_k() {
    for factor in [0.25, 0.5, 1.0, 2.0, 4.0] {
        for (distance, expected) in [(1.0, 2.0), (4.0, 4.0), (8.0, 6.0)] {
            assert_duration(
                angles(0.0, 0.0),
                angles(distance, 0.0),
                [2.0 * factor, factor],
                [factor * factor, factor * factor],
                expected / factor,
            );
        }
    }
}

#[test]
fn ordinary_extreme_scales_preserve_triangular_and_trapezoidal_times() {
    for exponent in [-400, -100, 0, 100, 400] {
        let scale = power_of_two(exponent);
        for (distance, expected) in [(1.0, 2.0), (4.0, 4.0), (8.0, 6.0)] {
            assert_duration(
                angles(0.0, 0.0),
                angles(distance * scale, 0.0),
                [2.0 * scale, 1.0],
                [scale, 1.0],
                expected,
            );
        }
    }
}

#[test]
fn positive_subnormal_acceleration_and_displacement_can_produce_finite_time() {
    for scale in [SMALLEST, f64::MIN_POSITIVE] {
        assert_duration(
            angles(0.0, 0.0),
            angles(scale, 0.0),
            [1.0, 1.0],
            [scale, 1.0],
            2.0,
        );
    }
}

#[test]
fn genuinely_unrepresentable_duration_returns_numerical_failure() {
    for (speed, a, distance) in [(0.5, 1.0, f64::MAX), (f64::MAX, SMALLEST, f64::MAX)] {
        for swap in [false, true] {
            let limits = if swap {
                acceleration(1.0, a)
            } else {
                acceleration(a, 1.0)
            };
            let speeds = if swap {
                velocity(1.0, speed)
            } else {
                velocity(speed, 1.0)
            };
            let end = if swap {
                angles(0.0, distance)
            } else {
                angles(distance, 0.0)
            };
            for (start, end) in [(angles(0.0, 0.0), end), (end, angles(0.0, 0.0))] {
                assert_eq!(
                    limits.min_duration_between(start, end, &speeds),
                    Err(PlannerError::NumericalFailure)
                );
            }
        }
    }
}

#[test]
fn overflowing_angle_subtraction_returns_numerical_failure_for_either_joint() {
    let limits = acceleration(1.0, 1.0);
    for (start, end) in [
        (angles(-f64::MAX, 0.0), angles(f64::MAX, 0.0)),
        (angles(0.0, -f64::MAX), angles(0.0, f64::MAX)),
        (angles(-f64::MAX, f64::MAX), angles(f64::MAX, -f64::MAX)),
    ] {
        for speeds in [
            velocity(SMALLEST, SMALLEST),
            velocity(1.0, 1.0),
            velocity(f64::MAX, f64::MAX),
        ] {
            for (start, end) in [(start, end), (end, start)] {
                assert_eq!(
                    limits.min_duration_between(start, end, &speeds),
                    Err(PlannerError::NumericalFailure)
                );
            }
        }
    }
}

// Maximum distance covered by a rest-to-rest velocity envelope over a fixed
// duration: triangle if the speed cap is not reached, trapezoid otherwise.
fn reachable_distance(duration: f64, speed: f64, a: f64) -> f64 {
    let ramp_time = (duration / 2.0).min(speed / a);
    let peak_speed = a * ramp_time;
    peak_speed * (duration - ramp_time)
}

#[test]
fn grid_durations_are_feasible_and_a_shorter_time_is_infeasible() {
    for (speeds, accelerations) in [
        ([0.5, 2.0], [4.0, 0.25]),
        ([4.0, 1.0], [0.5, 2.0]),
        ([2.0, 2.0], [1.0, 1.0]),
    ] {
        let limits = acceleration(accelerations[0], accelerations[1]);
        let velocities = velocity(speeds[0], speeds[1]);
        for first in -8..=8 {
            for second in -8..=8 {
                let distances = [(first as f64).abs(), (second as f64).abs()];
                let t = limits
                    .min_duration_between(
                        angles(0.0, 0.0),
                        angles(first as f64, second as f64),
                        &velocities,
                    )
                    .unwrap();
                for joint in 0..2 {
                    let capacity = reachable_distance(t, speeds[joint], accelerations[joint]);
                    assert!(distances[joint] <= capacity + 32.0 * f64::EPSILON * capacity);
                }
                if t > 0.0 {
                    let shorter = t * (1.0 - 1e-10);
                    assert!((0..2).any(|joint| distances[joint]
                        > reachable_distance(shorter, speeds[joint], accelerations[joint])));
                } else {
                    assert_eq!(distances, [0.0, 0.0]);
                }
            }
        }
    }
}

#[test]
fn increasing_either_limit_never_increases_duration() {
    let start = angles(0.0, 0.0);
    for distance in [0.125, 1.0, 4.0, 32.0] {
        let end = angles(distance, -distance / 2.0);
        let mut previous = f64::INFINITY;
        for speed in [0.25, 0.5, 1.0, 2.0, 8.0] {
            let t = acceleration(1.0, 2.0)
                .min_duration_between(start, end, &velocity(speed, speed))
                .unwrap();
            assert!(t <= previous);
            previous = t;
        }
        previous = f64::INFINITY;
        for a in [0.25, 0.5, 1.0, 2.0, 8.0] {
            let t = acceleration(a, a)
                .min_duration_between(start, end, &velocity(1.0, 2.0))
                .unwrap();
            assert!(t <= previous);
            previous = t;
        }
    }
}

#[test]
fn acceleration_limited_time_is_never_less_than_velocity_only_time() {
    for distance in [0.125, 1.0, 4.0, 32.0] {
        let start = angles(0.0, 0.0);
        let end = angles(distance, -2.0 * distance);
        for speed in [0.25, 1.0, 8.0] {
            let speeds = velocity(speed, 2.0 * speed);
            let baseline = speeds.min_duration_between(start, end).unwrap();
            for a in [0.25, 1.0, 8.0] {
                let t = acceleration(a, 2.0 * a)
                    .min_duration_between(start, end, &speeds)
                    .unwrap();
                assert!(t >= baseline);
            }
        }
    }
}

#[test]
fn duration_accepts_samples_from_a_validated_trajectory_in_both_configurations() {
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
        assert_duration(
            samples[0],
            samples[1],
            [100.0, 100.0],
            [100.0, FRAC_PI_2],
            2.0,
        );
    }
}

// Regressions intentionally describe the mathematically expected behavior.
// Production numerical fixes are outside this test-only review. Run with
// `cargo test -p planner --test joint_acceleration -- --ignored` to reproduce.

#[test]
fn regression_stationary_joint_takes_zero_time_even_when_speed_squared_underflows() {
    assert_duration(
        angles(0.0, 0.0),
        angles(0.0, 0.0),
        [SMALLEST, 1.0],
        [SMALLEST, 1.0],
        0.0,
    );
}

#[test]
fn regression_stationary_joint_cannot_delay_another_joints_valid_move() {
    assert_duration(
        angles(0.0, 0.0),
        angles(0.0, 1.0),
        [SMALLEST, 4.0],
        [SMALLEST, 16.0],
        0.5,
    );
}

#[test]
#[ignore = "known defect: v*v underflow selects a trapezoid for triangular motion"]
fn regression_small_common_scale_preserves_the_triangular_profile() {
    let scale = power_of_two(-600);
    assert_duration(
        angles(0.0, 0.0),
        angles(scale, 0.0),
        [2.0 * scale, 1.0],
        [scale, 1.0],
        2.0,
    );
}

#[test]
#[ignore = "known defect: v*v overflow selects a triangle and violates the speed cap"]
fn regression_large_common_scale_preserves_the_trapezoidal_profile() {
    let scale = power_of_two(600);
    assert_duration(
        angles(0.0, 0.0),
        angles(8.0 * scale, 0.0),
        [2.0 * scale, 1.0],
        [scale, 1.0],
        6.0,
    );
}

#[test]
fn extreme_critical_distance_can_have_a_representable_positive_duration() {
    assert_duration(
        angles(0.0, 0.0),
        angles(power_of_two(-800), 0.0),
        [1.0, 1.0],
        [power_of_two(800), 1.0],
        power_of_two(-799),
    );
}

#[test]
#[ignore = "known defect: delta/a underflow returns zero for nonzero motion"]
fn regression_underflowed_quotient_still_has_a_representable_positive_duration() {
    assert_duration(
        angles(0.0, 0.0),
        angles(power_of_two(-800), 0.0),
        [2.0, 1.0],
        [power_of_two(800), 1.0],
        power_of_two(-799),
    );
}

#[test]
#[ignore = "known defect: delta/a overflow rejects a representable triangular duration"]
fn regression_overflowed_quotient_still_has_a_representable_finite_duration() {
    assert_duration(
        angles(0.0, 0.0),
        angles(power_of_two(800), 0.0),
        [power_of_two(800), 1.0],
        [power_of_two(-800), 1.0],
        power_of_two(801),
    );
}
