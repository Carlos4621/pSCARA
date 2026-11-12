use planner::{MotionProfile, PlannerError};

fn close(actual: f64, expected: f64) {
    assert!(actual.is_finite(), "nonfinite value: {actual}");
    assert!(
        (actual - expected).abs() <= 128.0 * f64::EPSILON * expected.abs().max(1.0),
        "expected {expected:e}, got {actual:e}"
    );
}

fn motion(distance: f64, cap: f64, initial: f64, final_speed: f64) -> MotionProfile {
    MotionProfile::with_boundary_speeds(0.0, distance, 2.0, cap, initial, final_speed).unwrap()
}

#[test]
fn asymmetric_triangle_matches_analytical_position_velocity_and_acceleration() {
    // a=2, vi=1, vf=3, peak=5: accelerate for 2s, brake for 1s.
    for direction in [-1.0, 1.0] {
        let profile = motion(direction * 10.0, 8.0, 1.0, 3.0);
        close(profile.total_time(), 3.0);
        for (time, position, velocity, acceleration) in [
            (0.0, 0.0, 1.0, 2.0),
            (1.0, 2.0, 3.0, 2.0),
            (2.0, 6.0, 5.0, 2.0),
            (2.5, 8.25, 4.0, -2.0),
            (3.0, 10.0, 3.0, -2.0),
        ] {
            close(profile.position_at(time).unwrap(), direction * position);
            close(profile.velocity_at(time).unwrap(), direction * velocity);
            close(
                profile.acceleration_at(time).unwrap(),
                direction * acceleration,
            );
        }
    }
}

#[test]
fn asymmetric_trapezoid_uses_independent_ramp_durations() {
    // Same ramps, with 2s of cruise at speed 5, covering another 10 units.
    for direction in [-1.0, 1.0] {
        let profile = motion(direction * 20.0, 5.0, 1.0, 3.0);
        close(profile.total_time(), 5.0);
        for (time, position, velocity, acceleration) in [
            (0.0, 0.0, 1.0, 2.0),
            (1.0, 2.0, 3.0, 2.0),
            (2.0, 6.0, 5.0, 2.0),
            (3.0, 11.0, 5.0, 0.0),
            (4.0, 16.0, 5.0, 0.0),
            (4.5, 18.25, 4.0, -2.0),
            (5.0, 20.0, 3.0, -2.0),
        ] {
            close(profile.position_at(time).unwrap(), direction * position);
            close(profile.velocity_at(time).unwrap(), direction * velocity);
            close(
                profile.acceleration_at(time).unwrap(),
                direction * acceleration,
            );
        }
    }
}

#[test]
fn minimum_feasible_distance_allows_pure_acceleration_or_braking() {
    for (initial, final_speed, acceleration) in [(1.0, 3.0, 2.0), (3.0, 1.0, -2.0)] {
        for direction in [-1.0, 1.0] {
            let profile = motion(direction * 2.0, 8.0, initial, final_speed);
            close(profile.total_time(), 1.0);
            for i in 0..=16 {
                let time = f64::from(i) / 16.0;
                close(
                    profile.position_at(time).unwrap(),
                    direction * (initial * time + acceleration * time * time / 2.0),
                );
                close(
                    profile.velocity_at(time).unwrap(),
                    direction * (initial + acceleration * time),
                );
                close(
                    profile.acceleration_at(time).unwrap(),
                    direction * acceleration,
                );
            }
        }
    }
}

#[test]
fn insufficient_distance_is_infeasible_for_both_speed_change_directions() {
    for (initial, final_speed) in [(1.0_f64, 3.0), (3.0, 1.0), (0.0, 4.0), (4.0, 0.0)] {
        let minimum = (final_speed * final_speed - initial * initial).abs() / 4.0;
        for distance in [0.0, minimum / 2.0, minimum.next_down()] {
            for direction in [-1.0, 1.0] {
                assert_eq!(
                    MotionProfile::with_boundary_speeds(
                        0.0,
                        direction * distance,
                        2.0,
                        8.0,
                        initial,
                        final_speed
                    )
                    .unwrap_err(),
                    PlannerError::InfeasibleMotion
                );
            }
        }
    }
}

#[test]
fn equal_boundary_speeds_at_the_cap_produce_constant_velocity() {
    for direction in [-1.0, 1.0] {
        let profile = motion(direction * 6.0, 3.0, 3.0, 3.0);
        close(profile.total_time(), 2.0);
        for i in 0..=32 {
            let time = f64::from(i) / 16.0;
            close(profile.position_at(time).unwrap(), direction * 3.0 * time);
            assert_eq!(profile.velocity_at(time), Ok(direction * 3.0));
            assert_eq!(profile.acceleration_at(time), Ok(0.0));
        }
    }
}

#[test]
fn a_boundary_at_the_cap_omits_only_the_corresponding_ramp() {
    let accelerate = motion(11.0, 5.0, 1.0, 5.0);
    let brake = motion(11.0, 5.0, 5.0, 1.0);
    close(accelerate.total_time(), 3.0);
    close(brake.total_time(), 3.0);
    for (time, expected) in [(0.0, 2.0), (1.0, 2.0), (2.0, 2.0), (2.5, 0.0), (3.0, 0.0)] {
        close(accelerate.acceleration_at(time).unwrap(), expected);
    }
    for (time, expected) in [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (2.0, -2.0), (3.0, -2.0)] {
        close(brake.acceleration_at(time).unwrap(), expected);
    }
    close(accelerate.position_at(2.0).unwrap(), 6.0);
    close(brake.position_at(1.0).unwrap(), 5.0);
    assert_eq!(accelerate.velocity_at(3.0), Ok(5.0));
    assert_eq!(brake.velocity_at(0.0), Ok(5.0));
}

#[test]
fn critical_distance_has_no_cruise_and_nearby_distances_are_continuous() {
    let critical = motion(10.0, 5.0, 1.0, 3.0);
    close(critical.total_time(), 3.0);
    assert_eq!(critical.acceleration_at(2.0), Ok(2.0));
    assert_eq!(critical.acceleration_at(2.0_f64.next_up()), Ok(-2.0));
    for distance in [10.0_f64.next_down(), 10.0, 10.0_f64.next_up()] {
        let profile = motion(distance, 5.0, 1.0, 3.0);
        close(profile.total_time(), 3.0);
        close(profile.position_at(2.0).unwrap(), 6.0);
        assert_eq!(profile.velocity_at(0.0), Ok(1.0));
        assert_eq!(profile.velocity_at(profile.total_time()), Ok(3.0));
    }
}

#[test]
fn invalid_boundary_speeds_are_rejected_with_specific_errors() {
    for invalid in [
        -1.0,
        -f64::from_bits(1),
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
    ] {
        assert_eq!(
            MotionProfile::with_boundary_speeds(0.0, 10.0, 2.0, 5.0, invalid, 1.0).unwrap_err(),
            PlannerError::InvalidInitialSpeed
        );
        assert_eq!(
            MotionProfile::with_boundary_speeds(0.0, 10.0, 2.0, 5.0, 1.0, invalid).unwrap_err(),
            PlannerError::InvalidFinalSpeed
        );
    }
    for (initial, final_speed) in [
        (5.0_f64.next_up(), 1.0),
        (1.0, 5.0_f64.next_up()),
        (6.0, 6.0),
    ] {
        assert_eq!(
            MotionProfile::with_boundary_speeds(0.0, 10.0, 2.0, 5.0, initial, final_speed)
                .unwrap_err(),
            PlannerError::InvalidBoundarySpeed
        );
    }
}

#[test]
fn existing_angle_and_limit_validation_also_applies_to_boundary_profiles() {
    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(
            MotionProfile::with_boundary_speeds(invalid, 10.0, 2.0, 5.0, 1.0, 1.0).unwrap_err(),
            PlannerError::AngleNonFinite
        );
        assert_eq!(
            MotionProfile::with_boundary_speeds(0.0, invalid, 2.0, 5.0, 1.0, 1.0).unwrap_err(),
            PlannerError::AngleNonFinite
        );
    }
    for invalid in [0.0, -0.0, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(
            MotionProfile::with_boundary_speeds(0.0, 10.0, invalid, 5.0, 1.0, 1.0).unwrap_err(),
            PlannerError::InvalidMaxAcceleration
        );
        assert_eq!(
            MotionProfile::with_boundary_speeds(0.0, 10.0, 2.0, invalid, 1.0, 1.0).unwrap_err(),
            PlannerError::InvalidMaxVelocity
        );
    }
}

#[test]
fn invalid_times_are_rejected_in_every_phase_and_degenerate_case() {
    for profile in [
        motion(20.0, 5.0, 1.0, 3.0),
        motion(6.0, 3.0, 3.0, 3.0),
        motion(0.0, 5.0, 1.0, 1.0),
    ] {
        for time in [
            -1.0,
            -f64::from_bits(1),
            f64::NAN,
            f64::INFINITY,
            f64::NEG_INFINITY,
            profile.total_time().next_up(),
        ] {
            assert_eq!(profile.validate_time(time), Err(PlannerError::InvalidTime));
            assert_eq!(profile.position_at(time), Err(PlannerError::InvalidTime));
            assert_eq!(profile.velocity_at(time), Err(PlannerError::InvalidTime));
            assert_eq!(
                profile.acceleration_at(time),
                Err(PlannerError::InvalidTime)
            );
        }
    }
}

#[test]
fn zero_distance_is_instantaneous_only_when_boundary_speeds_match() {
    for angle in [-10.0, -0.0, 0.0, 10.0] {
        for speed in [0.0, 1.0, 5.0] {
            let profile =
                MotionProfile::with_boundary_speeds(angle, angle, 2.0, 5.0, speed, speed).unwrap();
            assert_eq!(profile.total_time(), 0.0);
            assert_eq!(profile.position_at(0.0), Ok(angle));
            assert_eq!(profile.velocity_at(0.0), Ok(speed));
            assert_eq!(profile.acceleration_at(-0.0), Ok(0.0));
        }
    }
}

#[test]
fn zero_boundary_speeds_preserve_the_rest_to_rest_api() {
    for (start, end) in [(0.0, 0.0), (-3.0, 7.0), (7.0, -3.0)] {
        for initial in [-0.0, 0.0] {
            for final_speed in [-0.0, 0.0] {
                let rest = MotionProfile::new(start, end, 2.0, 5.0).unwrap();
                let boundary =
                    MotionProfile::with_boundary_speeds(start, end, 2.0, 5.0, initial, final_speed)
                        .unwrap();
                assert_eq!(boundary.total_time(), rest.total_time());
                for i in 0..=16 {
                    let time = rest.total_time() * (f64::from(i) / 16.0);
                    assert_eq!(boundary.position_at(time), rest.position_at(time));
                    assert_eq!(boundary.velocity_at(time), rest.velocity_at(time));
                    assert_eq!(boundary.acceleration_at(time), rest.acceleration_at(time));
                }
            }
        }
    }
}

#[test]
fn independently_integrated_profiles_cover_boundary_speed_combinations() {
    // Generate the displacement by integrating a chosen velocity history,
    // then ask the constructor to recover it. All fixture inputs are dyadic.
    for a in [0.25, 1.0, 4.0] {
        for initial in [0.0_f64, 0.25, 1.0, 3.0] {
            for final_speed in [0.0_f64, 0.25, 1.0, 3.0] {
                for excess in [0.0, 1.0] {
                    for cruise in [0.0, 0.5, 2.0] {
                        let peak = initial.max(final_speed) + excess;
                        if peak == 0.0 {
                            continue;
                        }
                        let ta = (peak - initial) / a;
                        let td = (peak - final_speed) / a;
                        let da = (initial + peak) * ta / 2.0;
                        let distance = da + peak * cruise + (final_speed + peak) * td / 2.0;
                        // A zero-length profile has no negative travel direction;
                        // its matching instantaneous speeds are tested separately.
                        if distance == 0.0 {
                            continue;
                        }
                        let duration = ta + cruise + td;
                        let cap = if cruise == 0.0 { peak * 2.0 } else { peak };
                        for direction in [-1.0, 1.0] {
                            let profile = MotionProfile::with_boundary_speeds(
                                -2.0,
                                -2.0 + direction * distance,
                                a,
                                cap,
                                initial,
                                final_speed,
                            )
                            .unwrap();
                            close(profile.total_time(), duration);
                            // Midpoints and both ends of every nonempty phase.
                            for (length, offset, start_position, start_speed, acceleration) in [
                                (ta, 0.0, 0.0, initial, a),
                                (cruise, ta, da, peak, 0.0),
                                (td, ta + cruise, da + peak * cruise, peak, -a),
                            ] {
                                for fraction in [0.0, 0.25, 0.5, 0.75, 1.0] {
                                    let local = length * fraction;
                                    let time = offset + local;
                                    let position = start_position
                                        + start_speed * local
                                        + acceleration * local * local / 2.0;
                                    close(
                                        profile.position_at(time).unwrap(),
                                        -2.0 + direction * position,
                                    );
                                    close(
                                        profile.velocity_at(time).unwrap(),
                                        direction * (start_speed + acceleration * local),
                                    );
                                    if length > 0.0 && fraction > 0.0 && fraction < 1.0 {
                                        close(
                                            profile.acceleration_at(time).unwrap(),
                                            direction * acceleration,
                                        );
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn swapping_boundary_speeds_and_endpoints_reverses_time() {
    for cap in [5.0, 8.0] {
        let forward = MotionProfile::with_boundary_speeds(-2.0, 8.0, 2.0, cap, 1.0, 3.0).unwrap();
        let backward = MotionProfile::with_boundary_speeds(8.0, -2.0, 2.0, cap, 3.0, 1.0).unwrap();
        close(forward.total_time(), backward.total_time());
        for i in 0..=64 {
            let time = forward.total_time() * (f64::from(i) / 64.0);
            let reverse_time = backward.total_time() - time;
            close(
                forward.position_at(time).unwrap(),
                backward.position_at(reverse_time).unwrap(),
            );
            close(
                forward.velocity_at(time).unwrap(),
                -backward.velocity_at(reverse_time).unwrap(),
            );
        }
    }
}

#[test]
fn consecutive_moves_can_share_nonzero_position_and_velocity() {
    for direction in [-1.0, 1.0] {
        let first = motion(direction * 10.0, 8.0, 1.0, 3.0);
        let second = MotionProfile::with_boundary_speeds(
            direction * 10.0,
            direction * 20.0,
            2.0,
            8.0,
            3.0,
            1.0,
        )
        .unwrap();
        assert_eq!(
            first.position_at(first.total_time()),
            second.position_at(0.0)
        );
        assert_eq!(
            first.velocity_at(first.total_time()),
            second.velocity_at(0.0)
        );
        let h = 1e-6;
        let left = (first.position_at(first.total_time()).unwrap()
            - first.position_at(first.total_time() - h).unwrap())
            / h;
        let right = (second.position_at(h).unwrap() - second.position_at(0.0).unwrap()) / h;
        assert!((left - direction * 3.0).abs() < 2e-6);
        assert!((right - direction * 3.0).abs() < 2e-6);
    }
}

#[test]
fn scaling_all_speeds_and_acceleration_preserves_the_geometric_path() {
    for cap in [5.0, 8.0] {
        let reference = motion(20.0, cap, 1.0, 3.0);
        for factor in [0.25, 0.5, 2.0, 4.0] {
            let scaled = MotionProfile::with_boundary_speeds(
                0.0,
                20.0,
                2.0 * factor * factor,
                cap * factor,
                factor,
                3.0 * factor,
            )
            .unwrap();
            close(scaled.total_time(), reference.total_time() / factor);
            for i in 0..=64 {
                let time = reference.total_time() * (f64::from(i) / 64.0);
                close(
                    scaled.position_at(time / factor).unwrap(),
                    reference.position_at(time).unwrap(),
                );
                close(
                    scaled.velocity_at(time / factor).unwrap(),
                    reference.velocity_at(time).unwrap() * factor,
                );
                close(
                    scaled.acceleration_at(time / factor).unwrap(),
                    reference.acceleration_at(time).unwrap() * factor * factor,
                );
            }
        }
    }
}

#[test]
fn derivatives_and_integrals_agree_with_nonzero_boundary_speeds() {
    let profile = motion(20.0, 5.0, 1.0, 3.0);
    let h = 1e-4;
    for time in [0.5, 1.5, 2.5, 3.5, 4.5] {
        let dp = (profile.position_at(time + h).unwrap() - profile.position_at(time - h).unwrap())
            / (2.0 * h);
        let dv = (profile.velocity_at(time + h).unwrap() - profile.velocity_at(time - h).unwrap())
            / (2.0 * h);
        assert!((dp - profile.velocity_at(time).unwrap()).abs() < 1e-9);
        assert!((dv - profile.acceleration_at(time).unwrap()).abs() < 1e-9);
    }
    let mut integrated_position = 0.0;
    let mut integrated_velocity = 1.0;
    // The 2s/4s phase boundaries align exactly with this quadrature grid.
    for i in 0..500 {
        let middle = (f64::from(i) + 0.5) / 100.0;
        integrated_position += profile.velocity_at(middle).unwrap() / 100.0;
        integrated_velocity += profile.acceleration_at(middle).unwrap() / 100.0;
    }
    assert!((integrated_position - 20.0).abs() < 1e-11);
    assert!((integrated_velocity - 3.0).abs() < 1e-11);
}

#[test]
fn adjacent_phase_times_preserve_position_and_velocity_continuity() {
    for (distance, cap, boundaries) in [(10.0, 8.0, &[2.0][..]), (20.0, 5.0, &[2.0, 4.0][..])] {
        let profile = motion(distance, cap, 1.0, 3.0);
        for &boundary in boundaries {
            let position = profile.position_at(boundary).unwrap();
            let velocity = profile.velocity_at(boundary).unwrap();
            for time in [boundary.next_down(), boundary.next_up()] {
                close(profile.position_at(time).unwrap(), position);
                close(profile.velocity_at(time).unwrap(), velocity);
            }
        }
    }
}

#[test]
fn asymmetric_profiles_preserve_dynamics_across_extreme_binary_scales() {
    for exponent in [-1000, -800, -600, 0, 600, 800, 1000] {
        let scale = 2.0_f64.powi(exponent);
        for (distance, cap, total) in [(10.0, 8.0, 3.0), (20.0, 5.0, 5.0)] {
            for direction in [-1.0, 1.0] {
                let profile = MotionProfile::with_boundary_speeds(
                    0.0,
                    direction * distance * scale,
                    2.0 * scale,
                    cap * scale,
                    scale,
                    3.0 * scale,
                )
                .unwrap();
                let reference = motion(direction * distance, cap, 1.0, 3.0);
                close(profile.total_time(), total);
                for i in 0..=64 {
                    let time = total * (f64::from(i) / 64.0);
                    close(
                        profile.position_at(time).unwrap() / scale,
                        reference.position_at(time).unwrap(),
                    );
                    close(
                        profile.velocity_at(time).unwrap() / scale,
                        reference.velocity_at(time).unwrap(),
                    );
                    close(
                        profile.acceleration_at(time).unwrap() / scale,
                        reference.acceleration_at(time).unwrap(),
                    );
                }
            }
        }
    }
}

#[test]
fn tiny_peak_increments_do_not_disappear_from_the_duration() {
    // vp rounds to 1, but travelling a positive distance still takes time.
    let distance = 2.0_f64.powi(-60);
    let profile = MotionProfile::with_boundary_speeds(0.0, distance, 1.0, 2.0, 1.0, 1.0).unwrap();
    assert_eq!(profile.total_time(), distance);
    assert_eq!(profile.position_at(distance / 2.0), Ok(distance / 2.0));
    assert_eq!(profile.velocity_at(distance / 2.0), Ok(1.0));
}

#[test]
fn subnormal_boundary_speeds_preserve_constant_motion_and_speed_changes() {
    let smallest = f64::from_bits(1);
    let constant =
        MotionProfile::with_boundary_speeds(0.0, smallest, 1.0, smallest, smallest, smallest)
            .unwrap();
    assert_eq!(constant.total_time(), 1.0);
    assert_eq!(constant.velocity_at(0.5), Ok(smallest));
    // vi=1u, vf=3u, a=2u: d_min=2u, duration=1, no second ramp.
    for (initial, final_speed) in [(smallest, 3.0 * smallest), (3.0 * smallest, smallest)] {
        let profile = MotionProfile::with_boundary_speeds(
            0.0,
            2.0 * smallest,
            2.0 * smallest,
            4.0 * smallest,
            initial,
            final_speed,
        )
        .unwrap();
        assert_eq!(profile.total_time(), 1.0);
        assert_eq!(profile.velocity_at(0.0), Ok(initial));
        assert_eq!(profile.velocity_at(1.0), Ok(final_speed));
    }
}

#[test]
fn maximum_finite_speeds_do_not_overflow_intermediate_sums_or_squares() {
    let profile = MotionProfile::with_boundary_speeds(
        0.0,
        f64::MAX,
        f64::MAX,
        f64::MAX,
        f64::MAX / 2.0,
        f64::MAX / 2.0,
    )
    .unwrap();
    for i in 0..=64 {
        let time = profile.total_time() * (f64::from(i) / 64.0);
        assert!(profile.position_at(time).unwrap().is_finite());
        assert!(profile.velocity_at(time).unwrap().is_finite());
        assert!(profile.acceleration_at(time).unwrap().is_finite());
    }
    assert_eq!(profile.velocity_at(0.0), Ok(f64::MAX / 2.0));
    assert_eq!(
        profile.velocity_at(profile.total_time()),
        Ok(f64::MAX / 2.0)
    );
}

#[test]
fn unrepresentable_displacement_or_duration_returns_numerical_failure() {
    for (start, end, a, cap, initial, final_speed) in [
        (-f64::MAX, f64::MAX, 1.0, 2.0, 1.0, 1.0),
        (0.0, f64::MAX, 1.0, f64::from_bits(1), 0.0, 0.0),
        (0.0, f64::from_bits(1), 1.0, f64::MAX, f64::MAX, f64::MAX),
    ] {
        assert_eq!(
            MotionProfile::with_boundary_speeds(start, end, a, cap, initial, final_speed)
                .unwrap_err(),
            PlannerError::NumericalFailure
        );
    }
}
