use super::*;

#[path = "motion_profile_progress_limits.rs"]
mod progress_limits;

fn assert_unscaled_position_matches_scaled(profile: &MotionProfile, time: f64) {
    let expected = profile
        .position_at_scaled(time, profile.total_time())
        .map(f64::to_bits);
    let actual = profile.position_at(time).map(f64::to_bits);
    assert_eq!(actual, expected, "profile={profile:?}, time={time}");
}

#[test]
fn unscaled_position_matches_scaled_reference() {
    for start in [-10.0, 0.0, 10.0] {
        for end in [-10.0, 0.0, 10.0] {
            for velocity in [0.5, 1.0, 2.0, 4.0] {
                for acceleration in [0.1, 1.0, 10.0, 100.0] {
                    let profile = MotionProfile::new(start, end, acceleration, velocity).unwrap();
                    let duration = profile.total_time();
                    for index in 0..=1024 {
                        let time = duration * (f64::from(index) / 1024.0);
                        assert_unscaled_position_matches_scaled(&profile, time);
                    }
                }
            }
        }
    }
}

#[test]
fn unscaled_position_matches_at_adjacent_phase_times_and_numeric_limits() {
    let smallest = f64::from_bits(1);
    let profiles = [
        MotionProfile::with_boundary_speeds(0.0, 10.0, 2.0, 8.0, 1.0, 3.0).unwrap(),
        MotionProfile::with_boundary_speeds(0.0, -20.0, 2.0, 5.0, 1.0, 3.0).unwrap(),
        MotionProfile::with_boundary_speeds(0.0, 2.0, 2.0, 4.0, 1.0, 3.0).unwrap(),
        MotionProfile::with_boundary_speeds(0.0, 2.0, 2.0, 4.0, 3.0, 1.0).unwrap(),
        MotionProfile::with_boundary_speeds(0.0, 3.0, 2.0, 3.0, 3.0, 3.0).unwrap(),
        MotionProfile::with_boundary_speeds(-0.0, 0.0, 2.0, 4.0, 1.0, 1.0).unwrap(),
        MotionProfile::with_boundary_speeds(0.0, smallest, 1.0, smallest, smallest, smallest)
            .unwrap(),
        MotionProfile::with_boundary_speeds(
            0.0,
            f64::MAX,
            f64::MAX,
            f64::MAX,
            f64::MAX / 2.0,
            f64::MAX / 2.0,
        )
        .unwrap(),
    ];

    for profile in profiles {
        let duration = profile.total_time();
        for boundary in [
            0.0,
            profile.acceleration_time,
            duration - profile.deceleration_time,
            duration,
        ] {
            for time in [boundary.next_down(), boundary, boundary.next_up()] {
                assert_unscaled_position_matches_scaled(&profile, time);
            }
        }
        for time in [
            duration / 2.0,
            -0.0,
            f64::NAN,
            f64::INFINITY,
            f64::NEG_INFINITY,
        ] {
            assert_unscaled_position_matches_scaled(&profile, time);
        }
    }
}

#[test]
fn unscaled_cruise_includes_the_boundary_rounded_by_subtraction() {
    let profile = MotionProfile::with_boundary_speeds(0.0, 2.0, 3.0, 0.5, 0.25, 0.0).unwrap();
    let cruise_end = profile.total_time() - profile.deceleration_time;
    assert!(cruise_end > profile.acceleration_time + profile.cruise_time);
    assert!(matches!(
        profile.phase_at_unscaled(cruise_end),
        MotionPhase::Cruise
    ));
    assert!(matches!(
        profile.phase_at_unscaled(cruise_end.next_up()),
        MotionPhase::Deceleration
    ));
    assert_unscaled_position_matches_scaled(&profile, cruise_end);
    assert_unscaled_position_matches_scaled(&profile, cruise_end.next_up());
}

#[test]
fn unscaled_ramp_preserves_rounding_when_acceleration_equals_duration() {
    let profile = MotionProfile::new(0.0, 6.75, 3.0, 5.0).unwrap();
    let time = 0.01;
    assert_eq!(profile.total_time(), 3.0);
    assert_ne!(
        product_ratio(&[3.0, time, time], &[2.0]).to_bits(),
        profile.position_at_scaled(time, 3.0).unwrap().to_bits()
    );
    assert_unscaled_position_matches_scaled(&profile, time);
}

#[test]
fn unscaled_cruise_preserves_ramp_rounding_when_acceleration_equals_duration() {
    let profile = MotionProfile::new(0.0, 2.45, 5.0, 0.5).unwrap();
    assert_eq!(profile.total_time(), 5.0);
    let ramp_time = profile.acceleration_time;
    let time = ramp_time.next_up();
    assert!(matches!(
        profile.phase_at_unscaled(time),
        MotionPhase::Cruise
    ));
    let simplified_ramp = product_ratio(&[5.0, ramp_time, ramp_time], &[2.0]);
    let cruise_distance = product_ratio(&[0.5, time - ramp_time], &[]);
    assert_ne!(
        (simplified_ramp + cruise_distance).to_bits(),
        profile.position_at_scaled(time, 5.0).unwrap().to_bits()
    );
    assert_unscaled_position_matches_scaled(&profile, time);
}

#[test]
fn stretched_asymmetric_profiles_scale_boundary_velocities_and_accelerations() {
    for end in [-20.0, 20.0] {
        let profile = MotionProfile::with_boundary_speeds(0.0, end, 2.0, 5.0, 1.0, 3.0).unwrap();
        for stretch in [1.0, 2.0, 4.0, 16.0] {
            let duration = profile.total_time() * stretch;
            for time in [0.0, 1.0, 2.0, 3.0, 4.0, 4.5, 5.0] {
                let scaled_time = time * stretch;
                assert_eq!(
                    profile.position_at_scaled(scaled_time, duration),
                    profile.position_at(time)
                );
                assert_eq!(
                    profile.velocity_at_scaled(scaled_time, duration).unwrap(),
                    profile.velocity_at(time).unwrap() / stretch
                );
                assert_eq!(
                    profile
                        .acceleration_at_scaled(scaled_time, duration)
                        .unwrap(),
                    profile.acceleration_at(time).unwrap() / (stretch * stretch)
                );
            }
            for invalid in [-1.0, duration.next_up(), f64::NAN, f64::INFINITY] {
                assert_eq!(
                    profile.position_at_scaled(invalid, duration),
                    Err(PlannerError::InvalidTime)
                );
                assert_eq!(
                    profile.velocity_at_scaled(invalid, duration),
                    Err(PlannerError::InvalidTime)
                );
                assert_eq!(
                    profile.acceleration_at_scaled(invalid, duration),
                    Err(PlannerError::InvalidTime)
                );
            }
        }
    }
}

#[test]
fn stretching_an_instantaneous_profile_keeps_its_position_constant() {
    let profile = MotionProfile::with_boundary_speeds(2.0, 2.0, 2.0, 5.0, 1.0, 1.0).unwrap();
    for time in [0.0, 0.5, 1.0] {
        assert_eq!(profile.position_at_scaled(time, 1.0), Ok(2.0));
        assert_eq!(profile.velocity_at_scaled(time, 1.0), Ok(0.0));
        assert_eq!(profile.acceleration_at_scaled(time, 1.0), Ok(0.0));
    }
}
