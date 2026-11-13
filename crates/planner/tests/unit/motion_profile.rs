use super::*;

#[path = "motion_profile_progress_limits.rs"]
mod progress_limits;

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
