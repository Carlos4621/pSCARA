use kinematics::JointAngles;
use planner::{
    JointAccelerationLimits, JointMotionProfile, JointVelocityLimits, MotionProfile, PlannerError,
};

fn assert_close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() <= 64.0 * f64::EPSILON * expected.abs().max(1.0),
        "expected {expected:?}, got {actual:?}"
    );
}

#[test]
fn integrated_triangle_and_trapezoid_samples_are_invariant_across_extreme_binary_scales() {
    for exponent in [-1000, -800, -600, -200, 0, 200, 600, 800, 1000] {
        let scale = 2.0_f64.powi(exponent);
        let triangular = MotionProfile::new(0.0, scale, scale, 2.0 * scale).unwrap();
        assert_eq!(triangular.total_time(), 2.0);
        for (time, position, velocity) in [
            (0.0, 0.0, 0.0),
            (0.5, 0.125, 0.5),
            (1.0, 0.5, 1.0),
            (1.5, 0.875, 0.5),
            (2.0, 1.0, 0.0),
        ] {
            assert_close(triangular.position_at(time).unwrap() / scale, position);
            assert_close(triangular.velocity_at(time).unwrap() / scale, velocity);
        }
        let trapezoidal = MotionProfile::new(0.0, 8.0 * scale, scale, 2.0 * scale).unwrap();
        assert_eq!(trapezoidal.total_time(), 6.0);
        for (time, position, velocity) in [
            (0.0, 0.0, 0.0),
            (1.0, 0.5, 1.0),
            (2.0, 2.0, 2.0),
            (3.0, 4.0, 2.0),
            (4.0, 6.0, 2.0),
            (5.0, 7.5, 1.0),
            (6.0, 8.0, 0.0),
        ] {
            assert_close(trapezoidal.position_at(time).unwrap() / scale, position);
            assert_close(trapezoidal.velocity_at(time).unwrap() / scale, velocity);
        }
    }
}

#[test]
fn stationary_profiles_have_no_ramps_for_every_extreme_limit_combination() {
    let limits = [f64::from_bits(1), f64::MIN_POSITIVE, 1.0, f64::MAX];
    for acceleration in limits {
        for velocity in limits {
            for angle in [-1000.0_f64, -0.0, 0.0, f64::MAX] {
                let profile = MotionProfile::new(angle, angle, acceleration, velocity).unwrap();
                assert_eq!(profile.total_time(), 0.0);
                assert_eq!(profile.position_at(0.0).unwrap().to_bits(), angle.to_bits());
                assert_eq!(profile.velocity_at(0.0), Ok(0.0));
                assert_eq!(profile.acceleration_at(0.0), Ok(0.0));
                assert_eq!(
                    profile.position_at(f64::from_bits(1)),
                    Err(PlannerError::InvalidTime)
                );
            }
        }
    }
}

#[test]
fn synchronized_velocity_survives_even_when_the_corresponding_local_time_underflows() {
    let motion = JointMotionProfile::new(
        JointAngles::new(0.0, 0.0).unwrap(),
        JointAngles::new(f64::from_bits(1), 2.0_f64.powi(-1000)).unwrap(),
        &JointVelocityLimits::new(1.0, 1.0).unwrap(),
        &JointAccelerationLimits::new(2.0_f64.powi(1022), 2.0_f64.powi(432)).unwrap(),
    )
    .unwrap();
    assert_eq!(motion.total_time(), 2.0_f64.powi(-715));
    let time = 2.0_f64.powi(-800);
    assert_eq!(time * 2.0_f64.powi(-332), 0.0);
    let (first, second) = motion.velocity_at(time).unwrap();
    assert_eq!(first, 2.0_f64.powi(-442));
    assert_eq!(second, 2.0_f64.powi(-368));
    assert_eq!(motion.acceleration_at(time).unwrap().0, 2.0_f64.powi(358));
}

#[test]
fn near_identity_synchronization_keeps_maximum_finite_limits_finite() {
    let slightly_longer_distance = f64::from_bits(1.0_f64.to_bits() + 2);
    let motion = JointMotionProfile::new(
        JointAngles::new(0.0, 0.0).unwrap(),
        JointAngles::new(f64::MAX, slightly_longer_distance).unwrap(),
        &JointVelocityLimits::new(f64::MAX, 10.0).unwrap(),
        &JointAccelerationLimits::new(f64::MAX, 1.0).unwrap(),
    )
    .unwrap();
    assert!(motion.total_time() > 2.0);
    for fraction in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let time = motion.total_time() * fraction;
        let position = motion.position_at(time).unwrap().first_theta();
        let velocity = motion.velocity_at(time).unwrap().0;
        let acceleration = motion.acceleration_at(time).unwrap().0;
        assert!(position.is_finite() && position >= 0.0);
        assert!(velocity.is_finite() && velocity >= 0.0);
        assert!(acceleration.is_finite());
    }
}
