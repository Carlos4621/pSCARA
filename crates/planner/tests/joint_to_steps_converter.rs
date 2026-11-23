use core::f64::consts::TAU;

use kinematics::JointAngles;
use planner::{
    CoordinatedJointMotionProfile, CoordinatedJointPathProfiles, CoordinatedJointTrajectory,
    CoordinatedJointTrajectoryCursor, JointAccelerationLimits, JointToStepsConverter,
    JointVelocityLimits, MotorSteps, PlannerError, TimedCoordinatedJointPathProfiles,
};

const I64_UPPER_BOUND: f64 = (1_u64 << 63) as f64;
const SMALLEST: f64 = f64::from_bits(1);

fn angles(first: f64, second: f64) -> JointAngles {
    JointAngles::new(first, second).unwrap()
}

fn steps(first: i64, second: i64) -> MotorSteps {
    MotorSteps { first, second }
}

#[test]
fn the_constructor_accepts_positive_finite_calibrations_on_either_axis() {
    for calibration in [SMALLEST, f64::MIN_POSITIVE, 0.25, 1.0, 1000.0, f64::MAX] {
        assert!(JointToStepsConverter::new(calibration, 1.0).is_ok());
        assert!(JointToStepsConverter::new(1.0, calibration).is_ok());
    }
}

#[test]
fn the_constructor_rejects_invalid_calibrations_on_either_axis() {
    for calibration in [
        0.0,
        -0.0,
        -SMALLEST,
        -f64::MIN_POSITIVE,
        -1.0,
        -f64::MAX,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NAN,
        -f64::NAN,
    ] {
        assert_eq!(
            JointToStepsConverter::new(calibration, 1.0).unwrap_err(),
            PlannerError::InvalidSteps,
            "first calibration: {calibration}"
        );
        assert_eq!(
            JointToStepsConverter::new(1.0, calibration).unwrap_err(),
            PlannerError::InvalidSteps,
            "second calibration: {calibration}"
        );
    }
}

#[test]
fn each_joint_uses_its_own_calibration_and_keeps_its_direction() {
    let converter = JointToStepsConverter::new(4.0, 8.0).unwrap();
    assert_eq!(converter.convert(angles(1.25, -1.25)), Ok(steps(5, -10)));
    assert_eq!(converter.convert(angles(-1.25, 1.25)), Ok(steps(-5, 10)));
    assert_eq!(converter.convert(angles(0.375, -0.3125)), Ok(steps(2, -3)));
    assert_eq!(converter.convert(angles(0.0, 1.25)), Ok(steps(0, 10)));
    assert_eq!(converter.convert(angles(-1.25, 0.0)), Ok(steps(-5, 0)));
}

#[test]
fn multiple_turns_keep_their_absolute_step_count() {
    let converter = JointToStepsConverter::new(256.0 / TAU, 160.0 / TAU).unwrap();
    assert_eq!(
        converter.convert(angles(3.0 * TAU, -2.0 * TAU)),
        Ok(steps(768, -320))
    );
    assert_eq!(
        converter.convert(angles(-TAU / 4.0, 2.5 * TAU)),
        Ok(steps(-64, 400))
    );
}

#[test]
fn half_steps_round_away_from_zero_and_their_neighbors_round_to_the_nearest_step() {
    let converter = JointToStepsConverter::new(1.0, 1.0).unwrap();
    for (half, lower, upper) in [(0.5_f64, 0, 1), (1.5, 1, 2), (2.5, 2, 3), (511.5, 511, 512)] {
        for (angle, expected) in [
            (half.next_down(), lower),
            (half, upper),
            (half.next_up(), upper),
        ] {
            assert_eq!(
                converter.convert(angles(angle, -angle)),
                Ok(steps(expected, -expected))
            );
            assert_eq!(
                converter.convert(angles(-angle, angle)),
                Ok(steps(-expected, expected))
            );
        }
    }
}

#[test]
fn quarter_step_grid_matches_an_integer_rounding_oracle_after_calibration() {
    let converter = JointToStepsConverter::new(0.125, 32.0).unwrap();
    for quarters in -1024_i32..=1024 {
        let scaled = f64::from(quarters);
        let rounded = i64::from((quarters.unsigned_abs() + 2) / 4) * i64::from(quarters.signum());
        // These binary fractions give exactly `quarters / 4` and its negation in steps.
        assert_eq!(
            converter.convert(angles(scaled * 2.0, -scaled / 128.0)),
            Ok(steps(rounded, -rounded))
        );
    }
}

#[test]
fn signed_zeros_and_products_that_underflow_to_zero_yield_zero_steps() {
    let converter = JointToStepsConverter::new(f64::MAX, SMALLEST).unwrap();
    for (first, second) in [(0.0, -0.0), (-0.0, 0.0), (0.0, 0.0), (-0.0, -0.0)] {
        assert_eq!(converter.convert(angles(first, second)), Ok(steps(0, 0)));
    }
    let tiny = JointToStepsConverter::new(SMALLEST, SMALLEST).unwrap();
    assert_eq!(tiny.convert(angles(0.5, -0.5)), Ok(steps(0, 0)));
    assert_eq!(tiny.convert(angles(f64::MAX, -f64::MAX)), Ok(steps(0, 0)));
}

#[test]
fn extreme_reciprocal_scales_can_produce_small_finite_step_counts() {
    let large = 1.0 / f64::MIN_POSITIVE;
    let small = f64::MIN_POSITIVE;
    let converter = JointToStepsConverter::new(small, large).unwrap();
    assert_eq!(
        converter.convert(angles(1.5 * large, -2.5 * small)),
        Ok(steps(2, -3))
    );
    assert_eq!(
        converter.convert(angles(-1.5 * large, 2.5 * small)),
        Ok(steps(-2, 3))
    );
}

#[test]
fn exact_i64_lower_bound_and_largest_representable_positive_step_are_accepted() {
    let converter = JointToStepsConverter::new(1.0, 1.0).unwrap();
    let lower = -I64_UPPER_BOUND;
    let largest_positive = I64_UPPER_BOUND.next_down();
    assert_eq!(
        converter.convert(angles(lower, largest_positive)),
        Ok(steps(i64::MIN, i64::MAX - 1023))
    );
    assert_eq!(
        converter.convert(angles(largest_positive, lower.next_up())),
        Ok(steps(i64::MAX - 1023, i64::MIN + 1024))
    );
}

#[test]
fn finite_out_of_range_products_are_rejected_without_saturating() {
    let converter = JointToStepsConverter::new(2.0, 4.0).unwrap();
    for value in [
        I64_UPPER_BOUND,
        I64_UPPER_BOUND.next_up(),
        (-I64_UPPER_BOUND).next_down(),
    ] {
        assert_eq!(
            converter.convert(angles(value / 2.0, 1.0)),
            Err(PlannerError::NumericalFailure)
        );
        assert_eq!(
            converter.convert(angles(1.0, value / 4.0)),
            Err(PlannerError::NumericalFailure)
        );
    }
    assert_eq!(
        converter.convert(angles(
            -I64_UPPER_BOUND / 2.0,
            I64_UPPER_BOUND.next_down() / 4.0
        )),
        Ok(steps(i64::MIN, i64::MAX - 1023))
    );
}

#[test]
fn overflowing_a_product_on_either_axis_does_not_poison_later_conversions() {
    let converter = JointToStepsConverter::new(2.0, 4.0).unwrap();
    for angle in [f64::MAX, -f64::MAX] {
        assert_eq!(
            converter.convert(angles(angle, 1.0)),
            Err(PlannerError::NumericalFailure)
        );
        assert_eq!(
            converter.convert(angles(1.0, angle)),
            Err(PlannerError::NumericalFailure)
        );
        assert_eq!(converter.convert(angles(1.0, -1.0)), Ok(steps(2, -4)));
    }
}

#[test]
fn peak_step_frequencies_use_each_motors_resolution_without_rounding_to_integers() {
    for (velocities, resolutions, expected) in [
        ((2.0, 1.0), (400.0, 800.0), (800.0, 800.0)),
        ((0.0, 3.0), (400.0, 200.0), (0.0, 600.0)),
        ((1.5, 2.0), (100.0, 100.0), (150.0, 200.0)),
        ((0.0, 0.0), (400.0, 400.0), (0.0, 0.0)),
        ((1.5, 0.5), (0.25, 3.0), (0.375, 1.5)),
    ] {
        let converter = JointToStepsConverter::new(resolutions.0, resolutions.1).unwrap();
        assert_eq!(converter.peak_step_frequencies(velocities), Ok(expected));
    }
}

#[test]
fn peak_step_frequencies_reject_negative_and_nonfinite_velocities_on_either_axis() {
    let converter = JointToStepsConverter::new(400.0, 800.0).unwrap();
    for invalid in [
        -SMALLEST,
        -1.0,
        -f64::MAX,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NAN,
        -f64::NAN,
    ] {
        assert_eq!(
            converter.peak_step_frequencies((invalid, 0.0)),
            Err(PlannerError::InvalidJointVelocity)
        );
        assert_eq!(
            converter.peak_step_frequencies((0.0, invalid)),
            Err(PlannerError::InvalidJointVelocity)
        );
    }
}

#[test]
fn peak_step_frequencies_return_positive_zero_for_either_signed_zero() {
    let converter = JointToStepsConverter::new(f64::MAX, SMALLEST).unwrap();
    for velocities in [(0.0, -0.0), (-0.0, 0.0), (-0.0, -0.0)] {
        let (first, second) = converter.peak_step_frequencies(velocities).unwrap();
        assert_eq!(first.to_bits(), 0.0_f64.to_bits());
        assert_eq!(second.to_bits(), 0.0_f64.to_bits());
    }
}

#[test]
fn peak_step_frequencies_accept_subnormal_and_underflowed_products() {
    let converter = JointToStepsConverter::new(1.0, SMALLEST).unwrap();
    assert_eq!(
        converter.peak_step_frequencies((SMALLEST, 0.25)),
        Ok((SMALLEST, 0.0))
    );
}

#[test]
fn peak_step_frequencies_accept_the_maximum_finite_product_and_reject_overflow() {
    let converter = JointToStepsConverter::new(2.0, 4.0).unwrap();
    let first_limit = f64::MAX / 2.0;
    let second_limit = f64::MAX / 4.0;
    assert_eq!(
        converter.peak_step_frequencies((first_limit, second_limit)),
        Ok((f64::MAX, f64::MAX))
    );
    assert_eq!(
        converter.peak_step_frequencies((first_limit.next_up(), second_limit)),
        Err(PlannerError::NumericalFailure)
    );
    assert_eq!(
        converter.peak_step_frequencies((first_limit, second_limit.next_up())),
        Err(PlannerError::NumericalFailure)
    );
}

#[test]
fn peak_step_frequencies_accept_the_magnitudes_from_a_coordinated_profile()
-> Result<(), PlannerError> {
    let velocity = JointVelocityLimits::new(10.0, 10.0)?;
    let acceleration = JointAccelerationLimits::new(2.0, 1.0)?;
    let profile = CoordinatedJointMotionProfile::new(
        angles(0.0, 0.0),
        angles(2.0, -1.0),
        &velocity,
        &acceleration,
        0.0,
        0.0,
    )?;
    let converter = JointToStepsConverter::new(400.0, 800.0)?;

    assert_eq!(
        converter.peak_step_frequencies(profile.peak_joint_velocities()?),
        Ok((800.0, 800.0))
    );
    Ok(())
}

#[test]
fn coordinated_trajectory_samples_convert_to_absolute_motor_positions() {
    let points = [angles(0.0, 0.0), angles(1.0, -0.5), angles(1.0, -1.5)];
    let velocity = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let acceleration = JointAccelerationLimits::new(1.0, 1.0).unwrap();
    let profiles =
        CoordinatedJointPathProfiles::new(&points, &[0.0; 3], &velocity, &acceleration).unwrap();
    let timed = TimedCoordinatedJointPathProfiles::new(profiles).unwrap();
    let trajectory = CoordinatedJointTrajectory::new(timed).unwrap();
    let mut cursor = CoordinatedJointTrajectoryCursor::new(trajectory);
    let converter = JointToStepsConverter::new(8.0, 16.0).unwrap();

    for (time, expected) in [
        (0.0, steps(0, 0)),
        (1.0, steps(4, -4)),
        (2.0, steps(8, -8)),
        (3.0, steps(8, -16)),
        (4.0, steps(8, -24)),
    ] {
        assert_eq!(
            cursor
                .sample_at(time)
                .and_then(|sample| converter.convert(sample.position)),
            Ok(expected)
        );
    }
}
