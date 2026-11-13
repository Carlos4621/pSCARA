use super::*;

fn junction_with_limit(acceleration: f64, adjustments: u8) -> Result<(f64, f64), PlannerError> {
    calculate_with_rounding_limit(
        JointAngles::new(-1.0, -1.0).unwrap(),
        JointAngles::new(0.0, 0.0).unwrap(),
        JointAngles::new(1.0, 1.0).unwrap(),
        &JointVelocityLimits::new(f64::MAX, f64::MAX).unwrap(),
        &JointAccelerationLimits::new(acceleration, acceleration).unwrap(),
        adjustments,
    )
}

#[test]
fn a_feasible_theoretical_candidate_needs_no_rounding_adjustment() {
    assert_eq!(junction_with_limit(2.0, 0), Ok((2.0, 2.0)));
}

#[test]
fn exhausting_the_adjustment_budget_returns_numerical_failure() {
    let theoretical = sqrt_product_ratio(&[2.0, 1.0], &[]);
    // Establish that this is a real rounding failure of the scalar contract,
    // rather than relying on a mocked profile or an arbitrary small speed.
    assert!(matches!(
        MotionProfile::with_boundary_speeds(0.0, 1.0, 1.0, f64::MAX, 0.0, theoretical),
        Err(PlannerError::InfeasibleMotion)
    ));
    assert_eq!(
        junction_with_limit(1.0, 0),
        Err(PlannerError::NumericalFailure)
    );
}

#[test]
fn a_candidate_that_becomes_feasible_on_the_last_allowed_adjustment_is_accepted() {
    let expected = sqrt_product_ratio(&[2.0, 1.0], &[]).next_down();
    let speeds = junction_with_limit(1.0, 1).unwrap();
    assert_eq!(speeds, (expected, expected));
    assert!(MotionProfile::with_boundary_speeds(0.0, 1.0, 1.0, f64::MAX, 0.0, speeds.0).is_ok());
    assert!(MotionProfile::with_boundary_speeds(0.0, 1.0, 1.0, f64::MAX, speeds.1, 0.0).is_ok());
}

#[test]
fn the_public_function_never_exceeds_its_explicit_rounding_distance() {
    for acceleration in [f64::from_bits(1), 0.1, 0.5, 1.0, 3.0, 7.0, 10.0, f64::MAX] {
        let theoretical = sqrt_product_ratio(&[2.0, acceleration], &[]);
        let speeds = calculate_max_junction_progress_speeds(
            JointAngles::new(-1.0, -1.0).unwrap(),
            JointAngles::new(0.0, 0.0).unwrap(),
            JointAngles::new(1.0, 1.0).unwrap(),
            &JointVelocityLimits::new(f64::MAX, f64::MAX).unwrap(),
            &JointAccelerationLimits::new(acceleration, acceleration).unwrap(),
        )
        .unwrap();
        assert!(speeds.0 <= theoretical && speeds.1 <= theoretical);
        for speed in [speeds.0, speeds.1] {
            assert!(theoretical.to_bits() - speed.to_bits() <= 4);
        }
    }
}

#[test]
fn a_nonrounding_error_in_the_second_profile_is_not_hidden_by_rounding_in_the_first() {
    let tiny = f64::from_bits(1);
    for budget in [0, MAX_ROUNDING_ADJUSTMENTS] {
        // AB has unit progress acceleration and an upward-rounded sqrt(2)
        // boundary speed. BC's tiny velocity cap makes its duration overflow.
        assert_eq!(
            calculate_with_rounding_limit(
                JointAngles::new(-tiny, -tiny).unwrap(),
                JointAngles::new(0.0, 0.0).unwrap(),
                JointAngles::new(1.0, 1.0).unwrap(),
                &JointVelocityLimits::new(2.0 * tiny, 2.0 * tiny).unwrap(),
                &JointAccelerationLimits::new(tiny, tiny).unwrap(),
                budget,
            ),
            Err(PlannerError::NumericalFailure)
        );
    }
}
