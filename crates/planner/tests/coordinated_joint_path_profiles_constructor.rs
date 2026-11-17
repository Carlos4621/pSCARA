use kinematics::JointAngles;
use planner::{
    CoordinatedJointMotionProfile, CoordinatedJointPathProfiles, JointAccelerationLimits,
    JointVelocityLimits, PlannerError, plan_joint_path_speeds,
};

type Pair = [f64; 2];

fn angles(point: Pair) -> JointAngles {
    JointAngles::new(point[0], point[1]).unwrap()
}

fn construct<const N: usize>(
    coordinates: [Pair; N],
    speeds: &[f64],
    velocity: Pair,
    acceleration: Pair,
) -> Result<(), PlannerError> {
    let points = coordinates.map(angles);
    let vl = JointVelocityLimits::new(velocity[0], velocity[1]).unwrap();
    let al = JointAccelerationLimits::new(acceleration[0], acceleration[1]).unwrap();
    CoordinatedJointPathProfiles::new(&points, speeds, &vl, &al).map(|_| ())
}

#[test]
fn at_least_two_points_are_required() {
    for speeds in [&[][..], &[0.0][..], &[0.0, 0.0][..]] {
        assert_eq!(
            construct([], speeds, [10.0; 2], [2.0; 2]),
            Err(PlannerError::InsufficientPoints)
        );
        assert_eq!(
            construct([[3.0, -2.0]], speeds, [10.0; 2], [2.0; 2]),
            Err(PlannerError::InsufficientPoints)
        );
    }
}

#[test]
fn shorter_and_longer_speed_slices_are_rejected() {
    let points = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]];
    for count in [0, 1, 2, 4, 5] {
        assert_eq!(
            construct(points, &[0.0; 5][..count], [10.0; 2], [2.0; 2]),
            Err(PlannerError::MorePointsThanSpeedsBuffer)
        );
    }
}

#[test]
fn point_and_buffer_counts_are_checked_before_speed_values() {
    assert_eq!(
        construct([], &[f64::NAN], [10.0; 2], [2.0; 2]),
        Err(PlannerError::InsufficientPoints)
    );
    assert_eq!(
        construct(
            [[0.0, 0.0], [1.0, 0.0]],
            &[f64::NAN; 3],
            [10.0; 2],
            [2.0; 2]
        ),
        Err(PlannerError::MorePointsThanSpeedsBuffer)
    );
}

#[test]
fn nonfinite_speeds_are_rejected_at_every_position() {
    let points = [0.0, 1.0, 2.0, 3.0, 4.0].map(|x| [x, 0.0]);
    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        for index in 0..points.len() {
            let mut speeds = [0.0; 5];
            speeds[index] = invalid;
            assert_eq!(
                construct(points, &speeds, [10.0; 2], [2.0; 2]),
                Err(PlannerError::InvalidBoundarySpeed),
                "index={index}, value={invalid:?}"
            );
        }
    }
}

#[test]
fn negative_speeds_are_rejected_at_every_position() {
    let points = [0.0, 1.0, 2.0, 3.0, 4.0].map(|x| [x, 0.0]);
    for invalid in [-f64::from_bits(1), -1.0, -f64::MAX] {
        for index in 0..points.len() {
            let mut speeds = [0.0; 5];
            speeds[index] = invalid;
            assert_eq!(
                construct(points, &speeds, [10.0; 2], [2.0; 2]),
                Err(PlannerError::InvalidBoundarySpeed)
            );
        }
    }
}

#[test]
fn both_endpoints_must_be_at_rest_even_for_the_smallest_positive_speed() {
    let points = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]];
    for nonzero in [f64::from_bits(1), f64::MIN_POSITIVE, 1.0, f64::MAX] {
        for speeds in [
            [nonzero, 0.0, 0.0],
            [0.0, 0.0, nonzero],
            [nonzero, 0.0, nonzero],
        ] {
            assert_eq!(
                construct(points, &speeds, [10.0; 2], [2.0; 2]),
                Err(PlannerError::InvalidBoundarySpeed)
            );
        }
    }
}

#[test]
fn signed_zero_is_accepted_at_every_boundary() {
    let points = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
    for mask in 0..16 {
        let speeds =
            core::array::from_fn::<_, 4, _>(
                |index| if mask & (1 << index) == 0 { 0.0 } else { -0.0 },
            );
        assert_eq!(construct(points, &speeds, [10.0; 2], [2.0; 2]), Ok(()));
    }
}

#[test]
fn one_nonzero_segment_with_zero_boundary_speeds_is_valid() {
    for end in [[1.0, 0.0], [0.0, -2.0], [4.0, -3.0]] {
        assert_eq!(
            construct([[0.0, 0.0], end], &[0.0, 0.0], [10.0; 2], [2.0; 2]),
            Ok(())
        );
    }
}

#[test]
fn equal_collinear_segments_accept_a_feasible_nonzero_junction_speed() {
    assert_eq!(
        construct(
            [[0.0, 0.0], [2.0, -1.0], [4.0, -2.0]],
            &[0.0, 4.0, 0.0],
            [20.0, 10.0],
            [4.0, 2.0]
        ),
        Ok(())
    );
}

#[test]
fn unequal_segment_lengths_use_segment_specific_progress_speeds() {
    assert_eq!(
        construct(
            [[0.0, 0.0], [2.0, -1.0], [6.0, -3.0]],
            &[0.0, 4.0, 0.0],
            [20.0, 10.0],
            [4.0, 2.0]
        ),
        Ok(())
    );
}

#[test]
fn stopping_at_each_junction_allows_corners_and_reversals() {
    assert_eq!(
        construct(
            [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [1.0, 0.0], [0.0, 0.0]],
            &[0.0; 5],
            [10.0; 2],
            [2.0; 2]
        ),
        Ok(())
    );
}

#[test]
fn nonzero_junction_speed_at_a_corner_is_rejected() {
    assert_eq!(
        construct(
            [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]],
            &[0.0, 1.0, 0.0],
            [10.0; 2],
            [2.0; 2]
        ),
        Err(PlannerError::DiscontinuousJointVelocity)
    );
}

#[test]
fn nonzero_junction_speed_at_a_reversal_is_rejected() {
    assert_eq!(
        construct(
            [[0.0, 0.0], [1.0, -1.0], [0.0, 0.0]],
            &[0.0, 1.0, 0.0],
            [10.0; 2],
            [2.0; 2]
        ),
        Err(PlannerError::DiscontinuousJointVelocity)
    );
}

#[test]
fn even_the_smallest_positive_corner_speed_is_not_a_stop() {
    assert_eq!(
        construct(
            [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]],
            &[0.0, f64::from_bits(1), 0.0],
            [10.0; 2],
            [2.0; 2]
        ),
        Err(PlannerError::DiscontinuousJointVelocity)
    );
}

#[test]
fn almost_parallel_segments_are_not_treated_as_exactly_parallel() {
    assert_eq!(
        construct(
            [[0.0, 0.0], [1.0, 1.0], [2.0, 2.0_f64.next_up()]],
            &[0.0, 1.0, 0.0],
            [10.0; 2],
            [2.0; 2]
        ),
        Err(PlannerError::DiscontinuousJointVelocity)
    );
}

#[test]
fn continuity_is_checked_at_the_last_interior_point_too() {
    assert_eq!(
        construct(
            [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [2.0, 1.0]],
            &[0.0, 1.0, 1.0, 0.0],
            [10.0; 2],
            [2.0; 2]
        ),
        Err(PlannerError::DiscontinuousJointVelocity)
    );
}

#[test]
fn a_stationary_joint_does_not_make_a_segment_degenerate() {
    for zero in [0.0, -0.0] {
        for points in [
            [[zero, 0.0], [-zero, 1.0], [zero, 2.0]],
            [[0.0, zero], [1.0, -zero], [2.0, zero]],
        ] {
            assert_eq!(
                construct(points, &[0.0, 2.0, 0.0], [10.0; 2], [2.0; 2]),
                Ok(())
            );
        }
    }
}

#[test]
fn duplicate_points_are_rejected_even_with_zero_junction_speed() {
    for points in [
        [[0.0, 0.0], [0.0, 0.0], [1.0, 0.0], [2.0, 0.0]],
        [[0.0, 0.0], [1.0, 0.0], [1.0, 0.0], [2.0, 0.0]],
        [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [2.0, 0.0]],
    ] {
        assert_eq!(
            construct(points, &[0.0; 4], [10.0; 2], [2.0; 2]),
            Err(PlannerError::ZeroDisplacementMotion)
        );
    }
}

#[test]
fn duplicate_points_with_positive_junction_speed_propagate_zero_displacement() {
    assert_eq!(
        construct(
            [[0.0, 0.0], [1.0, 0.0], [1.0, 0.0]],
            &[0.0, 1.0, 0.0],
            [10.0; 2],
            [2.0; 2]
        ),
        Err(PlannerError::ZeroDisplacementMotion)
    );
}

#[test]
fn overflowed_displacement_is_rejected_for_either_axis_and_direction() {
    for joint in 0..2 {
        for sign in [-1.0, 1.0] {
            let mut points = [[0.0; 2]; 2];
            points[0][joint] = sign * f64::MAX;
            points[1][joint] = -sign * f64::MAX;
            assert_eq!(
                construct(points, &[0.0; 2], [10.0; 2], [2.0; 2]),
                Err(PlannerError::NumericalFailure)
            );
        }
    }
}

#[test]
fn an_overflowed_last_segment_is_not_skipped() {
    assert_eq!(
        construct(
            [[0.0, 0.0], [f64::MAX, 0.0], [-f64::MAX, 0.0]],
            &[0.0; 3],
            [f64::MAX; 2],
            [f64::MAX; 2]
        ),
        Err(PlannerError::NumericalFailure)
    );
}

#[test]
fn a_boundary_speed_above_either_joint_velocity_cap_is_rejected() {
    for (velocity, speeds) in [
        ([2.0, 100.0], [0.0, 3.0, 0.0]),
        ([100.0, 1.0], [0.0, 3.0, 0.0]),
    ] {
        assert_eq!(
            construct(
                [[0.0, 0.0], [2.0, -1.0], [4.0, -2.0]],
                &speeds,
                velocity,
                [100.0; 2]
            ),
            Err(PlannerError::InvalidBoundarySpeed)
        );
    }
}

#[test]
fn insufficient_incoming_acceleration_distance_is_rejected() {
    assert_eq!(
        construct(
            [[0.0, 0.0], [1.0, 0.0], [9.0, 0.0]],
            &[0.0, 2.0, 0.0],
            [10.0; 2],
            [1.0; 2]
        ),
        Err(PlannerError::InfeasibleMotion)
    );
}

#[test]
fn insufficient_outgoing_braking_distance_is_rejected() {
    assert_eq!(
        construct(
            [[0.0, 0.0], [8.0, 0.0], [9.0, 0.0]],
            &[0.0, 2.0, 0.0],
            [10.0; 2],
            [1.0; 2]
        ),
        Err(PlannerError::InfeasibleMotion)
    );
}

#[test]
fn the_last_segment_must_be_feasible_after_valid_earlier_segments() {
    assert_eq!(
        construct(
            [[0.0, 0.0], [4.0, 0.0], [8.0, 0.0], [9.0, 0.0]],
            &[0.0, 1.0, 3.0, 0.0],
            [10.0; 2],
            [1.0; 2]
        ),
        Err(PlannerError::InfeasibleMotion)
    );
}

#[test]
fn acceleration_boundary_is_inclusive_but_the_next_larger_speed_is_not() {
    let points = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]];
    assert_eq!(
        construct(points, &[0.0, 2.0, 0.0], [10.0; 2], [2.0; 2]),
        Ok(())
    );
    assert_eq!(
        construct(points, &[0.0, 2.0_f64.next_up(), 0.0], [10.0; 2], [2.0; 2]),
        Err(PlannerError::InfeasibleMotion)
    );
}

#[test]
fn rounded_up_theoretical_speed_is_rejected_without_silently_adjusting_it() {
    let points = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]];
    let speed = core::f64::consts::SQRT_2;
    assert_eq!(
        construct(points, &[0.0, speed, 0.0], [10.0; 2], [1.0; 2]),
        Err(PlannerError::InfeasibleMotion)
    );
    assert_eq!(
        construct(points, &[0.0, speed.next_down(), 0.0], [10.0; 2], [1.0; 2]),
        Ok(())
    );
}

#[test]
fn normalization_overflow_is_reported_as_numerical_failure() {
    let tiny = f64::from_bits(1);
    assert!((1.0 / tiny).is_infinite());
    assert_eq!(
        construct(
            [[0.0, 0.0], [tiny, 0.0], [2.0 * tiny, 0.0]],
            &[0.0, 1.0, 0.0],
            [1.0; 2],
            [1.0; 2]
        ),
        Err(PlannerError::NumericalFailure)
    );
}

#[test]
fn positive_boundary_speed_that_underflows_during_normalization_is_rejected() {
    let tiny = f64::from_bits(1);
    assert_eq!(tiny / 2.0, 0.0);
    // The first profile preserves the requested speed, while the second
    // silently sees zero after division by its length. Both inputs are finite.
    let points = [[0.0, 0.0], [1.0, 0.0], [3.0, 0.0]];
    let vl = JointVelocityLimits::new(10.0, 10.0).unwrap();
    let al = JointAccelerationLimits::new(2.0, 2.0).unwrap();
    let first = CoordinatedJointMotionProfile::new(
        angles(points[0]),
        angles(points[1]),
        &vl,
        &al,
        0.0,
        tiny,
    )
    .unwrap();
    let second = CoordinatedJointMotionProfile::new(
        angles(points[1]),
        angles(points[2]),
        &vl,
        &al,
        tiny / 2.0,
        0.0,
    )
    .unwrap();
    assert_eq!(first.velocity_at(first.total_time()), Ok((tiny, 0.0)));
    assert_eq!(second.velocity_at(0.0), Ok((0.0, 0.0)));
    assert_eq!(
        construct(points, &[0.0, tiny, 0.0], [10.0; 2], [2.0; 2]),
        Err(PlannerError::NumericalFailure)
    );
}

#[test]
fn positive_final_speed_that_underflows_during_normalization_is_rejected() {
    // Here the first segment loses its final speed, complementing the
    // regression where the second segment loses its initial speed.
    assert_eq!(
        construct(
            [[0.0, 0.0], [2.0, 0.0], [3.0, 0.0]],
            &[0.0, f64::from_bits(1), 0.0],
            [10.0; 2],
            [2.0; 2],
        ),
        Err(PlannerError::NumericalFailure)
    );
}

#[test]
fn smallest_positive_normalized_speed_remains_valid_after_division() {
    let tiny = f64::from_bits(1);
    assert_eq!((2.0 * tiny) / 2.0, tiny);
    assert_eq!(
        construct(
            [[0.0, 0.0], [2.0, 0.0], [4.0, 0.0]],
            &[0.0, 2.0 * tiny, 0.0],
            [10.0; 2],
            [2.0; 2],
        ),
        Ok(())
    );
}

#[test]
fn representable_subnormal_progress_speed_is_preserved() {
    assert_eq!(
        construct(
            [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]],
            &[0.0, f64::from_bits(1), 0.0],
            [10.0; 2],
            [2.0; 2]
        ),
        Ok(())
    );
}

#[test]
fn subnormal_geometry_with_proportionally_scaled_limits_is_valid() {
    let tiny = f64::from_bits(1);
    assert_eq!(
        construct(
            [[0.0, 0.0], [tiny, 0.0], [2.0 * tiny, 0.0]],
            &[0.0, tiny, 0.0],
            [tiny; 2],
            [tiny; 2]
        ),
        Ok(())
    );
}

#[test]
fn large_lengths_and_speeds_normalize_without_intermediate_overflow() {
    assert_eq!(
        construct(
            [[-f64::MAX, 0.0], [0.0, 0.0], [f64::MAX, 0.0]],
            &[0.0, f64::MAX, 0.0],
            [f64::MAX; 2],
            [f64::MAX; 2]
        ),
        Ok(())
    );
}

#[test]
fn unrepresentable_progress_limits_are_propagated_as_numerical_failure() {
    assert_eq!(
        construct(
            [[0.0, 0.0], [f64::from_bits(1), 0.0]],
            &[0.0; 2],
            [1.0; 2],
            [1.0; 2]
        ),
        Err(PlannerError::NumericalFailure)
    );
}

#[test]
fn unrepresentable_segment_duration_is_propagated_as_numerical_failure() {
    assert_eq!(
        construct(
            [[0.0, 0.0], [1.0, 0.0]],
            &[0.0; 2],
            [f64::from_bits(1); 2],
            [1.0; 2]
        ),
        Err(PlannerError::NumericalFailure)
    );
}

#[test]
fn constructor_accepts_planner_output_for_corners_unequal_segments_and_reversals() {
    for coordinates in [
        [[0.0, 0.0], [2.0, -1.0], [6.0, -3.0], [12.0, -6.0]],
        [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]],
        [[0.0, 0.0], [2.0, 0.0], [0.0, 0.0], [-2.0, 0.0]],
    ] {
        let points = coordinates.map(angles);
        let vl = JointVelocityLimits::new(20.0, 10.0).unwrap();
        let al = JointAccelerationLimits::new(4.0, 2.0).unwrap();
        let mut speeds = [f64::NAN; 4];
        plan_joint_path_speeds(&points, &vl, &al, &mut speeds).unwrap();
        assert!(CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).is_ok());
    }
}

#[test]
fn reflection_translation_and_joint_exchange_preserve_constructor_acceptance() {
    let original = [[0.0, 0.0], [2.0, -1.0], [6.0, -3.0]];
    for sx in [-1.0, 1.0] {
        for sy in [-1.0, 1.0] {
            assert_eq!(
                construct(
                    original.map(|q| [32.0 + sx * q[0], -64.0 + sy * q[1]]),
                    &[0.0, 4.0, 0.0],
                    [20.0, 10.0],
                    [4.0, 2.0]
                ),
                Ok(())
            );
        }
    }
    assert_eq!(
        construct(
            original.map(|q| [q[1], q[0]]),
            &[0.0, 4.0, 0.0],
            [10.0, 20.0],
            [2.0, 4.0]
        ),
        Ok(())
    );
}

#[test]
fn constructor_agrees_with_independent_per_segment_construction_on_a_grid() {
    for incoming in 1..=4 {
        for outgoing in 1..=4 {
            for boundary in [0.0, 0.5, 1.0, 2.0, 4.0] {
                let x = f64::from(incoming);
                let end = f64::from(incoming + outgoing);
                let points = [[0.0, 0.0], [x, -x / 2.0], [end, -end / 2.0]].map(angles);
                let speeds = [0.0, boundary, 0.0];
                let vl = JointVelocityLimits::new(3.0, 1.5).unwrap();
                let al = JointAccelerationLimits::new(2.0, 1.0).unwrap();
                let expected =
                    [(0, x), (1, f64::from(outgoing))]
                        .into_iter()
                        .try_for_each(|(i, length)| {
                            CoordinatedJointMotionProfile::new(
                                points[i],
                                points[i + 1],
                                &vl,
                                &al,
                                speeds[i] / length,
                                speeds[i + 1] / length,
                            )
                            .map(|_| ())
                        });
                let actual =
                    CoordinatedJointPathProfiles::new(&points, &speeds, &vl, &al).map(|_| ());
                assert_eq!(
                    actual, expected,
                    "incoming={incoming}, outgoing={outgoing}, boundary={boundary}"
                );
            }
        }
    }
}
