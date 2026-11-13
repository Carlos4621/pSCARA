use kinematics::{JointAngles, KinematicsError};
use planner::{CoordinatedJointMotionProfile, PlannerError};

fn same_direction(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> Result<bool, PlannerError> {
    let angles = |q: [f64; 2]| JointAngles::new(q[0], q[1]).unwrap();
    CoordinatedJointMotionProfile::are_segments_same_direction(angles(a), angles(b), angles(c))
}

// For chaining at nonzero speed, direction means d2 = k*d1 with k > 0.
// Nonzero vectors in the same quadrant alone do not satisfy that condition.
#[test]
fn positive_proportional_displacements_have_the_same_direction() {
    for first_sign in [-1.0, 1.0] {
        for second_sign in [-1.0, 1.0] {
            for factor in [0.25, 1.0, 4.0] {
                let b = [2.0 * first_sign, second_sign];
                let c = [b[0] * (1.0 + factor), b[1] * (1.0 + factor)];
                assert_eq!(same_direction([0.0; 2], b, c), Ok(true));
            }
        }
    }
}

#[test]
fn negative_proportional_displacements_have_opposite_directions() {
    for factor in [0.25, 1.0, 4.0] {
        assert_eq!(
            same_direction([0.0; 2], [2.0, -1.0], [2.0 - 2.0 * factor, -1.0 + factor]),
            Ok(false)
        );
    }
}

#[test]
fn a_corner_with_both_displacements_in_the_same_quadrant_is_not_parallel() {
    // d1=(2,-1), d2=(4,-1): component ratios .5 and 1 disagree.
    assert_eq!(
        same_direction([0.0, 0.0], [2.0, -1.0], [6.0, -2.0]),
        Ok(false)
    );
}

#[test]
fn perpendicular_displacements_do_not_have_the_same_direction() {
    // d1=(2,1), d2=(1,-2): dot product is zero.
    assert_eq!(
        same_direction([0.0, 0.0], [2.0, 1.0], [3.0, -1.0]),
        Ok(false)
    );
}

#[test]
fn an_obtuse_turn_does_not_have_the_same_direction() {
    // d1=(2,1), d2=(.25,-2): dot product is negative.
    assert_eq!(
        same_direction([0.0, 0.0], [2.0, 1.0], [2.25, -1.0]),
        Ok(false)
    );
}

#[test]
fn first_joint_may_remain_stationary_in_parallel_segments() {
    for sign in [-1.0, 1.0] {
        assert_eq!(
            same_direction([3.0, 0.0], [3.0, sign], [3.0, 3.0 * sign]),
            Ok(true)
        );
    }
}

#[test]
fn second_joint_may_remain_stationary_in_parallel_segments() {
    for sign in [-1.0, 1.0] {
        assert_eq!(
            same_direction([0.0, -3.0], [sign, -3.0], [3.0 * sign, -3.0]),
            Ok(true)
        );
    }
}

#[test]
fn axis_aligned_reversals_have_opposite_directions() {
    for (a, b, c) in [
        ([3.0, 0.0], [3.0, 1.0], [3.0, -1.0]),
        ([0.0, -3.0], [1.0, -3.0], [-1.0, -3.0]),
    ] {
        assert_eq!(same_direction(a, b, c), Ok(false));
    }
}

#[test]
fn a_turn_between_different_moving_joints_is_not_parallel() {
    assert_eq!(
        same_direction([0.0, 0.0], [1.0, 0.0], [1.0, 1.0]),
        Ok(false)
    );
}

#[test]
fn reversing_the_entire_chain_preserves_the_direction_relationship() {
    let (a, b, c) = ([0.0, 0.0], [3.0, 1.0], [4.0, -1.0]);
    // Both vectors change sign and exchange places: parallelism cannot change.
    assert_eq!(same_direction(a, b, c), same_direction(c, b, a));
}

#[test]
fn translation_preserves_the_direction_relationship() {
    for c in [[6.0, -3.0], [6.0, -2.0], [-2.0, 1.0]] {
        assert_eq!(
            same_direction([0.0, 0.0], [2.0, -1.0], c),
            same_direction([32.0, -64.0], [34.0, -65.0], [32.0 + c[0], -64.0 + c[1]])
        );
    }
}

#[test]
fn exchanging_joints_preserves_the_direction_relationship() {
    for c in [[6.0, -3.0], [6.0, -2.0], [-2.0, 1.0]] {
        assert_eq!(
            same_direction([0.0, 0.0], [2.0, -1.0], c),
            same_direction([0.0, 0.0], [-1.0, 2.0], [c[1], c[0]])
        );
    }
}

#[test]
fn parallel_segments_are_recognized_across_extreme_common_scales() {
    for exponent in [-1000, -500, 0, 500, 1000] {
        let scale = 2.0_f64.powi(exponent);
        assert_eq!(
            same_direction([0.0, 0.0], [scale, -scale], [3.0 * scale, -3.0 * scale]),
            Ok(true)
        );
    }
}

#[test]
fn parallel_segments_do_not_become_nonparallel_when_component_ratios_underflow() {
    let tiny = f64::from_bits(1);
    let large = f64::MAX / 2.0;
    assert_eq!(
        same_direction([-tiny, -tiny], [0.0; 2], [large, large]),
        Ok(true)
    );
}

#[test]
fn parallel_segments_do_not_fail_when_component_ratios_overflow() {
    let tiny = f64::from_bits(1);
    let large = f64::MAX / 2.0;
    assert_eq!(
        same_direction([-large, -large], [0.0; 2], [tiny, tiny]),
        Ok(true)
    );
}

#[test]
fn nonparallel_vectors_are_not_accepted_just_because_they_are_nearly_parallel() {
    // There is no tolerance parameter; these representable vectors are distinct.
    assert_eq!(
        same_direction([-1.0, -1.0], [0.0; 2], [1.0, 1.0_f64.next_up()]),
        Ok(false)
    );
}

#[test]
fn subtraction_overflow_is_propagated_from_either_segment_and_joint() {
    let large = f64::MAX;
    for (a, b, c) in [
        ([-large, 0.0], [large, 1.0], [large, 2.0]),
        ([0.0, -large], [1.0, large], [2.0, large]),
        ([large, 0.0], [large, 1.0], [-large, 2.0]),
        ([0.0, large], [1.0, large], [2.0, -large]),
    ] {
        assert_eq!(
            same_direction(a, b, c),
            Err(PlannerError::Kinematics(KinematicsError::InvalidPoint))
        );
    }
}

#[test]
fn zero_displacement_never_reports_a_shared_direction() {
    // Zero vectors have no direction. This checks that invariant without
    // prescribing an as-yet undocumented error policy for degenerate segments.
    for (a, b, c) in [
        ([0.0; 2], [0.0; 2], [1.0; 2]),
        ([0.0; 2], [1.0; 2], [1.0; 2]),
        ([0.0, -0.0], [-0.0, 0.0], [0.0, -0.0]),
    ] {
        assert!(!matches!(same_direction(a, b, c), Ok(true)));
    }
}

#[test]
fn rounded_equal_cross_products_do_not_hide_a_nonzero_determinant() {
    let n = 2.0_f64.powi(27);
    // n*n - (n-1)*(n+1) = 1 exactly, although both f64 products round
    // to n*n. Normalizing the vectors must not erase this difference either.
    assert_eq!(n * n, (n - 1.0) * (n + 1.0));
    assert_eq!(
        same_direction([-n, -(n - 1.0)], [0.0; 2], [n + 1.0, n]),
        Ok(false)
    );
}

#[test]
fn underflowed_determinant_does_not_hide_a_turn_with_nonzero_dot_product() {
    let x = 2.0_f64.powi(-500);
    let y = 2.0_f64.powi(-1000);
    assert_eq!(x * y, 0.0);
    assert!(x * x > 0.0);
    assert_eq!(same_direction([-x, -y], [0.0; 2], [x, 2.0 * y]), Ok(false));
}

#[test]
fn skinny_nonparallel_vectors_remain_distinct_after_normalization_would_underflow() {
    let large = f64::MAX;
    let tiny = f64::from_bits(1);
    assert_eq!(tiny / large, 0.0);
    assert_eq!(
        same_direction([-large, -tiny], [0.0; 2], [large, 2.0 * tiny]),
        Ok(false)
    );
}

#[test]
fn axis_aligned_directions_survive_subnormal_and_maximum_displacements() {
    for scale in [f64::from_bits(1), f64::MIN_POSITIVE, 1.0, f64::MAX] {
        for (a, c) in [
            ([-scale, 0.0], [scale, -0.0]),
            ([0.0, -scale], [-0.0, scale]),
        ] {
            assert_eq!(same_direction(a, [0.0; 2], c), Ok(true));
            assert_eq!(same_direction(a, [0.0; 2], a), Ok(false));
        }
    }
}

#[test]
fn all_small_integer_displacements_match_the_exact_geometric_oracle() {
    for x1 in -3_i32..=3 {
        for y1 in -3_i32..=3 {
            for x2 in -3_i32..=3 {
                for y2 in -3_i32..=3 {
                    let expected = x1 * y2 == y1 * x2 && x1 * x2 + y1 * y2 > 0;
                    let a = [-f64::from(x1), -f64::from(y1)];
                    let c = [f64::from(x2), f64::from(y2)];
                    assert_eq!(
                        same_direction(a, [0.0; 2], c),
                        Ok(expected),
                        "d1=({x1},{y1}), d2=({x2},{y2})"
                    );
                }
            }
        }
    }
}

#[test]
fn subtraction_overflow_is_rejected_even_when_the_other_segment_is_stationary() {
    let large = f64::MAX;
    for (a, b, c) in [
        ([-large; 2], [-large; 2], [large; 2]),
        ([-large; 2], [large; 2], [large; 2]),
    ] {
        assert_eq!(
            same_direction(a, b, c),
            Err(PlannerError::Kinematics(KinematicsError::InvalidPoint))
        );
    }
}
