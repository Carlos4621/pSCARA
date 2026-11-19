use core::iter::FusedIterator;

use kinematics::JointAngles;
use planner::{
    CoordinatedStepInterpolator, JointToStepsConverter, MotorStepMove, MotorSteps, StepDirection,
    StepEvent,
};

fn steps(first: i64, second: i64) -> MotorSteps {
    MotorSteps { first, second }
}

fn event(first_step: bool, second_step: bool) -> StepEvent {
    StepEvent {
        first_step,
        second_step,
    }
}

fn from_pulse_counts(first: u64, second: u64) -> CoordinatedStepInterpolator {
    let origin = steps(i64::MIN, i64::MIN);
    let target_for = |count| i64::try_from(i128::from(i64::MIN) + i128::from(count)).unwrap();
    CoordinatedStepInterpolator::new(MotorStepMove::new(
        origin,
        steps(target_for(first), target_for(second)),
    ))
}

fn assert_exhausted(iter: &mut impl FusedIterator<Item = StepEvent>) {
    assert!(iter.next().is_none());
    assert!(iter.next().is_none());
}

fn assert_prefixes(interpolator: &mut CoordinatedStepInterpolator, pulses: [u64; 2], through: u64) {
    let total = pulses[0].max(pulses[1]);
    let mut observed = [0_u128; 2];
    for index in 1..=through {
        let next = interpolator
            .next()
            .expect("movement ended before the expected event");
        assert!(
            next.first_step || next.second_step,
            "empty event at index {index}"
        );
        observed[0] += u128::from(next.first_step);
        observed[1] += u128::from(next.second_step);
        // A prefix of length k emits floor(k * pulses / total) on each axis.
        // Computing this directly avoids copying the running-error algorithm.
        let expected =
            pulses.map(|count| u128::from(index) * u128::from(count) / u128::from(total));
        assert_eq!(observed, expected, "pulses={pulses:?}, prefix={index}");
    }
}

#[test]
fn unchanged_positions_have_no_events_and_remain_exhausted() {
    let position = steps(41, -9);
    let mut interpolator = CoordinatedStepInterpolator::new(MotorStepMove::new(position, position));
    assert_exhausted(&mut interpolator);
}

#[test]
fn a_single_moving_axis_emits_all_its_pulses_in_either_direction() {
    let origin = steps(10, -4);
    for (target, expected) in [
        (steps(15, -4), event(true, false)),
        (steps(5, -4), event(true, false)),
        (steps(10, 1), event(false, true)),
        (steps(10, -9), event(false, true)),
    ] {
        let mut interpolator = CoordinatedStepInterpolator::new(MotorStepMove::new(origin, target));
        assert_eq!(interpolator.by_ref().collect::<Vec<_>>(), [expected; 5]);
        assert_exhausted(&mut interpolator);
    }
}

#[test]
fn equal_nonzero_pulse_counts_step_together_even_in_opposite_directions() {
    let movement = MotorStepMove::new(steps(10, 20), steps(13, 17));
    let mut interpolator = CoordinatedStepInterpolator::new(movement);
    for _ in 0..3 {
        assert_eq!(interpolator.next(), Some(event(true, true)));
    }
    assert_exhausted(&mut interpolator);
}

#[test]
fn a_five_to_two_move_pulses_the_secondary_axis_on_events_three_and_five() {
    for (first, second, primary_only) in [(5, 2, event(true, false)), (2, 5, event(false, true))] {
        let mut interpolator = from_pulse_counts(first, second);
        let both = event(true, true);
        let expected = [primary_only, primary_only, both, primary_only, both];
        assert_eq!(interpolator.by_ref().collect::<Vec<_>>(), expected);
        assert_exhausted(&mut interpolator);
    }
}

#[test]
fn every_prefix_matches_the_expected_pulse_counts_across_a_grid_of_ratios() {
    for first in 0..=32 {
        for second in 0..=32 {
            let mut interpolator = from_pulse_counts(first, second);
            assert_prefixes(&mut interpolator, [first, second], first.max(second));
            assert_exhausted(&mut interpolator);
        }
    }
}

#[test]
fn resuming_after_taking_or_skipping_events_preserves_the_interleaving() {
    let mut interpolator = from_pulse_counts(8, 3);
    let primary = event(true, false);
    let both = event(true, true);
    assert_eq!(
        interpolator.by_ref().take(2).collect::<Vec<_>>(),
        [primary, primary]
    );

    let iter: &mut dyn FusedIterator<Item = StepEvent> = &mut interpolator;
    // Skip event 3, which pulses both axes; event 4 pulses only the primary.
    assert_eq!(iter.nth(1), Some(primary));
    assert_eq!(iter.next(), Some(primary));
    assert_eq!(iter.next(), Some(both));
    assert_eq!(iter.next(), Some(primary));
    assert_eq!(iter.next(), Some(both));
    assert_exhausted(&mut interpolator);
}

#[test]
fn prefixes_at_full_i64_displacements_handle_an_accumulator_wider_than_u64() {
    let max = u64::MAX;
    for (first, second) in [
        (max, max),
        (max, max - 1),
        (max - 1, max),
        (max, max / 2),
        (1, max),
    ] {
        let mut interpolator = from_pulse_counts(first, second);
        // For (max, max - 1), even the second update exceeds u64::MAX before subtraction.
        assert_prefixes(&mut interpolator, [first, second], 64);
    }
}

#[test]
fn interpolated_pulses_and_motor_directions_reach_the_converted_target() {
    let converter = JointToStepsConverter::new(4.0, 8.0).unwrap();
    for (from, to) in [
        ([1.0, -0.5], [2.25, -0.75]),
        ([2.25, -0.75], [1.0, -0.5]),
        ([0.5, -0.5], [0.5, 0.25]),
    ] {
        let convert = |[first, second]: [f64; 2]| {
            converter
                .convert(JointAngles::new(first, second).unwrap())
                .unwrap()
        };
        let origin = convert(from);
        let target = convert(to);
        let movement = MotorStepMove::new(origin, target);
        let signed_step = |direction| match direction {
            StepDirection::Positive => 1,
            StepDirection::Negative => -1,
        };
        let mut observed = origin;
        for next in CoordinatedStepInterpolator::new(movement) {
            if next.first_step {
                observed.first += signed_step(movement.first().direction);
            }
            if next.second_step {
                observed.second += signed_step(movement.second().direction);
            }
        }
        assert_eq!(observed, target);
    }
}
