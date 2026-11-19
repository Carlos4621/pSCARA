use super::{CoordinatedStepInterpolator, MotorStepMove, StepEvent};
use crate::MotorSteps;

#[test]
fn the_last_event_at_u64_max_is_emitted_once_and_leaves_the_iterator_exhausted() {
    let total = u64::MAX;
    let origin = MotorSteps {
        first: i64::MIN,
        second: i64::MIN,
    };
    for secondary_pulses in [0, 1, total / 2, total - 1, total] {
        let target = MotorSteps {
            first: i64::MAX,
            second: i64::try_from(i128::from(i64::MIN) + i128::from(secondary_pulses)).unwrap(),
        };
        let mut interpolator = CoordinatedStepInterpolator::new(MotorStepMove::new(origin, target));

        // Seed the penultimate prefix with its closed-form remainder. Reaching it
        // via the public API would require iterating almost u64::MAX events.
        let previous = total - 1;
        interpolator.emitted_events = previous;
        interpolator.error =
            (u128::from(previous) * u128::from(secondary_pulses)) % u128::from(total);

        assert_eq!(
            interpolator.next(),
            Some(StepEvent {
                first_step: true,
                second_step: secondary_pulses > 0
            })
        );
        assert!(interpolator.next().is_none());
        assert!(interpolator.next().is_none());
    }
}
