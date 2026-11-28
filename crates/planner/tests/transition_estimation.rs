#![cfg(feature = "transition-estimation")]

use kinematics::JointAngles;
use planner::{
    CoordinatedJointMotionProfile, JointAccelerationLimits, JointToStepsConverter,
    JointVelocityLimits, MotorStepMove, StepTimeBase, TimedCoordinatedJointProfile,
    TransitionEstimationStatistics, find_first_transition_tick_exponential,
    find_first_transition_tick_hinted,
};

fn constant_speed_profile(start: [f64; 2], end: [f64; 2]) -> TimedCoordinatedJointProfile {
    let motion = CoordinatedJointMotionProfile::new(
        JointAngles::new(start[0], start[1]).unwrap(),
        JointAngles::new(end[0], end[1]).unwrap(),
        &JointVelocityLimits::new(1.0, 1.0).unwrap(),
        &JointAccelerationLimits::new(1.0, 1.0).unwrap(),
        1.0,
        1.0,
    )
    .unwrap();
    TimedCoordinatedJointProfile::new(2.0, motion).unwrap()
}

#[test]
fn estimates_the_first_motor_to_change_and_verifies_signed_rounding_ties() {
    let timebase = StepTimeBase::new(8).unwrap();
    for (start, end, second_scale, estimated, actual) in [
        ([0.0, 0.0], [1.0, -1.0], 1.0, 20, 20),
        ([0.0, 0.0], [1.0, -1.0], 2.0, 18, 18),
        ([-1.0, 0.0], [0.0, 0.0], 1.0, 20, 21),
        ([1.0, 0.0], [0.0, 0.0], 1.0, 20, 21),
    ] {
        let profile = constant_speed_profile(start, end);
        let converter = JointToStepsConverter::new(1.0, second_scale).unwrap();
        let sample =
            |tick| converter.convert(profile.position_at(timebase.ticks_to_seconds(tick))?);
        let start_tick = 16;
        let end_tick = 24;
        let position = sample(start_tick).unwrap();
        let hint = profile.estimate_next_step_tick(&converter, position, timebase);
        assert_eq!(hint, Some(estimated));
        let event = Some((
            actual,
            MotorStepMove::new(position, sample(actual).unwrap()),
        ));
        assert_eq!(
            find_first_transition_tick_hinted(start_tick, end_tick, position, hint, sample),
            Ok(event)
        );
        assert_eq!(
            find_first_transition_tick_exponential(start_tick, end_tick, position, sample),
            Ok(event)
        );
    }
}

#[test]
fn returns_no_estimate_when_threshold_or_clock_conversion_is_out_of_range() {
    let profile = constant_speed_profile([0.0, 0.0], [1.0, 0.0]);
    let converter = JointToStepsConverter::new(1.0, 1.0).unwrap();
    let initial = converter
        .convert(profile.position_at(2.0).unwrap())
        .unwrap();
    let final_position = converter
        .convert(profile.position_at(3.0).unwrap())
        .unwrap();
    assert_eq!(
        profile.estimate_next_step_tick(&converter, initial, StepTimeBase::new(u64::MAX).unwrap()),
        None
    );
    assert_eq!(
        profile.estimate_next_step_tick(&converter, final_position, StepTimeBase::new(8).unwrap()),
        None
    );
}

#[test]
fn preserves_complete_events_for_triangular_trapezoidal_and_asymmetric_profiles() {
    let converter = JointToStepsConverter::new(2037.0, 2037.0).unwrap();
    let timebase = StepTimeBase::new(100_000).unwrap();
    for (velocity, initial, final_speed) in [
        (0.5, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (2.0, 0.0, 0.0),
        (4.0, 0.0, 0.0),
        (1.0, 0.125, 0.25),
        (4.0, 0.25, 0.125),
    ] {
        let start = JointAngles::new(0.0, 0.0).unwrap();
        let end = JointAngles::new(1.0, -0.75).unwrap();
        let motion = CoordinatedJointMotionProfile::new(
            start,
            end,
            &JointVelocityLimits::new(velocity, velocity).unwrap(),
            &JointAccelerationLimits::new(10.0, 10.0).unwrap(),
            initial,
            final_speed,
        )
        .unwrap();
        let profile = TimedCoordinatedJointProfile::new(0.0, motion).unwrap();
        let end_time = profile.end_time().unwrap();
        let (_, end_tick) = timebase.bracketing_ticks(end_time).unwrap();
        let sample = |tick| {
            converter.convert(profile.position_at(timebase.ticks_to_seconds(tick).min(end_time))?)
        };
        let mut tick = 0;
        let mut position = sample(tick).unwrap();
        let mut stats = TransitionEstimationStatistics::default();
        let mut pulses = [0, 0];
        loop {
            let hint = profile.estimate_next_step_tick(&converter, position, timebase);
            let expected =
                find_first_transition_tick_exponential(tick, end_tick, position, sample).unwrap();
            assert_eq!(
                find_first_transition_tick_hinted(tick, end_tick, position, hint, sample),
                Ok(expected),
                "velocity={velocity}, initial={initial}, final={final_speed}, from_tick={tick}"
            );
            let Some((actual_tick, movement)) = expected else {
                break;
            };
            stats.record(actual_tick, hint);
            pulses[0] += movement.first().pulses;
            pulses[1] += movement.second().pulses;
            tick = actual_tick;
            position = sample(tick).unwrap();
        }
        assert_eq!(pulses, [2037, 1528]);
        assert_eq!(position, converter.convert(end).unwrap());
        assert_eq!(stats.estimates, stats.transitions, "{stats:?}");
        assert_eq!(
            stats.estimates_within_one_tick, stats.transitions,
            "{stats:?}"
        );
    }
}
