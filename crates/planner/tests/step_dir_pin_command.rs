use planner::{
    StepDirEvent, StepDirPinCommand, StepDirState,
    StepDirection::{Negative, Positive},
};

#[test]
fn sets_unknown_direction_and_only_updates_it_again_on_reversal() {
    let mut state = StepDirState::new(None, None);

    for (tick, direction, expected_update) in [
        (100, Positive, Some(Positive)),
        (120, Positive, None),
        (140, Positive, None),
        (160, Negative, Some(Negative)),
        (180, Negative, None),
    ] {
        let command = state.prepare(StepDirEvent {
            tick,
            first_direction: Some(direction),
            second_direction: None,
        });

        assert_eq!(
            command,
            StepDirPinCommand {
                tick,
                first_dir_update: expected_update,
                second_dir_update: None,
                first_step: true,
                second_step: false,
            }
        );
    }
}

#[test]
fn an_idle_motor_retains_its_known_direction() {
    let mut state = StepDirState::new(Some(Positive), Some(Negative));

    for (tick, first_direction, second_direction, first_step, second_step) in [
        (10, None, None, false, false),
        (20, Some(Positive), None, true, false),
        (30, None, Some(Negative), false, true),
        (40, Some(Positive), Some(Negative), true, true),
    ] {
        let command = state.prepare(StepDirEvent {
            tick,
            first_direction,
            second_direction,
        });

        assert_eq!(
            command,
            StepDirPinCommand {
                tick,
                first_dir_update: None,
                second_dir_update: None,
                first_step,
                second_step,
            }
        );
    }
}

#[test]
fn both_motors_step_and_each_direction_is_updated_independently() {
    let mut state = StepDirState::new(None, None);

    for (tick, first, second, first_dir_update, second_dir_update) in [
        (10, Positive, Negative, Some(Positive), Some(Negative)),
        (20, Negative, Negative, Some(Negative), None),
        (30, Negative, Positive, None, Some(Positive)),
        (40, Negative, Positive, None, None),
    ] {
        let command = state.prepare(StepDirEvent {
            tick,
            first_direction: Some(first),
            second_direction: Some(second),
        });

        assert_eq!(
            command,
            StepDirPinCommand {
                tick,
                first_dir_update,
                second_dir_update,
                first_step: true,
                second_step: true,
            }
        );
    }
}
