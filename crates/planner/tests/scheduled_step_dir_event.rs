use planner::{
    PlannerError, ScheduledDirChange, ScheduledStepDirEvent, StepDirEvent, StepDirPinCommand,
    StepDirState, StepDirTiming,
    StepDirection::{Negative, Positive},
    StepTimeBase, schedule_step_dir_command,
};

fn schedule(
    command: StepDirPinCommand,
    previous_first_step: Option<u64>,
    previous_second_step: Option<u64>,
) -> Result<ScheduledStepDirEvent, PlannerError> {
    let timing = StepDirTiming::new(1_000, 1_000, 3_000, 2_000).unwrap();
    let timebase = StepTimeBase::new(1_000_000).unwrap();
    schedule_step_dir_command(
        command,
        previous_first_step,
        previous_second_step,
        timing,
        timebase,
    )
}

#[test]
fn chooses_the_latest_tick_for_initial_setup_and_reversals() {
    let max = u64::MAX;
    for (previous, step_tick, dir_tick) in [
        (None, 10, 7),
        (Some(100), 110, 107),
        (Some(100), 105, 102),
        (Some(max - 5), max, max - 3),
    ] {
        let command = StepDirState::new(previous.map(|_| Positive), None).prepare(StepDirEvent {
            tick: step_tick,
            first_direction: Some(Negative),
            second_direction: None,
        });

        assert_eq!(
            schedule(command, previous, None),
            Ok(ScheduledStepDirEvent {
                step_tick,
                first_dir_change: Some(ScheduledDirChange {
                    tick: dir_tick,
                    direction: Negative,
                }),
                second_dir_change: None,
                first_step: true,
                second_step: false,
            }),
            "previous={previous:?}, step_tick={step_tick}"
        );
    }
}

#[test]
fn propagates_errors_from_the_direction_window() {
    let max = u64::MAX;
    for (previous, tick, error) in [
        (100, 104, PlannerError::DirChangeNotSchedulable),
        (100, 100, PlannerError::InvalidTicks),
        (max - 1, max, PlannerError::DirChangeNotSchedulable),
    ] {
        let command = StepDirState::new(Some(Positive), None).prepare(StepDirEvent {
            tick,
            first_direction: Some(Negative),
            second_direction: None,
        });

        assert_eq!(schedule(command, Some(previous), None), Err(error));
    }
}

#[test]
fn preserves_a_step_with_the_same_direction_even_inside_the_minimum_step_interval() {
    let command = StepDirState::new(Some(Positive), None).prepare(StepDirEvent {
        tick: 101,
        first_direction: Some(Positive),
        second_direction: None,
    });

    assert_eq!(
        schedule(command, Some(100), None),
        Ok(ScheduledStepDirEvent {
            step_tick: 101,
            first_dir_change: None,
            second_dir_change: None,
            first_step: true,
            second_step: false,
        })
    );
}

#[test]
fn schedules_a_change_for_the_second_motor_while_the_first_motor_is_idle() {
    let command = StepDirState::new(None, Some(Negative)).prepare(StepDirEvent {
        tick: 110,
        first_direction: None,
        second_direction: Some(Positive),
    });

    assert_eq!(
        schedule(command, None, Some(100)),
        Ok(ScheduledStepDirEvent {
            step_tick: 110,
            first_dir_change: None,
            second_dir_change: Some(ScheduledDirChange {
                tick: 107,
                direction: Positive,
            }),
            first_step: false,
            second_step: true,
        })
    );
}

#[test]
fn schedules_both_changes_using_each_motors_previous_step() {
    let command = StepDirState::new(Some(Positive), Some(Negative)).prepare(StepDirEvent {
        tick: 110,
        first_direction: Some(Negative),
        second_direction: Some(Positive),
    });

    assert_eq!(
        schedule(command, Some(100), Some(105)),
        Ok(ScheduledStepDirEvent {
            step_tick: 110,
            first_dir_change: Some(ScheduledDirChange {
                tick: 107,
                direction: Negative,
            }),
            second_dir_change: Some(ScheduledDirChange {
                tick: 107,
                direction: Positive,
            }),
            first_step: true,
            second_step: true,
        })
    );

    for (first, second) in [(100, 106), (106, 105)] {
        assert_eq!(
            schedule(command, Some(first), Some(second)),
            Err(PlannerError::DirChangeNotSchedulable)
        );
    }
}
