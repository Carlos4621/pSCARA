use executor::{PinFrame, PinFrameEncoder};
use planner::{
    PinOperationBatch, PlannerError,
    StepDirection::{Negative, Positive},
    StepPinLevel::{High, Low},
};

#[test]
fn starts_with_step_low_and_applies_each_initial_dir_polarity() {
    for (first, second, first_positive_is_high, second_positive_is_high, levels) in [
        (Negative, Negative, true, true, 0b0000),
        (Positive, Positive, true, true, 0b1010),
        (Positive, Positive, false, true, 0b1000),
        (Positive, Positive, true, false, 0b0010),
        (Positive, Negative, false, false, 0b1000),
        (Negative, Positive, false, false, 0b0010),
    ] {
        let encoder = PinFrameEncoder::new(
            first,
            second,
            first_positive_is_high,
            second_positive_is_high,
        );

        assert_eq!(encoder.levels(), levels);
    }
}

#[test]
fn encodes_the_example_as_relative_delays_and_complete_pin_masks() {
    let mut encoder = PinFrameEncoder::new(Negative, Negative, true, true);

    for (batch, delay_ticks, levels) in [
        (
            PinOperationBatch {
                first_direction: Some(Positive),
                ..PinOperationBatch::new(10)
            },
            10,
            0b0010,
        ),
        (
            PinOperationBatch {
                first_step: Some(High),
                ..PinOperationBatch::new(13)
            },
            3,
            0b0011,
        ),
        (
            PinOperationBatch {
                first_step: Some(Low),
                ..PinOperationBatch::new(15)
            },
            2,
            0b0010,
        ),
        (
            PinOperationBatch {
                second_direction: Some(Positive),
                ..PinOperationBatch::new(20)
            },
            5,
            0b1010,
        ),
        (
            PinOperationBatch {
                first_step: Some(High),
                second_step: Some(High),
                ..PinOperationBatch::new(23)
            },
            3,
            0b1111,
        ),
        (
            PinOperationBatch {
                first_step: Some(Low),
                second_step: Some(Low),
                ..PinOperationBatch::new(25)
            },
            2,
            0b1010,
        ),
    ] {
        assert_eq!(
            encoder.encode(batch),
            Ok(PinFrame {
                delay_ticks,
                levels
            })
        );
        assert_eq!(encoder.levels(), levels);
    }
}

#[test]
fn encodes_tick_zero_and_independent_dir_polarities() {
    for (first_high, second_high, positive_levels, negative_levels) in
        [(true, false, 0b0010, 0b1000), (false, true, 0b1000, 0b0010)]
    {
        let mut encoder = PinFrameEncoder::new(Negative, Negative, first_high, second_high);
        let positive = PinOperationBatch {
            first_direction: Some(Positive),
            second_direction: Some(Positive),
            ..PinOperationBatch::new(0)
        };
        let negative = PinOperationBatch {
            first_direction: Some(Negative),
            second_direction: Some(Negative),
            ..PinOperationBatch::new(10)
        };

        assert_eq!(
            encoder.encode(positive),
            Ok(PinFrame {
                delay_ticks: 0,
                levels: positive_levels
            })
        );
        assert_eq!(
            encoder.encode(negative),
            Ok(PinFrame {
                delay_ticks: 10,
                levels: negative_levels
            })
        );
    }
}

#[test]
fn preserves_all_pin_levels_when_a_batch_leaves_them_unspecified() {
    let mut encoder = PinFrameEncoder::new(Positive, Positive, true, true);
    let rising = PinOperationBatch {
        first_step: Some(High),
        second_step: Some(High),
        ..PinOperationBatch::new(1)
    };
    assert_eq!(
        encoder.encode(rising),
        Ok(PinFrame {
            delay_ticks: 1,
            levels: 0b1111
        })
    );
    assert_eq!(
        encoder.encode(PinOperationBatch::new(4)),
        Ok(PinFrame {
            delay_ticks: 3,
            levels: 0b1111
        })
    );

    let second_falling = PinOperationBatch {
        second_step: Some(Low),
        ..PinOperationBatch::new(7)
    };
    assert_eq!(
        encoder.encode(second_falling),
        Ok(PinFrame {
            delay_ticks: 3,
            levels: 0b1011
        })
    );
}

#[test]
fn rejects_repeated_and_decreasing_ticks_without_changing_levels_or_time() {
    let mut encoder = PinFrameEncoder::new(Negative, Negative, true, true);
    let first = PinOperationBatch {
        first_direction: Some(Positive),
        second_step: Some(High),
        ..PinOperationBatch::new(10)
    };
    assert_eq!(
        encoder.encode(first),
        Ok(PinFrame {
            delay_ticks: 10,
            levels: 0b0110
        })
    );

    for tick in [10, 9] {
        let invalid = PinOperationBatch {
            first_direction: Some(Negative),
            second_direction: Some(Positive),
            first_step: Some(High),
            second_step: Some(Low),
            ..PinOperationBatch::new(tick)
        };
        assert_eq!(encoder.encode(invalid), Err(PlannerError::InvalidTicks));
        assert_eq!(encoder.levels(), 0b0110);
    }

    let next = PinOperationBatch {
        first_step: Some(High),
        ..PinOperationBatch::new(13)
    };
    assert_eq!(
        encoder.encode(next),
        Ok(PinFrame {
            delay_ticks: 3,
            levels: 0b0111
        })
    );
}

#[test]
fn encodes_delays_and_ticks_at_the_u64_limit() {
    let max = u64::MAX;
    let mut encoder = PinFrameEncoder::new(Positive, Negative, true, true);
    assert_eq!(
        encoder.encode(PinOperationBatch::new(max - 1)),
        Ok(PinFrame {
            delay_ticks: max - 1,
            levels: 0b0010
        })
    );
    assert_eq!(
        encoder.encode(PinOperationBatch {
            second_step: Some(High),
            ..PinOperationBatch::new(max)
        }),
        Ok(PinFrame {
            delay_ticks: 1,
            levels: 0b0110
        })
    );

    let mut encoder = PinFrameEncoder::new(Negative, Negative, true, true);
    assert_eq!(
        encoder.encode(PinOperationBatch::new(max)),
        Ok(PinFrame {
            delay_ticks: max,
            levels: 0
        })
    );
}
