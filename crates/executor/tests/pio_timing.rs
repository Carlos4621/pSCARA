use executor::{
    ExecutorError, PIO_FRAME_OVERHEAD_CYCLES, PIO_MAX_FRAME_CYCLES, PioCommandWord,
    pio_cycles_per_tick, pio_frame_cycles, pio_wait_counter,
};
use planner::StepTimeBase;

#[test]
fn converts_between_counters_and_expected_frame_cycles() {
    assert_eq!(PIO_FRAME_OVERHEAD_CYCLES, 6);
    assert_eq!(PIO_MAX_FRAME_CYCLES, 134_217_733);

    for (counter, cycles) in [
        (0, 6),
        (1, 7),
        (10, 16),
        (94, 100),
        (134_217_727, 134_217_733),
    ] {
        assert_eq!(pio_frame_cycles(counter), Ok(cycles));
        assert_eq!(pio_wait_counter(cycles), Ok(counter));
    }
}

#[test]
fn rejects_every_interval_below_the_minimum() {
    for cycles in 0..6 {
        assert_eq!(pio_wait_counter(cycles), Err(ExecutorError::DelayTooShort));
    }
}

#[test]
fn rejects_intervals_above_the_maximum_before_narrowing_the_result() {
    for cycles in [134_217_734, u64::from(u32::MAX) + 7, u64::MAX] {
        assert_eq!(
            pio_wait_counter(cycles),
            Err(ExecutorError::WaitCounterOutOfRange)
        );
    }
}

#[test]
fn rejects_wait_counters_above_the_maximum() {
    for counter in [PioCommandWord::MAX_WAIT_COUNTER + 1, u32::MAX] {
        assert_eq!(
            pio_frame_cycles(counter),
            Err(ExecutorError::WaitCounterOutOfRange)
        );
    }
}

#[test]
fn conversions_are_inverses_around_each_counter_bit_boundary() {
    for bit in 0..27 {
        let boundary = 1_u32 << bit;
        for counter in [boundary - 1, boundary, boundary + 1] {
            let cycles = u64::from(counter) + 6;

            assert_eq!(
                pio_frame_cycles(pio_wait_counter(cycles).unwrap()),
                Ok(cycles)
            );
            assert_eq!(
                pio_wait_counter(pio_frame_cycles(counter).unwrap()),
                Ok(counter)
            );
        }
    }
}

#[test]
fn returns_exact_ratios_at_and_above_six_cycles_per_tick() {
    let largest_tick_rate_with_ratio_six = u64::MAX / 6;
    for (ticks_per_second, pio_frequency_hz, expected_ratio) in [
        (1_000_000, 6_000_000, 6),
        (1_000_000, 100_000_000, 100),
        (
            largest_tick_rate_with_ratio_six,
            largest_tick_rate_with_ratio_six * 6,
            6,
        ),
        (3, u64::MAX, u64::MAX / 3),
        (1, u64::MAX, u64::MAX),
    ] {
        let timebase = StepTimeBase::new(ticks_per_second).unwrap();
        assert_eq!(
            pio_cycles_per_tick(timebase, pio_frequency_hz),
            Ok(expected_ratio)
        );
    }
}

#[test]
fn rejects_zero_fractional_and_too_small_clock_ratios() {
    for (ticks_per_second, pio_frequency_hz) in [
        (10, 0),
        (10, 9),
        (10, 10),
        (10, 50),
        (10, 59),
        (10, 61),
        (10, 99),
        (2, u64::MAX),
        (u64::MAX, u64::MAX),
    ] {
        let timebase = StepTimeBase::new(ticks_per_second).unwrap();
        assert_eq!(
            pio_cycles_per_tick(timebase, pio_frequency_hz),
            Err(ExecutorError::InvalidPioClockRatio),
            "frecuencias PIO y del planificador: {pio_frequency_hz} y {ticks_per_second}"
        );
    }
}
