use executor::{
    ExecutorError, PIO_FRAME_OVERHEAD_CYCLES, PIO_MAX_FRAME_CYCLES, PioCommandWord,
    pio_frame_cycles, pio_wait_counter,
};

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
