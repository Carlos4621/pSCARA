use super::StepTimeBase;

const MAX_EXACT_INTEGER: u64 = 1 << 53;

fn assert_same_conversion(frequency: u64, ticks: u64) {
    let timebase = StepTimeBase::new(frequency).unwrap();
    let expected = timebase.ticks_to_seconds_exact(ticks).to_bits();
    let actual = timebase.ticks_to_seconds(ticks).to_bits();
    assert_eq!(actual, expected, "frequency={frequency}, ticks={ticks}");
}

#[test]
fn fast_tick_conversion_matches_exact_conversion_at_boundaries() {
    let frequencies = [
        1,
        8,
        100,
        1_000,
        100_000,
        1_000_000,
        MAX_EXACT_INTEGER - 1,
        MAX_EXACT_INTEGER,
        MAX_EXACT_INTEGER + 1,
        u64::MAX,
    ];
    let ticks = [
        0,
        1,
        2,
        3,
        100,
        1_000,
        100_000,
        1_000_000,
        MAX_EXACT_INTEGER - 1,
        MAX_EXACT_INTEGER,
        MAX_EXACT_INTEGER + 1,
        u64::MAX,
    ];

    for frequency in frequencies {
        for tick in ticks {
            assert_same_conversion(frequency, tick);
        }
    }
}

#[test]
fn fast_tick_conversion_matches_exact_conversion_for_pseudorandom_inputs() {
    let mut state = 0x243f_6a88_85a3_08d3_u64;
    let mut next = || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state
    };

    for _ in 0..8192 {
        let ticks = next();
        let frequency = next().max(1);
        let tick_shift = next() % u64::BITS as u64;
        let frequency_shift = next() % u64::BITS as u64;
        let fast_mask = MAX_EXACT_INTEGER - 1;
        assert_same_conversion(
            ((frequency & fast_mask) >> frequency_shift).max(1),
            (ticks & fast_mask) >> tick_shift,
        );
        assert_same_conversion(100_000, ticks & ((1 << 24) - 1));
        assert_same_conversion(frequency, ticks);
    }
}
