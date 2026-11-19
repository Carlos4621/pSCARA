use utilities::{UtilitiesErrors, checked_time_sumatory};

const SMALLEST: f64 = f64::from_bits(1);
const TWO_TO_54: f64 = 18_014_398_509_481_984.0;

#[test]
fn an_empty_slice_and_signed_zeros_sum_to_zero() {
    for times in [&[][..], &[0.0], &[-0.0], &[0.0, -0.0, 0.0]] {
        assert_eq!(checked_time_sumatory(times), Ok(0.0));
    }
}

#[test]
fn sums_finite_times_over_several_scales() {
    assert_eq!(checked_time_sumatory(&[0.25, 1.5, 0.0, 2.75, 4.0]), Ok(8.5));
    for time in [1e-200, 1.5, 1e200, f64::MAX] {
        assert_eq!(checked_time_sumatory(&[time]), Ok(time));
        assert_eq!(
            checked_time_sumatory(&[-0.0, time / 2.0, 0.0, time / 2.0]),
            Ok(time)
        );
    }
}

#[test]
fn preserves_representable_subnormal_times() {
    assert_eq!(checked_time_sumatory(&[SMALLEST]), Ok(SMALLEST));
    assert_eq!(
        checked_time_sumatory(&[SMALLEST, SMALLEST]),
        Ok(f64::from_bits(2))
    );
    assert_eq!(
        checked_time_sumatory(&[f64::MIN_POSITIVE.next_down(), SMALLEST]),
        Ok(f64::MIN_POSITIVE)
    );
}

#[test]
fn rejects_negative_times_even_if_other_terms_hide_them() {
    for times in [
        &[2.0, -1.0][..],
        &[-1.0],
        &[2.0, -SMALLEST, 4.0],
        &[-1.0, 2.0],
    ] {
        assert_eq!(
            checked_time_sumatory(times),
            Err(UtilitiesErrors::InvalidTime),
            "times={times:?}"
        );
    }
}

#[test]
fn rejects_nonfinite_terms_at_any_position_as_invalid_time() {
    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        for times in [
            [invalid, 1.0, 2.0],
            [1.0, invalid, 2.0],
            [1.0, 2.0, invalid],
        ] {
            assert_eq!(
                checked_time_sumatory(&times),
                Err(UtilitiesErrors::InvalidTime),
                "times={times:?}"
            );
        }
    }
}

#[test]
fn finite_terms_can_overflow_at_the_last_addition() {
    assert_eq!(
        checked_time_sumatory(&[f64::MAX, f64::MAX]),
        Err(UtilitiesErrors::TimeOverflow)
    );
}

#[test]
fn overflow_has_the_same_error_with_remaining_terms() {
    assert_eq!(
        checked_time_sumatory(&[f64::MAX, f64::MAX, 0.0]),
        Err(UtilitiesErrors::TimeOverflow)
    );
}

#[test]
fn a_positive_time_that_cannot_advance_the_sum_loses_precision() {
    assert_eq!(
        checked_time_sumatory(&[TWO_TO_54, 2.0]),
        Err(UtilitiesErrors::TimePrecisionLost)
    );
}

#[test]
fn a_large_term_can_also_absorb_the_previous_positive_total() {
    assert_eq!(
        checked_time_sumatory(&[2.0, TWO_TO_54]),
        Err(UtilitiesErrors::TimePrecisionLost)
    );
}

#[test]
fn precision_loss_is_detected_before_later_terms_change_the_total() {
    let cases = [[0.0, TWO_TO_54, 2.0, 8.0], [0.0, 2.0, TWO_TO_54, 8.0]];
    assert_eq!(
        cases.map(|times| checked_time_sumatory(&times)),
        [Err(UtilitiesErrors::TimePrecisionLost); 2]
    );
}

#[test]
fn precision_loss_is_reported_before_a_later_overflow() {
    assert_eq!(
        checked_time_sumatory(&[TWO_TO_54, 2.0, f64::MAX, f64::MAX]),
        Err(UtilitiesErrors::TimePrecisionLost)
    );
}

#[test]
fn rounding_is_allowed_when_both_positive_terms_advance_the_result() {
    let odd = TWO_TO_54.next_up();
    assert_eq!(checked_time_sumatory(&[odd, 2.0]), Ok(odd.next_up()));
    assert_eq!(checked_time_sumatory(&[2.0, odd]), Ok(odd.next_up()));
}
