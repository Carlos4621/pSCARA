use super::{JointToStepsConverter, MotorSteps, next_step_threshold};

#[test]
fn estimates_half_step_thresholds_in_both_directions_and_ignores_stationary_axes() {
    let converter = JointToStepsConverter::new(2.0, 4.0).unwrap();
    let position = MotorSteps {
        first: 10,
        second: -4,
    };
    assert_eq!(
        converter.next_step_thresholds(position, 1.0, -1.0),
        (Some(5.25), Some(-1.125))
    );
    assert_eq!(
        converter.next_step_thresholds(position, -1.0, 1.0),
        (Some(4.75), Some(-0.875))
    );
    assert_eq!(
        converter.next_step_thresholds(position, 0.0, -0.0),
        (None, None)
    );
}

#[test]
fn accepts_only_representable_half_steps_and_finite_angles() {
    let limit = 1_i64 << 52;
    let last = limit as f64 - 0.5;
    assert_eq!(next_step_threshold(limit - 1, 1.0, 1.0), Some(last));
    assert_eq!(next_step_threshold(1 - limit, -1.0, 1.0), Some(-last));
    for step in [limit, -limit, i64::MIN, i64::MAX] {
        assert_eq!(next_step_threshold(step, 1.0, 1.0), None);
    }
    for displacement in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(next_step_threshold(1, displacement, 1.0), None);
    }
    assert_eq!(next_step_threshold(1, 1.0, f64::from_bits(1)), None);
}
