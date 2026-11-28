use std::fmt::Write;

use scara_wokwi::{CASES, Case, config};

use crate::{
    check::{
        Change, DEFAULT_NAMES, Wiring, check_limits, compare, compare_changes, pulses, reference,
        signed_steps,
    },
    trace::{FS_PER_NS, Time, Trace},
};

const US: Time = 1000 * FS_PER_NS;
const FIXTURE: &str = include_str!("../tests/fixtures/two-motors.vcd");

fn read(text: &str) -> Trace {
    Trace::read(text.as_bytes(), DEFAULT_NAMES).unwrap()
}

#[test]
fn parses_multiple_scales_and_signal_paths() {
    let fine = FIXTURE
        .lines()
        .map(|line| {
            if line.starts_with("$timescale") {
                "$timescale 10 ns $end".to_owned()
            } else if let Some(tick) = line.strip_prefix('#') {
                format!("#{}", tick.parse::<u32>().unwrap() * 100)
            } else {
                line.to_owned()
            }
        })
        .collect::<Vec<_>>()
        .join("\n");
    let fine = Trace::read(fine.as_bytes(), ["logic.D0", "D1", "D2", "D3", "D4"]).unwrap();
    let coarse = read(FIXTURE);
    assert_eq!(coarse.quantum, US);
    assert_eq!(fine.quantum, 10 * FS_PER_NS);
    assert_eq!(fine.signals, coarse.signals);
    assert_eq!(fine.end, 180 * US);
}

#[test]
fn measures_width_period_simultaneity_setup_hold_and_signed_steps() {
    let trace = read(FIXTURE);
    let first = pulses(&trace, 0, Wiring::default()).unwrap();
    let second = pulses(&trace, 1, Wiring::default()).unwrap();
    assert_eq!(first.len(), 2);
    assert_eq!(second.len(), 1);
    assert_eq!(first[0].time, second[0].time);
    assert_eq!(first[0].width, 10 * US);
    assert_eq!(first[1].time - first[0].time, 40 * US);
    assert_eq!(first[1].time - first[0].time - first[0].width, 30 * US);
    assert_eq!(first[0].dir_hold, Some(25 * US));
    assert_eq!(first[1].dir_setup, 15 * US);
    assert!(first[0].positive);
    assert!(!first[1].positive);
    assert_eq!(signed_steps(&first), 0);
    assert_eq!(signed_steps(&second), -1);
    check_limits(&first, 0, 0).unwrap();
}

#[test]
fn accepts_active_low_steps_and_positive_low_direction() {
    let inverted = FIXTURE
        .lines()
        .map(|line| match line {
            "0a" => "1a",
            "1a" => "0a",
            _ => line,
        })
        .collect::<Vec<_>>()
        .join("\n");
    let wiring = Wiring {
        step_active_high: [false, true],
        dir_positive_high: [false, true],
    };
    let first = pulses(&read(&inverted), 0, wiring).unwrap();
    assert_eq!(first[0].width, 10 * US);
    assert!(!first[0].positive);
    assert!(first[1].positive);
}

#[test]
fn identifies_missing_extra_and_misaligned_events() {
    let a = Change {
        time: 20 * US,
        channel: 0,
        high: true,
    };
    let b = Change {
        time: 30 * US,
        channel: 0,
        high: false,
    };
    let shifted_a = Change {
        time: 120 * US,
        ..a
    };
    let shifted_b = Change {
        time: 130 * US,
        ..b
    };
    assert_eq!(
        compare_changes(&[a, b], &[shifted_a, shifted_b], 0, 0),
        Ok(0)
    );
    assert!(
        compare_changes(&[a, b], &[shifted_a], 0, 0)
            .unwrap_err()
            .contains("Falta")
    );
    assert!(
        compare_changes(&[a], &[shifted_a, shifted_b], 0, 0)
            .unwrap_err()
            .contains("adicional")
    );
    let late = Change {
        time: 132 * US,
        ..shifted_b
    };
    assert!(
        compare_changes(&[a, b], &[shifted_a, late], US, 0)
            .unwrap_err()
            .contains("desviación")
    );
}

#[test]
fn rejects_lost_simultaneity_and_collapsed_distinct_times() {
    let a = Change {
        time: 100 * US,
        channel: 0,
        high: true,
    };
    let b = Change { channel: 2, ..a };
    let later = Change {
        time: b.time + 100 * FS_PER_NS,
        ..b
    };
    assert!(
        compare_changes(&[a, b], &[a, later], US, FS_PER_NS)
            .unwrap_err()
            .contains("simultaneidad")
    );
    assert!(
        compare_changes(&[a, later], &[a, b], US, FS_PER_NS)
            .unwrap_err()
            .contains("superpuestos")
    );
}

#[test]
fn rejects_electrical_timing_violations() {
    let examples = [
        (FIXTURE.replace("#30", "#29"), "STEP activo"),
        (
            FIXTURE
                .replace("#60", "#38")
                .replace("#45", "#35")
                .replace("#70", "#49"),
            "establecimiento",
        ),
    ];
    for (fixture, cause) in examples {
        let trace = read(&fixture);
        assert!(
            check_limits(&pulses(&trace, 0, Wiring::default()).unwrap(), 0, 0)
                .unwrap_err()
                .contains(cause)
        );
    }
    let trace = read(&FIXTURE.replace("#30\n0a\n0c\n#45\n0b", "#25\n0b\n#30\n0a\n0c"));
    assert!(
        check_limits(&pulses(&trace, 0, Wiring::default()).unwrap(), 0, 0)
            .unwrap_err()
            .contains("mantenimiento")
    );
    let trace = read(&FIXTURE.replace("#45\n0b\n#60", "#31\n#35"));
    assert!(
        check_limits(&pulses(&trace, 0, Wiring::default()).unwrap(), 0, 0)
            .unwrap_err()
            .contains("STEP inactivo")
    );
}

#[test]
fn rejects_unknown_invalid_and_incomplete_capture() {
    for replacement in ["Xa", "za"] {
        let unknown = FIXTURE.replace("1a", replacement);
        let error = Trace::read(unknown.as_bytes(), DEFAULT_NAMES)
            .err()
            .unwrap();
        assert!(error.contains("D0: estado"), "{error}");
    }
    let invalid = FIXTURE.replace("$timescale 1 us", "$timescale ancho");
    assert!(Trace::read(invalid.as_bytes(), DEFAULT_NAMES).is_err());
    assert!(Trace::read(FIXTURE.replace("#60", "#35").as_bytes(), DEFAULT_NAMES).is_err());
    let partial = read(&FIXTURE.replace("#70\n0a\n", ""));
    assert!(
        pulses(&partial, 0, Wiring::default())
            .unwrap_err()
            .contains("sin cierre")
    );
    let incomplete = read(&FIXTURE.replace("#180\n0e", "#180"));
    assert!(
        compare(&incomplete, Wiring::default(), US, true)
            .unwrap_err()
            .contains("sin cierre")
    );
    assert!(
        compare(&read(FIXTURE), Wiring::default(), US, true)
            .unwrap_err()
            .contains("1 de 11")
    );
}

#[test]
fn first_axis_matches_independent_cruise_calculation() {
    let changes = reference(Case::First, Wiring::default()).unwrap();
    let rises: Vec<_> = changes
        .iter()
        .filter(|change| change.channel == 0 && change.high)
        .map(|change| change.time)
        .collect();
    // v=12.5 rad/s, a=2500 rad/s². La rampa recorre 1/32 rad en 5 ms.
    // Umbrales de cuantización de 8 pasos/rad: 1/16, 3/16 y 5/16 rad.
    // DIR en el tick cero añade los seis ciclos de la primera trama.
    // El muestreo numérico puede detectar el cruce un tick del planner después.
    for (&rise, nominal_us) in rises.iter().zip([7_506, 17_506, 27_506]) {
        assert!((nominal_us * US..=(nominal_us + 10) * US).contains(&rise));
    }
    assert_eq!(rises.len(), 3);
    for &start in &rises {
        assert!(changes.contains(&Change {
            time: start + 10 * US,
            channel: 0,
            high: false
        }));
    }
    assert!(!changes.iter().any(|change| change.channel == 2));
}

#[test]
fn coordinated_move_has_independent_simultaneous_steps_and_ramp_has_three_phases() {
    let both = reference(Case::Both, Wiring::default()).unwrap();
    let joint_rises = both
        .windows(2)
        .filter(|pair| {
            pair[0].channel == 0
                && pair[1].channel == 2
                && pair[0].high
                && pair[1].high
                && pair[0].time == pair[1].time
        })
        .count();
    assert_eq!(joint_rises, 2);
    let ramp = reference(Case::Ramp, Wiring::default()).unwrap();
    let rises: Vec<_> = ramp
        .iter()
        .filter(|change| change.channel == 0 && change.high)
        .map(|change| change.time)
        .collect();
    let intervals: Vec<_> = rises
        .windows(2)
        .map(|window| window[1] - window[0])
        .collect();
    assert_eq!(rises.len(), 16);
    assert!(intervals[0] > intervals[1]);
    assert_eq!(intervals[5], 1_250 * US);
    assert_eq!(intervals[6], intervals[5]);
    assert!(intervals[14] > intervals[13]);
}

#[test]
fn compares_all_scenarios_in_a_generated_fixture() {
    let header = FIXTURE.split("$dumpvars").next().unwrap();
    let mut text = format!("{header}$dumpvars\n0a\n0b\n0c\n1d\n0e\n$end\n");
    let mut start = 100;
    let initial = config().initial_levels();
    let names = ['a', 'b', 'c', 'd'];
    let mut levels = initial;
    for case in CASES {
        if case == Case::Starved {
            compare(&read(&text), Wiring::default(), US, false).unwrap();
        }
        let changes = reference(case, Wiring::default()).unwrap();
        writeln!(text, "#{}\n1e", start).unwrap();
        for change in &changes {
            let bit = 1 << change.channel;
            if change.high {
                levels |= bit;
            } else {
                levels &= !bit;
            }
            writeln!(
                text,
                "#{}\n{}{}",
                start + 10 + change.time / US,
                u8::from(change.high),
                names[change.channel]
            )
            .unwrap();
        }
        let end = start + 200 + changes.last().map_or(0, |change| change.time / US);
        writeln!(text, "#{end}\n0e\n#{}", end + 20).unwrap();
        for (channel, &name) in names.iter().enumerate() {
            let bit = 1 << channel;
            if (initial ^ levels) & bit != 0 {
                writeln!(text, "{}{name}", u8::from(initial & bit != 0)).unwrap();
            }
        }
        start = end + 200;
        levels = initial;
    }
    compare(&read(&text), Wiring::default(), US, true).unwrap();
}
