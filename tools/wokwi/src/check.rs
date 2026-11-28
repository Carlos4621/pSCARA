use executor::{PioCommandWord, pio_frame_cycles};
use scara_wokwi::{
    AFTER_WINDOW_US, COMMAND_CAPACITY, Case, PIO_HZ, SYSTEM_CLOCK_HZ, cases, config,
};
use std::ops::Range;

use crate::trace::{Check, FS_PER_NS, FS_PER_SECOND, Time, Trace};

const MARKER: usize = 4;
pub const DEFAULT_NAMES: [&str; 5] = ["D0", "D1", "D2", "D3", "D4"];

#[derive(Clone, Copy)]
pub struct Wiring {
    pub step_active_high: [bool; 2],
    pub dir_positive_high: [bool; 2],
}

impl Default for Wiring {
    fn default() -> Self {
        Self {
            step_active_high: [true; 2],
            dir_positive_high: [true; 2],
        }
    }
}

impl Wiring {
    fn physical_level(self, channel: usize, high: bool) -> bool {
        let motor = channel / 2;
        let same_polarity = if channel.is_multiple_of(2) {
            self.step_active_high[motor]
        } else {
            let config = config();
            let reference = [
                config.first_positive_is_high,
                config.second_positive_is_high,
            ];
            self.dir_positive_high[motor] == reference[motor]
        };
        high == same_polarity
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Change {
    pub time: Time,
    pub channel: usize,
    pub high: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct Pulse {
    pub time: Time,
    pub width: Time,
    pub positive: bool,
    pub dir_setup: Time,
    pub dir_hold: Option<Time>,
}

pub fn pulses(trace: &Trace, motor: usize, wiring: Wiring) -> Check<Vec<Pulse>> {
    let step = &trace.signals[2 * motor];
    let dir = &trace.signals[2 * motor + 1];
    let active = wiring.step_active_high[motor];
    if step[0].high == active {
        return Err(format!(
            "Motor {} STEP activo en la primera muestra ({} fs)",
            motor + 1,
            step[0].time
        ));
    }
    let (pairs, remainder) = step[1..].as_chunks::<2>();
    let mut pulses = Vec::new();
    for pair in pairs {
        let start = pair[0].time;
        let index = dir.partition_point(|edge| edge.time <= start);
        let previous = index
            .checked_sub(1)
            .map(|index| dir[index])
            .ok_or_else(|| format!("Motor {} sin DIR para el paso en {start} fs", motor + 1))?;
        pulses.push(Pulse {
            time: start,
            width: pair[1].time - start,
            positive: previous.high == wiring.dir_positive_high[motor],
            dir_setup: start - previous.time,
            dir_hold: dir.get(index).map(|next| next.time - start),
        });
    }
    if let [unfinished] = remainder {
        return Err(format!(
            "Motor {} pulso sin cierre desde {} fs",
            motor + 1,
            unfinished.time
        ));
    }
    Ok(pulses)
}

pub fn check_limits(pulses: &[Pulse], motor: usize, tolerance: Time) -> Check<()> {
    let timing = config().timing;
    let mut previous_end = None;
    for pulse in pulses {
        let check = |name: &str, measured: Time, minimum_ns: u64| -> Check<()> {
            let minimum = Time::from(minimum_ns) * FS_PER_NS;
            if measured < minimum.saturating_sub(tolerance) {
                return Err(format!(
                    "Motor {} en {} fs: {name} de {measured} fs, mínimo {minimum} fs",
                    motor + 1,
                    pulse.time,
                ));
            }
            Ok(())
        };
        check("STEP activo", pulse.width, timing.step_high_ns())?;
        check(
            "establecimiento DIR",
            pulse.dir_setup,
            timing.dir_setup_ns(),
        )?;
        if let Some(hold) = pulse.dir_hold {
            check("mantenimiento DIR", hold, timing.dir_hold_ns())?;
        }
        if let Some(previous) = previous_end {
            check("STEP inactivo", pulse.time - previous, timing.step_low_ns())?;
        }
        previous_end = Some(pulse.time + pulse.width);
    }
    Ok(())
}

pub fn signed_steps(pulses: &[Pulse]) -> i64 {
    pulses
        .iter()
        .map(|pulse| if pulse.positive { 1 } else { -1 })
        .sum()
}

pub fn reference(case: Case, wiring: Wiring) -> Check<Vec<Change>> {
    let mut storage = [PioCommandWord::END; COMMAND_CAPACITY];
    let prepared = case
        .prepare(&mut storage)
        .map_err(|error| format!("{}: {error:?}", case.name()))?;
    let mut previous = config().initial_levels();
    let mut time = 0;
    let mut result = Vec::new();
    for word in case.delivered_words(prepared) {
        let (Some(wait), Some(levels)) = (word.wait_counter(), word.levels()) else {
            break;
        };
        let cycles =
            pio_frame_cycles(wait).map_err(|error| format!("{}: {error:?}", case.name()))?;
        time += Time::from(cycles) * (FS_PER_SECOND / Time::from(PIO_HZ));
        for channel in 0..MARKER {
            let mask = 1 << channel;
            if (previous ^ levels) & mask != 0 {
                result.push(Change {
                    time,
                    channel,
                    high: wiring.physical_level(channel, levels & mask != 0),
                });
            }
        }
        previous = levels;
    }
    Ok(result)
}

fn windows(trace: &Trace) -> Check<Vec<Range<Time>>> {
    let marker = &trace.signals[MARKER];
    if marker[0].high {
        return Err("Marcador activo en la primera muestra".into());
    }
    let (pairs, remainder) = marker[1..].as_chunks::<2>();
    let windows: Vec<_> = pairs
        .iter()
        .map(|pair| pair[0].time..pair[1].time)
        .collect();
    if let [open] = remainder {
        return Err(format!("Escenario sin cierre desde {} fs", open.time));
    }
    Ok(windows)
}

fn window_changes(trace: &Trace, window: Range<Time>, wiring: Wiring) -> Check<Vec<Change>> {
    let mut changes = Vec::new();
    let initial = config().initial_levels();
    for (channel, signal) in trace.signals[..MARKER].iter().enumerate() {
        let expected = wiring.physical_level(channel, initial & (1 << channel) != 0);
        let start = signal.partition_point(|edge| edge.time < window.start);
        let end = signal.partition_point(|edge| edge.time < window.end);
        if start == 0 || signal[start - 1].high != expected {
            return Err(format!(
                "Canal D{channel}: nivel inicial incorrecto antes de {} fs",
                window.start
            ));
        }
        for edge in &signal[start..end] {
            if edge.time == window.start {
                return Err(format!(
                    "Canal D{channel}: cambia junto al marcador en {} fs",
                    edge.time
                ));
            }
            changes.push(Change {
                time: edge.time,
                channel,
                high: edge.high,
            });
        }
    }
    changes.sort_unstable();
    Ok(changes)
}

pub fn compare_changes(
    expected: &[Change],
    actual: &[Change],
    tolerance: Time,
    quantum: Time,
) -> Check<Time> {
    let expected_start = expected.first().map_or(0, |change| change.time);
    let actual_start = actual.first().map_or(0, |change| change.time);
    let mut max_deviation = 0;
    let mut previous = None;
    for (index, (expected, actual)) in expected.iter().zip(actual).enumerate() {
        let observed_time = actual.time - actual_start;
        let expected_time = expected.time - expected_start;
        let deviation = observed_time.abs_diff(expected_time);
        if actual.channel != expected.channel
            || actual.high != expected.high
            || deviation > tolerance
        {
            return Err(format!(
                "Evento {}: D{}={} esperado a {expected_time} fs; D{}={} observado a {observed_time} fs (captura {} fs, desviación {deviation} fs)",
                index + 1,
                expected.channel,
                u8::from(expected.high),
                actual.channel,
                u8::from(actual.high),
                actual.time,
            ));
        }
        if let Some((previous_expected, previous_actual)) = previous {
            if expected.time == previous_expected && actual.time - previous_actual > quantum {
                return Err(format!(
                    "D{} en {} fs: perdió simultaneidad con el evento anterior",
                    actual.channel, actual.time
                ));
            }
            if expected.time > previous_expected && actual.time == previous_actual {
                return Err(format!(
                    "D{} en {} fs: dos instantes distintos quedaron superpuestos",
                    actual.channel, actual.time
                ));
            }
        }
        previous = Some((expected.time, actual.time));
        max_deviation = max_deviation.max(deviation);
    }
    match expected.len().cmp(&actual.len()) {
        std::cmp::Ordering::Greater => {
            let change = expected[actual.len()];
            Err(format!(
                "Falta un evento de D{} a {} fs relativos",
                change.channel,
                change.time - expected_start
            ))
        }
        std::cmp::Ordering::Less => {
            let change = actual[expected.len()];
            Err(format!(
                "Evento adicional de D{} en {} fs",
                change.channel, change.time
            ))
        }
        std::cmp::Ordering::Equal => Ok(max_deviation),
    }
}

pub fn compare(trace: &Trace, wiring: Wiring, tolerance: Time, dma: bool) -> Check<()> {
    if trace.quantum > FS_PER_SECOND / Time::from(PIO_HZ) {
        return Err(format!(
            "Resolución VCD de {} fs inferior a la resolución del PIO",
            trace.quantum
        ));
    }
    let cases = cases(dma);
    let windows = windows(trace)?;
    if windows.len() != cases.len() {
        return Err(format!(
            "Captura incompleta: {} de {} ventanas",
            windows.len(),
            cases.len()
        ));
    }
    let axes = [pulses(trace, 0, wiring)?, pulses(trace, 1, wiring)?];
    // Cada flanco de un intervalo puede desplazarse un ciclo de clk_sys
    // al ser publicado por el simulador.
    let interval_uncertainty =
        (2 * FS_PER_SECOND).div_ceil(Time::from(SYSTEM_CLOCK_HZ)) + trace.quantum;
    for (motor, pulses) in axes.iter().enumerate() {
        check_limits(pulses, motor, interval_uncertainty)?;
        let mut window = 0;
        for pulse in pulses {
            while window < windows.len() && windows[window].end <= pulse.time {
                window += 1;
            }
            if windows.get(window).is_none_or(|window| {
                pulse.time <= window.start || pulse.time + pulse.width >= window.end
            }) {
                return Err(format!(
                    "Motor {}: pulso fuera de una ventana completa en {} fs",
                    motor + 1,
                    pulse.time
                ));
            }
        }
    }
    for (&case, window) in cases.iter().zip(windows) {
        let verify = || -> Check<_> {
            if window.end - window.start
                < (Time::from(AFTER_WINDOW_US) * 1000 * FS_PER_NS).saturating_sub(trace.quantum)
            {
                return Err(format!("Ventana demasiado corta en {} fs", window.start));
            }
            let actual = window_changes(trace, window.clone(), wiring)?;
            let expected = reference(case, wiring)?;
            let deviation = compare_changes(&expected, &actual, tolerance, trace.quantum)?;
            let (counts, displacement) = case.expected_steps();
            for motor in 0..2 {
                let in_window: Vec<_> = axes[motor]
                    .iter()
                    .copied()
                    .filter(|pulse| window.contains(&pulse.time))
                    .collect();
                let measured = signed_steps(&in_window);
                if in_window.len() != counts[motor] || measured != displacement[motor] {
                    return Err(format!(
                        "Motor {}: {} pasos, desplazamiento {measured}; se esperaban {}, {}",
                        motor + 1,
                        in_window.len(),
                        counts[motor],
                        displacement[motor]
                    ));
                }
            }
            if case == Case::Cartesian {
                let endpoint = scara_wokwi::cartesian_endpoint(displacement)
                    .map_err(|error| format!("Reconstrucción cartesiana: {error:?}"))?;
                let distance = (endpoint.x() - 1.25).hypot(endpoint.y() - 0.75);
                if distance > 3.0 / 64.0 {
                    return Err(format!(
                        "Error cartesiano de {distance} fuera de la cota de cuantización"
                    ));
                }
                println!(
                    "cartesiana: mandato reconstruido ({:.6}, {:.6}), error {:.6}",
                    endpoint.x(),
                    endpoint.y(),
                    distance
                );
            }
            Ok((counts, displacement, deviation))
        };
        let (counts, displacement, deviation) =
            verify().map_err(|error| format!("{}: {error}", case.name()))?;
        println!(
            "{}: pasos {counts:?}, desplazamiento {displacement:?}, desviación máxima {deviation} fs",
            case.name()
        );
    }
    Ok(())
}

pub fn inspect(trace: &Trace, wiring: Wiring) -> Check<()> {
    println!(
        "Resolución {} fs, fin de captura {} fs",
        trace.quantum, trace.end
    );
    for motor in 0..2 {
        let pulses = pulses(trace, motor, wiring)?;
        let dir_changes = trace.signals[2 * motor + 1].len() - 1;
        println!(
            "Motor {}: {} pulsos, desplazamiento {}, {} cambios DIR",
            motor + 1,
            pulses.len(),
            signed_steps(&pulses),
            dir_changes
        );
        for (index, pulse) in pulses.iter().enumerate() {
            let direction = if pulse.positive { '+' } else { '-' };
            print!(
                "  Paso {} en {} fs, activo {} fs, DIR {direction}, setup {} fs",
                index + 1,
                pulse.time,
                pulse.width,
                pulse.dir_setup
            );
            if let Some(hold) = pulse.dir_hold {
                print!(", hold {hold} fs");
            }
            if let Some(previous) = index.checked_sub(1).map(|previous| pulses[previous]) {
                let period = pulse.time - previous.time;
                print!(
                    ", inactivo {} fs, período {period} fs, {:.3} Hz",
                    period - previous.width,
                    FS_PER_SECOND as f64 / period as f64
                );
            }
            println!();
        }
    }
    Ok(())
}
