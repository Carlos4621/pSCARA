mod check;
mod trace;

use check::{DEFAULT_NAMES, Wiring, compare, inspect};
use std::{env, fs::File, io::BufReader, process::ExitCode};
use trace::{Check, FS_PER_NS, Time, Trace};

fn pair(value: &str) -> Check<[bool; 2]> {
    fn polarity(value: &str) -> Check<bool> {
        match value {
            "high" => Ok(true),
            "low" => Ok(false),
            _ => Err(format!(
                "Polaridad inválida {value}, se esperaba high o low"
            )),
        }
    }
    let (first, second) = value
        .split_once(',')
        .ok_or("Indica ambas polaridades separadas por coma")?;
    Ok([polarity(first)?, polarity(second)?])
}

fn run(mut args: impl Iterator<Item = String>) -> Check<()> {
    let usage = "Uso: scara-wokwi compare|inspect CAPTURA.vcd [--profile pio|dma] [--signals STEP1,DIR1,STEP2,DIR2,LED] [--step-active high,high] [--dir-positive high,high] [--tolerance-ns 1000]";
    let mode = args.next().ok_or(usage)?;
    if mode == "--help" {
        println!("{usage}");
        return Ok(());
    }
    if !matches!(mode.as_str(), "compare" | "inspect") {
        return Err(usage.into());
    }
    let path = args.next().ok_or(usage)?;
    let mut names = DEFAULT_NAMES.map(str::to_owned);
    let mut wiring = Wiring::default();
    let mut tolerance_ns = 1_000u64;
    let mut dma = false;
    while let Some(flag) = args.next() {
        let value = args
            .next()
            .ok_or_else(|| format!("Falta el valor de {flag}"))?;
        match flag.as_str() {
            "--signals" => {
                names = value
                    .split(',')
                    .map(str::to_owned)
                    .collect::<Vec<_>>()
                    .try_into()
                    .map_err(|_| "Se requieren cinco nombres de señales")?;
            }
            "--step-active" => wiring.step_active_high = pair(&value)?,
            "--dir-positive" => wiring.dir_positive_high = pair(&value)?,
            "--profile" if mode == "compare" => {
                dma = match value.as_str() {
                    "pio" => false,
                    "dma" => true,
                    _ => return Err(format!("Perfil desconocido {value}, se esperaba pio o dma")),
                }
            }
            "--tolerance-ns" if mode == "compare" => {
                tolerance_ns = value.parse().map_err(|_| "Tolerancia inválida en ns")?;
            }
            _ => return Err(format!("Opción incompatible o desconocida {flag}")),
        }
    }
    let file = File::open(&path).map_err(|error| format!("{path}: {error}"))?;
    let trace = Trace::read(BufReader::new(file), names.each_ref().map(String::as_str))?;
    if mode == "compare" {
        compare(
            &trace,
            wiring,
            Time::from(tolerance_ns) * FS_PER_NS + trace.quantum,
            dma,
        )
    } else {
        inspect(&trace, wiring)
    }
}

fn main() -> ExitCode {
    match run(env::args().skip(1)) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests;
