use std::io::BufRead;

use vcd::{Command, IdCode, Parser, ScopeItem, SimulationCommand, Value};

pub type Time = u128;
pub const FS_PER_NS: Time = 1_000_000;
pub const FS_PER_SECOND: Time = 1_000_000_000 * FS_PER_NS;
pub const CHANNELS: usize = 5;
pub type Check<T> = Result<T, String>;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Edge {
    pub time: Time,
    pub high: bool,
}

pub struct Trace {
    pub signals: [Vec<Edge>; CHANNELS],
    pub quantum: Time,
    pub end: Time,
}

impl Trace {
    pub fn read(input: impl BufRead, names: [&str; CHANNELS]) -> Check<Self> {
        // El parser espera un delimitador tras el último token.
        let mut parser = Parser::new(input.chain(&b"\n"[..]));
        let header = parser
            .parse_header()
            .map_err(|error| format!("Cabecera VCD: {error}"))?;
        let (scale, unit) = header.timescale.ok_or("VCD sin escala temporal")?;
        if scale == 0 {
            return Err("Escala temporal igual a cero".into());
        }
        let quantum = Time::from(scale) * (FS_PER_SECOND / Time::from(unit.divisor()));
        let mut codes = Vec::new();
        for name in names {
            let mut matches = Vec::new();
            find_signal(&header.items, "", name, &mut matches);
            let [(code, 1)] = matches.as_slice() else {
                return Err(format!("{name}: se requiere una señal escalar inequívoca"));
            };
            if codes.contains(code) {
                return Err(format!("{name}: dos canales apuntan a la misma señal"));
            }
            codes.push(*code);
        }

        let mut signals: [Vec<Edge>; CHANNELS] = std::array::from_fn(|_| Vec::new());
        let mut time = 0;
        let mut dump_open = false;
        for command in parser {
            let (code, value) = match command.map_err(|error| format!("VCD: {error}"))? {
                Command::Timestamp(tick) => {
                    let next = Time::from(tick)
                        .checked_mul(quantum)
                        .ok_or("Marca temporal VCD fuera de rango")?;
                    if next < time {
                        return Err(format!("Marcas temporales decrecientes en {next} fs"));
                    }
                    time = next;
                    continue;
                }
                Command::Begin(SimulationCommand::Dumpoff | SimulationCommand::Dumpon) => {
                    return Err(format!("Interrupción de captura en {time} fs"));
                }
                Command::Begin(_) => {
                    dump_open = true;
                    continue;
                }
                Command::End(_) => {
                    dump_open = false;
                    continue;
                }
                Command::ChangeScalar(code, value) => (code, value),
                Command::ChangeVector(code, bits) if codes.contains(&code) => {
                    if bits.len() != 1 {
                        return Err(format!("Ancho de señal inválido en {time} fs"));
                    }
                    (code, bits.get(0).ok_or("Vector VCD vacío")?)
                }
                Command::ChangeReal(code, _) | Command::ChangeString(code, _)
                    if codes.contains(&code) =>
                {
                    return Err(format!("Muestra no digital en {time} fs"));
                }
                _ => continue,
            };
            let Some(channel) = codes.iter().position(|&id| id == code) else {
                continue;
            };
            let high = match value {
                Value::V0 => false,
                Value::V1 => true,
                Value::X | Value::Z => {
                    return Err(format!("{}: estado {value} en {time} fs", names[channel]));
                }
            };
            let signal = &mut signals[channel];
            if let Some(previous) = signal.last() {
                if previous.high == high {
                    continue;
                }
                if previous.time == time {
                    return Err(format!("{}: cambios opuestos en {time} fs", names[channel]));
                }
            }
            signal.push(Edge { time, high });
        }
        if dump_open {
            return Err(format!("Bloque de captura sin cierre en {time} fs"));
        }
        for (name, edges) in names.into_iter().zip(&signals) {
            if edges.is_empty() {
                return Err(format!("{name}: captura sin muestras"));
            }
        }
        Ok(Self {
            signals,
            quantum,
            end: time,
        })
    }
}

fn find_signal(items: &[ScopeItem], prefix: &str, name: &str, result: &mut Vec<(IdCode, u32)>) {
    for item in items {
        match item {
            ScopeItem::Scope(scope) => {
                find_signal(
                    &scope.items,
                    &format!("{prefix}{}.", scope.identifier),
                    name,
                    result,
                );
            }
            ScopeItem::Var(var)
                if var.reference == name || format!("{prefix}{}", var.reference) == name =>
            {
                result.push((var.code, var.size));
            }
            _ => {}
        }
    }
}
