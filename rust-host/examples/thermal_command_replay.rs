// SPDX-License-Identifier: GPL-3.0-only
//! Run only the inert thermal command model with a fixed heater table.
use klipper_host_protocol::thermal::commands::{
    parse_gcode, Bank, Command, Error, Gate, Heater, Observation, Output, Profile, Role, Source,
    Wait,
};
use std::io::{self, BufRead};

fn main() {
    let profiles = [
        Profile {
            name: "extruder",
            role: Role::Extruder(0),
            minimum: 0.,
            maximum: 300.,
        },
        Profile {
            name: "extruder1",
            role: Role::Extruder(1),
            minimum: 0.,
            maximum: 280.,
        },
        Profile {
            name: "heater_bed",
            role: Role::Bed,
            minimum: 0.,
            maximum: 130.,
        },
        Profile {
            name: "chamber",
            role: Role::Named,
            minimum: 0.,
            maximum: 90.,
        },
    ];
    let flags = std::env::args().skip(1).collect::<Vec<_>>();
    let debug = flags.iter().any(|v| v == "debug");
    let lexer = flags.iter().any(|v| v == "--gcode");
    let mut bank = Bank::new(&profiles, &["ambient"], Some(0), debug).unwrap();
    for row in io::stdin().lock().lines() {
        let row = row.unwrap();
        let (line, readings) = if lexer && !row.starts_with("select ") {
            let (line, sample) = row.split_once('|').expect("fixture reading delimiter");
            (line, Some(sample))
        } else {
            (row.as_str(), None)
        };
        let mut parts = line.split_ascii_whitespace();
        let opcode = parts.next().unwrap();
        let next = |p: Option<&str>| p.unwrap().parse::<f64>().unwrap();
        let range = |p: Option<&str>| p.filter(|&x| x != "-").map(|x| x.parse::<f64>().unwrap());
        let result = if readings.is_some() {
            parse_gcode(line).and_then(|command| bank.command(command.ok_or(Error::Syntax)?))
        } else {
            match opcode {
                "select" => bank
                    .activate(parts.next().unwrap().parse().unwrap())
                    .map(|()| Output::Ignored),
                "M104" | "M109" => {
                    let id = parts
                        .next()
                        .filter(|&x| x != "-")
                        .map(|x| x.parse().unwrap());
                    bank.command(Command::Set {
                        heater: Heater::Extruder(id),
                        target: next(parts.next()),
                        wait: opcode == "M109",
                    })
                }
                "M140" | "M190" => bank.command(Command::Set {
                    heater: Heater::Bed,
                    target: next(parts.next()),
                    wait: opcode == "M190",
                }),
                "set" => bank.command(Command::Set {
                    heater: Heater::Named(parts.next().unwrap()),
                    target: next(parts.next()),
                    wait: false,
                }),
                "off" => bank.command(Command::Off),
                "range" => bank.command(Command::WaitSensor {
                    name: parts.next().unwrap(),
                    minimum: range(parts.next()),
                    maximum: range(parts.next()),
                }),
                _ => panic!("fixture opcode"),
            }
        };
        let output = result;
        let mut pauses = 0;
        if let Ok(Output::Set { wait: Some(w), .. } | Output::Range(w)) = output {
            let source = match w {
                Wait::Controller { index, .. } => Source::Heater(index),
                Wait::Range { source, .. } => source,
            };
            let mut samples = readings
                .map(str::split_ascii_whitespace)
                .unwrap_or(parts)
                .enumerate();
            loop {
                let (time, value) = samples.next().expect("fixture must settle");
                let temperature = next(Some(value));
                let busy =
                    matches!(w, Wait::Controller { target, .. } if temperature < target - 2.);
                let obs = Observation {
                    source,
                    sampled_at: time as f64,
                    temperature,
                    controller_busy: busy,
                };
                match w.poll(time as f64, 0., Some(obs), false).unwrap() {
                    Gate::Ready => break,
                    Gate::Pending => pauses += 1,
                    Gate::Shutdown => unreachable!(),
                }
            }
        }
        print!("{} {pauses}", if output.is_ok() { "ok" } else { "error" });
        for target in bank.targets() {
            print!(" {target:.1}");
        }
        println!();
    }
}
