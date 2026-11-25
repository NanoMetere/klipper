// SPDX-License-Identifier: GPL-3.0-only
#[cfg(input_shaper_fixture)]
use klipper_host_protocol::input_shaper::{Pulses, Shaper};
use klipper_host_protocol::{
    config,
    extrusion::{Extruder, Settings},
    machine_preview::{Machine, SimulatedExtruder},
    pressure_advance::Advance,
    step_clock::{Clock, Command},
};
use std::{
    collections::BTreeMap,
    fs,
    io::{self, Read},
};

extern "C" {
    fn machine_trace_cpu_ns() -> u64;
}

fn cpu_ns() -> u64 {
    // This stateless C helper returns the POSIX process CPU clock in ns.
    unsafe { machine_trace_cpu_ns() }
}

fn main() {
    let filename = std::env::args_os().nth(1).unwrap();
    let config_text = fs::read_to_string(filename).unwrap();
    let started_config = cpu_ns();
    let cfg = config::parse(
        &BTreeMap::from([("machine.cfg".into(), config_text)]),
        "machine.cfg",
    )
    .unwrap();
    let machine = Machine::from_config(&cfg).unwrap();
    let args: Vec<_> = std::env::args().skip(2).collect();
    let has = |flag| args.iter().any(|v| v == flag);
    #[cfg(input_shaper_fixture)]
    let machine = if has("--shape") {
        machine.with_input_shaper(Shaper::new([
            Pulses::new(&[(0.0, 1.0), (0.013, 0.8)]).unwrap(),
            Pulses::new(&[(0.0, 0.9), (0.010, 1.6), (0.020, 0.7)]).unwrap(),
            Pulses::new(&[(0.0, 1.0), (0.030, 0.6)]).unwrap(),
        ]))
    } else {
        machine
    };
    let config_ns = cpu_ns() - started_config;
    let mut script = String::new();
    io::stdin().read_to_string(&mut script).unwrap();
    let e = Extruder::new(Settings {
        nozzle: 0.4,
        filament: 1.75,
        max_area: 0.64,
        retraction_distance: 15.0,
        retraction_velocity: 20.0,
        retraction_accel: 300.0,
        corner_velocity: 1.0,
        min_simulated_temperature: 170.0,
    })
    .unwrap();
    let runtime_pa = has("--runtime-pressure");
    let pressure = Advance::new(0.04, if runtime_pa { &[(0.0, 0.05)] } else { &[(0.0, 0.05), (2.46, 0.075)] }).unwrap();
    let extra = (has("--extruder") || has("--pressure") || runtime_pa).then_some(SimulatedExtruder {
        geometry: &e,
        temperature: 210.0,
        step_distance: 0.004,
        inverse_direction: true,
        pressure_advance: (has("--pressure") || runtime_pa).then_some(&pressure),
    });
    let started = cpu_ns();
    let trace = machine
        .trace_with_extruder(
            &script,
            [1., 1., 0.2],
            2.0,
            Clock {
                last_encoded: 0,
                time_origin: 0.,
                hz: 20e6,
            },
            20,
            [false, true, false],
            extra,
        )
        .unwrap();
    eprintln!("TIMED_NS {}", config_ns + (cpu_ns() - started));
    for motor in trace.motors.into_iter().chain(trace.extruder) {
        for cmd in motor {
            match cmd {
                Command::SetDirection(v) => println!("d {}", u8::from(v)),
                Command::Steps(b) => {
                    println!("s {} {} {} {}", b.interval, b.count, b.add, b.last_clock)
                }
            }
        }
        println!("end");
    }
    println!("t {:.17e}", trace.duration);
}
