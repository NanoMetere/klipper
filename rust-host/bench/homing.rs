// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::homing::{self, Counter, Error, Kinematics};
use std::io::{self, BufRead};

fn main() {
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let mut fields = line.split_whitespace();
        let op = fields.next().unwrap();
        let kin = match fields.next().unwrap() {
            "cart" => Kinematics::Cartesian,
            "core" => Kinematics::CoreXy,
            _ => panic!("invalid kinematics"),
        };
        let mut f = || fields.next().unwrap().parse::<f64>().unwrap();
        let origin = [f(), f(), f()];
        if op == "rate" {
            let end = [f(), f(), f()];
            let dist = [f(), f(), f()];
            let speed = f();
            let mask = fields.next().unwrap().parse().unwrap();
            println!(
                "{:.17e}",
                homing::sample_interval(kin, origin, end, speed, dist, mask).unwrap()
            );
        } else if op == "retract" {
            let end = [f(), f(), f()];
            let (target, second) = homing::retract(origin, end, f()).unwrap();
            for p in target.into_iter().chain(second) {
                print!("{p:.17e} ");
            }
            println!();
        } else if op == "positions" {
            let dist = [f(), f(), f()];
            let mut i = || fields.next().unwrap().parse::<i32>().unwrap();
            let probe = i() != 0;
            let triggered = i() != 0;
            let require = i() != 0;
            let readings = std::array::from_fn(|_| {
                let present = i() != 0;
                let counter = Counter {
                    start: i(),
                    trigger: i(),
                    halt: i(),
                };
                present.then_some(counter)
            });
            let p = homing::recorded_positions(kin, origin, dist, readings, probe).unwrap();
            let status = match homing::check_trigger(triggered, readings, require) {
                Ok(()) => -1,
                Err(Error::NoTrigger) => -2,
                Err(Error::StillTriggered { motor }) => motor as i32,
                error => panic!("unexpected result {error:?}"),
            };
            print!("{status} ");
            for p in p.trigger.into_iter().chain(p.halt) {
                print!("{p:.17e} ");
            }
            println!();
        } else {
            panic!("invalid operation {op}");
        }
        assert!(fields.next().is_none());
    }
}
