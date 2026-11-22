// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::{
    step_clock::{encode_trace, Clock, Command},
    step_times::Event,
};
use std::io::{self, BufRead};
use std::str::FromStr;

fn read<T: FromStr>(w: &mut std::str::SplitAsciiWhitespace<'_>) -> T {
    w.next().unwrap().parse().ok().unwrap()
}

fn main() {
    let mut clock = Clock {
        last_encoded: 0,
        time_origin: 0.0,
        hz: 1.0,
    };
    let (mut err, mut invert) = (0, false);
    let mut events = Vec::new();
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let mut w = line.split_ascii_whitespace();
        match w.next().unwrap() {
            "r" => {
                clock = Clock {
                    last_encoded: read(&mut w),
                    time_origin: read(&mut w),
                    hz: read(&mut w),
                };
                err = read(&mut w);
                invert = read::<u8>(&mut w) != 0;
            }
            "a" => events.push(Event::Crossing {
                direction_positive: read::<u8>(&mut w) != 0,
                segment_time: read(&mut w),
                offset: read(&mut w),
            }),
            "c" => events.push(Event::Commit),
            "f" => {
                for item in encode_trace(clock, &events, err, invert).unwrap() {
                    match item {
                        Command::SetDirection(v) => println!("d {}", u8::from(v)),
                        Command::Steps(b) => {
                            println!("s {} {} {} {}", b.interval, b.count, b.add, b.last_clock)
                        }
                    }
                }
                events.clear();
                println!("done");
            }
            _ => panic!(),
        }
    }
}
