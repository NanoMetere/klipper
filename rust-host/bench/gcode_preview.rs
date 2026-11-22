// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::{
    gcode::{Output, Preview},
    lookahead::Limits,
};
use std::{
    hint::black_box,
    io::{self, BufRead},
    time::Instant,
};

fn log(p: &Preview, o: Output) {
    println!(
        "line {} {:.17e}",
        o.moves.len(),
        o.dwell_seconds.unwrap_or(-1.0)
    );
    print!("state");
    for v in p
        .coords()
        .position()
        .into_iter()
        .chain(p.coords().gcode_position())
        .chain(p.coords().homing_origin())
        .chain([p.coords().physical_feed_rate()])
    {
        print!(" {v:.17e}");
    }
    println!();
    for m in o.moves {
        print!("move {}", u8::from(m.kinematic));
        for v in m.start.into_iter().chain(m.end).chain(m.direction).chain([
            m.start_v, m.cruise_v, m.end_v, m.accel, m.accel_t, m.cruise_t, m.decel_t,
        ]) {
            print!(" {v:.17e}");
        }
        println!();
    }
}

fn solve(lines: &[String], emit: bool) -> (usize, f64) {
    let limits = Limits::new(160.0, 2500.0, 5.0, 0.5).unwrap();
    let mut p = Preview::new(limits, 4096, Some(1.0)).unwrap();
    let mut n = 0;
    let mut time = 0.0;
    let mut consume = |p: &Preview, out: Output| {
        n += out.moves.len();
        for m in &out.moves {
            time += m.accel_t + m.cruise_t + m.decel_t;
        }
        time += out.dwell_seconds.unwrap_or(0.0);
        if emit {
            log(p, out);
        }
    };
    for line in lines {
        let out = p.line(line).unwrap_or_else(|e| panic!("{line}: {e:?}"));
        consume(&p, out);
    }
    let moves = p.finish().unwrap();
    consume(
        &p,
        Output {
            moves,
            dwell_seconds: None,
        },
    );
    (n, time)
}

fn main() {
    let lines: Vec<_> = io::stdin().lock().lines().map(Result::unwrap).collect();
    if std::env::args().nth(1).is_some() {
        black_box(solve(&lines, false));
        let start = Instant::now();
        let (n, time) = black_box(solve(&lines, false));
        println!("{} {n} {time:.17e}", start.elapsed().as_nanos());
    } else {
        solve(&lines, true);
    }
}
