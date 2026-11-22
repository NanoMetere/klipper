// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::{
    bed_mesh::{Algorithm, Mesh, Transform},
    gcode::{Output, Preview, TransformOwner},
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
        .chain(p.physical_position())
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

fn directive(preview: &mut Preview, line: &str) {
    let mut words = line.split_ascii_whitespace();
    let cmd = words.next().unwrap();
    if cmd == "!mesh" {
        let z: Vec<_> = (0..81)
            .map(|i| {
                let x = i % 9;
                let y = i / 9;
                f64::from((x * 13 + y * 19 + x * y * 7) % 23 - 11) * 0.019
            })
            .collect();
        let mesh = Mesh::new(
            [[25.0; 2], [325.0; 2]],
            [9, 9],
            &z,
            Algorithm::Bicubic {
                points_per_segment: [2, 2],
                tension: 0.2,
            },
        )
        .unwrap();
        let mesh = Transform::new(mesh, Some((0.6, 10.0, 0.0)), 0.0, 0.025, 5.0).unwrap();
        preview.install_bed_mesh(mesh).unwrap();
        return;
    }
    if cmd == "!reset" {
        preview.reset_last_position().unwrap();
        return;
    }
    let owner = TransformOwner(words.next().unwrap().parse().unwrap());
    if cmd == "!unwrap" {
        preview.unwrap_translation(owner).unwrap();
        return;
    }
    let offset = std::array::from_fn(|_| words.next().unwrap().parse().unwrap());
    if cmd == "!wrap" {
        let force = words.next().unwrap() == "1";
        preview.wrap_translation(owner, offset, force).unwrap();
    } else if cmd == "!set" {
        preview.set_translation(owner, offset).unwrap();
    } else {
        panic!("{line}");
    }
}

fn solve(lines: &[String], emit: bool, arcs: bool) -> (usize, f64) {
    let limits = Limits::new(160.0, 2500.0, 5.0, 0.5).unwrap();
    let mut p = Preview::new(limits, 4096, Some(1.0)).unwrap();
    if arcs {
        p.enable_arcs(0.28).unwrap();
    }
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
        let out = if line.starts_with('!') {
            directive(&mut p, line);
            Output::default()
        } else {
            p.line(line).unwrap_or_else(|e| panic!("{line}: {e:?}"))
        };
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
    let args: Vec<_> = std::env::args().collect();
    let arcs = args.iter().any(|a| a == "--arcs");
    if args.iter().any(|a| a == "--bench") {
        black_box(solve(&lines, false, arcs));
        let start = Instant::now();
        let (n, time) = black_box(solve(&lines, false, arcs));
        println!("{} {n} {time:.17e}", start.elapsed().as_nanos());
    } else {
        solve(&lines, true, arcs);
    }
}
