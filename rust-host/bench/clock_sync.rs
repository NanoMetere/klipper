// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::clock_sync::{PrimaryClock, Sample, SecondaryClock, Update};
use std::{
    hint::black_box,
    io::{self, Read},
    time::Instant,
};

fn make_model(parts: &mut std::str::SplitAsciiWhitespace<'_>) -> PrimaryClock {
    let hz = parts.next().unwrap().parse::<f64>().unwrap();
    let tick = parts.next().unwrap().parse::<u64>().unwrap();
    let at = parts.next().unwrap().parse::<f64>().unwrap();
    PrimaryClock::from_uptime(hz, tick, at).unwrap()
}

fn get_sample(parts: &mut std::str::SplitAsciiWhitespace<'_>) -> (u32, Sample) {
    let low = parts.next().unwrap().parse::<u32>().unwrap();
    let sent = parts.next().unwrap().parse::<f64>().unwrap();
    let received = parts.next().unwrap().parse::<f64>().unwrap();
    (
        low,
        if sent == 0.0 {
            Sample::Untimed
        } else {
            Sample::Timed { sent, received }
        },
    )
}

fn secondary() {
    let mut buf = String::new();
    io::stdin().read_to_string(&mut buf).unwrap();
    let mut rows = buf.lines().map(|line| line.split_ascii_whitespace());
    let mut a = make_model(&mut rows.next().unwrap());
    let mut b = make_model(&mut rows.next().unwrap());
    let mut adj: Option<SecondaryClock> = None;
    for (index, mut fields) in rows.enumerate() {
        let (low_a, ts_a) = get_sample(&mut fields);
        let (low_b, ts_b) = get_sample(&mut fields);
        let at = fields.next().unwrap().parse::<f64>().unwrap();
        let print = fields.next().unwrap().parse::<f64>().unwrap();
        a.observe(low_a, ts_a, index < 8).unwrap();
        b.observe(low_b, ts_b, index < 8).unwrap();
        if index < 7 {
            continue;
        }
        if adj.is_none() {
            adj = Some(SecondaryClock::connect(&a, &b, at).unwrap());
        }
        let adj = adj.as_mut().unwrap();
        let map = adj.calibrate(&a, &b, print, at).unwrap();
        let tick = map.tick_at(adj.horizon()).unwrap();
        println!(
            "{:.17} {:.17} {:.17} {} {:.17}",
            map.offset,
            map.hz,
            adj.horizon(),
            tick,
            map.print_time_at(tick)
        );
    }
}

fn main() {
    if std::env::args().nth(1).as_deref() == Some("--secondary") {
        return secondary();
    }
    let repeats: usize = std::env::args()
        .nth(1)
        .unwrap_or("0".into())
        .parse()
        .unwrap();
    let mut buf = String::new();
    io::stdin().read_to_string(&mut buf).unwrap();
    let mut rows = buf.lines().map(|line| line.split_ascii_whitespace());
    let mut head = rows.next().unwrap();
    let hz = head.next().unwrap().parse::<f64>().unwrap();
    let tick0 = head.next().unwrap().parse::<u64>().unwrap();
    let at0 = head.next().unwrap().parse::<f64>().unwrap();
    let rows: Vec<_> = rows
        .map(|mut fields| {
            let tick = fields.next().unwrap().parse::<u32>().unwrap();
            let sent = fields.next().unwrap().parse::<f64>().unwrap();
            let received = fields.next().unwrap().parse::<f64>().unwrap();
            let warmup = fields.next().unwrap() == "1";
            (
                tick,
                if sent == 0.0 {
                    Sample::Untimed
                } else {
                    Sample::Timed { sent, received }
                },
                warmup,
            )
        })
        .collect();
    if repeats > 0 {
        let start = Instant::now();
        for _ in 0..repeats {
            let mut model = PrimaryClock::from_uptime(hz, tick0, at0).unwrap();
            for &(tick, ts, warm) in black_box(&rows) {
                black_box(model.observe(tick, ts, warm).unwrap());
            }
        }
        println!("{}", start.elapsed().as_nanos());
        return;
    }
    let mut model = PrimaryClock::from_uptime(hz, tick0, at0).unwrap();
    for (tick, ts, warm) in rows {
        let result = model.observe(tick, ts, warm).unwrap();
        let m = model.estimate();
        println!(
            "{} {:.17} {:.17} {:.17} {} {:.17} {:.17}",
            model.last_clock(),
            m.at,
            m.ticks,
            m.hz,
            model.nearest_tick(tick.wrapping_sub(331)).unwrap(),
            m.at_time(m.at + 0.5),
            m.time_for_tick(model.last_clock())
        );
        match result {
            Update::Untimed => println!("untimed"),
            Update::Outlier => println!("outlier"),
            Update::Updated(e) => println!("updated {:.17} {} {:.17}", e.at, e.ticks, e.hz),
        }
    }
}
