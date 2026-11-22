// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::probe::{Aggregate, Decision, Error, ResultPoint, Sampler};
use std::io::{self, BufRead};
fn main() {
    let mut sampler = None;
    let mut xyz = [0.; 3];
    let (mut reads, mut retries) = (0, 0);
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let mut p = line.split_ascii_whitespace();
        let op = p.next().unwrap();
        let v: Vec<f64> = p.map(|w| w.parse().unwrap()).collect();
        if op == "r" {
            let mode = if v[0] == 0. {
                Aggregate::Mean
            } else {
                Aggregate::Median
            };
            sampler = Some(Sampler::new(mode, v[1] as usize, v[2], v[3] as u8, true).unwrap());
            xyz = v[4..7].try_into().unwrap();
            reads = 0;
            retries = 0;
        } else if let Some(active) = &mut sampler {
            reads += 1;
            let tracked = if op == "x" {
                Err(active.timeout())
            } else {
                active.push(ResultPoint::from_trigger(v.try_into().unwrap(), xyz).unwrap())
            };
            match tracked {
                Ok(Decision::Continue) => {}
                Ok(Decision::Retry { .. }) => retries += 1,
                Ok(Decision::Done(result)) => {
                    print!("done {reads} {retries}");
                    for value in result.bed.into_iter().chain(result.tool) {
                        print!(" {value:.17e}");
                    }
                    println!();
                    sampler = None;
                }
                Err(Error::Tolerance) => {
                    println!("tolerance {reads} {retries}");
                    sampler = None;
                }
                Err(Error::TriggerTimeout) => {
                    println!("timeout {reads} {retries}");
                    sampler = None;
                }
                e => panic!("{e:?}"),
            }
        }
    }
}
