// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::{
    encode,
    flight::{Flight, Retry, SendError},
};
use std::io::{self, BufRead};

fn hex(bytes: &[u8]) -> io::Result<usize> {
    for byte in bytes {
        print!("{byte:02x}");
    }
    println!();
    Ok(bytes.len())
}

fn main() {
    let mut flight = Flight::established(14);
    let mut step = 1;
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let mut fields = line.split_ascii_whitespace();
        let op = fields.next().unwrap();
        let mut value = || fields.next().unwrap().parse::<usize>().unwrap();
        match op {
            "s" => {
                let len = value();
                let byte = value() as u8;
                let window = value();
                match flight.send(&[byte; 59][..len], window, hex) {
                    Ok(_) => {}
                    Err(SendError::Backpressure) => {
                        println!("blocked");
                        continue;
                    }
                    Err(other) => panic!("{other:?}"),
                }
            }
            "a" => {
                let seq = value();
                let len = value();
                if let Ok(p) = flight.observe(&encode(seq as u8, &[37][..len]).unwrap()) {
                    println!(
                        "{} {} {} {} {} {}",
                        p.receive_sequence,
                        flight.pending_frames(),
                        flight.outstanding_bytes(),
                        flight.prior_ack_bytes(),
                        u8::from(p.fast_retry),
                        u8::from(p.payload_timing_valid)
                    );
                    if p.fast_retry && step & 1 != 0 {
                        flight.retry(Retry::DuplicateAck, hex).unwrap();
                    }
                } else {
                    println!("invalid");
                }
            }
            "t" => {
                if flight.pending_frames() == 0 {
                    println!("empty");
                    continue;
                }
                flight.retry(Retry::Timeout, hex).unwrap();
            }
            _ => panic!("{op}"),
        }
        step += 1;
    }
}
