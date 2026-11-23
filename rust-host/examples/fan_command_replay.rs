// SPDX-License-Identifier: GPL-3.0-only
//! Output typed test demands for comparison with the stock parser.
use klipper_host_protocol::fan::command::print_demand;
use std::io::{self, BufRead};

fn main() {
    for line in io::stdin().lock().lines() {
        match print_demand(&line.unwrap()) {
            Ok(Some(speed)) => println!("{speed:.17}"),
            Ok(None) => println!("other"),
            Err(_) => println!("rejected"),
        }
    }
}
