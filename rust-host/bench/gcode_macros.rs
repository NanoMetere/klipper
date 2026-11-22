// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::{config, gcode_macros::Registry};
use std::{
    collections::BTreeMap,
    fs,
    io::{self, Read},
};

fn main() {
    let input = std::env::args_os().nth(1).unwrap();
    let cfg = config::parse(
        &BTreeMap::from([("printer.cfg".into(), fs::read_to_string(input).unwrap())]),
        "printer.cfg",
    )
    .unwrap();
    let registry = Registry::from_config(&cfg).unwrap();
    let mut text = String::new();
    io::stdin().read_to_string(&mut text).unwrap();
    match registry.expand(&text) {
        Ok(expanded) => print!("{expanded}"),
        Err(e) => {
            eprintln!(
                "{:?} {} {}",
                e.kind,
                e.line,
                e.macro_name.as_deref().unwrap_or("")
            );
            std::process::exit(2);
        }
    }
}
