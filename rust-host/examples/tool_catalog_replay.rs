// SPDX-License-Identifier: GPL-3.0-only
//! Expose tool metadata from simulated configuration fixtures.

use klipper_host_protocol::{config, toolchange::Catalog};
use std::{
    collections::BTreeMap,
    io::{self, Read},
};

fn main() {
    let mut all = String::new();
    io::stdin().read_to_string(&mut all).unwrap();
    for fixture in all.split('\x0c').filter(|text| !text.is_empty()) {
        let cfg = config::parse(
            &BTreeMap::from([("printer.cfg".into(), fixture.to_owned())]),
            "printer.cfg",
        )
        .unwrap();
        let tc = Catalog::from_config(&cfg).unwrap();
        let p = tc.policy();
        println!(
            "P {} {} {}",
            p.axes,
            u8::from(p.allow_empty),
            u8::from(p.verify_pickup)
        );
        for slot in tc.tools() {
            let [x, y, z] = slot.nozzle_offset;
            println!(
                "T {} {} {} {x:.17} {y:.17} {z:.17} {}",
                slot.number,
                slot.restore_axes,
                u8::from(slot.has_detection),
                slot.section.replace(' ', "_")
            );
        }
    }
}
