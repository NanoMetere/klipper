// SPDX-License-Identifier: GPL-3.0-only
//! Emit the offline tool-change operation order for a comparison fixture.

use klipper_host_protocol::toolchange::{Action, Changer, Completion, Policy};
use std::io::{self, BufRead};

fn main() {
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let mut fields = line.split_whitespace();
        let tool = |v: &str| {
            if v == "-" {
                None
            } else {
                Some(v.parse().unwrap())
            }
        };
        let previous = tool(fields.next().unwrap());
        let target = tool(fields.next().unwrap());
        let verify_pickup = fields.next().unwrap() == "1";
        let mut changer = Changer::new(
            vec![0, 2, 5],
            Policy {
                axes: 7,
                allow_empty: true,
                verify_pickup,
            },
        )
        .unwrap();
        changer.initialize_from_detection(previous).unwrap();
        let mut next = changer.select(target, 7).unwrap();
        let name = |value: Option<u8>| value.map_or("-".into(), |i| i.to_string());
        while let Some(req) = next {
            let mut completion = Completion::Completed;
            match req.action {
                Action::Save => print!("save "),
                Action::Before(id) => print!("before:{} ", name(id)),
                Action::SelectDockTransform => print!("dock "),
                Action::SuspendMissingWatch => print!("suspend "),
                Action::Dropoff(id) => print!("drop:{id} "),
                Action::ApplyToolConfig(id) => print!("configure:{} ", name(id)),
                Action::Pickup(id) => print!("pickup:{id} "),
                Action::Verify(id) => {
                    print!("verify:{id} ");
                    completion = Completion::Detected(Some(id));
                }
                Action::ResumeMissingWatch(id) => print!("watch:{id} "),
                Action::After(id) => print!("after:{id} "),
                Action::Restore => print!("restore "),
            }
            next = changer.finish_step(req.ticket, completion).unwrap();
        }
        println!("active:{}", name(changer.active()));
    }
}
