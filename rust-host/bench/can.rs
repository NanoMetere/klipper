// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::{
    can::{Datagram, Demux, Node},
    encode,
};
use std::io::{self, BufRead};

fn hex(data: &[u8]) -> String {
    data.iter().map(|v| format!("{v:02x}")).collect()
}

fn main() {
    let mut demux = Demux::default();
    for n in 4..=9 {
        demux.register(Node::new(n).unwrap()).unwrap();
    }
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let mut w = line.split_ascii_whitespace();
        let op = w.next().unwrap();
        let id: u32 = w.next().unwrap().parse().unwrap();
        let length: usize = w.next().unwrap().parse().unwrap();
        let data: Vec<u8> = w.map(|b| b.parse().unwrap()).collect();
        assert_eq!(length, data.len());
        if op == "t" {
            for packet in Node::new(id as u8).unwrap().fragments(&data).unwrap() {
                println!("tx {} {}", packet.id(), hex(packet.data()));
            }
        } else {
            let packet = Datagram::new(id, &data);
            let accepted = packet.ok().and_then(|f| {
                demux.feed(f, |frame| {
                    let original = encode(frame.sequence, frame.payload).unwrap();
                    println!("rx {}", hex(&original));
                })
            });
            match accepted {
                Some(s) => println!("rx-count {} {}", s.frames, s.discarded),
                None => println!("filtered"),
            }
        }
    }
}
