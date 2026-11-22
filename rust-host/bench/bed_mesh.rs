// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::bed_mesh::{Algorithm, Mesh, Transform};
use std::io::{self, BufRead};

fn main() {
    let mut mesh = None;
    let mut transform = None;
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let mut w = line.split_ascii_whitespace();
        let op = w.next().unwrap();
        let v: Vec<f64> = w.map(|n| n.parse().unwrap()).collect();
        match op {
            "m" => {
                let pps = [v[7] as usize, v[8] as usize];
                let algo = match v[0] as u8 {
                    0 => Algorithm::Direct,
                    1 => Algorithm::Lagrange {
                        points_per_segment: pps,
                    },
                    _ => Algorithm::Bicubic {
                        points_per_segment: pps,
                        tension: v[9],
                    },
                };
                let n = [v[5] as usize, v[6] as usize];
                mesh = Some(Mesh::new([[v[1], v[2]], [v[3], v[4]]], n, &v[10..], algo).unwrap());
            }
            "o" => mesh.as_mut().unwrap().set_offsets([v[0], v[1]]).unwrap(),
            "z" => println!(
                "z {:.17e}",
                mesh.as_ref().unwrap().sample([v[0], v[1]]).unwrap()
            ),
            "f" => {
                transform = Some(
                    Transform::new(
                        mesh.take().unwrap(),
                        (v[0] != 0.).then_some((v[1], v[2], v[3])),
                        v[4],
                        v[5],
                        v[6],
                    )
                    .unwrap(),
                );
            }
            "g" => {
                let pos = transform
                    .as_ref()
                    .unwrap()
                    .reported(v.try_into().unwrap())
                    .unwrap();
                println!(
                    "g {:.17e} {:.17e} {:.17e} {:.17e}",
                    pos[0], pos[1], pos[2], pos[3]
                );
            }
            "s" => {
                let out = transform
                    .as_ref()
                    .unwrap()
                    .split(v[..4].try_into().unwrap(), v[4..].try_into().unwrap())
                    .unwrap();
                println!("s {}", out.len());
                for row in out {
                    println!(
                        "p {:.17e} {:.17e} {:.17e} {:.17e}",
                        row[0], row[1], row[2], row[3]
                    );
                }
            }
            _ => panic!("{op}"),
        }
    }
}
