// SPDX-License-Identifier: GPL-3.0-only
//! Offline dock-state recipe for reference verification.

use klipper_host_protocol::{
    config,
    gcode::{Preview, TransformOwner},
    lookahead::Limits,
    toolchange::{Catalog, ChangeRequest, Dock, ExtruderPositions, Scripts},
};
use std::{
    collections::BTreeMap,
    io::{self, BufRead},
};

fn main() {
    let lanes = std::env::args().nth(1).as_deref() == Some("--lanes");
    for input in io::stdin().lock().lines() {
        let input = input.unwrap();
        let mut fields = input.split_whitespace();
        let mask = fields.next().unwrap().parse().unwrap();
        let mut vector =
            || std::array::from_fn::<_, 3, _>(|_| fields.next().unwrap().parse::<f64>().unwrap());
        let old = vector();
        let new = vector();
        let [x, y, z] = vector();
        let [a, b, c] = vector();
        assert!(fields.next().is_none());
        let owner = TransformOwner(1);
        let limits = Limits::new(172.0, 3420.0, 5.7, 0.63).unwrap();
        let mut p = Preview::new(limits, 200, Some(1.0)).unwrap();
        p.wrap_translation(owner, old, true).unwrap();
        p.reset_last_position().unwrap();
        p.line(&format!("G1 X{x:.13} Y{y:.13} Z{z:.13} E3 F1500"))
            .unwrap();
        p.line("SET_GCODE_OFFSET X=.3 Y=-.1 Z=.12").unwrap();
        p.line("M221 S110").unwrap();
        let dropoff = format!("G90\nG0 X{a:.13} Y{b:.13} Z{c:.13}\nG91\nM83\nG1 E-.3 F600");
        let mut totals = [0.; 2];
        let (mut p, positions) = if lanes {
            let axes: String = "XYZ"
                .chars()
                .enumerate()
                .filter(|(i, _)| mask & (1 << i) != 0)
                .map(|(_, c)| c)
                .collect();
            let conf = format!("[toolchanger]\n[tool A]\ntool_number:0\ndetection_pin:A\n[tool B]\ntool_number:3\ndetection_pin:B\ngcode_x_offset:{}\ngcode_y_offset:{}\ngcode_z_offset:{}\nt_command_restore_axis:{axes}\n", new[0],new[1],new[2]);
            let cfg = config::parse(&BTreeMap::from([("tc.cfg".into(), conf)]), "tc.cfg").unwrap();
            let catalog = Catalog::from_config(&cfg).unwrap();
            let mut changer = catalog.build().unwrap();
            changer.initialize_from_detection(Some(0)).unwrap();
            let positions = ExtruderPositions::new(&[(0, 0.), (3, 1.3)], 0).unwrap();
            let change = catalog
                .preview_change_lanes(
                    &mut changer,
                    p,
                    owner,
                    ChangeRequest {
                        target: Some(3),
                        homed_axes: 7,
                        detected_after_pickup: Some(3),
                        scripts: Scripts {
                            before: "",
                            dropoff: &dropoff,
                            pickup: "G91\nM83\nG1 E.1 F600",
                            after: "G1 E.2 F600",
                        },
                    },
                    positions,
                    Some(1.),
                )
                .unwrap();
            assert_eq!(changer.active(), Some(3));
            assert_eq!(
                change.groups.iter().map(|g| g.tool).collect::<Vec<_>>(),
                [0, 3]
            );
            for (group, total) in change.groups.iter().zip(&mut totals) {
                *total = group
                    .phases
                    .iter()
                    .flat_map(|o| &o.moves)
                    .map(|m| m.end[3] - m.start[3])
                    .sum();
            }
            (change.preview, Some(change.positions))
        } else {
            let dock = Dock::enter(p, owner, mask)
                .unwrap()
                .preview_lines(&dropoff)
                .unwrap();
            (dock.finish(new).unwrap().0, None)
        };
        p.line("G1 E3.1").unwrap();
        p.line("M400").unwrap();
        for v in p
            .coords()
            .position()
            .into_iter()
            .chain(p.coords().gcode_position())
            .chain(p.coords().homing_origin())
            .chain(p.physical_position())
            .chain([p.coords().physical_feed_rate()])
        {
            print!("{v:.17e} ");
        }
        if let Some(positions) = positions {
            for v in [
                positions.observed_position(0, &p).unwrap(),
                positions.observed_position(3, &p).unwrap(),
                totals[0],
                totals[1],
            ] {
                print!("{v:.17e} ");
            }
        }
        println!();
    }
}
