// SPDX-License-Identifier: GPL-3.0-only
#![cfg(all(feature = "identify", feature = "webhooks"))]

use klipper_host_protocol::{
    config,
    gcode_macros::Registry,
    homing::Counter,
    identify::Dictionary,
    machine_preview::Machine,
    simulated_printer::{self, Input, Simulation},
    step_wire::{StepMessages, QUEUE, SET_DIR},
    virtual_job::State,
    webhooks::{Session, Snapshot, ETX},
};
use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};

fn source() -> config::Config {
    config::parse(
        &BTreeMap::from([
            (
                "printer.cfg".into(),
                include_str!("../examples/preview/printer.cfg").into(),
            ),
            (
                "rails.cfg".into(),
                include_str!("../examples/preview/rails.cfg").into(),
            ),
        ]),
        "printer.cfg",
    )
    .unwrap()
}

fn codec() -> StepMessages {
    let raw = serde_json::from_value::<Dictionary>(json!({
        "commands": {QUEUE: 13, SET_DIR: 14}, "responses": {},
        "version": "sim", "build_versions": "fixture"
    }))
    .unwrap();
    StepMessages::from_dictionary(&raw).unwrap()
}

fn plan(script: &str) -> Simulation {
    let cfg = source();
    let machine = Machine::from_config(&cfg).unwrap();
    let mut home = machine.virtual_homing([1., 1., 0.2]).unwrap();
    let registry = Registry::from_config(&cfg).unwrap();
    let codec = codec();
    let input = || Input {
        name: "test.gcode",
        script,
        macros: Some(&registry),
        discovered: &codec,
        oids: &[4, 5, 6],
        extruder: None,
        hz: 20e6,
    };
    assert!(matches!(
        Simulation::prepare(&machine, &home, input()),
        Err(simulated_printer::Error::Machine(_))
    ));
    for (axis, coordinate) in [1., 1., 0.2].into_iter().enumerate() {
        let token = home.arm(axis as u8, coordinate, true).unwrap();
        let mark = Some(Counter {
            start: 9,
            trigger: 6,
            halt: 6,
        });
        let readings = if axis < 2 {
            [mark, mark, None]
        } else {
            [None, None, mark]
        };
        home.complete(token, readings, true).unwrap();
    }
    Simulation::prepare(&machine, &home, input()).unwrap()
}

fn collect(frame: &[u8], replies: &mut Vec<Value>) {
    assert_eq!(frame.last(), Some(&ETX));
    replies.push(serde_json::from_slice(&frame[..frame.len() - 1]).unwrap());
}

#[test]
fn virtual_files_and_motion_share_a_read_only_status_session() {
    let script = "TEST_PATH\nG1 X5000";
    let mut simulation = plan(script);
    let mut snapshot = Snapshot::default();
    assert_eq!(simulation.preflight_boundary(), (10, 18));
    assert!(simulation.publish(&mut snapshot).unwrap());
    let mut client = Session::default();
    let mut replies = Vec::new();
    let mut query = serde_json::to_vec(&json!({
        "id": 1,"method": "objects/subscribe", "params": {
            "objects": {"print_stats":["state"],"virtual_job":["state","issued"],"virtual_input":null},
            "response_template":{"method":"status"}
        }
    })).unwrap();
    query.push(ETX);
    client.feed(&query, &snapshot, 0., |frame| collect(frame, &mut replies));
    assert_eq!(
        replies[0]["result"]["status"]["print_stats"]["state"],
        "standby"
    );

    simulation.start(100).unwrap();
    simulation.advance(200, 8).unwrap();
    simulation.publish(&mut snapshot).unwrap();
    assert!(client.update(&snapshot, 1., |frame| collect(frame, &mut replies)));
    assert_eq!(
        replies.last().unwrap()["params"]["status"]["print_stats"]["state"],
        "printing"
    );

    simulation.pause().unwrap();
    assert!(simulation.advance(1_000, 16).unwrap().is_empty());
    simulation.publish(&mut snapshot).unwrap();
    assert!(client.update(&snapshot, 2., |frame| collect(frame, &mut replies)));
    assert_eq!(
        replies.last().unwrap()["params"]["status"]["print_stats"]["state"],
        "paused"
    );

    simulation.resume(1_000).unwrap();
    let planned = simulation.snapshot().planned_end;
    let mut used = BTreeSet::new();
    for _ in 0..4 {
        for issue in simulation.advance(planned + 2_000, 256).unwrap() {
            used.insert(issue.lane);
        }
    }
    simulation.publish(&mut snapshot).unwrap();
    assert!(client.update(&snapshot, 3., |frame| collect(frame, &mut replies)));
    assert_eq!(
        replies.last().unwrap()["params"]["status"]["print_stats"]["state"],
        "complete"
    );
    assert_eq!(simulation.snapshot().state, State::Finished);
    assert_eq!(simulation.snapshot().issued, simulation.snapshot().total);
    assert_eq!(used, BTreeSet::from([0, 1, 2]));
}

#[test]
fn contact_fault_drops_the_remaining_simulated_commands() {
    let mut simulation = plan("TEST_PATH\n");
    simulation.start(10).unwrap();
    simulation.advance(50, 1).unwrap();
    simulation.fail("fixture disconnect").unwrap();
    assert_eq!(simulation.snapshot().state, State::Faulted);
    assert!(simulation.advance(100_000_000, 256).unwrap().is_empty());
    assert!(simulation.resume(100_000_000).is_err());
}

#[test]
fn simulated_heater_failure_prevents_any_remaining_motion() {
    use klipper_host_protocol::verify_heater::{Decision, Fault, Monitor, Settings};
    let mut simulation = plan("G4 P9000\nTEST_PATH\n");
    simulation.start(0).unwrap();
    let mut heater = Monitor::new(Settings {
        check_gain_time: 1.,
        ..Settings::standard(false)
    })
    .unwrap();
    assert!(matches!(
        simulation.heater_sample(&mut heater, 20., 220.).unwrap(),
        Decision::Continue { .. }
    ));
    simulation.advance(20_000_000, 256).unwrap();
    assert!(matches!(
        simulation.heater_sample(&mut heater, 20., 220.).unwrap(),
        Decision::Continue { .. }
    ));
    simulation.advance(40_000_000, 256).unwrap();
    assert_eq!(
        simulation.heater_sample(&mut heater, 20., 220.).unwrap(),
        Decision::Fault(Fault::NotHeating)
    );
    assert_eq!(simulation.snapshot().state, State::Faulted);
    assert!(simulation.advance(500_000_000, 256).unwrap().is_empty());
}

#[test]
fn optional_fourth_lane_rejects_cold_extrusion_and_tracks_inert_observations() {
    use klipper_host_protocol::{
        extrusion::{Extruder, Settings},
        machine_preview::SimulatedExtruder,
        pressure_advance::Advance,
        print_stats::Extrusion,
    };
    let cfg = source();
    let machine = Machine::from_config(&cfg).unwrap();
    let xyz = [1., 1., 0.2];
    let mut home = machine.virtual_homing(xyz).unwrap();
    for (axis, coordinate) in xyz.into_iter().enumerate() {
        let arm = home.arm(axis as u8, coordinate, true).unwrap();
        let c = Some(Counter {
            start: 10,
            trigger: 9,
            halt: 9,
        });
        let readings = if axis < 2 {
            [c, c, None]
        } else {
            [None, None, c]
        };
        home.complete(arm, readings, true).unwrap();
    }
    let geometry = Extruder::new(Settings {
        nozzle: 0.4,
        filament: 1.75,
        max_area: 0.64,
        retraction_distance: 15.,
        retraction_velocity: 20.,
        retraction_accel: 300.,
        corner_velocity: 1.,
        min_simulated_temperature: 170.,
    })
    .unwrap();
    let pressure = Advance::new(0.04, &[(0., 0.04)]).unwrap();
    let code = codec();
    let input = |temperature| Input {
        name: "extrusion.gcode",
        script: include_str!("../examples/preview/path_extrusion.gcode"),
        macros: None,
        discovered: &code,
        oids: &[1, 3, 8, 13],
        hz: 20e6,
        extruder: Some(SimulatedExtruder {
            geometry: &geometry,
            temperature,
            step_distance: 0.004,
            inverse_direction: false,
            pressure_advance: Some(&pressure),
        }),
    };
    assert!(matches!(
        Simulation::prepare(&machine, &home, input(30.)),
        Err(simulated_printer::Error::Machine(
            klipper_host_protocol::machine_preview::Error::Extrusion(_)
        ))
    ));
    let mut sim = Simulation::prepare(&machine, &home, input(210.)).unwrap();
    sim.start(0).unwrap();
    sim.advance(50_000, 256).unwrap();
    sim.extrusion_observation(Extrusion {
        position: 1.2,
        factor: 1.,
    })
    .unwrap();
    let mut source = Snapshot::default();
    sim.publish(&mut source).unwrap();
    let request = b"{\"id\":1,\"method\":\"objects/query\",\"params\":{\"objects\":{\"print_stats\":[\"filament_used\"]}}}\x03";
    let mut replies = Vec::new();
    Session::default().feed(request, &source, 1., |response| {
        collect(response, &mut replies)
    });
    assert_eq!(
        replies[0]["result"]["status"]["print_stats"]["filament_used"],
        1.2
    );
    let mut lanes = BTreeSet::new();
    let end = sim.snapshot().planned_end;
    for _ in 0..4 {
        lanes.extend(
            sim.advance(end, 256)
                .unwrap()
                .into_iter()
                .map(|item| item.lane),
        );
    }
    assert_eq!(sim.snapshot().state, State::Finished);
    assert!(lanes.contains(&3));
}

#[cfg(unix)]
mod socket_tests {
    use super::*;
    use klipper_host_protocol::status_socket::{StatusConnection, StatusSocket};
    use std::{
        fs,
        io::{BufRead, BufReader, Write},
        os::unix::{fs::DirBuilderExt, net::UnixStream},
        time::{Duration, SystemTime, UNIX_EPOCH},
    };

    #[test]
    fn unix_clients_observe_simulated_lifecycle_without_enabling_command_execution() {
        let (stream, mut client) = UnixStream::pair().unwrap();
        let mut server = StatusConnection::from_stream(stream).unwrap();
        client
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        let mut reader = BufReader::new(client.try_clone().unwrap());
        let mut simulation = plan("TEST_PATH\n");
        let mut state = Snapshot::default();
        simulation.publish(&mut state).unwrap();
        let mut response = || {
            let mut bytes = Vec::new();
            reader.read_until(ETX, &mut bytes).unwrap();
            assert_eq!(bytes.pop(), Some(ETX));
            serde_json::from_slice::<Value>(&bytes).unwrap()
        };

        client
            .write_all(br#"{"id":2,"method":"objects/sub"#)
            .unwrap();
        assert_eq!(server.poll(&state, 0.).unwrap().requests, 0);
        client.write_all(br#"scribe","params":{"objects":{"print_stats":["state"]},"response_template":{"method":"notify_status_update"}}}"#).unwrap();
        client.write_all(&[ETX]).unwrap();
        assert_eq!(server.poll(&state, 0.).unwrap().requests, 1);
        assert_eq!(
            response()["result"]["status"]["print_stats"]["state"],
            "standby"
        );

        simulation.start(0).unwrap();
        simulation.publish(&mut state).unwrap();
        server.poll(&state, 1.).unwrap();
        let update = response();
        assert_eq!(update["method"], "notify_status_update");
        assert_eq!(
            update["params"]["status"]["print_stats"]["state"],
            "printing"
        );

        client
            .write_all(br#"{"id":3,"method":"gcode/script","params":{"script":"M112"}}"#)
            .unwrap();
        client.write_all(&[ETX]).unwrap();
        server.poll(&state, 2.).unwrap();
        assert_eq!(response()["error"]["error"], "WebRequestError");
        assert_eq!(simulation.snapshot().state, State::Running);

        // A client requesting several large replies at once is dropped; the
        // existing subscriber still receives its next simulation delta.
        state
            .info
            .insert("fixture".into(), Value::String("x".repeat(58 * 1024)));
        let (slow_stream, mut slow) = UnixStream::pair().unwrap();
        let mut slow_session = StatusConnection::from_stream(slow_stream).unwrap();
        for id in 10..14 {
            write!(slow, "{{\"id\":{id},\"method\":\"info\"}}").unwrap();
            slow.write_all(&[ETX]).unwrap();
        }
        simulation.fail("fixture interrupted").unwrap();
        simulation.publish(&mut state).unwrap();
        assert_eq!(
            slow_session.poll(&state, 3.).unwrap_err().kind(),
            std::io::ErrorKind::OutOfMemory
        );
        server.poll(&state, 3.).unwrap();
        assert_eq!(
            response()["params"]["status"]["print_stats"]["state"],
            "error"
        );
    }

    #[test]
    #[ignore = "requires an environment that permits binding named local Unix sockets"]
    fn listener_binds_only_a_new_path_and_unlinks_its_own_socket() {
        let stamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let dir = std::env::temp_dir().join(format!("klipper-test-{}-{stamp}", std::process::id()));
        fs::DirBuilder::new().mode(0o700).create(&dir).unwrap();
        let path = dir.join("status.sock");
        let server = StatusSocket::bind(&path).unwrap();
        assert!(StatusSocket::bind(&path).is_err());
        drop(server);
        assert!(!path.exists());
        fs::remove_dir(dir).unwrap();
    }
}
