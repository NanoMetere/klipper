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
    fn independent_subscribers_can_replace_requests_without_crossing_streams() {
        use std::io::Read;
        let (server_a, mut a) = UnixStream::pair().unwrap();
        let (server_b, mut b) = UnixStream::pair().unwrap();
        let mut a_session = StatusConnection::from_stream(server_a).unwrap();
        let mut b_session = StatusConnection::from_stream(server_b).unwrap();
        let mut a_reader = BufReader::new(a.try_clone().unwrap());
        let mut b_reader = BufReader::new(b.try_clone().unwrap());
        for client in [&a, &b] {
            client
                .set_read_timeout(Some(Duration::from_secs(1)))
                .unwrap();
        }
        let mut source = Snapshot::default();
        let publish = |source: &mut Snapshot, x, y| {
            source.set_object(
                "print_stats".into(),
                json!({"x":x,"y":y}).as_object().unwrap().clone(),
            );
        };
        let encode = |v: Value| {
            let mut bytes = serde_json::to_vec(&v).unwrap();
            bytes.push(ETX);
            bytes
        };
        let read = |stream: &mut BufReader<UnixStream>| {
            let mut raw = Vec::new();
            stream.read_until(ETX, &mut raw).unwrap();
            assert_eq!(raw.pop(), Some(ETX));
            serde_json::from_slice::<Value>(&raw).unwrap()
        };
        let quiet = |stream: &mut BufReader<UnixStream>| {
            let mut byte = [0];
            stream.get_ref().set_nonblocking(true).unwrap();
            assert_eq!(
                stream.read(&mut byte).unwrap_err().kind(),
                std::io::ErrorKind::WouldBlock
            );
            stream.get_ref().set_nonblocking(false).unwrap();
        };
        let subscribe = |id, name, fields: &[&str]| {
            encode(json!({
                "id":id,"method":"objects/subscribe","params":{
                    "objects":{"print_stats":fields},"response_template":{"method":name}
                }
            }))
        };
        publish(&mut source, 1, 2);
        let mut coalesced = encode(json!({"id":1,"method":"objects/list"}));
        coalesced.extend(subscribe(2, "client_a", &["x", "missing"]));
        a.write_all(&coalesced).unwrap();
        b.write_all(&subscribe(3, "client_b", &["y"])).unwrap();
        assert_eq!(a_session.poll(&source, 0.).unwrap().requests, 2);
        b_session.poll(&source, 0.).unwrap();
        assert_eq!(
            read(&mut a_reader)["result"]["objects"],
            json!(["print_stats"])
        );
        assert_eq!(
            read(&mut a_reader)["result"]["status"]["print_stats"],
            json!({"x":1,"missing":null})
        );
        assert_eq!(
            read(&mut b_reader)["result"]["status"]["print_stats"],
            json!({"y":2})
        );
        publish(&mut source, 3, 2);
        a_session.poll(&source, 1.).unwrap();
        b_session.poll(&source, 1.).unwrap();
        assert_eq!(
            read(&mut a_reader)["params"]["status"]["print_stats"],
            json!({"x":3})
        );
        quiet(&mut b_reader);
        a.write_all(&subscribe(4, "client_a_replaced", &["y"]))
            .unwrap();
        a_session.poll(&source, 2.).unwrap();
        assert_eq!(
            read(&mut a_reader)["result"]["status"]["print_stats"],
            json!({"y":2})
        );
        publish(&mut source, 4, 2);
        a_session.poll(&source, 3.).unwrap();
        b_session.poll(&source, 3.).unwrap();
        quiet(&mut a_reader);
        quiet(&mut b_reader);
        publish(&mut source, 4, 6);
        a_session.poll(&source, 4.).unwrap();
        b_session.poll(&source, 4.).unwrap();
        assert_eq!(read(&mut a_reader)["method"], "client_a_replaced");
        assert_eq!(read(&mut b_reader)["method"], "client_b");
        drop(a_reader);
        drop(a);
        assert!(a_session.poll(&source, 5.).is_err());
        publish(&mut source, 4, 7);
        b_session.poll(&source, 5.).unwrap();
        assert_eq!(
            read(&mut b_reader)["params"]["status"]["print_stats"],
            json!({"y":7})
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

#[test]
fn verified_two_tool_segments_yield_independent_inert_motor_lanes() {
    use klipper_host_protocol::{
        extrusion::{Extruder, Settings},
        machine_preview::{Error, SimulatedExtruder, ToolLane, ToolSegment},
        pressure_advance::Advance,
        step_clock::Clock,
        virtual_stepper::Stepper,
    };
    let machine = Machine::from_config(&source()).unwrap();
    let initial = [1., 1., 0.2];
    let mut home = machine.virtual_homing(initial).unwrap();
    let count = Some(Counter {
        start: 14,
        trigger: 11,
        halt: 11,
    });
    for (i, x) in initial.into_iter().enumerate() {
        let arm = home.arm(i as u8, x, true).unwrap();
        let readings = if i < 2 {
            [count, count, None]
        } else {
            [None, None, count]
        };
        home.complete(arm, readings, true).unwrap();
    }
    let e = Extruder::new(Settings {
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
    let pressure = Advance::new(0.04, &[(0., 0.05)]).unwrap();
    let mk = |id, parked_e, temperature| ToolLane {
        id,
        parked_e,
        extruder: SimulatedExtruder {
            geometry: &e,
            temperature,
            step_distance: 0.004,
            inverse_direction: false,
            pressure_advance: Some(&pressure),
        },
    };
    let pieces = [
        ToolSegment {
            tool: 0,
            gcode: "G90\nM82\nG1 X8 Y10 Z.3 E.4 F1800\n",
        },
        ToolSegment {
            tool: 1,
            gcode: "G1 X20 Y20 E.7 F1800\nG4 P80\n",
        },
        ToolSegment {
            tool: 0,
            gcode: "G1 X25 Y26 E.2 F1800\nM400\n",
        },
    ];
    let clock = Clock {
        last_encoded: 0,
        time_origin: 0.,
        hz: 20e6,
    };
    let cold = [mk(0, 0., 210.), mk(1, 1.3, 30.)];
    assert!(matches!(
        machine.trace_tool_segments(&home, &pieces, &cold, 2., clock, 20, [false; 3]),
        Err(Error::Extrusion(_))
    ));
    let lanes = [mk(0, 0., 210.), mk(1, 1.3, 210.)];
    assert!(matches!(
        machine.trace_tool_gcode(
            &home,
            "G90\nT255 ; no motion",
            0,
            &lanes,
            2.,
            clock,
            20,
            [false; 3]
        ),
        Err(Error::ToolSegment)
    ));
    let result = machine
        .trace_tool_segments(&home, &pieces, &lanes, 2., clock, 20, [false; 3])
        .unwrap();
    assert!(result.motors.iter().all(|lane| !lane.is_empty()));
    let wire = codec();
    let mut final_positions = Vec::new();
    for (id, lane) in &result.tools {
        let mut motor = Stepper::new(*id, 0, 0);
        for record in wire.payloads(*id, 0, lane).unwrap() {
            motor.accept_payload(&wire, &record.bytes).unwrap();
        }
        final_positions.push(motor.advance(u32::MAX.into()).unwrap().position_steps);
    }
    assert_eq!(final_positions, [150, 175]);
}

#[test]
fn sparse_job_switch_status_follows_paused_virtual_clock_and_rebases_filament() {
    use klipper_host_protocol::{
        machine_preview::tool_profiles, print_stats::Extrusion, simulated_printer::MultiInput,
        step_clock::Clock,
    };
    let cfg = config::parse(
        &BTreeMap::from([
            (
                "sparse.cfg".into(),
                include_str!("../examples/preview/sparse-tool.cfg").into(),
            ),
            (
                "printer.cfg".into(),
                include_str!("../examples/preview/printer.cfg").into(),
            ),
            (
                "rails.cfg".into(),
                include_str!("../examples/preview/rails.cfg").into(),
            ),
        ]),
        "sparse.cfg",
    )
    .unwrap();
    let machine = Machine::from_config(&cfg).unwrap();
    let origin = [1., 1., 0.2];
    let mut home = machine.virtual_homing(origin).unwrap();
    let codec = codec();
    let profiles = tool_profiles::read(&cfg).unwrap();
    let lanes = profiles
        .iter()
        .zip([0., 1.3, 2.1])
        .map(|(p, e)| p.lane(210., e).unwrap())
        .collect::<Vec<_>>();
    let script = include_str!("../examples/preview/sparse-tool.gcode");
    let hz = 20e6;
    let input = || MultiInput {
        name: "virtual-sparse.gcode",
        script,
        macros: None,
        discovered: &codec,
        oids: &[12, 14, 16, 23, 55, 102],
        lanes: &lanes,
        initial_tool: 0,
        hz,
    };
    assert!(matches!(
        Simulation::prepare_tools(&machine, &home, input()),
        Err(simulated_printer::Error::Machine(_))
    ));
    for (axis, coordinate) in origin.into_iter().enumerate() {
        let token = home.arm(axis as u8, coordinate, true).unwrap();
        let counter = Some(Counter {
            start: 10,
            trigger: 9,
            halt: 9,
        });
        home.complete(
            token,
            if axis < 2 {
                [counter, counter, None]
            } else {
                [None, None, counter]
            },
            true,
        )
        .unwrap();
    }
    let trace = machine
        .trace_tool_gcode(
            &home,
            script,
            0,
            &lanes,
            2.,
            Clock {
                last_encoded: 0,
                time_origin: 0.,
                hz,
            },
            20,
            [false; 3],
        )
        .unwrap();
    assert_eq!(
        trace.switches.iter().map(|t| t.id).collect::<Vec<_>>(),
        [2, 6, 2]
    );
    let times = trace
        .switches
        .iter()
        .map(|t| (t.at * hz).ceil() as u64)
        .collect::<Vec<_>>();
    let mut sim = Simulation::prepare_tools(&machine, &home, input()).unwrap();
    assert_eq!(sim.planned_tool(), Some(0));
    assert_eq!(sim.preflight_boundary(), (script.len(), script.len()));
    sim.start(0).unwrap();
    let mut lanes_seen = BTreeSet::new();
    let mut due = |clock: u64, sim: &mut Simulation| {
        lanes_seen.extend(sim.advance(clock, 256).unwrap().into_iter().map(|p| p.lane));
    };
    let reading = |position| Extrusion {
        position,
        factor: 1.,
    };
    due(times[0] - 1, &mut sim);
    sim.tool_extrusion_observation(0, reading(0.4)).unwrap();
    assert!(sim.tool_extrusion_observation(2, reading(1.3)).is_err());
    assert!(sim.extrusion_observation(reading(10.)).is_err());
    sim.pause().unwrap();
    assert!(sim.advance(times[0] + 10000, 256).unwrap().is_empty());
    assert_eq!(sim.planned_tool(), Some(0));
    sim.resume(times[0] + 10000).unwrap();
    due(times[0] + 10001, &mut sim);
    assert_eq!(sim.planned_tool(), Some(2));
    sim.tool_extrusion_observation(2, reading(2.0)).unwrap();
    due(times[1] + 10001, &mut sim);
    assert_eq!(sim.planned_tool(), Some(6));
    sim.tool_extrusion_observation(6, reading(2.3)).unwrap();
    due(times[2] + 10001, &mut sim);
    assert_eq!(sim.planned_tool(), Some(2));
    sim.tool_extrusion_observation(2, reading(1.92)).unwrap();
    let end = sim.snapshot().planned_end;
    for _ in 0..5 {
        due(end + 10001, &mut sim);
    }
    assert_eq!(sim.snapshot().state, State::Finished);
    assert_eq!(lanes_seen, BTreeSet::from([0, 1, 2, 3, 4, 5]));

    let mut document = Snapshot::default();
    sim.publish(&mut document).unwrap();
    let mut replies = Vec::new();
    let mut client = Session::default();
    client.feed(
        b"{\"id\":1,\"method\":\"objects/query\",\"params\":{\"objects\":{\"virtual_tool_lanes\":null,\"print_stats\":[\"filament_used\"]}}}\x03",
        &document, 1., |frame| collect(frame, &mut replies)
    );
    let status = &replies[0]["result"]["status"];
    assert_eq!(
        status["virtual_tool_lanes"],
        json!({
            "mode":"planned_virtual","pending_tool_changes":0,"planned_active_tool":2,"tool_ids":[0,2,6]
        })
    );
    assert!((status["print_stats"]["filament_used"].as_f64().unwrap() - 1.22).abs() < 1e-10);
    let mut cancelled = Simulation::prepare_tools(&machine, &home, input()).unwrap();
    cancelled.start(0).unwrap();
    cancelled.cancel().unwrap();
    assert!(cancelled.advance(end, 256).unwrap().is_empty());
    assert_eq!(cancelled.planned_tool(), Some(0));
    cancelled.publish(&mut document).unwrap();
    client.feed(
        b"{\"id\":2,\"method\":\"objects/query\",\"params\":{\"objects\":{\"virtual_tool_lanes\":null}}}\x03",
        &document, 2., |frame| collect(frame, &mut replies)
    );
    assert_eq!(
        replies[1]["result"]["status"]["virtual_tool_lanes"]["pending_tool_changes"],
        0
    );
}

#[test]
fn checked_dock_applies_lane_geometry_to_staged_and_restore_moves() {
    use klipper_host_protocol::{
        extrusion::{Error as ExtrusionError, Extruder, Settings},
        gcode,
        machine_preview::{DockInput, Error as MachineError, SimulatedExtruder, ToolLane},
        toolchange::{Catalog, Failure, Scripts, State as ChangerState},
    };
    let machine = Machine::from_config(&source()).unwrap();
    let origin = [1., 1., 0.2];
    let mut home = machine.virtual_homing(origin).unwrap();
    for (axis, coordinate) in origin.into_iter().enumerate() {
        let arm = home.arm(axis as u8, coordinate, true).unwrap();
        let count = Some(Counter {
            start: 10,
            trigger: 9,
            halt: 9,
        });
        home.complete(
            arm,
            if axis < 2 {
                [count, count, None]
            } else {
                [None, None, count]
            },
            true,
        )
        .unwrap();
    }
    let geometry = Extruder::new(Settings {
        nozzle: 0.4,
        filament: 1.75,
        max_area: 0.64,
        retraction_distance: 1.0,
        retraction_velocity: 20.,
        retraction_accel: 300.,
        corner_velocity: 1.,
        min_simulated_temperature: 170.,
    })
    .unwrap();
    let catalog = |offset| {
        let cfg = config::parse(&BTreeMap::from([("dock.cfg".into(), format!(
            "[toolchanger]\n[tool T0]\ntool_number:0\ndetection_pin:A\n[tool T2]\ntool_number:2\ndetection_pin:B\ngcode_x_offset:{offset}\n"
        ))]), "dock.cfg").unwrap();
        Catalog::from_config(&cfg).unwrap()
    };
    let trial = |catalog: &Catalog, temperature, dropoff, pickup| {
        let lane = |id, parked_e, temperature| ToolLane {
            id,
            parked_e,
            extruder: SimulatedExtruder {
                geometry: &geometry,
                temperature,
                step_distance: 0.004,
                inverse_direction: false,
                pressure_advance: None,
            },
        };
        let mut changer = catalog.build().unwrap();
        changer.initialize_from_detection(Some(0)).unwrap();
        let result = machine.preview_dock(
            &home,
            &mut changer,
            DockInput {
                catalog,
                outgoing: lane(0, 0., 210.),
                incoming: lane(2, 1.3, temperature),
                scripts: Scripts {
                    before: "M83\nG1 E-.1 F600",
                    dropoff,
                    pickup,
                    after: "",
                },
                detected_after_pickup: Some(2),
            },
        );
        (result, changer.state())
    };
    let reference = catalog(0.25);
    let (ok, state) = trial(&reference, 210., "G0 X20 Y30 Z3 F1200", "M83\nG1 E.15 F600");
    assert_eq!(state, ChangerState::Ready);
    let out = ok.unwrap();
    assert_eq!(
        out.groups.iter().map(|g| g.tool).collect::<Vec<_>>(),
        [0, 2]
    );
    let p = out.preview.physical_position();
    assert!((p[0] - 1.25).abs() < 1e-12 && (p[3] - 1.45).abs() < 1e-12);
    assert!(matches!(
        trial(&reference, 30., "G0 X20", "M83\nG1 E.15 F600"),
        (
            Err(MachineError::Extrusion(ExtrusionError::Cold)),
            ChangerState::Faulted
        )
    ));
    assert!(matches!(
        trial(&reference, 210., "G0 X201", ""),
        (
            Err(MachineError::Dock(Failure::Coordinates(
                gcode::Error::MoveOutsidePreview
            ))),
            ChangerState::Faulted
        )
    ));
    assert!(matches!(
        trial(&catalog(200.), 210., "G0 X20 Y30 Z3 F1200", ""),
        (
            Err(MachineError::Dock(Failure::Coordinates(
                gcode::Error::MoveOutsidePreview
            ))),
            ChangerState::Faulted
        )
    ));
}
