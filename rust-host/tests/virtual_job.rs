// SPDX-License-Identifier: GPL-3.0-only
#![cfg(feature = "identify")]
use klipper_host_protocol::{
    identify::Dictionary,
    machine_preview::Trace,
    step_clock::Command,
    step_compress::Block,
    step_wire::{StepMessages, QUEUE, SET_DIR},
    virtual_job::{Error, Job, State},
};

fn codec() -> StepMessages {
    let dictionary: Dictionary = serde_json::from_value(serde_json::json!({
        "commands": {QUEUE: 15, SET_DIR: 16}, "responses": {},
        "version": "synthetic", "build_versions": "test"
    }))
    .unwrap();
    StepMessages::from_dictionary(&dictionary).unwrap()
}

fn fixture() -> Trace {
    let block = |at| {
        Command::Steps(Block {
            interval: 100,
            count: 2,
            add: 0,
            last_clock: at,
        })
    };
    Trace {
        duration: 0.2,
        extruder: None,
        motors: [
            vec![Command::SetDirection(true), block(200), block(400)],
            vec![Command::SetDirection(false), block(200)],
            Vec::new(),
        ],
    }
}

#[test]
fn pauses_and_resumes_without_replaying_an_issued_command() {
    let trace = fixture();
    let mut j = Job::for_trace(&codec(), &trace, &[1, 2, 3], 0, 500, 9).unwrap();
    j.start(100).unwrap();
    let first = j.poll(100, 10).unwrap();
    assert_eq!(
        first.iter().map(|c| c.lane).collect::<Vec<_>>(),
        [0, 0, 1, 1]
    );
    assert!(first.iter().all(|c| c.generation == 9));
    j.pause().unwrap();
    assert!(j.poll(2_000, 10).unwrap().is_empty());
    assert_eq!(j.snapshot().simulated_tick, 0);
    assert!(matches!(j.resume(1_999), Err(Error::InvalidClock)));
    j.resume(2_000).unwrap();
    assert!(j.poll(2_199, 10).unwrap().is_empty());
    assert_eq!(j.poll(2_200, 10).unwrap().len(), 1);
    assert_eq!(j.snapshot().state, State::Running);
    assert!(j.poll(2_500, 10).unwrap().is_empty());
    assert_eq!(j.snapshot().state, State::Finished);
    assert_eq!(j.snapshot().issued, 5);
}

#[test]
fn faults_discard_unissued_work_and_preflight_rejects_unsafe_metadata() {
    let trace = fixture();
    let codec = codec();
    assert!(matches!(
        Job::for_trace(&codec, &trace, &[1, 1, 3], 0, 500, 3),
        Err(Error::InvalidTrace)
    ));
    assert!(matches!(
        Job::for_trace(&codec, &trace, &[1, 2, 3], 0, 399, 3),
        Err(Error::InvalidTrace)
    ));
    let mut j = Job::for_trace(&codec, &trace, &[1, 2, 3], 0, 500, 13).unwrap();
    j.start(10).unwrap();
    let sent = j.poll(10, 1).unwrap();
    assert_eq!(sent.len(), 1);
    j.fault().unwrap();
    assert_eq!(j.snapshot().state, State::Faulted);
    assert_eq!(j.snapshot().issued, 1);
    assert!(j.poll(10_000, 100).unwrap().is_empty());
    assert!(j.resume(10_000).is_err());
    assert!(j.start(10_000).is_err());
}

#[test]
fn routes_a_real_offline_preview_through_virtual_nodes_without_io() {
    use klipper_host_protocol::{
        can::{Datagram, Node},
        config, decode_prefix, encode,
        homing::{Counter, Error as HomingError},
        machine_preview::{Error as MachineError, Machine},
        planner::Priority,
        step_clock::Clock,
        virtual_can::{Action, Bus, Peer},
    };
    use std::collections::BTreeMap;
    let config = config::parse(
        &BTreeMap::from([
            (
                "printer.cfg".to_owned(),
                include_str!("../examples/preview/printer.cfg").to_owned(),
            ),
            (
                "rails.cfg".to_owned(),
                include_str!("../examples/preview/rails.cfg").to_owned(),
            ),
        ]),
        "printer.cfg",
    )
    .unwrap();
    let machine = Machine::from_config(&config).unwrap();
    let clock = Clock {
        last_encoded: 0,
        time_origin: 0.0,
        hz: 20_000_000.0,
    };
    let script = include_str!("../examples/preview/path.gcode");
    let mut home = machine.virtual_homing([1., 1., 0.2]).unwrap();
    assert!(matches!(
        machine.trace_homed(script, &home, 2., clock, 20, [false; 3]),
        Err(MachineError::VirtualHoming(HomingError::Unhomed))
    ));
    for (axis, at) in [1., 1., 0.2].into_iter().enumerate() {
        let token = home.arm(axis as u8, at, true).unwrap();
        let counter = Some(Counter {
            start: 12,
            trigger: 9,
            halt: 9,
        });
        let readings = if axis < 2 {
            [counter, counter, None]
        } else {
            [None, None, counter]
        };
        home.complete(token, readings, true).unwrap();
    }
    let trace = machine
        .trace_homed(script, &home, 2., clock, 20, [false; 3])
        .unwrap();
    let end = ((2.0 + trace.duration) * clock.hz).ceil() as u64 + 1;
    let step_messages = codec();
    let oids = [3, 14, 111];
    let mut job = Job::for_trace(&step_messages, &trace, &oids, 0, end, 2).unwrap();
    let nodes = [
        Node::new(4).unwrap(),
        Node::new(5).unwrap(),
        Node::new(9).unwrap(),
    ];
    let peers = nodes
        .iter()
        .copied()
        .map(|node| Peer {
            node,
            sequence: 22,
            receive_window: 160,
            clock: None,
        })
        .collect();
    let mut bus = Bus::new(1_000_000, peers, 1, 4096).unwrap();

    job.start(0).unwrap();
    let mut host_time = 1.0;
    let mut delivered: [Vec<u8>; 3] = std::array::from_fn(|_| Vec::new());
    for readout in [0, 1_000, 25_000_000, end, end, end] {
        let work = job.poll(readout, 256).unwrap();
        for event in work {
            bus.enqueue(
                nodes[event.lane],
                0,
                0,
                Priority::Deadline(0),
                &event.command.bytes,
            )
            .unwrap();
        }
        loop {
            match bus.poll(host_time).unwrap() {
                Action::Transmit {
                    node,
                    datagrams,
                    completes_at,
                    retry: None,
                } => {
                    let lane = nodes.iter().position(|id| *id == node).unwrap();
                    let mut bytes = Vec::new();
                    for packet in datagrams {
                        assert_eq!(packet.id(), node.host_id());
                        bytes.extend_from_slice(packet.data());
                    }
                    let (wire, size) = decode_prefix(&bytes).unwrap().unwrap();
                    assert_eq!(size, bytes.len());
                    delivered[lane].extend_from_slice(wire.payload);
                    let ack = encode(wire.sequence + 1, &[]).unwrap();
                    let datagram = Datagram::new(u32::from(node.response_id()), &ack).unwrap();
                    bus.receive(datagram, completes_at).unwrap().unwrap();
                    host_time = completes_at;
                }
                Action::WaitUntil(t) => host_time = t,
                Action::Idle => break,
                other => panic!("{other:?}"),
            }
        }
    }
    assert_eq!(job.snapshot().state, State::Finished);
    for i in 0..3 {
        let expected: Vec<_> = step_messages
            .payloads(oids[i], 0, &trace.motors[i])
            .unwrap()
            .into_iter()
            .flat_map(|part| part.bytes)
            .collect();
        assert_eq!(delivered[i], expected);
    }
}

#[cfg(feature = "webhooks")]
#[test]
fn virtual_job_exposes_status_without_webhook_mutations() {
    use klipper_host_protocol::webhooks::{Session, Snapshot};
    use serde_json::{json, Value};
    let mut job = Job::for_trace(&codec(), &fixture(), &[1, 2, 3], 0, 500, 7).unwrap();
    let mut source = Snapshot::default();
    let mut client = Session::default();
    let mut seen = Vec::new();
    let mut record = |bytes: &[u8]| {
        let v: Value = serde_json::from_slice(&bytes[..bytes.len() - 1]).unwrap();
        seen.push(v);
    };
    assert!(job.publish_status(&mut source));
    client.feed(b"{\"id\":1,\"method\":\"objects/subscribe\",\"params\":{\"objects\":{\"virtual_job\":[\"state\",\"mode\"]}}}\x03",
                &source, 1.0, &mut record);
    job.start(10).unwrap();
    job.pause().unwrap();
    assert!(job.publish_status(&mut source));
    assert!(client.update(&source, 1.25, &mut record));
    client.feed(
        b"{\"id\":2,\"method\":\"gcode/script\",\"params\":{\"script\":\"RESUME\"}}\x03",
        &source,
        2.0,
        &mut record,
    );
    assert_eq!(job.snapshot().state, State::Paused);
    assert_eq!(
        seen[0]["result"]["status"]["virtual_job"],
        json!({"state":"ready","mode":"simulation"})
    );
    assert_eq!(
        seen[1]["params"]["status"]["virtual_job"],
        json!({"state":"paused"})
    );
    assert_eq!(seen[2]["error"]["error"], "WebRequestError");
}

#[test]
fn wide_index_preserves_ties_and_discards_remaining_heads_on_fault() {
    let fixture = fixture();
    let codec = codec();
    let indices = (0..12_u8).collect::<Vec<_>>();
    let traces = (0..indices.len())
        .map(|i| fixture.motors[i % 3].as_slice())
        .collect::<Vec<_>>();
    let mut want = Vec::new();
    for (n, (&oid, &commands)) in indices.iter().zip(&traces).enumerate() {
        want.extend(
            codec
                .payloads(oid, 0, commands)
                .unwrap()
                .into_iter()
                .map(|p| (n, p)),
        );
    }
    // Stable sort retains within-lane ordering at equal ticks.
    want.sort_by_key(|(n, p)| (p.required_tick, *n));
    let mut j = Job::for_lanes(&codec, &traces, &indices, 0, 500, 26).unwrap();
    j.start(1).unwrap();
    for cap in [1, 2, 5] {
        let batch = j.poll(1, cap).unwrap();
        assert_eq!(batch.len(), cap);
        for (actual, (lane, payload)) in batch.into_iter().zip(want.drain(..cap)) {
            assert_eq!(actual.lane, lane);
            assert_eq!(actual.command, payload);
            assert_eq!(actual.generation, 26);
        }
    }
    let batch = j.poll(201, 12).unwrap();
    for (actual, (lane, payload)) in batch.into_iter().zip(want.drain(..12)) {
        assert_eq!(actual.lane, lane);
        assert_eq!(actual.command, payload);
    }
    assert_eq!(j.snapshot().issued, 20);
    assert_eq!(j.snapshot().total, 20);
    // A trailing dwell remains after all queues drained.
    j.fault().unwrap();
    assert!(j.poll(1500, 256).unwrap().is_empty());
    assert_eq!(j.snapshot().state, State::Faulted);

    let mut j = Job::for_lanes(&codec, &traces, &indices, 0, 500, 27).unwrap();
    j.start(0).unwrap();
    assert_eq!(j.poll(0, 3).unwrap().len(), 3);
    j.cancel().unwrap();
    assert!(j.poll(1500, 256).unwrap().is_empty());
    assert_eq!(j.snapshot().issued, 3);
    assert_eq!(j.snapshot().state, State::Cancelled);
}
