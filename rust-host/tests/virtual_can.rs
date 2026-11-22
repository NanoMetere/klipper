// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::{
    can::{Datagram, Node},
    decode_prefix, encode,
    flight::Retry,
    planner::Priority,
    virtual_can::{Action, Bus, Error, Peer},
};

fn topology(count: u8) -> Vec<Peer> {
    (4..4 + count)
        .map(|n| Peer {
            node: Node::new(n).unwrap(),
            sequence: 14,
            receive_window: 160,
            clock: None,
        })
        .collect()
}

fn node(n: u8) -> Node {
    Node::new(n).unwrap()
}

fn host_payload(datagrams: Vec<Datagram>) -> Vec<u8> {
    let mut bytes = Vec::new();
    for data in datagrams {
        bytes.extend_from_slice(data.data());
    }
    bytes
}

fn respond(bus: &mut Bus, node: Node, sequence: u8, tag: u8, at: f64) {
    let reply = encode(sequence + 1, &[tag; 10]).unwrap();
    let mut found = Vec::new();
    for fragment in reply.chunks(8) {
        let packet = Datagram::new(u32::from(node.response_id()), fragment).unwrap();
        let got = bus.receive(packet, at).unwrap().unwrap();
        assert_eq!(got.out_of_window, 0);
        found.extend(got.accepted_payloads);
    }
    assert_eq!(found, [vec![tag; 10]]);
    let ack = encode(sequence + 1, &[]).unwrap();
    let got = bus
        .receive(Datagram::new(node.response_id().into(), &ack).unwrap(), at)
        .unwrap()
        .unwrap();
    assert!(got.accepted_payloads.is_empty());
}

#[test]
fn processes_six_interleaved_links_and_replays_only_the_lost_link() {
    let mut bus = Bus::new(1_000_000, topology(6), 1, 64).unwrap();
    let mut start = 1.0;
    for n in 4..=9 {
        bus.enqueue(node(n), 0, 0, Priority::Deadline(0), &[n; 59])
            .unwrap();
    }
    let mut valid = Vec::new();
    let mut missing = None;
    for n in 4..=9 {
        match bus.poll(start).unwrap() {
            Action::Transmit {
                node: dst,
                datagrams,
                completes_at,
                retry: None,
            } => {
                assert_eq!(dst, node(n));
                assert!(datagrams.iter().all(|d| d.id() == dst.host_id()));
                let bytes = host_payload(datagrams);
                let (frame, count) = decode_prefix(&bytes).unwrap().unwrap();
                assert_eq!(count, bytes.len());
                assert_eq!(frame.sequence, 14);
                assert_eq!(frame.payload, &[n; 59]);
                if n == 4 {
                    missing = Some(bytes);
                } else {
                    valid.push((dst, n));
                }
                start = completes_at;
            }
            other => panic!("{other:?}"),
        }
    }
    // Replies arrive in a different order and their first fragments interleave.
    let mut replies = Vec::new();
    for (dst, tag) in valid.into_iter().rev() {
        let bytes = encode(15, &[tag; 14]).unwrap();
        let packet = Datagram::new(dst.response_id().into(), &bytes[..8]).unwrap();
        assert!(bus
            .receive(packet, start)
            .unwrap()
            .unwrap()
            .accepted_payloads
            .is_empty());
        replies.push((dst, tag, bytes));
    }
    for (dst, tag, bytes) in replies.into_iter().rev() {
        let mut got = Vec::new();
        for data in bytes[8..].chunks(8) {
            got.extend(
                bus.receive(
                    Datagram::new(dst.response_id().into(), data).unwrap(),
                    start,
                )
                .unwrap()
                .unwrap()
                .accepted_payloads,
            );
        }
        assert_eq!(got, [vec![tag; 14]]);
    }
    let Action::WaitUntil(at) = bus.poll(start).unwrap() else {
        panic!();
    };
    let Action::Transmit {
        node: dst,
        datagrams,
        completes_at,
        retry,
    } = bus.poll(at).unwrap()
    else {
        panic!();
    };
    assert_eq!(dst, node(4));
    assert_eq!(retry, Some(Retry::Timeout));
    assert_eq!(
        host_payload(datagrams),
        [&[0x7e][..], &missing.unwrap()].concat()
    );
    respond(&mut bus, dst, 14, 4, completes_at);
    assert!(matches!(bus.poll(completes_at).unwrap(), Action::Idle));
}

#[test]
fn a_silent_peer_does_not_prevent_another_peer_from_wrapping_its_sequence() {
    let mut bus = Bus::new(1_000_000, topology(2), 1, 64).unwrap();
    bus.enqueue(node(4), 0, 0, Priority::Deadline(0), &[4])
        .unwrap();
    let Action::Transmit { completes_at, .. } = bus.poll(1.0).unwrap() else {
        panic!();
    };
    let mut now = completes_at;
    for i in 0..40 {
        bus.enqueue(node(5), 0, 0, Priority::Deadline(0), &[5])
            .unwrap();
        let Action::Transmit {
            node: dst,
            datagrams,
            completes_at,
            ..
        } = bus.poll(now).unwrap()
        else {
            panic!();
        };
        let bytes = host_payload(datagrams);
        let (frame, _) = decode_prefix(&bytes).unwrap().unwrap();
        assert_eq!(dst, node(5));
        assert_eq!(frame.sequence, (14 + i) % 16);
        respond(&mut bus, dst, frame.sequence, 5, completes_at);
        now = completes_at;
    }
    let mut retries = 0;
    loop {
        match bus.poll(now).unwrap() {
            Action::WaitUntil(at) => now = at,
            Action::Transmit {
                completes_at,
                node: dst,
                retry: Some(Retry::Timeout),
                ..
            } => {
                assert_eq!(dst, node(4));
                now = completes_at;
                retries += 1;
            }
            Action::PeerTimedOut(dst) => {
                assert_eq!(dst, node(4));
                break;
            }
            other => panic!("{other:?}"),
        }
    }
    assert_eq!(retries, 6);
    assert!(matches!(
        bus.enqueue(node(4), 0, 0, Priority::Deadline(0), &[4]),
        Err(Error::Unavailable)
    ));
    assert!(bus
        .enqueue(node(5), 0, 0, Priority::Deadline(0), &[5])
        .is_ok());
    assert!(matches!(bus.poll(now).unwrap(), Action::Transmit { node: dst, .. } if dst == node(5)));
}
