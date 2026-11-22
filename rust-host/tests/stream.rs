// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::{
    encode,
    stream::{Received, Receiver},
    MAX_PAYLOAD,
};

#[test]
fn matches_every_split_and_preserves_sequence_wrap() {
    let expected = [(15, vec![0x7e; MAX_PAYLOAD]), (0, vec![]), (1, vec![1, 2])];
    let wire: Vec<u8> = expected
        .iter()
        .flat_map(|(seq, p)| encode(*seq, p).unwrap())
        .collect();
    for split in 0..=wire.len() {
        let mut rx = Receiver::new();
        let mut actual = Vec::new();
        for part in [&wire[..split], &wire[split..]] {
            let result = rx.feed(part, |frame| {
                actual.push((frame.sequence, frame.payload.to_vec()))
            });
            assert_eq!(result.discarded, 0);
            assert!(rx.pending().len() < 64);
        }
        assert_eq!(actual, expected);
        assert!(rx.pending().is_empty());
        assert!(!rx.seeking_sync());
    }
}

#[test]
fn discards_through_next_sync_even_beyond_bounded_buffer() {
    let mut rx = Receiver::new();
    assert_eq!(rx.feed(&[0xff; 3], |_| unreachable!()), Received::default());
    let mut data = vec![0xff; 120];
    data.push(0x7e);
    data.extend(encode(7, b"recover").unwrap());
    assert_eq!(
        rx.feed(&data, |frame| assert_eq!(frame.payload, b"recover")),
        Received {
            frames: 1,
            discarded: 124
        }
    );
    assert!(!rx.seeking_sync());
    assert!(rx.pending().is_empty());
}

#[test]
fn carries_sync_state_across_short_reads() {
    let mut rx = Receiver::new();
    assert_eq!(rx.feed(&[0x00; 99], |_| unreachable!()).discarded, 99);
    assert!(rx.seeking_sync());
    assert_eq!(rx.feed(&[0x7e, 0xff], |_| unreachable!()).discarded, 0);
    assert_eq!(rx.pending(), &[0x7e, 0xff]);
    let part = encode(2, b"ok").unwrap();
    let mut input = vec![0xff, 0x7e];
    input.extend(part);
    assert_eq!(
        rx.feed(&input, |f| assert_eq!(f.sequence, 2)),
        Received {
            frames: 1,
            discarded: 4
        }
    );
    assert!(!rx.seeking_sync());
}
