// SPDX-License-Identifier: GPL-3.0-only
#![cfg(feature = "identify")]

use flate2::{write::ZlibEncoder, Compression};
use klipper_host_protocol::{
    clock::{ClockMessages, Response},
    decode_prefix,
    identify::{Error, Identify},
    Frame,
};
use std::io::Write;

fn unhex(s: &str) -> Vec<u8> {
    assert_eq!(s.len() % 2, 0);
    s.as_bytes()
        .chunks_exact(2)
        .map(|c| u8::from_str_radix(std::str::from_utf8(c).unwrap(), 16).unwrap())
        .collect()
}

fn upstream_fixture() -> Identify {
    let mut reader = Identify::default();
    let mut done = false;
    for (index, line) in include_str!("fixtures/identify_wire.txt")
        .lines()
        .enumerate()
    {
        let (request, response) = line.split_once(' ').unwrap();
        let seq = (index % 16) as u8;
        assert!(!done);
        assert_eq!(reader.request(seq).unwrap(), unhex(request));
        let response = unhex(response);
        let (frame, used) = decode_prefix(&response).unwrap().unwrap();
        assert_eq!(used, response.len());
        done = reader.accept(frame).unwrap();
    }
    assert!(done);
    reader
}

#[test]
fn replays_upstream_identify_frames() {
    let reader = upstream_fixture();
    let dictionary = reader.dictionary().unwrap();
    assert!(dictionary
        .commands
        .contains_key("identify offset=%u count=%c"));
    assert!(dictionary
        .responses
        .contains_key("identify_response offset=%u data=%.*s"));
    assert_eq!(dictionary.version, "native-linux-identify-fixture");
    assert_eq!(dictionary.config["MCU"], "linux");
    assert_eq!(dictionary.commands.len(), 121);
    assert_eq!(dictionary.responses.len(), 26);
    assert_eq!(dictionary.commands.get("get_clock"), Some(&5));
    assert_eq!(dictionary.responses.get("clock clock=%u"), Some(&-1));
    assert!(matches!(reader.request(0), Err(Error::AlreadyComplete)));
}

#[test]
fn rejected_chunks_do_not_advance() {
    let mut reader = Identify::default();
    let initial = reader.request(1).unwrap();
    let mut oversized = vec![0, 0, 41];
    oversized.extend([0; 41]);
    for (payload, expected) in [
        (&[1, 0, 0][..], "UnexpectedMessage"),
        (&[0, 0x80][..], "InvalidOffset"),
        (&[0, 0x60, 0][..], "InvalidOffset"),
        (&[0, 0x80, 0x80, 0x80, 0x80, 0x80, 0][..], "InvalidOffset"),
        (&[0, 1, 0][..], "OffsetMismatch"),
        (&[0, 0, 2, 9][..], "InvalidChunk"),
        (&oversized, "InvalidChunk"),
    ] {
        let error = reader
            .accept(Frame {
                sequence: 0,
                payload,
            })
            .unwrap_err();
        assert!(format!("{error:?}").starts_with(expected));
        assert_eq!(reader.request(1).unwrap(), initial);
    }
    assert!(matches!(reader.dictionary(), Err(Error::Incomplete)));
}

fn compress(data: &[u8]) -> Vec<u8> {
    let mut z = ZlibEncoder::new(Vec::new(), Compression::fast());
    z.write_all(data).unwrap();
    z.finish().unwrap()
}

// Construct response payloads using the codec for the accepted request offsets,
// replacing only the bootstrap message ID and requested count.
fn from_chunks(data: &[u8]) -> Result<Identify, Error> {
    let mut reader = Identify::default();
    for chunk in data.chunks(40).chain(std::iter::once(&[][..])) {
        let request = reader.request(0)?;
        let (frame, _) = decode_prefix(&request).unwrap().unwrap();
        let mut payload = frame.payload.to_vec();
        payload[0] = 0;
        *payload.last_mut().unwrap() = chunk.len() as u8;
        payload.extend_from_slice(chunk);
        reader.accept(Frame {
            sequence: 0,
            payload: &payload,
        })?;
    }
    Ok(reader)
}

#[test]
fn enforces_dictionary_size_and_integrity() {
    assert!(matches!(
        from_chunks(&vec![0; 256 * 1024 + 1]),
        Err(Error::CompressedLimit)
    ));
    let large = compress(&vec![b' '; 2 * 1024 * 1024 + 16]);
    assert!(matches!(
        from_chunks(&large).unwrap().dictionary(),
        Err(Error::JsonLimit)
    ));
    assert!(matches!(
        from_chunks(&compress(b"{}")).unwrap().dictionary(),
        Err(Error::InvalidJson(_))
    ));
    let mut ok = compress(br#"{"commands":{},"responses":{}}"#);
    assert!(from_chunks(&ok).unwrap().dictionary().is_ok());
    assert!(matches!(
        from_chunks(&ok[..ok.len() - 1]).unwrap().dictionary(),
        Err(Error::InvalidCompression)
    ));
    ok.push(0);
    assert!(matches!(
        from_chunks(&ok).unwrap().dictionary(),
        Err(Error::InvalidCompression)
    ));
}

#[test]
fn uses_dictionary_ids_and_reads_clock_wrap_cases() {
    let dictionary = upstream_fixture().dictionary().unwrap();
    let messages = ClockMessages::from_dictionary(&dictionary).unwrap();
    for (index, line) in include_str!("fixtures/clock_wire.txt").lines().enumerate() {
        let mut fields = line.split_whitespace();
        let kind = fields.next().unwrap();
        let high: u32 = fields.next().unwrap().parse().unwrap();
        let low: u32 = fields.next().unwrap().parse().unwrap();
        let request = unhex(fields.next().unwrap());
        let response = unhex(fields.next().unwrap());
        let seq = (index / 2) as u8;
        let (actual_request, expected) = if kind == "clock" {
            (messages.get_clock(seq).unwrap(), Response::Clock(low))
        } else {
            (
                messages.get_uptime(seq).unwrap(),
                Response::Uptime((u64::from(high) << 32) | u64::from(low)),
            )
        };
        assert_eq!(actual_request, request);
        let (frame, _) = decode_prefix(&response).unwrap().unwrap();
        assert_eq!(messages.response(frame), Ok(expected));
    }
}
