// SPDX-License-Identifier: GPL-3.0-only
//! Differential test driver linked against stock Klippy msgblock.c.
use klipper_host_protocol::{
    encode,
    stream::{Received, Receiver},
    MIN_FRAME,
};

unsafe extern "C" {
    fn msgblock_check(need_sync: *mut u8, buf: *mut u8, buf_len: i32) -> i32;
}

#[derive(Default)]
struct Reference {
    pending: Vec<u8>,
    need_sync: u8,
}

type Record = (u8, Vec<u8>);

impl Reference {
    fn feed(&mut self, bytes: &[u8], decoded: &mut Vec<Record>) -> Received {
        self.pending.extend_from_slice(bytes);
        let mut start = 0;
        let mut counts = Received::default();
        while start < self.pending.len() {
            let buf = &mut self.pending[start..];
            // Stock C does not mutate its buffer despite the non-const argument.
            let n = unsafe {
                msgblock_check(
                    &mut self.need_sync,
                    buf.as_mut_ptr(),
                    i32::try_from(buf.len()).unwrap(),
                )
            };
            if n == 0 {
                break;
            }
            if n > 0 {
                let len = n as usize;
                decoded.push((buf[1] & 15, buf[2..len - (MIN_FRAME - 2)].to_vec()));
                counts.frames += 1;
            } else {
                counts.discarded += n.unsigned_abs() as usize;
            }
            start += n.unsigned_abs() as usize;
        }
        self.pending.drain(..start);
        counts
    }
}

fn random(rng: &mut u32) -> u32 {
    *rng ^= *rng << 13;
    *rng ^= *rng >> 17;
    *rng ^= *rng << 5;
    *rng
}

fn main() {
    let mut checked = 0;
    for seed in 1u32..=4096 {
        let mut rng = seed;
        let mut stream = Vec::new();
        for frame in 0..48 {
            let len = (random(&mut rng) % 60) as usize;
            let payload: Vec<_> = (0..len).map(|_| random(&mut rng) as u8).collect();
            let mut wire = encode(frame, &payload).unwrap();
            match random(&mut rng) % 8 {
                0 => wire[0] = 0xff,
                1 => wire[len + 2] ^= 1,
                2 => {
                    let n = (random(&mut rng) % 130) as usize;
                    stream.extend((0..n).map(|_| random(&mut rng) as u8));
                    stream.push(0x7e);
                }
                3 => {
                    wire.pop();
                }
                _ => {}
            }
            stream.extend(wire);
        }
        let mut rust = Receiver::new();
        let mut c = Reference::default();
        let mut index = 0;
        while index < stream.len() {
            let step = match seed % 5 {
                0 => 1,
                1 => 64,
                _ => (random(&mut rng) % 161) as usize,
            };
            let end = stream.len().min(index + step);
            let bytes = &stream[index..end];
            let mut a = Vec::new();
            let mut b = Vec::new();
            let rust_stats = rust.feed(bytes, |frame| {
                a.push((frame.sequence, frame.payload.to_vec()))
            });
            let c_stats = c.feed(bytes, &mut b);
            assert_eq!(
                (
                    rust_stats,
                    a.as_slice(),
                    rust.pending(),
                    rust.seeking_sync()
                ),
                (
                    c_stats,
                    b.as_slice(),
                    c.pending.as_slice(),
                    c.need_sync != 0
                ),
                "seed={seed}, range={index}..{end}",
            );
            index = end;
            checked += 1;
        }
    }
    println!("matched {checked} reads across 4096 streams");
}
