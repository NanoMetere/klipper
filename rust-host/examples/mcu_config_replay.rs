// SPDX-License-Identifier: GPL-3.0-only
//! Inert config payload replay from a provided firmware dictionary.

use klipper_host_protocol::{
    identify::Dictionary,
    mcu_config::{Messages, Profile},
};
use std::{
    env, fs,
    io::{self, Write},
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = env::args_os()
        .nth(1)
        .ok_or("expected a firmware dictionary file")?;
    let dictionary: Dictionary = serde_json::from_slice(&fs::read(path)?)?;
    let codec = Messages::from_dictionary(&dictionary).map_err(|e| format!("{e:?}"))?;
    let mut output = io::BufWriter::new(io::stdout().lock());
    for count in 0..=u8::MAX {
        let profile = Profile::trsync_only(count, 3);
        write!(output, "{count} {}", profile.crc())?;
        for payload in codec.payloads(&profile) {
            write!(output, " ")?;
            for byte in payload {
                write!(output, "{byte:02x}")?;
            }
        }
        writeln!(output)?;
    }
    Ok(())
}
