// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::{decode_prefix, identify::Identify};
use std::io::{self, Read};

fn run() -> Result<(), Box<dyn std::error::Error>> {
    // Input is an offline sequence of raw inbound frames, not a serial device.
    const MAX_REPLAY: u64 = 1024 * 1024;
    let mut bytes = Vec::new();
    io::stdin().take(MAX_REPLAY + 1).read_to_end(&mut bytes)?;
    if bytes.len() as u64 > MAX_REPLAY {
        return Err("replay exceeds 1 MiB".into());
    }
    let mut input = bytes.as_slice();
    let mut identify = Identify::default();
    while !input.is_empty() {
        let (frame, count) = decode_prefix(input)
            .map_err(|e| format!("bad frame: {e:?}"))?
            .ok_or("truncated final frame")?;
        if !frame.payload.is_empty() {
            identify.accept(frame).map_err(|e| format!("identify: {e:?}"))?;
        }
        input = &input[count..];
    }
    let dictionary = identify.dictionary().map_err(|e| format!("dictionary: {e:?}"))?;
    println!("{}: {} commands, {} responses, {} output formats",
             dictionary.version, dictionary.commands.len(),
             dictionary.responses.len(), dictionary.output.len());
    Ok(())
}

fn main() -> std::process::ExitCode {
    match run() {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("{error}");
            std::process::ExitCode::FAILURE
        }
    }
}
