// SPDX-License-Identifier: GPL-3.0-only
//! Bounded software-peer writer. Never merge commands across peer switches.
use klipper_host_protocol::MAX_PAYLOAD;

pub(super) struct Batch {
    data: [u8; MAX_PAYLOAD],
    used: usize,
    owner: usize,
    count: usize,
}

impl Default for Batch {
    fn default() -> Self {
        Self {
            data: [0; MAX_PAYLOAD],
            used: 0,
            owner: 0,
            count: 0,
        }
    }
}

impl Batch {
    pub(super) fn push(
        &mut self,
        board: usize,
        command: &[u8],
        send: &mut impl FnMut(usize, &[u8], usize) -> Result<(), String>,
    ) -> Result<(), String> {
        if command.is_empty() || command.len() > MAX_PAYLOAD {
            return Err("invalid fixture command length".into());
        }
        if self.used != 0 && (self.owner != board || self.used + command.len() > MAX_PAYLOAD) {
            self.flush(send)?;
        }
        self.owner = board;
        self.data[self.used..self.used + command.len()].copy_from_slice(command);
        self.used += command.len();
        self.count += 1;
        Ok(())
    }

    pub(super) fn flush(
        &mut self,
        send: &mut impl FnMut(usize, &[u8], usize) -> Result<(), String>,
    ) -> Result<(), String> {
        if self.used != 0 {
            // Consume before transmitting. A failed ACK must not permit
            // this software-only buffer to replay the same command.
            let len = std::mem::take(&mut self.used);
            let count = std::mem::take(&mut self.count);
            send(self.owner, &self.data[..len], count)?;
        }
        Ok(())
    }
}
