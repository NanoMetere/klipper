// SPDX-License-Identifier: GPL-3.0-only
//! Offline Classic CAN transport pieces matching Klipper's host packetization.
//! No socket, interface setup, UUID assignment, or physical bus access.

use crate::{
    flight::FRAME_WINDOW,
    stream::{Received, Receiver},
    Frame, MAX_FRAME,
};

const CAN_PAYLOAD: usize = 8;
const FIRST_NODE: u8 = 4;
const MAX_WRITE: usize = FRAME_WINDOW * MAX_FRAME + 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Node,
    DuplicateNode,
    Frame,
    WriteLimit,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Node(u8);

impl Node {
    pub fn new(id: u8) -> Result<Self, Error> {
        if id < FIRST_NODE {
            Err(Error::Node)
        } else {
            Ok(Self(id))
        }
    }

    pub const fn host_id(self) -> u16 {
        0x100 + self.0 as u16 * 2
    }

    pub const fn response_id(self) -> u16 {
        self.host_id() + 1
    }

    /// Split bytes from one bounded Klipper write into Classic CAN data fields.
    /// Adjacent chunks are at most eight bytes, as in stock.
    pub fn fragments(self, write: &[u8]) -> Result<impl Iterator<Item = Datagram> + '_, Error> {
        if write.len() > MAX_WRITE {
            return Err(Error::WriteLimit);
        }
        Ok(write
            .chunks(CAN_PAYLOAD)
            .map(move |payload| Datagram::from_valid(self.host_id(), payload)))
    }
}

/// Standard 11-bit Classic CAN data only; not OS struct layout or CAN FD.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Datagram {
    id: u16,
    data: [u8; CAN_PAYLOAD],
    len: u8,
}

impl Datagram {
    pub fn new(raw_id: u32, data: &[u8]) -> Result<Self, Error> {
        if raw_id > 0x7ff || data.len() > CAN_PAYLOAD {
            return Err(Error::Frame);
        }
        Ok(Self::from_valid(raw_id as u16, data))
    }

    fn from_valid(id: u16, data: &[u8]) -> Self {
        let mut bytes = [0; CAN_PAYLOAD];
        bytes[..data.len()].copy_from_slice(data);
        Self {
            id,
            data: bytes,
            len: data.len() as u8,
        }
    }

    pub const fn id(&self) -> u16 {
        self.id
    }

    pub fn data(&self) -> &[u8] {
        &self.data[..usize::from(self.len)]
    }
}

/// Independent stream decoder per assigned node. Frames not matching a
/// response ID are ignored and never mutate another node's partial input.
pub struct Demux {
    receiver: [Option<Receiver>; 256],
}

impl Default for Demux {
    fn default() -> Self {
        Self {
            receiver: std::array::from_fn(|_| None),
        }
    }
}

impl Demux {
    pub fn register(&mut self, node: Node) -> Result<(), Error> {
        let slot = &mut self.receiver[usize::from(node.0)];
        if slot.is_some() {
            return Err(Error::DuplicateNode);
        }
        *slot = Some(Receiver::new());
        Ok(())
    }

    /// A disconnected node loses its partial frame, without affecting others.
    pub fn remove(&mut self, node: Node) {
        self.receiver[usize::from(node.0)] = None;
    }

    pub fn feed(&mut self, packet: Datagram, on_frame: impl FnMut(Frame<'_>)) -> Option<Received> {
        let raw = packet.id().checked_sub(0x101)?;
        if !raw.is_multiple_of(2) {
            return None;
        }
        let slot = self.receiver.get_mut(usize::from(raw / 2))?.as_mut()?;
        Some(slot.feed(packet.data(), on_frame))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::encode;

    #[test]
    fn isolates_partial_streams_and_rejects_wrong_ids_flags_and_lengths() {
        let a = Node::new(4).unwrap();
        let b = Node::new(255).unwrap();
        assert_eq!(
            (a.host_id(), a.response_id(), b.response_id()),
            (264, 265, 767)
        );
        assert_eq!(Node::new(3), Err(Error::Node));
        assert_eq!(Datagram::new(0x8000_0109, &[1]), Err(Error::Frame));
        assert_eq!(Datagram::new(265, &[1; 9]), Err(Error::Frame));
        let mut mux = Demux::default();
        mux.register(a).unwrap();
        mux.register(b).unwrap();
        assert_eq!(mux.register(b), Err(Error::DuplicateNode));
        let wa = encode(14, &[9; 7]).unwrap();
        let wb = encode(2, &[2; 4]).unwrap();
        let mut received = Vec::new();
        let mut feed = |id: u16, bytes| {
            mux.feed(Datagram::new(id.into(), bytes).unwrap(), |f| {
                received.push((f.sequence, f.payload.to_vec()))
            })
        };
        assert_eq!(feed(a.host_id(), &wa[..8]), None);
        feed(a.response_id(), &wa[..8]);
        feed(b.response_id(), &wb[..8]);
        feed(a.response_id(), &wa[8..]);
        feed(b.response_id(), &wb[8..]);
        assert_eq!(received, [(14, vec![9; 7]), (2, vec![2; 4])]);
    }
}
