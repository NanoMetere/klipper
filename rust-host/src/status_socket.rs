// SPDX-License-Identifier: GPL-3.0-only
//! Bounded local Unix transport for the read-only simulated WebHooks subset.
//! The caller must use a private socket directory. No network or printer commands.

use crate::webhooks::{self, Session, Snapshot};
use std::{
    fs,
    io::{self, Read, Write},
    os::unix::{
        fs::{FileTypeExt, MetadataExt},
        net::{UnixListener, UnixStream},
    },
    path::{Path, PathBuf},
};

const MAX_CLIENTS: usize = 8;
const MAX_PENDING: usize = 128 * 1024;
const READ_CHUNK: usize = 8192;
const READ_LIMIT: usize = 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Activity {
    pub accepted: usize,
    pub disconnected: usize,
    pub requests: usize,
}

pub struct StatusSocket {
    server: UnixListener,
    socket: PathBuf,
    identity: (u64, u64),
    clients: Vec<StatusConnection>,
}

pub struct StatusConnection {
    stream: UnixStream,
    session: Session,
    bytes: Vec<u8>,
    offset: usize,
}

impl StatusSocket {
    /// Bind a new socket. An existing path is never removed or replaced.
    pub fn bind(path: &Path) -> io::Result<Self> {
        let file = path
            .file_name()
            .ok_or_else(|| io::Error::from(io::ErrorKind::InvalidInput))?;
        let parent = path
            .parent()
            .filter(|p| !p.as_os_str().is_empty())
            .unwrap_or(Path::new("."));
        let parent = parent.canonicalize()?;
        if fs::metadata(&parent)?.mode() & 0o077 != 0 {
            return Err(io::Error::new(
                io::ErrorKind::PermissionDenied,
                "status socket needs a private directory",
            ));
        }
        let socket = parent.join(file);
        let server = UnixListener::bind(&socket)?;
        let meta = fs::symlink_metadata(&socket)?;
        let out = Self {
            server,
            socket,
            identity: (meta.dev(), meta.ino()),
            clients: Vec::new(),
        };
        out.server.set_nonblocking(true)?;
        Ok(out)
    }

    /// Progress all clients once without blocking or advancing simulated time.
    /// A slow or malformed client cannot grow the reply queue without bound.
    pub fn poll(&mut self, source: &Snapshot, eventtime: f64) -> io::Result<Activity> {
        let mut activity = Activity::default();
        for _ in 0..MAX_CLIENTS {
            match self.server.accept() {
                Ok((stream, _)) if self.clients.len() < MAX_CLIENTS => {
                    match StatusConnection::from_stream(stream) {
                        Ok(connection) => {
                            self.clients.push(connection);
                            activity.accepted += 1;
                        }
                        Err(_) => activity.disconnected += 1,
                    }
                }
                Ok(_) => activity.disconnected += 1,
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(e) => return Err(e),
            }
        }
        self.clients.retain_mut(|client| {
            let keep = match client.poll(source, eventtime) {
                Ok(webhooks::Feed { requests, .. }) => {
                    activity.requests += requests;
                    true
                }
                Err(_) => false,
            };
            activity.disconnected += usize::from(!keep);
            keep
        });
        Ok(activity)
    }
}

impl Drop for StatusSocket {
    fn drop(&mut self) {
        if let Ok(meta) = fs::symlink_metadata(&self.socket) {
            if meta.file_type().is_socket() && (meta.dev(), meta.ino()) == self.identity {
                let _ = fs::remove_file(&self.socket);
            }
        }
    }
}

impl StatusConnection {
    /// Use an existing connected Unix stream instead of opening a named socket.
    pub fn from_stream(stream: UnixStream) -> io::Result<Self> {
        stream.set_nonblocking(true)?;
        Ok(Self {
            stream,
            session: Session::default(),
            bytes: Vec::new(),
            offset: 0,
        })
    }

    pub fn poll(&mut self, source: &Snapshot, eventtime: f64) -> io::Result<webhooks::Feed> {
        // Move only the queued tail on a new poll. The cap covers everything
        // retained for this socket, irrespective of the client's read pace.
        if self.offset != 0 {
            self.bytes.drain(..self.offset);
            self.offset = 0;
        }
        let mut aggregate = webhooks::Feed::default();
        let mut exceeded = false;
        let mut enqueue = |message: &[u8]| {
            if !exceeded && self.bytes.len() + message.len() <= MAX_PENDING {
                self.bytes.extend_from_slice(message);
            } else {
                exceeded = true;
            }
        };
        let mut read = [0; READ_CHUNK];
        let mut closed = false;
        for _ in 0..READ_LIMIT {
            let size = match self.stream.read(&mut read) {
                Ok(0) => {
                    closed = true;
                    break;
                }
                Ok(n) => n,
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(e) => return Err(e),
            };
            let newest = self
                .session
                .feed(&read[..size], source, eventtime, &mut enqueue);
            aggregate.requests += newest.requests;
            aggregate.malformed += newest.malformed;
            aggregate.oversize += newest.oversize;
        }
        self.session.update(source, eventtime, &mut enqueue);
        if exceeded {
            return Err(io::Error::from(io::ErrorKind::OutOfMemory));
        }
        if !self.bytes.is_empty() {
            match self.stream.write(&self.bytes) {
                Ok(0) => return Err(io::Error::from(io::ErrorKind::WriteZero)),
                Ok(n) => self.offset = n,
                Err(e)
                    if matches!(
                        e.kind(),
                        io::ErrorKind::WouldBlock | io::ErrorKind::Interrupted
                    ) => {}
                Err(e) => return Err(e),
            }
        }
        if closed && self.offset == self.bytes.len() {
            return Err(io::Error::from(io::ErrorKind::UnexpectedEof));
        }
        Ok(aggregate)
    }
}
