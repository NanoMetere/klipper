// SPDX-License-Identifier: GPL-3.0-only
//! Offline toolchange lifecycle based on the viesturz/klipper-toolchanger
//! selection order. Upstream copyright (C) 2023 Viesturs Zarins.
//! Actions are purely descriptive and never run G-code, macros, or hardware.

#[path = "toolchange/catalog.rs"]
mod catalog;
pub use catalog::{Catalog, Tool};

const MAX_TOOLS: usize = 64;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Uninitialized,
    Ready,
    Changing,
    Faulted,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Config,
    State,
    Tool,
    NotHomed,
    Stale,
    Detection,
    Failed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    Save,
    Before(Option<u8>),
    SelectDockTransform,
    SuspendMissingWatch,
    Dropoff(u8),
    ApplyToolConfig(Option<u8>),
    Pickup(u8),
    Verify(u8),
    ResumeMissingWatch(u8),
    After(u8),
    Restore,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ticket {
    pub generation: u64,
    pub step: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Request {
    pub ticket: Ticket,
    pub action: Action,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Completion {
    Completed,
    Detected(Option<u8>),
    Failed,
}

#[derive(Debug, Clone, Copy)]
pub struct Policy {
    pub axes: u8,
    pub allow_empty: bool,
    pub verify_pickup: bool,
}

struct Change {
    target: Option<u8>,
    operations: Vec<Action>,
    step: usize,
}

pub struct Changer {
    tools: Vec<u8>,
    policy: Policy,
    state: State,
    active: Option<u8>,
    generation: u64,
    pending: Option<Change>,
}

impl Changer {
    pub fn new(mut tools: Vec<u8>, policy: Policy) -> Result<Self, Error> {
        if !(1..=MAX_TOOLS).contains(&tools.len()) || !(1..=7).contains(&policy.axes) {
            return Err(Error::Config);
        }
        tools.sort_unstable();
        if tools.windows(2).any(|ids| ids[0] == ids[1]) {
            return Err(Error::Config);
        }
        Ok(Self {
            tools,
            policy,
            state: State::Uninitialized,
            active: None,
            generation: 0,
            pending: None,
        })
    }

    pub fn state(&self) -> State {
        self.state
    }

    /// Indicates a verified tool identity only after initialization or a
    /// completed transaction. While changing/faulted, it is not a sensor result.
    pub fn active(&self) -> Option<u8> {
        self.active
    }

    /// Fixture supplies explicit observed identity, including after a fault.
    pub fn initialize_from_detection(&mut self, tool: Option<u8>) -> Result<(), Error> {
        if self.state == State::Changing {
            return Err(Error::State);
        }
        self.check_target(tool)?;
        self.state = State::Ready;
        self.active = tool;
        Ok(())
    }

    pub fn select(&mut self, tool: Option<u8>, homed_axes: u8) -> Result<Option<Request>, Error> {
        if self.state != State::Ready {
            return Err(Error::State);
        }
        self.check_target(tool)?;
        if self.active == tool {
            return Ok(None);
        }
        if homed_axes & self.policy.axes != self.policy.axes {
            return Err(Error::NotHomed);
        }
        self.generation = self.generation.checked_add(1).ok_or(Error::State)?;
        let mut operations = Vec::with_capacity(11);
        operations.extend([
            Action::Save,
            Action::Before(self.active),
            Action::SelectDockTransform,
            Action::SuspendMissingWatch,
        ]);
        if let Some(active) = self.active {
            operations.push(Action::Dropoff(active));
        }
        operations.push(Action::ApplyToolConfig(tool));
        if let Some(pickup) = tool {
            operations.push(Action::Pickup(pickup));
            if self.policy.verify_pickup {
                operations.push(Action::Verify(pickup));
            }
            operations.extend([Action::ResumeMissingWatch(pickup), Action::After(pickup)]);
        }
        operations.push(Action::Restore);
        self.state = State::Changing;
        self.pending = Some(Change {
            target: tool,
            operations,
            step: 0,
        });
        Ok(self.current())
    }

    pub fn current(&self) -> Option<Request> {
        let change = self.pending.as_ref()?;
        Some(Request {
            ticket: Ticket {
                generation: self.generation,
                step: change.step,
            },
            action: change.operations[change.step],
        })
    }

    pub fn finish_step(
        &mut self,
        ticket: Ticket,
        completion: Completion,
    ) -> Result<Option<Request>, Error> {
        let request = self.current().ok_or(Error::Stale)?;
        if request.ticket != ticket {
            return Err(Error::Stale);
        }
        match (request.action, completion) {
            (_, Completion::Failed) => return self.abort(Error::Failed),
            (Action::Verify(wanted), Completion::Detected(Some(got))) if wanted == got => (),
            (Action::Verify(_), _) => return self.abort(Error::Detection),
            (_, Completion::Completed) => (),
            (_, Completion::Detected(_)) => return Err(Error::State),
        }
        let change = self.pending.as_mut().ok_or(Error::State)?;
        change.step += 1;
        if change.step == change.operations.len() {
            self.active = change.target;
            self.pending = None;
            self.state = State::Ready;
        }
        Ok(self.current())
    }

    fn check_target(&self, tool: Option<u8>) -> Result<(), Error> {
        match tool {
            Some(id) if self.tools.binary_search(&id).is_ok() => Ok(()),
            None if self.policy.allow_empty => Ok(()),
            _ => Err(Error::Tool),
        }
    }

    fn abort<T>(&mut self, error: Error) -> Result<T, Error> {
        self.active = None;
        self.pending = None;
        self.state = State::Faulted;
        Err(error)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn virtual_pickup_requires_matching_observation_and_recovers_explicitly() {
        let policy = Policy {
            axes: 7,
            allow_empty: false,
            verify_pickup: true,
        };
        let mut c = Changer::new(vec![1, 4], policy).unwrap();
        c.initialize_from_detection(Some(1)).unwrap();
        assert_eq!(c.select(Some(4), 3), Err(Error::NotHomed));
        let mut req = c.select(Some(4), 7).unwrap().unwrap();
        assert_eq!(req.action, Action::Save);
        assert_eq!(c.active(), Some(1));
        let first = req.ticket;
        while !matches!(req.action, Action::Verify(_)) {
            req = c
                .finish_step(req.ticket, Completion::Completed)
                .unwrap()
                .unwrap();
        }
        assert_eq!(
            c.finish_step(first, Completion::Detected(Some(4))),
            Err(Error::Stale)
        );
        assert_eq!(
            c.finish_step(req.ticket, Completion::Detected(Some(1))),
            Err(Error::Detection)
        );
        assert_eq!(c.state(), State::Faulted);
        assert_eq!(c.active(), None);
        assert_eq!(c.select(Some(4), 7), Err(Error::State));
        c.initialize_from_detection(Some(4)).unwrap();
        assert_eq!(c.select(Some(4), 0), Ok(None));
    }
}
