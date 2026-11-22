// SPDX-License-Identifier: GPL-3.0-only
//! Offline read-only subset of Klipper's klippy/webhooks.py.
//! Upstream copyright (C) 2020 Eric Callahan <arksine.code@gmail.com>.
//! No UNIX socket, remote callbacks, command execution, or printer access.

use serde_json::{json, Map, Value};

pub const ETX: u8 = 3;
const MAX_FRAME: usize = 64 * 1024;
const MAX_OBJECTS: usize = 128;
const MAX_FIELDS: usize = 128;
const MAX_TEMPLATE: usize = 4096;
const ENDPOINTS: [&str; 6] = [
    "list_endpoints",
    "info",
    "gcode/help",
    "objects/list",
    "objects/query",
    "objects/subscribe",
];
type Dict = Map<String, Value>;

#[derive(Default)]
pub struct Snapshot {
    /// Offline metadata is explicitly supplied by the caller.
    pub info: Dict,
    pub help: Dict,
    objects: Vec<(String, Dict)>,
}

impl Snapshot {
    /// Insert or replace one object while preserving first-insertion order.
    pub fn set_object(&mut self, name: String, fields: Dict) -> bool {
        if name.is_empty() || name.len() > 128 || fields.len() > MAX_FIELDS {
            return false;
        }
        if let Some((_, old)) = self.objects.iter_mut().find(|(k, _)| k == &name) {
            *old = fields;
            return true;
        }
        if self.objects.len() >= MAX_OBJECTS {
            return false;
        }
        self.objects.push((name, fields));
        true
    }

    fn get_object(&self, name: &str) -> Option<&Dict> {
        self.objects.iter().find(|(k, _)| k == name).map(|(_, v)| v)
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Feed {
    pub requests: usize,
    pub malformed: usize,
    pub oversize: usize,
}

type Selection = Vec<(String, Option<Vec<String>>)>;

struct Subscription {
    selection: Selection,
    template: Dict,
    previous: Dict,
}

#[derive(Default)]
pub struct Session {
    partial: Vec<u8>,
    dropping: bool,
    subscription: Option<Subscription>,
}

impl Session {
    /// Consume fragmented or coalesced ETX-delimited requests. Malformed
    /// requests are discarded. Oversize frames are skipped through the next
    /// delimiter without retaining additional bytes. Output is delivered in
    /// order, one bounded message at a time.
    pub fn feed(
        &mut self,
        input: &[u8],
        source: &Snapshot,
        eventtime: f64,
        mut send: impl FnMut(&[u8]),
    ) -> Feed {
        let mut feed = Feed::default();
        let mut chunks = input.split_inclusive(|&b| b == ETX);
        for chunk in &mut chunks {
            let complete = chunk.last() == Some(&ETX);
            let body = &chunk[..chunk.len() - usize::from(complete)];
            if !self.dropping && body.len() > MAX_FRAME - self.partial.len() {
                self.partial.clear();
                self.dropping = true;
                feed.oversize += 1;
            }
            if !self.dropping {
                self.partial.extend_from_slice(body);
            }
            if !complete {
                continue;
            }
            if self.dropping {
                self.dropping = false;
                continue;
            }
            let parsed = serde_json::from_slice::<Value>(&self.partial);
            self.partial.clear();
            let Ok(Value::Object(mut request)) = parsed else {
                feed.malformed += 1;
                continue;
            };
            let (Some(Value::String(method)), Value::Object(params)) = (
                request.remove("method"),
                request.remove("params").unwrap_or_else(|| json!({})),
            ) else {
                feed.malformed += 1;
                continue;
            };
            feed.requests += 1;
            let result = self.request(source, eventtime, &method, &params);
            if let Some(id) = request.remove("id").filter(|v| !v.is_null()) {
                let reply = match result {
                    Ok(result) => json!({"id": id, "result": result}),
                    Err(message) => json!({"id": id, "error": {
                        "error": "WebRequestError", "message": message,
                    }}),
                };
                if !wire(&reply, &mut send) {
                    feed.oversize += 1;
                }
            }
        }
        feed
    }

    fn request(
        &mut self,
        source: &Snapshot,
        eventtime: f64,
        method: &str,
        params: &Dict,
    ) -> Result<Value, String> {
        match method {
            "list_endpoints" => Ok(json!({"endpoints": ENDPOINTS})),
            "info" => {
                if params.get("client_info").is_some_and(|v| !v.is_object()) {
                    return Err("Invalid Argument Type [client_info]".into());
                }
                Ok(Value::Object(source.info.clone()))
            }
            "gcode/help" => Ok(Value::Object(source.help.clone())),
            "objects/list" => {
                Ok(json!({"objects": source.objects.iter().map(|(k, _)| k).collect::<Vec<_>>()}))
            }
            "objects/query" | "objects/subscribe" => {
                if !eventtime.is_finite() {
                    return Err("Invalid clock".into());
                }
                let mut selection = parse_selection(params)?;
                let template = match params.get("response_template") {
                    None => Dict::new(),
                    Some(Value::Object(map)) => {
                        if serde_json::to_vec(map).map_or(true, |v| v.len() > MAX_TEMPLATE) {
                            return Err("Invalid argument".into());
                        }
                        map.clone()
                    }
                    _ => return Err("Invalid Argument Type [response_template]".into()),
                };
                let status = selections(source, &mut selection);
                if method == "objects/subscribe" {
                    self.subscription = Some(Subscription {
                        selection,
                        template,
                        previous: status.clone(),
                    });
                }
                Ok(json!({"eventtime": eventtime, "status": status}))
            }
            _ => Err(format!(
                "webhooks: No registered callback for path '{method}'"
            )),
        }
    }

    /// Emit one subscription update from a caller-supplied time and snapshot.
    /// Initial values are returned by subscribe; subsequent values are diffs.
    /// The caller owns timer cadence; empty updates are never emitted.
    pub fn update(
        &mut self,
        source: &Snapshot,
        eventtime: f64,
        mut send: impl FnMut(&[u8]),
    ) -> bool {
        let Some(s) = &mut self.subscription else {
            return false;
        };
        if !eventtime.is_finite() {
            return false;
        }
        let current = selections(source, &mut s.selection);
        let mut changes = Dict::new();
        for (key, fields) in &current {
            let fields = fields.as_object().unwrap();
            let old = s.previous.get(key).and_then(Value::as_object);
            let fields: Dict = fields
                .iter()
                .filter(|(k, v)| **v != *old.and_then(|o| o.get(*k)).unwrap_or(&Value::Null))
                .map(|(k, v)| (k.clone(), v.clone()))
                .collect();
            if !fields.is_empty() {
                changes.insert(key.clone(), Value::Object(fields));
            }
        }
        if changes.is_empty() {
            return false;
        }
        let mut response = s.template.clone();
        response.insert(
            "params".into(),
            json!({"eventtime": eventtime, "status": changes}),
        );
        if !wire(&Value::Object(response), &mut send) {
            return false;
        }
        s.previous = current;
        true
    }
}

fn wire(value: &Value, callback: &mut impl FnMut(&[u8])) -> bool {
    if let Ok(mut encoded) = serde_json::to_vec(value) {
        if encoded.len() <= MAX_FRAME {
            encoded.push(ETX);
            callback(&encoded);
            return true;
        }
    }
    false
}

fn parse_selection(params: &Dict) -> Result<Selection, String> {
    let objects = params
        .get("objects")
        .ok_or("Missing Argument [objects]")?
        .as_object()
        .ok_or("Invalid Argument Type [objects]")?;
    if objects.len() > MAX_OBJECTS {
        return Err("Invalid argument".into());
    }
    let mut result = Vec::with_capacity(objects.len());
    for (key, val) in objects {
        let fields = if val.is_null() {
            None
        } else if let Some(a) = val.as_array().filter(|v| v.len() <= MAX_FIELDS) {
            Some(
                a.iter()
                    .map(|v| v.as_str().map(str::to_owned).ok_or("Invalid argument"))
                    .collect::<Result<Vec<_>, _>>()?,
            )
        } else {
            return Err("Invalid argument".into());
        };
        result.push((key.clone(), fields));
    }
    Ok(result)
}

fn selections(source: &Snapshot, selection: &mut Selection) -> Dict {
    let mut status = Dict::new();
    for (object, keys) in selection {
        let existing = source.get_object(object);
        if keys.is_none() {
            if let Some(found) = existing.filter(|o| !o.is_empty()) {
                *keys = Some(found.keys().cloned().collect());
            }
        }
        let values: Dict = keys
            .iter()
            .flatten()
            .map(|key| {
                (
                    key.clone(),
                    existing
                        .and_then(|o| o.get(key))
                        .cloned()
                        .unwrap_or(Value::Null),
                )
            })
            .collect();
        status.insert(object.clone(), Value::Object(values));
    }
    status
}

#[cfg(test)]
mod tests {
    use super::*;

    fn value(frame: &[u8]) -> Value {
        assert_eq!(frame.last(), Some(&ETX));
        serde_json::from_slice(&frame[..frame.len() - 1]).unwrap()
    }

    #[test]
    fn handles_fragments_and_subscribes_to_changed_fields_only() {
        let mut session = Session::default();
        let mut status = Snapshot::default();
        let fields = |x| {
            json!({"position": [x,0,0], "axes":"xyz"})
                .as_object()
                .unwrap()
                .clone()
        };
        assert!(status.set_object("toolhead".into(), fields(1)));
        let mut output = Vec::new();
        assert_eq!(
            session.feed(
                br#"{"id":7,"method":"objects/sub"#,
                &status,
                1.0,
                |_| panic!()
            ),
            Feed::default()
        );
        let suffix = b"scribe\",\"params\":{\"objects\":{\"toolhead\":null},\"response_template\":{\"key\":1}}}\x03";
        session.feed(suffix, &status, 1.0, |b| output.push(value(b)));
        assert_eq!(output[0]["result"]["status"]["toolhead"]["axes"], "xyz");
        assert!(!session.update(&status, 1.25, |_| panic!()));
        assert!(status.set_object("toolhead".into(), fields(4)));
        assert!(session.update(&status, 1.5, |b| output.push(value(b))));
        assert_eq!(
            output[1],
            json!({"key":1,"params":{"eventtime":1.5,"status":{"toolhead":{"position":[4,0,0]}}}})
        );
    }

    #[test]
    fn rejects_mutations_and_resynchronizes_after_overlong_input() {
        let mut session = Session::default();
        let status = Snapshot::default();
        assert_eq!(
            session
                .feed(&vec![b'a'; MAX_FRAME + 1], &status, 1., |_| panic!())
                .oversize,
            1
        );
        let mut out = Vec::new();
        let in_ = b"ignored\x03not json\x03{\"method\":\"gcode/script\",\"id\":1}\x03{\"method\":\"gcode/help\"}\x03";
        let stats = session.feed(in_, &status, 1., |b| out.push(value(b)));
        assert_eq!(
            stats,
            Feed {
                requests: 2,
                malformed: 1,
                oversize: 0
            }
        );
        assert_eq!(out.len(), 1);
        assert_eq!(out[0]["error"]["error"], "WebRequestError");
    }
}
