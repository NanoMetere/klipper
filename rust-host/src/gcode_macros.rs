// SPDX-License-Identifier: GPL-3.0-only
//! Bounded offline expansion of a subset of Klipper gcode_macro sections.
//! Adapted from the dispatch semantics of klippy/extras/gcode_macro.py.
//! Upstream copyright (C) 2018-2021 Kevin O'Connor <kevin@koconnor.net>.
//! No Jinja execution, printer object access, external calls or emitted I/O.

use crate::config::Config;
use std::collections::BTreeMap;

const MAX_MACROS: usize = 512;
const MAX_BODY: usize = 64 * 1024;
const MAX_SCRIPT: usize = 8 * 1024 * 1024;
const MAX_LINES: usize = 32_768;
const MAX_LINE: usize = 4096;
const MAX_NESTING: usize = 32;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Config,
    UnsupportedTemplate,
    Arguments,
    Recursion,
    Limit,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Error {
    pub kind: Kind,
    /// One-based top-level script line. Zero denotes a configuration error.
    pub line: usize,
    pub macro_name: Option<String>,
}

#[derive(Debug)]
enum Template {
    Supported(Vec<Part>),
    Unsupported,
}

#[derive(Debug)]
enum Part {
    Text(String),
    Number {
        parameter: String,
        default: Option<String>,
    },
}

pub struct Registry {
    macros: BTreeMap<String, Template>,
}

impl Registry {
    /// Register only extended-name macros. A registered but unsupported
    /// macro fails when invoked; unrelated templates do not block a preview.
    pub fn from_config(config: &Config) -> Result<Self, Error> {
        let mut macros = BTreeMap::new();
        for section in config.sections() {
            let Some(suffix) = section.strip_prefix("gcode_macro ") else {
                continue;
            };
            let name = suffix.trim().to_ascii_uppercase();
            if macros.len() >= MAX_MACROS {
                return Err(error(Kind::Limit, 0, None));
            }
            if !(macro_name(&name) || traditional(&name)) || macros.contains_key(&name) {
                return Err(error(Kind::Config, 0, Some(&name)));
            }
            let script = config
                .get(section, "gcode")
                .ok_or_else(|| error(Kind::Config, 0, Some(&name)))?;
            if script.len() > MAX_BODY {
                return Err(error(Kind::Limit, 0, Some(&name)));
            }
            let variables = [section, "DEFAULT"].into_iter().any(|s| {
                config
                    .values(s)
                    .is_some_and(|v| v.keys().any(|k| k.starts_with("variable_")))
            });
            let rendered = if traditional(&name)
                || reserved(&name)
                || config.get(section, "rename_existing").is_some()
                || variables
            {
                Template::Unsupported
            } else {
                compile(script).map_or(Template::Unsupported, Template::Supported)
            };
            macros.insert(name, rendered);
        }
        Ok(Self { macros })
    }

    /// Preflight and expand recognized calls in memory. The complete result
    /// is returned only on success; there are no partial side effects.
    pub fn expand(&self, script: &str) -> Result<String, Error> {
        if script.len() > MAX_SCRIPT {
            return Err(error(Kind::Limit, 0, None));
        }
        let mut out = String::with_capacity(script.len().min(64 * 1024));
        let mut expanded = 0;
        let mut active = Vec::new();
        for (index, line) in script.lines().enumerate() {
            self.expand_line(line, index + 1, &mut active, &mut out, &mut expanded)?;
        }
        Ok(out)
    }

    fn expand_line<'a>(
        &'a self,
        line: &str,
        root: usize,
        active: &mut Vec<&'a str>,
        out: &mut String,
        expanded: &mut usize,
    ) -> Result<(), Error> {
        if line.len() > MAX_LINE || *expanded >= MAX_LINES {
            return Err(error(Kind::Limit, root, active.last().copied()));
        }
        *expanded += 1;
        if line.contains(['\0', '\r', '\n']) {
            return Err(error(Kind::Arguments, root, active.last().copied()));
        }
        let head = line.split(';').next().unwrap().trim();
        let name = head
            .split_ascii_whitespace()
            .next()
            .unwrap_or_default()
            .to_ascii_uppercase();
        let Some((key, template)) = self.macros.get_key_value(&name) else {
            if out.len() + line.len() + 1 > MAX_SCRIPT {
                return Err(error(Kind::Limit, root, active.last().copied()));
            }
            out.push_str(line);
            out.push('\n');
            return Ok(());
        };
        if active.contains(&key.as_str()) {
            return Err(error(Kind::Recursion, root, Some(key)));
        }
        if active.len() >= MAX_NESTING {
            return Err(error(Kind::Limit, root, Some(key)));
        }
        let Template::Supported(parts) = template else {
            return Err(error(Kind::UnsupportedTemplate, root, Some(key)));
        };
        let mut parameters = BTreeMap::new();
        let raw = head[name.len()..].split('#').next().unwrap();
        for word in raw.split_ascii_whitespace() {
            let (param, value) = word
                .split_once('=')
                .ok_or_else(|| error(Kind::Arguments, root, Some(key)))?;
            let param = param.to_ascii_uppercase();
            if !parameter_name(&param)
                || !number(value)
                || parameters.insert(param, value).is_some()
            {
                return Err(error(Kind::Arguments, root, Some(key)));
            }
        }
        let mut rendered = String::new();
        for part in parts {
            let s = match part {
                Part::Text(s) => s.as_str(),
                Part::Number { parameter, default } => parameters
                    .get(parameter)
                    .copied()
                    .or(default.as_deref())
                    .ok_or_else(|| error(Kind::Arguments, root, Some(key)))?,
            };
            if rendered.len() + s.len() > MAX_BODY {
                return Err(error(Kind::Limit, root, Some(key)));
            }
            rendered.push_str(s);
        }
        active.push(key);
        for inner in rendered.lines() {
            self.expand_line(inner, root, active, out, expanded)?;
        }
        active.pop();
        Ok(())
    }
}

fn error(kind: Kind, line: usize, name: Option<&str>) -> Error {
    Error {
        kind,
        line,
        macro_name: name.map(str::to_owned),
    }
}

fn macro_name(s: &str) -> bool {
    let name = s.as_bytes();
    (2..=64).contains(&name.len())
        && (name[0].is_ascii_uppercase() || name[0] == b'_')
        && !name[1].is_ascii_digit()
        && name.iter().all(|c| c.is_ascii_alphanumeric() || *c == b'_')
}

fn traditional(s: &str) -> bool {
    let name = s.as_bytes();
    (2..=64).contains(&name.len())
        && name[0].is_ascii_uppercase()
        && name[1].is_ascii_digit()
        && number(&s[1..])
}

fn reserved(s: &str) -> bool {
    matches!(
        s,
        "SAVE_GCODE_STATE"
            | "RESTORE_GCODE_STATE"
            | "SET_GCODE_OFFSET"
            | "SET_GCODE_VARIABLE"
            | "STATUS"
            | "RESTART"
            | "FIRMWARE_RESTART"
            | "HELP"
            | "PAUSE"
            | "RESUME"
            | "CANCEL_PRINT"
    )
}

fn parameter_name(s: &str) -> bool {
    let name = s.as_bytes();
    (1..=64).contains(&name.len())
        && name[0].is_ascii_uppercase()
        && name.iter().all(|c| c.is_ascii_alphanumeric() || *c == b'_')
}

// Strict finite decimal literals only. This avoids injecting lines or words
// through template values and keeps G-code semantics independent of Jinja.
fn number(s: &str) -> bool {
    (1..=64).contains(&s.len())
        && s.bytes()
            .all(|c| c.is_ascii_digit() || matches!(c, b'+' | b'-' | b'.'))
        && s.parse::<f64>().is_ok_and(f64::is_finite)
}

fn compile(mut text: &str) -> Option<Vec<Part>> {
    let mut parts = Vec::new();
    while let Some((before, after)) = text.split_once('{') {
        if before.contains('}') {
            return None;
        }
        let (expr, next) = after.split_once('}')?;
        let (param, default) = match expr.trim().split_once('|') {
            None => (expr.trim(), None),
            Some((parameter, clause)) => {
                let d = clause
                    .trim()
                    .strip_prefix("default(")?
                    .strip_suffix(')')?
                    .trim();
                if !number(d) {
                    return None;
                }
                (parameter.trim(), Some(d.to_owned()))
            }
        };
        let parameter = param.strip_prefix("params.")?;
        if !parameter_name(parameter) {
            return None;
        }
        parts.push(Part::Text(before.to_owned()));
        parts.push(Part::Number {
            parameter: parameter.to_owned(),
            default,
        });
        text = next;
    }
    if text.contains('}') {
        return None;
    }
    parts.push(Part::Text(text.to_owned()));
    Some(parts)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> Registry {
        let cfg = Config::default();
        let files = BTreeMap::from([("a.cfg".into(), "[gcode_macro PARK]\n gcode:\n  G90\n  MOVE_A X={params.X|default(4)}\n[gcode_macro MOVE_A]\n gcode:\n  G1 X{params.X}\n[gcode_macro LOOP]\n gcode: LOOP\n[gcode_macro DYNAMIC]\n gcode: {printer.foo}\n".into())]);
        assert!(Registry::from_config(&cfg).unwrap().macros.is_empty());
        Registry::from_config(&crate::config::parse(&files, "a.cfg").unwrap()).unwrap()
    }

    #[test]
    fn nested_macro_expands_with_numeric_arguments_and_case_insensitive_dispatch() {
        let registry = fixture();
        assert_eq!(
            registry.expand("park\nPARK x=8.5 ; comment\nM400").unwrap(),
            "\nG90\n\nG1 X4\n\nG90\n\nG1 X8.5\nM400\n"
        );
    }

    #[test]
    fn bounded_fail_closed_preflight() {
        let registry = fixture();
        for (script, kind) in [
            ("G90\nloop", Kind::Recursion),
            ("G90\nDYNAMIC", Kind::UnsupportedTemplate),
            ("G90\nPARK X=4.0M112", Kind::Arguments),
            ("G90\nPARK X=4 X=5", Kind::Arguments),
            ("G90\nMOVE_A", Kind::Arguments),
        ] {
            let err = registry.expand(script).unwrap_err();
            assert_eq!(err.kind, kind);
            assert_eq!(err.line, 2);
        }
        assert_eq!(
            registry
                .expand(&"G90\n".repeat(MAX_LINES + 1))
                .unwrap_err()
                .kind,
            Kind::Limit
        );
    }
}
