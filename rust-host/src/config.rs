// SPDX-License-Identifier: GPL-3.0-only
//! Bounded read-only config resolution based on Klipper's klippy/configfile.py.
//! Upstream copyright (C) 2017-2025 Kevin O'Connor <kevin@koconnor.net>.
//! Operates exclusively on caller-supplied in-memory files. No code execution.

use std::collections::{BTreeMap, BTreeSet};

const MAX_BYTES: usize = 8 * 1024 * 1024;
const MAX_FILES: usize = 1024;
const MAX_VISITS: usize = 2048;
const MAX_DEPTH: usize = 32;
const MAX_PATH: usize = 512;
const MAX_LINE: usize = 8192;
const MAX_VALUES: usize = 32768;
const GLOB_WORK: usize = 32 * 1024 * 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    InvalidPath,
    MissingInclude,
    IncludeCycle,
    InvalidLine,
    Bounds,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Error {
    pub kind: Kind,
    pub source: String,
    pub line: usize,
}

fn error(kind: Kind, source: &str, line: usize) -> Error {
    Error {
        kind,
        source: source.to_owned(),
        line,
    }
}

#[derive(Debug, Default)]
pub struct Config {
    order: Vec<String>,
    sections: BTreeMap<String, BTreeMap<String, String>>,
}

impl Config {
    /// Actual sections in first-definition order (excluding DEFAULT).
    pub fn sections(&self) -> impl Iterator<Item = &str> {
        self.order.iter().map(String::as_str)
    }

    pub fn get(&self, section: &str, option: &str) -> Option<&str> {
        let option = option.to_lowercase();
        self.sections
            .get(section)?
            .get(&option)
            .or_else(|| {
                self.sections
                    .get("DEFAULT")
                    .and_then(|defaults| defaults.get(&option))
            })
            .map(String::as_str)
    }

    /// Explicitly specified keys for a section. Use `get` for default fallback.
    pub fn values(&self, section: &str) -> Option<&BTreeMap<String, String>> {
        self.sections.get(section)
    }
}

/// Resolve includes in lexical order and apply definitions in encounter order.
/// File keys must be canonical, slash-separated relative paths. Includes may
/// use '.' and '..' but may not escape the provided root. Bracket ranges, '*' and
/// '?' match a single path component (including stock non-recursive '**').
pub fn parse(files: &BTreeMap<String, String>, entry: &str) -> Result<Config, Error> {
    if files.len() > MAX_FILES
        || files
            .values()
            .try_fold(0usize, |sum, s| sum.checked_add(s.len()))
            .is_none_or(|n| n > MAX_BYTES)
    {
        return Err(error(Kind::Bounds, entry, 0));
    }
    for k in files.keys() {
        if resolve("", k).as_ref() != Some(k) || has_magic(k) {
            return Err(error(Kind::InvalidPath, k, 0));
        }
    }
    let entry = resolve("", entry)
        .filter(|p| !has_magic(p))
        .ok_or_else(|| error(Kind::InvalidPath, entry, 0))?;
    let mut loader = Loader {
        files,
        config: Config::default(),
        active: BTreeSet::new(),
        visits: 0,
        parsed_bytes: 0,
        values: 0,
        glob_work: GLOB_WORK,
    };
    loader.visit(&entry, 0)?;
    Ok(loader.config)
}

struct Loader<'a> {
    files: &'a BTreeMap<String, String>,
    config: Config,
    active: BTreeSet<String>,
    visits: usize,
    parsed_bytes: usize,
    values: usize,
    glob_work: usize,
}

impl Loader<'_> {
    fn visit(&mut self, source: &str, depth: usize) -> Result<(), Error> {
        self.visits += 1;
        if depth >= MAX_DEPTH || self.visits > MAX_VISITS {
            return Err(error(Kind::Bounds, source, 0));
        }
        if !self.active.insert(source.to_owned()) {
            return Err(error(Kind::IncludeCycle, source, 0));
        }
        let data = self
            .files
            .get(source)
            .ok_or_else(|| error(Kind::MissingInclude, source, 0))?;
        self.parsed_bytes += data.len();
        if self.parsed_bytes > MAX_BYTES {
            return Err(error(Kind::Bounds, source, 0));
        }
        let mut section = None::<String>;
        let mut current = None::<(String, usize, String, usize)>;
        for (i, raw) in data.lines().enumerate() {
            let i = i + 1;
            if raw.len() > MAX_LINE {
                return Err(error(Kind::Bounds, source, i));
            }
            let hashless = raw.split_once('#').map_or(raw, |(left, _)| left);
            if let Some(spec) = header(hashless).and_then(|name| name.strip_prefix("include ")) {
                self.store(source, section.as_deref(), current.take())?;
                section = None;
                let path = resolve(
                    source.rsplit_once('/').map_or("", |(parent, _)| parent),
                    spec.trim(),
                )
                .ok_or_else(|| error(Kind::InvalidPath, source, i))?;
                if has_magic(&path) {
                    let mut matches = Vec::new();
                    for key in self.files.keys() {
                        self.glob_work = self
                            .glob_work
                            .checked_sub(path.len() * key.len())
                            .ok_or_else(|| error(Kind::Bounds, source, i))?;
                        if glob(&path, key) {
                            matches.push(key.clone());
                        }
                    }
                    for p in matches {
                        self.visit(&p, depth + 1)?;
                    }
                } else if self.files.contains_key(&path) {
                    self.visit(&path, depth + 1)?;
                } else {
                    return Err(error(Kind::MissingInclude, source, i));
                }
                continue;
            }

            let clean = strip_semicolon(hashless).trim_end();
            let value = clean.trim_start();
            if value.is_empty() {
                if let Some((_, _, text, _)) = &mut current {
                    if hashless.trim().is_empty() {
                        text.push('\n');
                    }
                }
                continue;
            }
            let indent = clean.len() - value.len();
            if let Some((_, previous, text, _)) = &mut current {
                if indent > *previous {
                    text.push('\n');
                    text.push_str(value);
                    continue;
                }
            }
            self.store(source, section.as_deref(), current.take())?;
            if let Some(name) = header(value) {
                if !self.config.sections.contains_key(name) {
                    if name != "DEFAULT" {
                        self.config.order.push(name.to_owned());
                    }
                    self.config
                        .sections
                        .insert(name.to_owned(), BTreeMap::new());
                }
                section = Some(name.to_owned());
                continue;
            }
            if section.is_none() {
                return Err(error(Kind::InvalidLine, source, i));
            }
            let (key, val) = value
                .split_once(['=', ':'])
                .ok_or_else(|| error(Kind::InvalidLine, source, i))?;
            let key = key.trim();
            if key.is_empty() {
                return Err(error(Kind::InvalidLine, source, i));
            }
            current = Some((key.to_lowercase(), indent, val.trim().to_owned(), i));
        }
        self.store(source, section.as_deref(), current)?;
        self.active.remove(source);
        Ok(())
    }

    fn store(
        &mut self,
        source: &str,
        section: Option<&str>,
        current: Option<(String, usize, String, usize)>,
    ) -> Result<(), Error> {
        if let Some((key, _, val, line)) = current {
            self.values += 1;
            if self.values > MAX_VALUES {
                return Err(error(Kind::Bounds, source, line));
            }
            self.config
                .sections
                .get_mut(section.unwrap())
                .unwrap()
                .insert(key, val.trim_end().to_owned());
        }
        Ok(())
    }
}

fn header(line: &str) -> Option<&str> {
    let (_, rest) = line.strip_prefix('[')?.rsplit_once(']')?;
    let close = line.len() - rest.len() - 1;
    (close > 1).then_some(&line[1..close])
}

fn strip_semicolon(s: &str) -> &str {
    for (i, c) in s.char_indices() {
        if c == ';' && (i == 0 || s[..i].ends_with(char::is_whitespace)) {
            return &s[..i];
        }
    }
    s
}

fn resolve(dir: &str, path: &str) -> Option<String> {
    if path.is_empty()
        || path.starts_with('/')
        || path.len() > MAX_PATH
        || path.contains(['\\', '\0', ':'])
    {
        return None;
    }
    let mut components: Vec<_> = dir.split('/').filter(|s| !s.is_empty()).collect();
    for c in path.split('/') {
        match c {
            "" | "." => {}
            ".." => {
                components.pop()?;
            }
            _ => components.push(c),
        }
    }
    let path = components.join("/");
    (!path.is_empty() && path.len() <= MAX_PATH).then_some(path)
}

fn has_magic(s: &str) -> bool {
    s.contains(['*', '?', '['])
}

fn glob(pattern: &str, path: &str) -> bool {
    let mut a = pattern.split('/');
    let mut b = path.split('/');
    loop {
        match (a.next(), b.next()) {
            (None, None) => return true,
            (Some(p), Some(t)) if !t.starts_with('.') || p.starts_with('.') => {
                if !match_component(p, t) {
                    return false;
                }
            }
            _ => return false,
        }
    }
}

fn match_component(pattern: &str, text: &str) -> bool {
    let p: Vec<_> = pattern.chars().collect();
    let t: Vec<_> = text.chars().collect();
    let mut state = vec![false; t.len() + 1];
    state[0] = true;
    let mut a = 0;
    while a < p.len() {
        let c = p[a];
        if c == '*' {
            for i in 1..=t.len() {
                state[i] |= state[i - 1];
            }
            a += 1;
            continue;
        }
        let mut class = None;
        if c == '[' {
            let start = a + 1 + usize::from(p.get(a + 1) == Some(&'!'));
            let search = start + usize::from(p.get(start) == Some(&']'));
            if let Some(end) = p
                .get(search..)
                .and_then(|s| s.iter().position(|v| *v == ']'))
            {
                let end = search + end;
                class = Some((start, end, start > a + 1));
            }
        }
        for i in (1..=t.len()).rev() {
            let value = t[i - 1];
            state[i] = state[i - 1]
                && if let Some((start, end, negate)) = class {
                    let mut hit = false;
                    let mut j = start;
                    while j < end {
                        if j + 2 < end && p[j + 1] == '-' {
                            hit |= p[j] <= value && value <= p[j + 2];
                            j += 3;
                        } else {
                            hit |= p[j] == value;
                            j += 1;
                        }
                    }
                    hit != negate
                } else {
                    c == '?' || c == value
                };
        }
        state[0] = false;
        a = class.map_or(a + 1, |(_, end, _)| end + 1);
    }
    state[t.len()]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture(files: &[(&str, &str)]) -> BTreeMap<String, String> {
        files
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect()
    }

    #[test]
    fn ordered_overrides_nested_includes_and_multiline_values() {
        let fs = fixture(&[
            (
                "main.cfg",
                "[DEFAULT]\nZ=2\n[printer]\nx:1\n[include extras/[ab]?.cfg]\n[printer]\nx:7",
            ),
            (
                "extras/b2.cfg",
                "[printer]\nx:2\nY=3\n[include ../macros.cfg]",
            ),
            ("extras/a1.cfg", "[printer]\nx:4\ny:6"),
            ("extras/.ax.cfg", "[printer]\ny:5"),
            (
                "macros.cfg",
                "[gcode_macro X]\ngcode:\n    G1 X1 ; comment\n    G1 X2",
            ),
        ]);
        let out = parse(&fs, "main.cfg").unwrap();
        assert_eq!(
            out.sections().collect::<Vec<_>>(),
            ["printer", "gcode_macro X"]
        );
        assert_eq!(out.get("printer", "X"), Some("7"));
        assert_eq!(out.get("printer", "y"), Some("3"));
        assert_eq!(out.get("printer", "z"), Some("2"));
        assert_eq!(out.get("gcode_macro X", "gcode"), Some("\nG1 X1\nG1 X2"));
    }

    #[test]
    fn rejects_escape_and_cycles_but_allows_pattern_without_matches() {
        for (content, err) in [
            ("[include ../private]", Kind::InvalidPath),
            ("[include main.cfg]", Kind::IncludeCycle),
            ("[include missing.cfg]", Kind::MissingInclude),
        ] {
            assert_eq!(
                parse(&fixture(&[("main.cfg", content)]), "main.cfg")
                    .unwrap_err()
                    .kind,
                err
            );
        }
        assert!(parse(
            &fixture(&[("main.cfg", "[include nothing/*.cfg]")]),
            "main.cfg"
        )
        .is_ok());
    }
}
