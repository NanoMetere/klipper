// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::config;
use std::{
    collections::BTreeMap,
    fs,
    io::{self, BufRead},
    path::Path,
};

fn hex(s: &str) -> String {
    s.as_bytes().iter().map(|b| format!("{b:02x}")).collect()
}

fn files(root: &Path, dir: &Path, map: &mut BTreeMap<String, String>) {
    for item in fs::read_dir(dir).unwrap() {
        let item = item.unwrap();
        let kind = item.file_type().unwrap();
        if kind.is_symlink() {
            continue;
        }
        let path = item.path();
        if kind.is_dir() {
            files(root, &path, map);
        } else if kind.is_file() {
            let key = path
                .strip_prefix(root)
                .unwrap()
                .to_str()
                .unwrap()
                .to_owned();
            map.insert(key, fs::read_to_string(&path).unwrap());
        }
    }
}

fn main() {
    let root = std::env::args_os().nth(1).unwrap();
    let root = Path::new(&root);
    let mut map = BTreeMap::new();
    files(root, root, &mut map);
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let result = config::parse(&map, &line);
        println!("ENTRY {}", hex(&line));
        match result {
            Ok(c) => {
                let default = c.values("DEFAULT");
                for name in c.sections() {
                    println!("SECTION {}", hex(name));
                    let explicit = c.values(name).unwrap();
                    let mut values = default.cloned().unwrap_or_default();
                    values.extend(explicit.clone());
                    for (k, v) in values {
                        println!("{}={}", hex(&k), hex(&v));
                    }
                }
            }
            Err(e) => println!("ERR {:?}", e.kind),
        }
    }
}
