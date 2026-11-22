// SPDX-License-Identifier: GPL-3.0-only
#![cfg(feature = "webhooks")]

use std::{path::Path, process::Command};

#[test]
fn previews_fixture_without_hardware_and_rejects_missing_includes() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/preview");
    let run = |files: &[&str]| {
        Command::new(env!("CARGO_BIN_EXE_klipper-preview"))
            .arg(&root)
            .arg("printer.cfg")
            .arg(root.join("path.gcode"))
            .args(["20000000", "1", "1", ".2"])
            .args(files)
            .output()
            .unwrap()
    };
    let reply = run(&["rails.cfg"]);
    assert!(
        reply.status.success(),
        "{:?}",
        String::from_utf8_lossy(&reply.stderr)
    );
    let result: serde_json::Value = serde_json::from_slice(&reply.stdout).unwrap();
    assert_eq!(result["mode"], "offline");
    assert_eq!(result["motors"].as_array().unwrap().len(), 3);
    assert!(result["motors"]
        .as_array()
        .unwrap()
        .iter()
        .all(|m| !m.as_array().unwrap().is_empty()));
    for paths in [&[][..], &["../printer.cfg"]] {
        let reply = run(paths);
        assert_eq!(reply.status.code(), Some(2));
        assert!(reply.stdout.is_empty());
    }
}
