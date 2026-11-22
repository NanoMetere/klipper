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

#[test]
fn opts_in_to_pure_macro_expansion_and_produces_the_same_motion() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/preview");
    let run = |flags: &[&str], script: &str| {
        Command::new(env!("CARGO_BIN_EXE_klipper-preview"))
            .args(flags)
            .arg(&root)
            .arg("printer.cfg")
            .arg(root.join(script))
            .args(["20000000", "1", "1", ".2", "rails.cfg"])
            .output()
            .unwrap()
    };
    let flat = run(&[], "path.gcode");
    let rejected = run(&[], "path_macro.gcode");
    let macro_run = run(&["--expand-macros"], "path_macro.gcode");
    assert!(flat.status.success() && macro_run.status.success());
    assert_eq!(rejected.status.code(), Some(2));
    assert!(rejected.stdout.is_empty());
    assert_eq!(macro_run.stdout, flat.stdout);
}

#[test]
fn virtual_sd_reports_the_stock_checkpoint_for_an_unterminated_last_line() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/preview");
    let run = |flag: &str, script: &str| {
        Command::new(env!("CARGO_BIN_EXE_klipper-preview"))
            .arg(flag)
            .arg(&root)
            .arg("printer.cfg")
            .arg(root.join(script))
            .args(["20000000", "1", "1", ".2", "rails.cfg"])
            .output()
            .unwrap()
    };
    let with_tail = run("--virtual-sd", "path_macro_tail.gcode");
    assert_eq!(with_tail.status.code(), Some(2)); // macro expansion remains opt-in
    let output = Command::new(env!("CARGO_BIN_EXE_klipper-preview"))
        .args(["--virtual-sd", "--expand-macros"])
        .arg(&root)
        .arg("printer.cfg")
        .arg(root.join("path_macro_tail.gcode"))
        .args(["20000000", "1", "1", ".2", "rails.cfg"])
        .output()
        .unwrap();
    assert!(output.status.success());
    let raw: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(
        raw["virtual_sd"],
        serde_json::json!({"position":10, "size":18})
    );
    assert!(raw["motors"]
        .as_array()
        .unwrap()
        .iter()
        .all(|m| !m.as_array().unwrap().is_empty()));
}
