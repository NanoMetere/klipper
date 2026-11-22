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

#[test]
fn explicit_sparse_tools_emit_separate_extrusion_lanes_from_config() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/preview");
    let run = |temps: &str, initial: &str| {
        Command::new(env!("CARGO_BIN_EXE_klipper-preview"))
            .args(["--tool-temps", temps, "--initial-tool", initial])
            .arg(&root)
            .arg("sparse-tool.cfg")
            .arg(root.join("sparse-tool.gcode"))
            .args(["20000000", "1", "1", ".2", "printer.cfg", "rails.cfg"])
            .output()
            .unwrap()
    };
    let out = run("0:210,2:210,6:210", "0");
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let result: serde_json::Value = serde_json::from_slice(&out.stdout).unwrap();
    assert_eq!(result["tool_mode"], "lane_selection_without_docking");
    let lanes = result["extruder_lanes"].as_array().unwrap();
    let mut all = Vec::new();
    for lane in lanes {
        let mut positive = false;
        let mut net = 0;
        for command in lane["commands"].as_array().unwrap() {
            if command["kind"] == "direction" {
                positive = command["positive"].as_bool().unwrap();
            } else {
                let count = command["count"].as_i64().unwrap();
                net += if positive { count } else { -count };
            }
        }
        assert!((lane["step_distance_mm"].as_f64().unwrap() - 0.004).abs() < 1e-12);
        all.push((
            lane["tool"].as_u64().unwrap(),
            lane["extruder"].as_str().unwrap(),
            net,
        ));
    }
    assert_eq!(
        all,
        [
            (0, "extruder", 100),
            (2, "extruder1", -155),
            (6, "extruder2", 50)
        ]
    );
    for (temps, initial, error) in [
        ("0:210,2:20,6:210", "0", "Cold"),
        (
            "0:210,2:210,3:210",
            "0",
            "missing modeled temperature for T6",
        ),
        ("0:210,2:210,6:210", "6", ""),
        ("0:210,2:210,6:210", "3", "ToolSegment"),
        ("0:210,2:210,6:NaN", "0", "nonfinite"),
    ] {
        let ret = run(temps, initial);
        if error.is_empty() {
            assert!(ret.status.success());
        } else {
            assert_eq!(ret.status.code(), Some(2));
            assert!(ret.stdout.is_empty());
            assert!(
                String::from_utf8_lossy(&ret.stderr).contains(error),
                "{:?}",
                ret
            );
        }
    }
}

#[test]
fn mixed_cli_reports_explicit_unresolved_waits_and_never_assumes_a_hot_extruder() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/preview");
    let execute = |flags: &[&str]| {
        Command::new(env!("CARGO_BIN_EXE_klipper-preview"))
            .arg("--mixed-plan")
            .args(flags)
            .arg(&root)
            .arg("mixed.cfg")
            .arg(root.join("path_mixed.gcode"))
            .args(["20000000", "1", "1", ".2", "printer.cfg", "rails.cfg"])
            .output()
            .unwrap()
    };
    let valid = execute(&["--tool-temps", "0:210"]);
    assert!(
        valid.status.success(),
        "{}",
        String::from_utf8_lossy(&valid.stderr)
    );
    let answer: serde_json::Value = serde_json::from_slice(&valid.stdout).unwrap();
    assert_eq!(answer["mode"], "offline_mixed_plan");
    assert_eq!(
        answer["heater_names"],
        serde_json::json!(["extruder", "heater_bed"])
    );
    let stages = answer["phases"].as_array().unwrap();
    assert_eq!(stages.len(), 7);
    assert_eq!(stages[0]["line"], 1);
    assert_eq!(
        stages[0]["wait"],
        serde_json::json!({"kind":"controller","heater_index":0,"target_c":210.})
    );
    assert_eq!(stages[2]["wait"]["heater_index"], 1);
    assert_eq!(
        stages[4]["wait"]["source"]["name"],
        "temperature_sensor chamber"
    );
    assert_eq!(stages[4]["wait"]["minimum_c"], 20.);
    assert!(stages[4]["wait"]["maximum_c"].is_null());
    assert_eq!(stages[6]["targets_c"], serde_json::json!([0., 0.]));
    assert!(stages[6]["wait"].is_null());
    for (flags, error) in [
        (&[][..], "ExtruderRequired"),
        (&["--tool-temps", "0:25"][..], "Cold"),
        (
            &["--tool-temps", "1:210"][..],
            "only one modeled temperature",
        ),
    ] {
        let result = execute(flags);
        assert_eq!(result.status.code(), Some(2));
        assert!(result.stdout.is_empty());
        assert!(String::from_utf8_lossy(&result.stderr).contains(error));
    }
}
