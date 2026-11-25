// SPDX-License-Identifier: GPL-3.0-only
#![cfg(feature = "webhooks")]

use std::{path::Path, process::Command};

#[test]
fn fixed_pressure_script_is_explicitly_offline_and_single_extruder() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/preview");
    let run = |flags: &[&str], entry: &str| {
        Command::new(env!("CARGO_BIN_EXE_klipper-preview"))
            .args(flags)
            .arg(&root)
            .arg(entry)
            .arg(root.join("single-pressure.gcode"))
            .args(["20000000", "1", "1", ".2", "printer.cfg", "rails.cfg"])
            .output()
            .unwrap()
    };
    let reply = run(
        &["--fixed-pressure-script", "--tool-temps", "0:210"],
        "single-pressure.cfg",
    );
    assert!(
        reply.status.success(),
        "{}",
        String::from_utf8_lossy(&reply.stderr)
    );
    let data: serde_json::Value = serde_json::from_slice(&reply.stdout).unwrap();
    assert_eq!(data["mode"], "offline");
    assert_eq!(data["tool_mode"], "fixed_extruder_without_docking");
    assert_eq!(data["extruder_lanes"][0]["extruder"], "extruder");
    assert!(
        data["extruder_lanes"][0]["commands"]
            .as_array()
            .unwrap()
            .len()
            > 10
    );
    for (flags, entry) in [
        (&["--tool-temps", "0:210"][..], "single-pressure.cfg"),
        (
            &[
                "--fixed-pressure-script",
                "--tool-temps",
                "0:210",
                "--mixed-plan",
            ][..],
            "single-pressure.cfg",
        ),
        (
            &["--fixed-pressure-script", "--tool-temps", "0:210"][..],
            "sparse-tool.cfg",
        ),
    ] {
        let denied = run(flags, entry);
        assert_eq!(denied.status.code(), Some(2));
        assert!(denied.stdout.is_empty());
    }
}

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
fn previews_runtime_acceleration_without_changing_end_positions() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/preview");
    let run = |script: &str| {
        let output = Command::new(env!("CARGO_BIN_EXE_klipper-preview"))
            .arg(&root)
            .arg("printer.cfg")
            .arg(root.join(script))
            .args(["20000000", "1", "1", ".2", "rails.cfg"])
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        serde_json::from_slice::<serde_json::Value>(&output.stdout).unwrap()
    };
    let fixed = run("path.gcode");
    let varied = run("runtime_limits.gcode");
    assert!(
        varied["planned_duration_seconds"].as_f64().unwrap()
            > fixed["planned_duration_seconds"].as_f64().unwrap()
    );
    let signed_positions = |value: &serde_json::Value| {
        value["motors"]
            .as_array()
            .unwrap()
            .iter()
            .map(|lane| {
                let mut sign = 1;
                let mut total = 0;
                for command in lane.as_array().unwrap() {
                    if command["kind"] == "direction" {
                        sign = if command["positive"].as_bool().unwrap() {
                            1
                        } else {
                            -1
                        };
                    } else {
                        total += sign * command["count"].as_i64().unwrap();
                    }
                }
                total
            })
            .collect::<Vec<_>>()
    };
    assert_eq!(signed_positions(&fixed), signed_positions(&varied));
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

#[test]
fn sparse_mixed_cli_distinguishes_tool_ids_from_heater_roles() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/preview");
    let execute = |flags: &[&str]| {
        Command::new(env!("CARGO_BIN_EXE_klipper-preview"))
            .arg("--mixed-plan")
            .args(flags)
            .arg(&root)
            .arg("sparse-tool.cfg")
            .arg(root.join("path_sparse_mixed.gcode"))
            .args(["20000000", "1", "1", ".2", "printer.cfg", "rails.cfg"])
            .output()
            .unwrap()
    };
    let valid = execute(&["--initial-tool", "0", "--tool-temps", "0:210,2:215,6:220"]);
    assert!(
        valid.status.success(),
        "{}",
        String::from_utf8_lossy(&valid.stderr)
    );
    let result: serde_json::Value = serde_json::from_slice(&valid.stdout).unwrap();
    assert_eq!(result["mode"], "offline_mixed_plan");
    assert_eq!(result["initial_tool"], 0);
    assert_eq!(
        result["heater_names"],
        serde_json::json!(["extruder", "extruder1", "extruder2"])
    );
    let phases = result["phases"].as_array().unwrap();
    let selects: Vec<_> = phases
        .iter()
        .filter(|p| p["stage"] == "select")
        .map(|p| {
            (
                p["line"].as_u64().unwrap(),
                p["from"].as_u64().unwrap(),
                p["to"].as_u64().unwrap(),
            )
        })
        .collect();
    assert_eq!(selects, [(7, 0, 2), (10, 2, 6), (13, 6, 0)]);
    let heaters: Vec<_> = phases.iter().filter(|p| p["stage"] == "heater").collect();
    assert_eq!(heaters[0]["line"], 4);
    assert_eq!(heaters[0]["active_tool"], 0);
    assert_eq!(heaters[0]["targets_c"], serde_json::json!([0., 0., 200.]));
    assert!(heaters[0]["wait"].is_null());
    for (row, (tool, heater)) in heaters[1..4].iter().zip([(0, 0), (2, 1), (6, 2)]) {
        assert_eq!(row["active_tool"], tool);
        assert_eq!(row["wait"]["heater_index"], heater);
    }
    assert_eq!(heaters[4]["targets_c"], serde_json::json!([0., 0., 0.]));
    let ext = result["final_extruders_mm"].as_array().unwrap();
    for (r, (tool, e)) in ext.iter().zip([(0, 0.12), (2, 0.2), (6, 0.04)]) {
        assert_eq!(r[0], tool);
        assert!((r[1].as_f64().unwrap() - e).abs() < 1e-12);
    }
    let emitting: Vec<_> = phases
        .iter()
        .filter(|p| {
            p["stage"] == "motion" && p["extruder"].as_array().is_some_and(|r| !r.is_empty())
        })
        .map(|p| p["tool"].as_u64().unwrap())
        .collect();
    assert_eq!(emitting, [0, 2, 6, 0]);
    for (flags, err) in [
        (
            &["--tool-temps", "0:210,2:215,6:220"][..],
            "requires --initial-tool",
        ),
        (
            &["--initial-tool", "5", "--tool-temps", "0:210,2:215,6:220"][..],
            "T5 is not configured",
        ),
        (
            &["--initial-tool", "0", "--tool-temps", "0:210,2:215"][..],
            "temperature for every configured tool",
        ),
        (
            &["--initial-tool", "0", "--tool-temps", "0:210,2:25,6:220"][..],
            "Cold",
        ),
    ] {
        let failure = execute(flags);
        assert_eq!(failure.status.code(), Some(2));
        assert!(failure.stdout.is_empty());
        let log = String::from_utf8_lossy(&failure.stderr);
        assert!(log.contains(err), "{log}");
    }
}

#[test]
fn sparse_fan_demands_are_only_previewed_with_a_configured_print_fan() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/preview");
    let run = |config: &str, included: Option<&str>| {
        let mut cmd = Command::new(env!("CARGO_BIN_EXE_klipper-preview"));
        cmd.args([
            "--mixed-plan",
            "--initial-tool",
            "0",
            "--tool-temps",
            "0:210,2:215,6:220",
        ])
        .arg(&root)
        .arg(config)
        .arg(root.join("path_sparse_fan.gcode"))
        .args(["20000000", "1", "1", ".2", "printer.cfg", "rails.cfg"]);
        if let Some(extra) = included {
            cmd.arg(extra);
        }
        cmd.output().unwrap()
    };
    let result = run("sparse-fan.cfg", Some("sparse-tool.cfg"));
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    let report: serde_json::Value = serde_json::from_slice(&result.stdout).unwrap();
    let fan: Vec<_> = report["phases"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|p| p["stage"] == "fan")
        .collect();
    assert_eq!(fan.len(), 2);
    assert_eq!(fan[0]["active_tool"], 0);
    assert_eq!(fan[0]["demand"], 0.5);
    assert_eq!(fan[1]["active_tool"], 2);
    assert_eq!(fan[1]["demand"], 0.);
    let invalid = run("sparse-tool.cfg", None);
    assert_eq!(invalid.status.code(), Some(2));
    assert!(invalid.stdout.is_empty());
    assert!(String::from_utf8_lossy(&invalid.stderr).contains("FanNotConfigured"));
}
