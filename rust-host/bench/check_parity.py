#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Run the Rust host's pinned correctness matrix without real hardware."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
import time

from planner import PIN, ROOT

OFFLINE = (
    "check_stream.sh", "clock_sync.py", "planner.py", "flight.py", "retry_timing.py", "can.py",
    "config.py", "lookahead.py", "gcode_preview.py", "gcode_arcs.py", "bed_mesh.py", "trajectory.py", "step_times.py",
    "step_compress.py", "step_clock.py", "machine_trace.py", "pressure_advance.py", "shared_windows.py",
    "tool_transform.py", "thermal.py", "probe.py", "homing.py",
)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--upstream", type=Path, required=True, help=f"git checkout at {PIN}")
    p.add_argument("--dictionary", type=Path, help="JSON from matching Linux MCU build")
    p.add_argument("--native-pty", action="store_true", help="run only against a newly created Linux MCU PTY")
    p.add_argument("--native-gpio", action="store_true", help="build a separate GPIO-free Linux motion fixture")
    p.add_argument("--macro-reference", action="store_true", help="also compare stock macro dispatch (requires Jinja2)")
    p.add_argument("--toolchanger-reference", type=Path, help="optional pinned viesturz comparator")
    p.add_argument("--legacy-klippy", type=Path, help="Klippy checkout matching the toolchanger smoke")
    p.add_argument("--modern-toolchanger", type=Path, help="reviewed PR #174 source checkout")
    p.add_argument("--klippain", type=Path, help="pinned Klippain smoke source")
    p.add_argument("--led-effect", type=Path, help="pinned LED Effect source")
    p.add_argument("--modern-bridge", type=Path, help="optional compiled Rust debug bridge for PR #174")
    p.add_argument("--dispatch-bridge", type=Path, help="optional compiled bridge for the stock dispatch comparison")
    p.add_argument("--moonraker", type=Path,
                   help="run the optional pinned Moonraker read-only socketpair fixture")
    p.add_argument("--moonraker-http", action="store_true",
                   help="also run pinned Moonraker's read-only loopback HTTP/WebSocket server")
    p.add_argument("--moonraker-job", action="store_true",
                   help="also stream a routed Rust virtual job through the read-only frontend")
    a = p.parse_args()
    if a.moonraker_job and not a.moonraker_http:
        p.error("--moonraker-job requires --moonraker-http")
    if a.moonraker_http and not a.moonraker:
        p.error("--moonraker-http requires --moonraker")
    modern = [a.modern_toolchanger, a.klippain, a.led_effect]
    if (any(modern) or a.modern_bridge) and not all(modern):
        p.error("--modern-toolchanger, --klippain and --led-effect must be supplied together")
    if bool(a.toolchanger_reference) != bool(a.legacy_klippy):
        p.error("--toolchanger-reference and --legacy-klippy must be supplied together")
    upstream = a.upstream.resolve()
    got = subprocess.check_output(["git", "-C", upstream, "rev-parse", "HEAD"], text=True).strip()
    if got != PIN:
        p.error(f"upstream must be pinned at {PIN}, got {got}")
    subprocess.run(["git", "-C", upstream, "diff", "--quiet", "HEAD", "--"], check=True)
    dictionary = (a.dictionary or upstream / "out/klipper.dict").resolve()
    target = Path(os.environ.get("CARGO_TARGET_DIR", ROOT / "target"))
    if not target.is_absolute():
        target = ROOT / target
    examples = target / "release/examples"
    phases = [
        ("rust_debug", ["cargo", "test", "--locked", "--all-targets", "-q"]),
        ("rust_release", ["cargo", "test", "--locked", "--release", "--all-targets", "-q"]),
        ("rust_nodefault", ["cargo", "test", "--locked", "--no-default-features", "--all-targets", "-q"]),
    ]
    for name in OFFLINE:
        runtime = "sh" if name.endswith(".sh") else sys.executable
        extra = ["--rounds", "0"] if name in ("lookahead.py", "gcode_preview.py") else []
        phases.append((name, [runtime, ROOT/"bench"/name, upstream, *extra]))
    phases.append(("shaped_machine_trace", [sys.executable, ROOT/"bench/machine_trace.py",
                                            upstream, "--shape-timing"]))
    phases.append(("pressure_advance_three_way", [sys.executable, ROOT/"bench/pa_three_way.py",
                                                  upstream, "--rounds", "6"]))
    if a.dispatch_bridge:
        phases.append(("canonical_dispatch", [sys.executable, ROOT/"bench/hybrid_dispatch_parity.py",
                                               upstream, a.dispatch_bridge.resolve()]))
        if dictionary.is_file():
            phases.append(("dispatch_factorial", [sys.executable, ROOT/"bench/hybrid_process_timing.py",
                                                  upstream, dictionary, a.dispatch_bridge.resolve(),
                                                  "--factorial", "--moves", "1200", "--rounds", "4",
                                                  "--profile", "layered", "--wait-interval", "160"]))
            phases.append(("dispatch_language", [sys.executable, ROOT/"bench/dispatch_language.py",
                                                 upstream, dictionary, a.dispatch_bridge.resolve(),
                                                 "--moves", "1200", "--rounds", "6",
                                                 "--native-rounds", "4", "--wait-interval", "160"]))
    if a.macro_reference:
        phases.append(("gcode_macros", [sys.executable, ROOT/"bench/gcode_macros.py", upstream]))
    phases.extend([
        ("build_fixture_binaries", ["cargo", "build", "--locked", "--release", "--examples", "--bins"]),
        ("build_file_debug_preview", ["cargo", "build", "--locked", "--release", "--features",
                                      "file-debug-preview", "--bin", "klipper-file-debug-preview",
                                      "--target-dir", target/"file-debug-preview"]),

        ("webhooks", [sys.executable, ROOT/"bench/webhooks.py", upstream, examples/"webhook_replay"]),
        ("trigger_timing", [sys.executable, ROOT/"bench/trigger_sync.py", upstream,
                            "--binary", examples/"trigger_sync_replay"]),
        ("trigger_dispatch", [sys.executable, ROOT/"bench/trigger_dispatch.py", upstream,
                              "--binary", examples/"trigger_dispatch_replay"]),
        ("virtual_sd", [sys.executable, ROOT/"bench/virtual_sd.py", upstream,
                        "--binary", examples/"virtual_sd_replay", "--status"]),
        ("print_stats", [sys.executable, ROOT/"bench/print_stats.py", upstream,
                         "--binary", examples/"print_stats_replay"]),
        ("multi_extruder", [sys.executable, ROOT/"bench/multi_extruder.py", upstream,
                              "--binary", examples/"multi_extruder_replay"]),
        ("tool_motion", [sys.executable, ROOT/"bench/tool_motion.py", upstream,
                           "--binary", examples/"tool_motion_replay", "--cli-binary", target/"release/klipper-preview"]),
        ("long_step_preview", [sys.executable, ROOT/"bench/long_step_preview.py", upstream,
                               "--target-dir", target/"long-preview",
                               "--ordinary-binary", target/"release/klipper-preview"]),
        ("long_sparse_tool_motion", [sys.executable, ROOT/"bench/tool_motion.py", upstream,
                                     "--binary", examples/"tool_motion_replay",
                                     "--cli-binary", target/"long-preview/release/klipper-preview",
                                     "--long-steps"]),
        ("long_scripted_mesh_motion", [sys.executable, ROOT/"bench/mesh_profile_motion.py", upstream,
                                        "--binary", target/"long-preview/release/klipper-preview",
                                        "--long-steps"]),
        ("mesh_profile_motion", [sys.executable, ROOT/"bench/mesh_profile_motion.py", upstream,
                                 "--binary", target/"release/klipper-preview"]),
        ("display_status", [sys.executable, ROOT/"bench/display_status.py", upstream,
                            examples/"display_status_replay", target/"release/klipper-preview"]),
        ("fan_commands", [sys.executable, ROOT/"bench/fan_commands.py", upstream,
                          "--binary", examples/"fan_command_replay"]),
        ("fan", [sys.executable, ROOT/"bench/fan.py", upstream,
                  "--binary", examples/"fan_replay"]),
        ("mixed_motion", [sys.executable, ROOT/"bench/mixed_motion.py", upstream,
                          "--binary", examples/"mixed_preflight"]),
        ("mixed_step_grid", [sys.executable, ROOT/"bench/mixed_step_grid.py", upstream,
                             "--binary", examples/"mixed_step_grid"]),
        ("mixed_tool_steps", [sys.executable, ROOT/"bench/mixed_tool_steps.py", upstream,
                              "--binary", examples/"mixed_tool_steps"]),
        ("thermal_commands", [sys.executable, ROOT/"bench/thermal_commands.py", upstream,
                              "--binary", examples/"thermal_command_replay"]),
        ("verify_heater", [sys.executable, ROOT/"bench/verify_heater.py", upstream,
                          "--binary", examples/"verify_heater_replay"]),
        ("virtual_stepper", [sys.executable, ROOT/"bench/virtual_stepper.py",
                             "--binary", examples/"virtual_stepper_replay"]),
    ])
    if a.moonraker:
        phases.append(("moonraker_status_client", [sys.executable, ROOT/"bench/moonraker_connection.py",
                                                  a.moonraker.resolve(),
                                                  examples/"moonraker_status_fixture"]))
    if a.moonraker_http:
        phases.append(("moonraker_status_frontend", [sys.executable, ROOT/"bench/moonraker_frontend.py",
                                                    a.moonraker.resolve(),
                                                    examples/"moonraker_status_fixture", "--delay-ready"]))
    if a.moonraker_job:
        phases.append(("moonraker_virtual_job", [sys.executable, ROOT/"bench/moonraker_frontend.py",
                                                a.moonraker.resolve(), examples/"moonraker_status_fixture",
                                                "--program"]))
    if a.toolchanger_reference:
        phases.append(("toolchange_order", [sys.executable, ROOT/"bench/toolchange_order.py",
                                           a.legacy_klippy.resolve(), a.toolchanger_reference.resolve(),
                                           "--binary", examples/"toolchange_replay"]))
        phases.append(("tool_catalog", [sys.executable, ROOT/"bench/tool_catalog.py",
                                        a.legacy_klippy.resolve(), a.toolchanger_reference.resolve(),
                                        "--binary", examples/"tool_catalog_replay"]))
        phases.append(("tool_dock", [sys.executable, ROOT/"bench/tool_dock.py",
                                     a.legacy_klippy.resolve(), a.toolchanger_reference.resolve(),
                                     "--binary", examples/"tool_dock_replay"]))
        phases.append(("fan_transfer", [sys.executable, ROOT/"bench/fan_transfer.py",
                                         a.legacy_klippy.resolve(), a.toolchanger_reference.resolve(),
                                         "--binary", examples/"fan_transfer_replay"]))
    if dictionary.is_file():
        phases.append(("shared_extruder_binding", [sys.executable, ROOT/"bench/shared_extruder.py",
                                                  upstream, "--dictionary", dictionary]))
        phases.append(("stock_captured_shared_axes", [sys.executable, ROOT/"bench/shared_window_capture.py",
                                                      upstream, "--dictionary", dictionary,
                                                      "--shared-preview", target / "release/klipper-shared-preview",
                                                      "--file-debug-preview", target / "file-debug-preview/release/klipper-file-debug-preview"]))
        shaped = target / "shaped-file-debug-preview"
        phases.append(("build_shaped_file_debug_preview", ["cargo", "build", "--locked",
                       "--release", "--features", "shaped-file-debug-preview", "--bin",
                       "klipper-shaped-file-debug-preview", "--target-dir", shaped]))
        phases.append(("stock_shaped_file_debug", [sys.executable, ROOT/"bench/shaped_file_debug.py",
                       upstream, dictionary, shaped / "release/klipper-shaped-file-debug-preview"]))
        runtime_shape = target / "runtime-shaper-preview"
        phases.append(("build_runtime_shaper_preview", ["cargo", "build", "--locked",
                       "--release", "--features", "runtime-shaper-preview", "--bin",
                       "klipper-runtime-shaper-preview", "--target-dir", runtime_shape]))
        phases.append(("stock_runtime_shaper_preview", [sys.executable, ROOT/"bench/runtime_shaper.py",
                       upstream, dictionary, runtime_shape / "release/klipper-runtime-shaper-preview"]))
        phases.append(("step_wire", [sys.executable, ROOT/"bench/step_wire.py", upstream,
                                     dictionary, examples/"step_wire_replay"]))
        phases.append(("mcu_config_payloads", [sys.executable, ROOT/"bench/mcu_config.py", upstream,
                                               "--payload-binary", examples/"mcu_config_replay",
                                               "--dictionary", dictionary]))
        phases.append(("integrated_simulated_job", [sys.executable, ROOT/"bench/simulated_job.py",
                                                    examples/"simulated_job", dictionary]))
    else:
        print("SKIP step_wire (no matching Linux firmware dictionary supplied)", flush=True)
    if a.native_pty:
        if not sys.platform.startswith("linux") or not (upstream/"out/klipper.elf").is_file():
            p.error("--native-pty requires Linux and a native MCU built in the pinned checkout")
        phases.extend([
            ("native_clock", [sys.executable, ROOT/"bench/native_mcu.py", upstream, "--binary",
                              examples/"native_mcu", "--live-clock"]),
            ("native_mcu_config", [sys.executable, ROOT/"bench/mcu_config.py", upstream, "--binary",
                                   examples/"native_mcu"]),
            ("native_trigger_sync", [sys.executable, ROOT/"bench/trigger_sync.py", upstream, "--native",
                                     "--binary", examples/"native_mcu"]),
            ("native_pipeline", [sys.executable, ROOT/"bench/pipeline_pty.py", upstream,
                                 examples/"native_mcu", "--waves", "8", "--rounds", "2"]),
        ])
    if a.native_gpio:
        if not a.native_pty:
            p.error("--native-gpio requires --native-pty")
        phases.append(("native_gpio_motion", [sys.executable, ROOT/"bench/native_gpio_motion.py",
                                              upstream, "--binary", examples/"native_mcu"]))
    if all(modern):
        if not dictionary.is_file():
            p.error("modern Toolchanger smoke requires a matched firmware dictionary")
        flags = ["--bridge", a.modern_bridge.resolve()] if a.modern_bridge else []
        phases.append(("modern_toolchanger", [sys.executable, ROOT/"bench/toolchanger_candidate.py",
            "--port", upstream, "--dictionary", dictionary,
            "--candidate", a.modern_toolchanger.resolve(),
            "--klippain", a.klippain.resolve(), "--led-effect", a.led_effect.resolve(), *flags]))
    failures = []
    for name, command in phases:
        start = time.monotonic()
        try:
            result = subprocess.run(command, cwd=ROOT, text=True, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, timeout=180)
            ok, output = result.returncode == 0, result.stdout
        except (OSError, subprocess.TimeoutExpired) as e:
            ok, output = False, str(e)
        print(f"{'PASS' if ok else 'FAIL'} {name} ({time.monotonic()-start:.2f}s)", flush=True)
        if not ok:
            print(output[-8000:], flush=True)
            failures.append(name)
    print(f"{len(phases)-len(failures)}/{len(phases)} phases passed")
    if failures:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
