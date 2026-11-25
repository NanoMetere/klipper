// SPDX-License-Identifier: GPL-3.0-only
//! Standalone bounded file-debug prediction for a shared-extruder pickup with multiple saved meshes.

#[path = "klipper-shared-preview.rs"]
mod shared_preview;

fn main() {
    shared_preview::entry_runtime_pickup_mesh_bank();
}
