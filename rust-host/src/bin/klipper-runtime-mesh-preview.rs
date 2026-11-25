// SPDX-License-Identifier: GPL-3.0-only
//! Standalone bounded file-debug prediction for saved mesh and X/Y shaper updates.

#[path = "klipper-shared-preview.rs"]
mod shared_preview;

fn main() {
    shared_preview::entry_runtime_mesh();
}
