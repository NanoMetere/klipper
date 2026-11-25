// SPDX-License-Identifier: GPL-3.0-only
//! Standalone bounded file-debug timing with fixed X/Y input shaping.

#[path = "klipper-shared-preview.rs"]
mod shared_preview;

fn main() {
    shared_preview::entry_shaped();
}
