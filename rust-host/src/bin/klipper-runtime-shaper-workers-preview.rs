// SPDX-License-Identifier: GPL-3.0-only
//! Opt-in bounded file-debug prediction with four independent lane workers.

#[path = "klipper-shared-preview.rs"]
mod shared_preview;

fn main() {
    shared_preview::entry_runtime_workers();
}
