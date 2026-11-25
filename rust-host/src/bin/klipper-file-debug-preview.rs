// SPDX-License-Identifier: GPL-3.0-only
//! Standalone offline file-debug timing. No serial transport or MCU output.

#[path = "klipper-shared-preview.rs"]
mod shared_preview;

fn main() {
    shared_preview::entry::<true>();
}
