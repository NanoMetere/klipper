#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
set -eu
upstream="${1:?usage: sh bench/check_stream.sh KLIPPER_UPSTREAM}"
here=$(CDPATH='' cd -- "$(dirname -- "$0")/.." && pwd)
tmp=$(mktemp -d)
trap 'rm -rf -- "$tmp"' EXIT
helper="$upstream/klippy/chelper"
"${CC:-cc}" -O3 -ffunction-sections -fdata-sections -I "$helper" -c "$helper/msgblock.c" -o "$tmp/msgblock.o"
rustc --edition=2021 -O --crate-type=rlib --crate-name=klipper_host_protocol \
    "$here/src/lib.rs" -o "$tmp/libklipper_host_protocol.rlib"
rustc --edition=2021 -O "$here/bench/check_stream.rs" \
    --extern "klipper_host_protocol=$tmp/libklipper_host_protocol.rlib" \
    -C "link-arg=$tmp/msgblock.o" -o "$tmp/check_stream"
"$tmp/check_stream"
