#!/bin/bash
#
# Boot the built U-Boot (raw gzip self-extracting u-boot-z.bin) on the
# emulated Hi3519DV500 (Cortex-A55, V5/aarch64) and watch the serial console.
#
# The image is the OpenIPC/u-boot-hi3519dv500 "smoke-hi3519dv500.bin" artifact
# (= `make u-boot-z.bin`).  QEMU's hi3519dv500 machine loads it at its DDR link
# address 0x48700000; the on-die HW gzip decompresses it to 0x48800000 and
# U-Boot runs from DDR.  Pass the image path as $1, e.g. the artifact built by
# OpenIPC/u-boot-hi3519dv500 (`make u-boot-z.bin` -> output/smoke-hi3519dv500.bin).
#
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-aarch64"
IMG="${1:?usage: $(basename "$0") <u-boot-z.bin> [extra qemu args]}"

exec "$QEMU" -M hi3519dv500,flash-file="$IMG" \
    -nographic -serial mon:stdio \
    -d unimp,guest_errors -D "$SCRIPT_DIR/qemu-hi3519dv500.log" \
    "${@:2}"
