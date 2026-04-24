#!/bin/bash
#
# Boot Hi3516EV300 from a SPI NOR flash dump.
#
# Usage:
#   bash qemu-boot/run-ev300-flash.sh /tmp/flash_dump_ev300.bin
#
# The flash dump is loaded into the emulated FMC flash controller.
# A built-in boot ROM at address 0 copies U-Boot from the flash
# memory window to DDR and jumps to it — just like real hardware.
# Serial output is saved to /tmp/ev300_serial.log.
#
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-arm"

FLASH_FILE="${1:-/tmp/flash_dump_ev300.bin}"

if [ ! -f "$FLASH_FILE" ]; then
    echo "Error: flash dump not found: $FLASH_FILE" >&2
    echo "Usage: $0 <flash_dump.bin>" >&2
    exit 1
fi

shift 2>/dev/null  # consume $1 so "$@" passes only extra args

TAP="${TAP:-tap0}"
if ip link show "$TAP" &>/dev/null 2>&1; then
    NIC_ARGS="-nic tap,ifname=$TAP,script=no,downscript=no"
    echo "Using bridged TAP networking ($TAP)"
else
    NIC_ARGS="-nic user"
    echo "TAP not available; using SLIRP user-mode networking"
fi

exec "$QEMU" -M hi3516ev300 \
    -global hisi-fmc.flash-file="$FLASH_FILE" \
    -nographic -serial mon:stdio \
    $NIC_ARGS \
    -d unimp,guest_errors \
    -D "$SCRIPT_DIR/qemu-ev300-flash.log" \
    "$@" 2>&1 | tee /tmp/ev300_serial.log
