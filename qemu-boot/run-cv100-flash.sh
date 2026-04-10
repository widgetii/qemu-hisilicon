#!/bin/bash
#
# Boot Hi3516CV100 from a SPI NOR flash dump.
#
# Usage:
#   bash qemu-boot/run-cv100-flash.sh /tmp/flash_dump_cv100.bin
#
# The flash dump is loaded into the emulated SFC350 flash controller.
# A built-in boot ROM at address 0 copies U-Boot from the flash
# memory window to DDR and jumps to it — just like real hardware.
#
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-arm"

FLASH_FILE="${1:-/tmp/openipc-hi3516cv100-nor-8m.bin}"

if [ ! -f "$FLASH_FILE" ]; then
    echo "Error: flash dump not found: $FLASH_FILE" >&2
    echo "Usage: $0 <flash_dump.bin>" >&2
    exit 1
fi

shift 2>/dev/null  # consume $1 so "$@" passes only extra args

NIC_ARGS="-nic user"

exec "$QEMU" -M hi3516cv100 -m 64M \
    -global hisi-sfc350.flash-file="$FLASH_FILE" \
    -nographic -serial mon:stdio \
    $NIC_ARGS \
    -d unimp,guest_errors \
    -D "$SCRIPT_DIR/qemu-cv100-flash.log" \
    "$@"
