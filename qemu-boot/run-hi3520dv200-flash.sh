#!/bin/bash
#
# Boot Hi3520Dv200 from a SPI NOR flash dump.
#
# Usage:
#   bash qemu-boot/run-hi3520dv200-flash.sh [flash_dump.bin]
#
# Defaults to the 32 MiB MX25L25635E image staged in /tmp.  The image is
# loaded into the emulated SFC350 controller; the built-in boot ROM at
# address 0 copies U-Boot from the flash memory window (0x58000000) to
# DDR and jumps to it.  The hi3520dv200 machine wires the SFC350
# registers at 0x10010000 (see qemu/hw/arm/hisilicon.c:2722).
#
# MEM_WINDOW_SIZE in hisi-sfc350.c was widened to 32 MiB to accommodate
# this layout (openipc/firmware#2089 reproduction).
#
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-arm"

FLASH_FILE="${1:-/tmp/hi3520dv200-mx25l25635e-cs1-32mib-openipc-noboot-20260513-150731.bin}"

if [ ! -f "$FLASH_FILE" ]; then
    echo "Error: flash dump not found: $FLASH_FILE" >&2
    echo "Usage: $0 <flash_dump.bin>" >&2
    exit 1
fi

shift 2>/dev/null  # consume $1 so "$@" passes only extra args

NIC_ARGS="-nic user"

exec "$QEMU" -M hi3520dv200 \
    -global hisi-sfc350.flash-file="$FLASH_FILE" \
    -nographic -serial mon:stdio \
    $NIC_ARGS \
    -d unimp,guest_errors \
    -D "$SCRIPT_DIR/qemu-hi3520dv200-flash.log" \
    "$@"
