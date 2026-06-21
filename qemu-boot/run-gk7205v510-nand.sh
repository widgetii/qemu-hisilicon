#!/bin/bash
#
# Boot OpenIPC on emulated Goke GK7205V510 ENTIRELY FROM a 128 MiB SPI-NAND
# dump (no -kernel / -initrd / -dtb).  The synthetic boot ROM parses the
# xm720/Goke-V500 "deadbeef" loader descriptor, copies U-Boot from NAND to
# DDR and jumps to it; U-Boot reads its env + kernel from NAND, attaches UBI
# and boots root=ubi0:ubifs / ubiblock — all from the emulated GD5F1GM7.
#
# The FMC controller auto-selects SPI-NAND mode because the image is > 16 MiB.
#
# Usage:
#   bash qemu-boot/run-gk7205v510-nand.sh <nand-dump.bin> [extra qemu args...]
#   FLASH=/path/to/nand.bin bash qemu-boot/run-gk7205v510-nand.sh
#
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-arm"

# NAND dump: first positional arg, or $FLASH.
FLASH="${1:-${FLASH:-}}"
[ -n "$1" ] && shift

if [ -z "$FLASH" ] || [ ! -f "$FLASH" ]; then
    echo "Error: NAND dump not found: '${FLASH:-<unset>}'" >&2
    echo "Usage: $0 <nand-dump.bin> [extra qemu args...]" >&2
    exit 1
fi

# Boot from a throwaway copy: U-Boot's saveenv writes back to the env region,
# so keep the source dump pristine and let each run start clean.
IMG="$(mktemp /tmp/gk7205v510-nand.XXXXXX.img)"
cp -f "$FLASH" "$IMG"

# restrict=on makes busybox ntpd's NTP-pool DNS fail fast so init doesn't
# stall (see openhisilicon#104); drop it if you need guest outbound traffic.
"$QEMU" -M gk7205v510,flash-file="$IMG" \
    -nographic -serial mon:stdio -nic user,restrict=on \
    -d unimp,guest_errors -D "$SCRIPT_DIR/qemu-gk7205v510-nand.log" \
    "$@"
rc=$?
rm -f "$IMG"
exit $rc
