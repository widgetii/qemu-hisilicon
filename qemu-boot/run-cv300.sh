#!/bin/bash
#
# Boot OpenIPC on emulated Hi3516CV300
#
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-arm"

exec "$QEMU" -M hi3516cv300 -m 128M \
    -kernel "$SCRIPT_DIR/uImage.hi3516cv300" \
    -initrd "$SCRIPT_DIR/rootfs.squashfs.hi3516cv300" \
    -nographic -serial mon:stdio \
    -append "console=ttyAMA0,115200 earlyprintk mem=128M root=/dev/ram0 rootfstype=squashfs initcall_blacklist=hisi_femac_driver_init,himci_init" \
    -d unimp,guest_errors -D "$SCRIPT_DIR/qemu-cv300.log" \
    "$@"
