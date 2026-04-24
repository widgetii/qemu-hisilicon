#!/bin/bash
#
# Boot OpenIPC on emulated Hi3519V101
#
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-arm"

exec "$QEMU" -M hi3519v101 \
    -kernel "$SCRIPT_DIR/uImage.hi3519v101" \
    -initrd "$SCRIPT_DIR/rootfs.squashfs.hi3519v101" \
    -nographic -serial mon:stdio \
    -append "console=ttyAMA0,115200 earlyprintk root=/dev/ram0 rootfstype=squashfs mtdparts=hi_sfc:256k(boot),64k(env),3072k(kernel),10240k(rootfs),-(rootfs_data)" \
    -d unimp,guest_errors -D "$SCRIPT_DIR/qemu-3519v101.log" \
    "$@"
