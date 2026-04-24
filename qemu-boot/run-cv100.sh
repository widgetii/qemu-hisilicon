#!/bin/bash
#
# Boot OpenIPC on emulated Hi3516CV100
#
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-arm"

exec "$QEMU" -M hi3516cv100,sensor=imx222 \
    -kernel "$SCRIPT_DIR/uImage.hi3516cv100" \
    -initrd "$SCRIPT_DIR/rootfs.squashfs.hi3516cv100" \
    -nographic -serial mon:stdio \
    -append "console=ttyAMA0,115200 root=/dev/ram0 rootfstype=squashfs mtdparts=hi_sfc:256k(boot),64k(env),3072k(kernel),10240k(rootfs),-(rootfs_data)" \
    -nic user \
    -d unimp,guest_errors -D "$SCRIPT_DIR/qemu-cv100.log" \
    "$@"
