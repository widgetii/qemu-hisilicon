#!/bin/bash
#
# Boot OpenIPC on emulated Hi3516CV610
# Uses extracted kernel + DTB from FIT image + separate rootfs
#
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-arm"

exec "$QEMU" -M hi3516cv610 -m 128M \
    -kernel "$SCRIPT_DIR/kernel.hi3516cv610" \
    -dtb "$SCRIPT_DIR/cv610.dtb" \
    -initrd "$SCRIPT_DIR/rootfs.squashfs.hi3516cv610" \
    -nographic -serial mon:stdio \
    -append "console=ttyAMA0,115200 earlyprintk mem=128M root=/dev/ram0 rootfstype=squashfs mtdparts=hi_sfc:256k(boot),64k(env),3072k(kernel),10240k(rootfs),-(rootfs_data)" \
    -nic user \
    -d unimp,guest_errors -D "$SCRIPT_DIR/qemu-cv610.log" \
    "$@"
