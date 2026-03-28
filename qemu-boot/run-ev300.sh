#!/bin/bash
#
# Boot OpenIPC on emulated Hi3516EV300
#
# NOTE: Linux 4.15+ kernels require vdso=0 on the command line (or
# CONFIG_VDSO=n at build time). The 4.15 ARM VDSO signal return
# trampoline uses instructions that this QEMU machine model does not
# fully emulate, causing SIGILL on init. Real hardware is unaffected.
#
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-arm"

exec "$QEMU" -M hi3516ev300 -m 128M \
    -kernel "$SCRIPT_DIR/uImage.hi3516ev300" \
    -initrd "$SCRIPT_DIR/rootfs.squashfs.hi3516ev300" \
    -nographic -serial mon:stdio \
    -append "console=ttyAMA0,115200 earlyprintk mem=96M root=/dev/ram0 rootfstype=squashfs mtdparts=hi_sfc:256k(boot),64k(env),3072k(kernel),10240k(rootfs),-(rootfs_data) mmz_allocator=hisi mmz=anonymous,0,0x46000000,32M" \
    -nic user \
    -d unimp,guest_errors -D "$SCRIPT_DIR/qemu-ev300.log" \
    "$@"
