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

TAP="${TAP:-tap0}"
if ip link show "$TAP" &>/dev/null 2>&1; then
    NIC_ARGS="-nic tap,ifname=$TAP,script=no,downscript=no"
    echo "Using bridged TAP networking ($TAP)"
else
    NIC_ARGS="-nic user"
    echo "TAP not available; using SLIRP user-mode networking"
fi

exec "$QEMU" -M hi3516ev300 \
    -kernel "$SCRIPT_DIR/uImage.hi3516ev300" \
    -initrd "$SCRIPT_DIR/rootfs.squashfs.hi3516ev300" \
    -nographic -serial mon:stdio \
    -append "console=ttyAMA0,115200 earlyprintk root=/dev/ram0 rootfstype=squashfs mtdparts=hi_sfc:256k(boot),64k(env),3072k(kernel),10240k(rootfs),-(rootfs_data)" \
    $NIC_ARGS \
    -d unimp,guest_errors -D "$SCRIPT_DIR/qemu-ev300.log" \
    "$@"
