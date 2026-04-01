#!/bin/bash
#
# Boot OpenIPC on emulated Hi3516CV300
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

exec "$QEMU" -M hi3516cv300 -m 128M \
    -kernel "$SCRIPT_DIR/uImage.hi3516cv300" \
    -initrd "$SCRIPT_DIR/rootfs.squashfs.hi3516cv300" \
    -nographic -serial mon:stdio \
    -append "console=ttyAMA0,115200 earlyprintk mem=128M root=/dev/ram0 rootfstype=squashfs mtdparts=hi_sfc:256k(boot),64k(env),3072k(kernel),10240k(rootfs),-(rootfs_data)" \
    $NIC_ARGS \
    -d unimp,guest_errors -D "$SCRIPT_DIR/qemu-cv300.log" \
    "$@"
