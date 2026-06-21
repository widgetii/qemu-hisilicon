#!/bin/bash
#
# Assemble a bootable SPI-NAND image for the emulated Hi3516EV300 from the
# OpenIPC hi3516ev300 NAND release, then boot it with run-ev300-nand.sh.
#
# OpenIPC ships the NAND build as three pieces:
#   u-boot-hi3516ev300-nand.bin          (U-Boot, gzip-loader format)
#   openipc.hi3516ev300-nand-ultimate.tgz -> uImage + rootfs.ubi
# The rootfs.ubi only contains the rootfs (+rootfs_data) volumes; OpenIPC adds
# the "kernel" volume at flash time and (on the ubiblock layout) the rootfs is
# a squashfs.  This script reproduces that: it builds a UBI image with the
# kernel volume + a squashfs rootfs + an autoresize rootfs_data, and lays it
# out per the default env (mtdparts=hinand:768k(boot),256k(env),-(ubi)):
#   0x000000  boot  (u-boot)
#   0x0C0000  env   (blank -> U-Boot default env, root=/dev/ubiblock0_1)
#   0x100000  ubi   (kernel + rootfs + rootfs_data)
#
# Requires: ubinize, mksquashfs, ubireader_extract_files (ubi_reader).
#
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORK="${WORK:-/tmp/ev300-nand-mk}"
DL="https://github.com/OpenIPC/firmware/releases/download/latest"
PAGE=2048; PEB=128KiB

mkdir -p "$WORK"; cd "$WORK"
[ -f u-boot-hi3516ev300-nand.bin ] || curl -sL -o u-boot-hi3516ev300-nand.bin "$DL/u-boot-hi3516ev300-nand.bin"
[ -f nand.tgz ]                    || curl -sL -o nand.tgz "$DL/openipc.hi3516ev300-nand-ultimate.tgz"
tar xzf nand.tgz

# Extract the OpenIPC rootfs (UBIFS) to a tree, restore exec bits that the
# extractor drops, and repack as squashfs to match the ubiblock root= env.
rm -rf rfs; ubireader_extract_files -k -o rfs rootfs.ubi.hi3516ev300 >/dev/null 2>&1 || true
RFS="$(find rfs -maxdepth 2 -type d -name rootfs | head -1)"
for d in overlay media utils proc sys tmp run dev mnt; do mkdir -p "$RFS/$d"; done
chmod +x "$RFS/init"
for d in bin sbin usr/bin usr/sbin usr/libexec etc/init.d; do
    [ -d "$RFS/$d" ] && chmod -R +x "$RFS/$d"
done
find "$RFS/lib" "$RFS/usr/lib" -name '*.so*' -exec chmod +x {} + 2>/dev/null || true
mksquashfs "$RFS" rootfs.squashfs -comp xz -b 131072 -noappend -all-root >/dev/null

cat > ubinize.cfg <<EOF
[kernel]
mode=ubi
image=uImage.hi3516ev300
vol_id=0
vol_type=static
vol_name=kernel

[rootfs]
mode=ubi
image=rootfs.squashfs
vol_id=1
vol_type=static
vol_name=rootfs

[rootfs_data]
mode=ubi
vol_id=2
vol_type=dynamic
vol_name=rootfs_data
vol_size=128KiB
vol_flags=autoresize
EOF
ubinize -o ev300.ubi -p "$PEB" -m "$PAGE" -s "$PAGE" ubinize.cfg

python3 - <<PY
img = bytearray(b'\xff' * 0x2000000)          # 32 MiB image (model pads to 128 MiB)
ub  = open('u-boot-hi3516ev300-nand.bin', 'rb').read(); img[0:len(ub)] = ub
ubi = open('ev300.ubi', 'rb').read();          img[0x100000:0x100000+len(ubi)] = ubi
open('$SCRIPT_DIR/ev300-nand.img', 'wb').write(img)
print('wrote ev300-nand.img (%d bytes, ubi %d)' % (len(img), len(ubi)))
PY
echo "Done: $SCRIPT_DIR/ev300-nand.img — boot with: bash qemu-boot/run-ev300-nand.sh"
