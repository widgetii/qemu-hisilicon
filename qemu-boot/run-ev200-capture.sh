#!/bin/bash
#
# Capture an ISP tuning profile from emulated Hi3516EV200 + Sofia
# under QEMU.  Boots, modprobes the vendor module chain, chroots
# into Sofia, waits for "ISP Dev 0 running", runs `isp-profile
# export`, and copies the output to the lab NFS share.
#
# Usage:
#   bash qemu-boot/run-ev200-capture.sh --sensor sc2315e [--mode day|night]
#
# Output: /mnt/noc/isp-captures/<sensor>_qemu_<mode>.isp on the host
# (i.e. /utils/isp-captures/<sensor>_qemu_<mode>.isp inside the guest).
#
# Fixtures expected on NFS at 10.216.128.227:/srv/nfsroot/:
#   sofia_full/      vendor sysroot tree
#   sofia_fonts/     Font.bin.web, Font.bin.custom
#   isp-profile      ARM static binary
#
# Exits 0 on successful capture, non-zero with the last 60 lines of
# QEMU output on failure.
#
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

SENSOR=""
MODE="day"
NFS_HOST="${NFS_HOST:-10.216.128.227}"
NFS_PATH="${NFS_PATH:-/srv/nfsroot}"
ISP_READY_TIMEOUT="${ISP_READY_TIMEOUT:-25}"
QEMU_TIMEOUT="${QEMU_TIMEOUT:-120}"

while [ $# -gt 0 ]; do
    case "$1" in
        --sensor)   SENSOR="$2"; shift 2;;
        --sensor=*) SENSOR="${1#--sensor=}"; shift;;
        --mode)     MODE="$2"; shift 2;;
        --mode=*)   MODE="${1#--mode=}"; shift;;
        --nfs)      NFS_HOST="$2"; shift 2;;
        -h|--help)
            sed -n '/^# /,/^[a-z]/p' "$0" | sed 's/^# \?//'
            exit 0;;
        *)
            echo "unknown arg: $1" >&2
            exit 2;;
    esac
done

if [ -z "$SENSOR" ]; then
    echo "error: --sensor required" >&2
    exit 2
fi

OUT_NAME="${SENSOR}_qemu_${MODE}.isp"
echo "==> Capturing ${OUT_NAME} (timeout ${QEMU_TIMEOUT}s)"

# Inline guest setup — same proven sequence as run-ev200-sofia.sh,
# but non-interactive: runs isp-profile export, copies result via
# NFS, then poweroffs.
read -r -d '' GUEST_SETUP <<EOF || true
set +e
mount -t proc proc /proc 2>/dev/null
mount -t tmpfs none /tmp
udhcpc -q -i eth0 2>/dev/null || ifconfig eth0 10.0.2.15 netmask 255.255.255.0 up
route add default gw 10.0.2.2 2>/dev/null

mkdir -p /utils
mount -o nolock,vers=3 ${NFS_HOST}:${NFS_PATH} /utils \\
    || { echo "NFS_MOUNT_FAILED"; poweroff -f; }
mkdir -p /utils/isp-captures

mkdir -p /tmp/vendor/proc /tmp/vendor/dev /tmp/vendor/tmp \\
         /tmp/vendor/mnt/custom/data/Fonts /tmp/vendor/mnt/web/Fonts \\
         /tmp/vendor/var
cp -a /utils/sofia_full/. /tmp/vendor/
rm -f /tmp/vendor/usr/Fonts/Font.bin
cp /utils/sofia_fonts/Font.bin.web    /tmp/vendor/mnt/web/Fonts/Font.bin
cp /utils/sofia_fonts/Font.bin.custom /tmp/vendor/mnt/custom/data/Fonts/Font.bin
ln -sf /mnt/web/Fonts/Font.bin /tmp/vendor/usr/Fonts/Font.bin
mount --bind /dev  /tmp/vendor/dev  2>/dev/null
mount -t proc proc /tmp/vendor/proc 2>/dev/null

cd /lib/modules/4.9.37/hisilicon
for m in sys_config hi_osal hi3516ev200_base hi3516ev200_sys \\
         hi3516ev200_rgn hi3516ev200_vgs hi_mipi_rx hi3516ev200_vi \\
         hi3516ev200_isp hi_sensor_i2c hi_sensor_spi \\
         hi3516ev200_vpss hi3516ev200_chnl hi3516ev200_vedu \\
         hi3516ev200_rc hi3516ev200_venc hi3516ev200_h264e \\
         hi3516ev200_h265e hi3516ev200_jpege \\
         hi3516ev200_aio hi3516ev200_ai hi3516ev200_ao \\
         hi3516ev200_aenc hi3516ev200_adec hi3516ev200_acodec \\
         hi3516ev200_pm hi3516ev200_ive hi_pwm; do
    [ -e \${m}.ko ] || continue
    case \$m in
        sys_config) ARGS="chip=hi3516ev200 sensors=${SENSOR} g_cmos_yuv_flag=0 board=demo";;
        hi_osal)    ARGS="anony=1 mmz_allocator=hisi mmz=anonymous,0,0x46000000,32M";;
        hi3516ev200_vgs) ARGS="max_vgs_job=20 max_vgs_node=20 max_vgs_task=20";;
        hi3516ev200_ive) ARGS="save_power=0";;
        *)          ARGS="";;
    esac
    insmod \${m}.ko \$ARGS > /dev/null 2>&1
done

chroot /tmp/vendor /Sofia > /tmp/sofia.log 2>&1 &
SPID=\$!

for i in \$(seq 1 ${ISP_READY_TIMEOUT}); do
    sleep 1
    if grep -q "ISP Dev 0 running" /tmp/sofia.log 2>/dev/null; then
        echo "ISP_READY_AT t=\${i}s"
        break
    fi
    if ! kill -0 \$SPID 2>/dev/null; then
        echo "SOFIA_DIED at t=\${i}s"
        tail -30 /tmp/sofia.log
        poweroff -f
    fi
done

if ! grep -q "ISP Dev 0 running" /tmp/sofia.log 2>/dev/null; then
    echo "ISP_NOT_READY_TIMEOUT"
    tail -30 /tmp/sofia.log
    poweroff -f
fi

# Let AE/AWB settle a few frames before snapshotting.
sleep 3

/utils/isp-profile export /utils/isp-captures/${OUT_NAME} ${SENSOR} 2>&1 | head -5
if [ -s /utils/isp-captures/${OUT_NAME} ]; then
    echo "CAPTURE_OK \$(wc -c < /utils/isp-captures/${OUT_NAME}) bytes"
    md5sum /utils/isp-captures/${OUT_NAME}
else
    echo "CAPTURE_FAILED"
fi
poweroff -f
EOF

LOG=$(mktemp -t cap-${SENSOR}-XXXXXX.log)
{
    sleep 8
    printf '%s\n' "$GUEST_SETUP"
} | env INIT=/bin/sh SENSOR="${SENSOR}" \
    timeout "$QEMU_TIMEOUT" bash "${SCRIPT_DIR}/run-ev200.sh" \
    > "$LOG" 2>&1 || true

if grep -q "CAPTURE_OK" "$LOG"; then
    grep -E "ISP_READY_AT|CAPTURE_OK|^[0-9a-f]{32}" "$LOG"
    echo "==> /mnt/noc/isp-captures/${OUT_NAME}"
    rm -f "$LOG"
    exit 0
fi

echo "==> CAPTURE FAILED (full log: $LOG)"
echo "--- last 60 lines ---"
tail -60 "$LOG"
exit 1
