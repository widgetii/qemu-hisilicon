#!/bin/bash
#
# SPI-NOR erase + program round-trip test (regression guard for the HiFMC
# erase-address / WEL fix that makes OpenIPC `firstboot` and `sysupgrade`
# work — see "hisi-fmc: fix NOR erase/program ...").
#
# Boots OpenIPC from the ramdisk (root=/dev/ram0) with a writable synthetic
# SPI-NOR attached via -machine flash-file=, then from Linux userspace:
#   1. flash_eraseall a multi-block partition  -> must read back 0xFF
#   2. flashcp -v random data over it          -> must verify (exit 0)
#
# Both fail with the pre-fix model: the erase address was read from the
# stale FMC_ADDRL instead of the IO buffer, so every block after the first
# erased the wrong offset (partition stayed unerased; flashcp mismatched
# at the first un-erased block).  A multi-block payload is required to catch
# it — a single-block write happens to land on whatever addrl held.
#
# Usage:
#   test-flash-rw.sh --soc hi3516ev300 --machine hi3516ev300,sensor=none \
#       --qemu ./qemu-src/build/qemu-system-arm --output-dir <dir> \
#       [--mem 128M] [--login-timeout 120]
#
set -u

SOC=""
MACHINE=""
QEMU="./qemu-src/build/qemu-system-arm"
OUTDIR="."
MEM="128M"
LOGIN_TIMEOUT=120

while [ $# -gt 0 ]; do
    case "$1" in
        --soc)           SOC="$2"; shift 2 ;;
        --machine)       MACHINE="$2"; shift 2 ;;
        --qemu)          QEMU="$2"; shift 2 ;;
        --output-dir)    OUTDIR="$2"; shift 2 ;;
        --mem)           MEM="$2"; shift 2 ;;
        --login-timeout) LOGIN_TIMEOUT="$2"; shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

[ -n "$SOC" ] && [ -n "$MACHINE" ] || { echo "--soc and --machine required" >&2; exit 2; }
mkdir -p "$OUTDIR"

KERNEL="qemu-boot/uImage.${SOC}"
INITRD="qemu-boot/rootfs.squashfs.${SOC}"
for f in "$KERNEL" "$INITRD"; do
    [ -f "$f" ] || { echo "=== FAIL: missing $f ==="; exit 1; }
done

# 8 MiB synthetic NOR.  mtdparts carves a 1 MiB writable "test" partition
# (mtd3, 16 erase blocks) that is NOT the rootfs (root is the ramdisk), so
# erasing/writing it is safe.
FLASH="$OUTDIR/flash.bin"
dd if=/dev/zero of="$FLASH" bs=1M count=8 2>/dev/null
MTDPARTS="hi_sfc:256k(boot),64k(env),2048k(kernel),1024k(test),-(rest)"
APPEND="console=ttyAMA0,115200 root=/dev/ram0 rootfstype=squashfs mtdparts=${MTDPARTS}"

CONSOLE="$OUTDIR/flash-rw-console.txt"
SER_IN="$OUTDIR/ser.in"
SER_OUT="$OUTDIR/ser.out"
rm -f "$SER_IN" "$SER_OUT" "$CONSOLE"
mkfifo "$SER_IN" "$SER_OUT"

# restrict=on: keep SLIRP DHCP reachable but block outbound (busybox ntpd,
# see openhisilicon#104) so init runs deterministically.
timeout 240 "$QEMU" \
    -M "$MACHINE",flash-file="$FLASH" -m "$MEM" \
    -kernel "$KERNEL" -initrd "$INITRD" \
    -nographic -serial "pipe:${OUTDIR}/ser" -monitor none \
    -nic user,restrict=on \
    -append "$APPEND" &
QEMU_PID=$!
( timeout 240 cat "$SER_OUT" > "$CONSOLE" 2>&1 ) &
exec 3<>"$SER_IN"

cleanup() { exec 3>&- 2>/dev/null; kill "$QEMU_PID" 2>/dev/null; wait 2>/dev/null; rm -f "$SER_IN" "$SER_OUT"; }
trap cleanup EXIT

wait_for() { # marker timeout
    local m="$1" t="$2" i
    for i in $(seq 1 "$t"); do
        grep -qF "$m" "$CONSOLE" 2>/dev/null && return 0
        sleep 1
    done
    return 1
}

echo "=== waiting for login prompt (<= ${LOGIN_TIMEOUT}s) ==="
if ! wait_for "login:" "$LOGIN_TIMEOUT"; then
    echo "=== FAIL: login prompt not reached ==="; tail -30 "$CONSOLE"; exit 1
fi
printf 'root\r' >&3; sleep 2
printf '12345\r' >&3; sleep 3

# Let init settle, then drive the round-trip.  NOTE: the serial console
# echoes each typed command back, so a literal success word (e.g. "OK")
# would also appear in the echoed command line.  All results are therefore
# reported as `NAME=<digit>` exit codes and extracted with a digit-anchored
# sed — the command echo carries the literal "$?", never a digit, so only
# the real output line matches.
printf 'echo RW_READY_$((6*7))\r' >&3
wait_for "RW_READY_42" 25 || { echo "=== FAIL: shell not ready ==="; tail -30 "$CONSOLE"; exit 1; }

# 1) erase a 1 MiB (16-block) partition, 2) confirm it reads back all-0xFF,
# 3) flashcp -v a 512 KiB (8-block) random payload over it.  Multi-block is
# essential: the pre-fix bug erased every block but the first at a stale
# address, so the readback stayed un-erased and flashcp's verify mismatched.
printf 'flash_eraseall /dev/mtd3 >/dev/null 2>&1; echo ERASE_RC=$?\r' >&3; sleep 2
printf 'xxd -l 16 /dev/mtd3 | grep -q "ffff ffff ffff ffff ffff ffff ffff ffff"; echo EFF_RC=$?\r' >&3; sleep 2
printf 'head -c 524288 /dev/urandom > /tmp/t; flashcp -v /tmp/t /dev/mtd3 >/dev/null 2>&1; echo FLASHCP_RC=$?\r' >&3; sleep 3
printf 'echo RW_DONE_$((21*2))\r' >&3
wait_for "RW_DONE_42" 60 || { echo "=== FAIL: round-trip did not complete ==="; tail -40 "$CONSOLE"; exit 1; }

sleep 1
sync_strip() { sed 's/\r//g' "$CONSOLE"; }
get_rc() { sync_strip | sed -n "s/.*$1=\\([0-9]\\).*/\\1/p" | head -1; }

erase_rc=$(get_rc ERASE_RC)      # flash_eraseall exit status
eff_rc=$(get_rc EFF_RC)          # 0 => mtd3 reads all-0xFF after erase
flashcp_rc=$(get_rc FLASHCP_RC)  # 0 => busybox flashcp -v erase+write+verify OK

echo "=== results: ERASE_RC=$erase_rc EFF_RC=$eff_rc FLASHCP_RC=$flashcp_rc ==="

fail=0
[ "$erase_rc" = "0" ]   || { echo "FAIL: flash_eraseall /dev/mtd3 returned '$erase_rc'"; fail=1; }
[ "$eff_rc" = "0" ]     || { echo "FAIL: mtd3 not 0xFF after erase (erase hit wrong address)"; fail=1; }
[ "$flashcp_rc" = "0" ] || { echo "FAIL: flashcp -v returned '$flashcp_rc' (verify mismatch)"; fail=1; }

if [ "$fail" = "0" ]; then
    echo "=== PASS: SPI-NOR erase + program round-trip OK ==="
    exit 0
fi
echo "--- last 40 console lines ---"; sync_strip | tail -40
exit 1
