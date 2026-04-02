#!/bin/bash
#
# Boot an OpenIPC guest locally, wait for a shell, run a command, and record
# timing markers plus the full serial transcript for root-cause analysis.
#
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  profile-linux-guest.sh --soc SOC --machine MACHINE --mem MEM_MB --append APPEND --command CMD [options]

Options:
  --qemu PATH              QEMU binary path
  --login-timeout SEC      Wait for login prompt (default: 300)
  --command-timeout SEC    Wait for command completion marker (default: 600)
  --output-dir DIR         Directory for logs (default: /tmp/profile-linux-guest-SOC-PID)
EOF
}

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-arm"
LOGIN_TIMEOUT=300
COMMAND_TIMEOUT=600
OUTPUT_DIR=""
SOC=""
MACHINE=""
MEM_MB=""
APPEND=""
COMMAND=""

while [ $# -gt 0 ]; do
    case "$1" in
        --soc) SOC="$2"; shift 2 ;;
        --machine) MACHINE="$2"; shift 2 ;;
        --mem) MEM_MB="$2"; shift 2 ;;
        --append) APPEND="$2"; shift 2 ;;
        --command) COMMAND="$2"; shift 2 ;;
        --qemu) QEMU="$2"; shift 2 ;;
        --login-timeout) LOGIN_TIMEOUT="$2"; shift 2 ;;
        --command-timeout) COMMAND_TIMEOUT="$2"; shift 2 ;;
        --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [ -z "$SOC" ] || [ -z "$MACHINE" ] || [ -z "$MEM_MB" ] || [ -z "$APPEND" ] || [ -z "$COMMAND" ]; then
    usage >&2
    exit 2
fi

MEM_MB="${MEM_MB%M}"

if [ -z "$OUTPUT_DIR" ]; then
    OUTPUT_DIR="/tmp/profile-linux-guest-${SOC}-$$"
fi

mkdir -p "$OUTPUT_DIR"

SER_PREFIX="/tmp/${SOC}.profile.$$"
SER_IN="${SER_PREFIX}.in"
SER_OUT="${SER_PREFIX}.out"
SERIAL_LOG="${OUTPUT_DIR}/serial.txt"
SUMMARY_LOG="${OUTPUT_DIR}/summary.txt"
QEMU_STDOUT="${OUTPUT_DIR}/qemu.stdout"
QEMU_STDERR="${OUTPUT_DIR}/qemu.stderr"

QEMU_PID=""
CAT_PID=""

cleanup() {
    set +e
    [ -n "$QEMU_PID" ] && kill "$QEMU_PID" 2>/dev/null || true
    [ -n "$CAT_PID" ] && kill "$CAT_PID" 2>/dev/null || true
    rm -f "$SER_IN" "$SER_OUT"
}
trap cleanup EXIT

wait_for_marker() {
    local marker="$1"
    local timeout_sec="$2"
    local start_wait
    start_wait=$(date +%s)
    while [ $(( $(date +%s) - start_wait )) -lt "$timeout_sec" ]; do
        if grep -a -q "$marker" "$SERIAL_LOG" 2>/dev/null; then
            return 0
        fi
        sleep 1
    done
    return 1
}

BOOT_START=$(date +%s)

{
    echo "soc=$SOC"
    echo "machine=$MACHINE"
    echo "mem_mb=$MEM_MB"
    echo "append=$APPEND"
    echo "command=$COMMAND"
    echo "boot_start=$BOOT_START"
    echo "host_start=$(date --iso-8601=seconds)"
    echo "host_uptime=$(uptime)"
} > "$SUMMARY_LOG"

mkfifo "$SER_IN" "$SER_OUT"
"$QEMU" \
    -M "$MACHINE" -m "${MEM_MB}M" \
    -kernel "$REPO_ROOT/qemu-boot/uImage.${SOC}" \
    -initrd "$REPO_ROOT/qemu-boot/rootfs.squashfs.${SOC}" \
    -nographic -serial "pipe:${SER_PREFIX}" \
    -append "$APPEND" \
    -nic user \
    -monitor none >"$QEMU_STDOUT" 2>"$QEMU_STDERR" &
QEMU_PID=$!

cat "$SER_OUT" > "$SERIAL_LOG" &
CAT_PID=$!

if wait_for_marker "login:" "$LOGIN_TIMEOUT"; then
    LOGIN_TS=$(date +%s)
    echo "login_elapsed=$(( LOGIN_TS - BOOT_START ))" >> "$SUMMARY_LOG"
else
    echo "login_elapsed=timeout" >> "$SUMMARY_LOG"
    tail -40 "$SERIAL_LOG" || true
    exit 1
fi

printf 'root\n' > "$SER_IN"
if ! wait_for_marker 'Password:' 30; then
    echo "password_prompt=timeout" >> "$SUMMARY_LOG"
    tail -40 "$SERIAL_LOG" || true
    exit 1
fi

printf '12345\n' > "$SER_IN"
printf 'echo SHELL_READY $(date +%%s)\n' > "$SER_IN"
if ! wait_for_marker '^SHELL_READY ' 30; then
    echo "shell_ready=timeout" >> "$SUMMARY_LOG"
    tail -40 "$SERIAL_LOG" || true
    exit 1
fi

printf 'echo HOSTMARK start $(date +%%s)\n' > "$SER_IN"
printf '%s\n' "$COMMAND" > "$SER_IN"
printf 'echo COMMAND_RC=$?\n' > "$SER_IN"
printf 'echo HOSTMARK done $(date +%%s)\n' > "$SER_IN"

if wait_for_marker '^HOSTMARK done ' "$COMMAND_TIMEOUT"; then
    DONE_TS=$(date +%s)
    echo "command_elapsed=$(( DONE_TS - LOGIN_TS ))" >> "$SUMMARY_LOG"
else
    echo "command_elapsed=timeout" >> "$SUMMARY_LOG"
    tail -80 "$SERIAL_LOG" || true
    exit 1
fi

echo "host_end=$(date --iso-8601=seconds)" >> "$SUMMARY_LOG"
echo "output_dir=$OUTPUT_DIR" >> "$SUMMARY_LOG"

kill "$QEMU_PID" 2>/dev/null || true
wait "$QEMU_PID" 2>/dev/null || true
kill "$CAT_PID" 2>/dev/null || true
rm -f "$SER_IN" "$SER_OUT"

echo "summary: $SUMMARY_LOG"
echo "serial:  $SERIAL_LOG"
