#!/bin/bash
#
# Reproduce the CI Linux ping+curl network check with explicit timing
# markers so slow guests can be diagnosed without relying on silent waits.
#
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  test-linux-network.sh --soc SOC --machine MACHINE --mem MEM_MB --append APPEND [options]

Options:
  --qemu PATH              QEMU binary path
  --tap IFACE              TAP interface name (default: tap0)
  --bridge IFACE           Bridge name when --setup-network is used (default: qbr0)
  --host-ip IP             Host bridge IP and HTTP target (default: 10.0.10.1)
  --setup-network          Create TAP/bridge/http/dnsmasq locally and tear it down
  --login-timeout SEC      Wait for login prompt (default: 240)
  --ping-timeout SEC       Wait for ping completion marker (default: 45)
  --curl-timeout SEC       Wait for curl completion marker (default: 100)
  --curl-max-time SEC      Guest curl --max-time value (default: 90)
  --output-dir DIR         Directory for diagnostic output (default: /tmp/linux-network-SOC-PID)
EOF
}

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-arm"
TAP="tap0"
BRIDGE="qbr0"
HOST_IP="10.0.10.1"
LOGIN_TIMEOUT=240
PING_TIMEOUT=45
CURL_TIMEOUT=100
CURL_MAX_TIME=90
SETUP_NETWORK=0
OUTPUT_DIR=""
SOC=""
MACHINE=""
MEM_MB=""
APPEND=""

while [ $# -gt 0 ]; do
    case "$1" in
        --soc) SOC="$2"; shift 2 ;;
        --machine) MACHINE="$2"; shift 2 ;;
        --mem) MEM_MB="$2"; shift 2 ;;
        --append) APPEND="$2"; shift 2 ;;
        --qemu) QEMU="$2"; shift 2 ;;
        --tap) TAP="$2"; shift 2 ;;
        --bridge) BRIDGE="$2"; shift 2 ;;
        --host-ip) HOST_IP="$2"; shift 2 ;;
        --setup-network) SETUP_NETWORK=1; shift ;;
        --login-timeout) LOGIN_TIMEOUT="$2"; shift 2 ;;
        --ping-timeout) PING_TIMEOUT="$2"; shift 2 ;;
        --curl-timeout) CURL_TIMEOUT="$2"; shift 2 ;;
        --curl-max-time) CURL_MAX_TIME="$2"; shift 2 ;;
        --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [ -z "$SOC" ] || [ -z "$MACHINE" ] || [ -z "$APPEND" ]; then
    usage >&2
    exit 2
fi

# --mem is optional — when omitted, QEMU uses the machine's default
# ram_size from the HisiSoCConfig table.
MEM_MB="${MEM_MB%M}"
MEM_ARG=""
[ -n "$MEM_MB" ] && MEM_ARG="-m ${MEM_MB}M"

if [ -z "$OUTPUT_DIR" ]; then
    OUTPUT_DIR="/tmp/linux-network-${SOC}-$$"
fi

SER_PREFIX="/tmp/${SOC}.ser"
SER_IN="${SER_PREFIX}.in"
SER_OUT="${SER_PREFIX}.out"
SERIAL_LOG="/tmp/linux-ping.txt"
SUMMARY_LOG="/tmp/linux-network-summary.txt"
HTTP_LOG="${OUTPUT_DIR}/http.log"
DNSMASQ_LOG="${OUTPUT_DIR}/dnsmasq.log"
QEMU_STDOUT="${OUTPUT_DIR}/qemu.stdout"
QEMU_STDERR="${OUTPUT_DIR}/qemu.stderr"
QEMU_PS_START="${OUTPUT_DIR}/qemu-ps-start.txt"
QEMU_PS_END="${OUTPUT_DIR}/qemu-ps-end.txt"
HOST_METRICS="${OUTPUT_DIR}/host-metrics.txt"

mkdir -p "$OUTPUT_DIR"
rm -f "$SER_IN" "$SER_OUT" "$SERIAL_LOG" "$SUMMARY_LOG"

QEMU_PID=""
HTTP_PID=""
CAT_PID=""

cleanup() {
    set +e
    [ -n "$QEMU_PID" ] && kill "$QEMU_PID" 2>/dev/null || true
    [ -n "$CAT_PID" ] && kill "$CAT_PID" 2>/dev/null || true
    [ -n "$HTTP_PID" ] && kill "$HTTP_PID" 2>/dev/null || true
    rm -f "$SER_IN" "$SER_OUT"
    if [ "$SETUP_NETWORK" = "1" ]; then
        sudo kill "$(cat /tmp/dnsmasq.pid 2>/dev/null)" 2>/dev/null || true
        sudo iptables -D INPUT -i "$BRIDGE" -j ACCEPT 2>/dev/null || true
        sudo ip link del "$BRIDGE" 2>/dev/null || true
        sudo ip tuntap del dev "$TAP" mode tap 2>/dev/null || true
    fi
}
trap cleanup EXIT

start_ts=$(date +%s)

{
    echo "soc=$SOC"
    echo "machine=$MACHINE"
    echo "mem_mb=$MEM_MB"
    echo "append=$APPEND"
    echo "qemu=$QEMU"
    echo "host_start=$(date --iso-8601=seconds)"
    echo "host_uptime_start=$(uptime)"
    echo "host_nproc=$(nproc)"
} > "$SUMMARY_LOG"

{
    echo "=== host start ==="
    uptime
    ps -eo pid,pcpu,pmem,etime,cmd --sort=-pcpu | sed -n '1,12p'
} > "$HOST_METRICS"

if [ "$SETUP_NETWORK" = "1" ]; then
    sudo ip link del "$BRIDGE" 2>/dev/null || true
    sudo ip tuntap del dev "$TAP" mode tap 2>/dev/null || true
    sudo kill "$(cat /tmp/dnsmasq.pid 2>/dev/null)" 2>/dev/null || true
    sudo iptables -D INPUT -i "$BRIDGE" -j ACCEPT 2>/dev/null || true

    sudo ip tuntap add dev "$TAP" mode tap user "$(whoami)"
    sudo ip link add "$BRIDGE" type bridge
    sudo ip link set "$BRIDGE" type bridge stp_state 0
    sudo ip link set "$TAP" master "$BRIDGE"
    sudo ip link set "$BRIDGE" up
    sudo ip link set "$TAP" up
    sudo ip addr add "${HOST_IP}/24" dev "$BRIDGE"
    sudo iptables -I INPUT -i "$BRIDGE" -j ACCEPT

    python3 -m http.server 8080 --bind "$HOST_IP" >"$HTTP_LOG" 2>&1 &
    HTTP_PID=$!

    sudo dnsmasq --interface="$BRIDGE" --bind-interfaces --listen-address="$HOST_IP" \
        --dhcp-range=10.0.10.50,10.0.10.99,255.255.255.0,1h \
        --dhcp-option=3,"$HOST_IP" --dhcp-option=6,"$HOST_IP" \
        --no-resolv --server=8.8.8.8 \
        --no-daemon --log-dhcp --pid-file=/tmp/dnsmasq.pid >"$DNSMASQ_LOG" 2>&1 &
    sleep 1
fi

mkfifo "$SER_IN" "$SER_OUT"
"$QEMU" \
    -M "$MACHINE" $MEM_ARG \
    -kernel "$REPO_ROOT/qemu-boot/uImage.${SOC}" \
    -initrd "$REPO_ROOT/qemu-boot/rootfs.squashfs.${SOC}" \
    -nographic -serial "pipe:${SER_PREFIX}" \
    -append "$APPEND" \
    -nic "tap,ifname=${TAP},script=no,downscript=no" \
    -monitor none >"$QEMU_STDOUT" 2>"$QEMU_STDERR" &
QEMU_PID=$!
ps -o pid,ppid,pcpu,pmem,etime,cmd -p "$QEMU_PID" > "$QEMU_PS_START"

cat "$SER_OUT" > "$SERIAL_LOG" &
CAT_PID=$!

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

if wait_for_marker "login:" "$LOGIN_TIMEOUT"; then
    echo "login_elapsed=$(( $(date +%s) - start_ts ))" >> "$SUMMARY_LOG"
else
    echo "login_elapsed=timeout" >> "$SUMMARY_LOG"
    echo "=== FAIL: login prompt not reached ==="
    tail -40 "$SERIAL_LOG" || true
    exit 1
fi

printf 'root\n' > "$SER_IN"
if ! wait_for_marker 'Password:' 30; then
    echo "password_prompt=timeout" >> "$SUMMARY_LOG"
    echo "=== FAIL: password prompt not reached after username ==="
    tail -40 "$SERIAL_LOG" || true
    exit 1
fi

printf '12345\n' > "$SER_IN"

printf 'echo SHELL_READY $(date +%%s)\n' > "$SER_IN"
if ! wait_for_marker '^SHELL_READY ' 30; then
    echo "shell_ready=timeout" >> "$SUMMARY_LOG"
    echo "=== FAIL: shell prompt did not become ready after login ==="
    tail -40 "$SERIAL_LOG" || true
    exit 1
fi

printf 'uname -a\n' > "$SER_IN"
printf 'cat /proc/loadavg\n' > "$SER_IN"
printf 'free\n' > "$SER_IN"
printf 'ip addr show\n' > "$SER_IN"
printf 'route -n\n' > "$SER_IN"

ping_cmd="ping -c 3 ${HOST_IP}; echo PING_DONE RC=\$? \$(date +%s)"
printf '%s\n' "$ping_cmd" > "$SER_IN"
if wait_for_marker '^PING_DONE RC=' "$PING_TIMEOUT"; then
    echo "ping_elapsed=$(( $(date +%s) - start_ts ))" >> "$SUMMARY_LOG"
else
    echo "ping_elapsed=timeout" >> "$SUMMARY_LOG"
fi

curl_cmd="echo CURL_START \$(date +%s); curl --connect-timeout 10 --max-time ${CURL_MAX_TIME} -s -o /dev/null -w 'HTTP_CODE=%{http_code}\n' http://${HOST_IP}:8080/; echo CURL_DONE RC=\$? \$(date +%s)"
printf '%s\n' "$curl_cmd" > "$SER_IN"
if wait_for_marker '^CURL_DONE RC=' "$CURL_TIMEOUT"; then
    echo "curl_elapsed=$(( $(date +%s) - start_ts ))" >> "$SUMMARY_LOG"
else
    echo "curl_elapsed=timeout" >> "$SUMMARY_LOG"
fi

ps -o pid,ppid,pcpu,pmem,etime,cmd -p "$QEMU_PID" > "$QEMU_PS_END"
{
    echo "=== host end ==="
    uptime
    ps -eo pid,pcpu,pmem,etime,cmd --sort=-pcpu | sed -n '1,12p'
} >> "$HOST_METRICS"
echo "host_uptime_end=$(uptime)" >> "$SUMMARY_LOG"
echo "host_end=$(date --iso-8601=seconds)" >> "$SUMMARY_LOG"
echo "output_dir=$OUTPUT_DIR" >> "$SUMMARY_LOG"

kill "$QEMU_PID" 2>/dev/null || true
wait "$QEMU_PID" 2>/dev/null || true
kill "$CAT_PID" 2>/dev/null || true
rm -f "$SER_IN" "$SER_OUT"

FAIL=0

if grep -a -q "PING_DONE RC=0" "$SERIAL_LOG"; then
    echo "=== PASS: Linux ping ==="
else
    echo "=== FAIL: Linux ping ==="
    grep -a -E "PING|bytes from|packet loss|PING_DONE" "$SERIAL_LOG" || true
    FAIL=1
fi

if grep -a -q "HTTP_CODE=200" "$SERIAL_LOG" && grep -a -q "CURL_DONE RC=0" "$SERIAL_LOG"; then
    echo "=== PASS: Linux curl (TCP) ==="
else
    echo "=== FAIL: Linux curl (TCP) ==="
    grep -a -E "CURL_START|HTTP_CODE|CURL_DONE|curl" "$SERIAL_LOG" || true
    FAIL=1
fi

cat "$SUMMARY_LOG"

if [ "$FAIL" -ne 0 ]; then
    exit 1
fi
