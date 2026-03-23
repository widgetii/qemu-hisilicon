#!/bin/bash
#
# Test HiSilicon BVT RTC (SPI-bridge) register access on V4 machines.
#
# Usage:  bash qemu/tests/test-hisi-rtc.sh
#
# Tests the hisi-rtc device at 0x120e0000 on EV300 and GK7605V100.
# Verifies MMIO reset values and that the device is properly mapped.
#
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-arm"

if [ ! -x "$QEMU" ]; then
    echo "FAIL: QEMU binary not found at $QEMU"
    echo "      Run 'bash qemu/setup.sh' first."
    exit 1
fi

PASS=0
FAIL=0

check() {
    local desc="$1" expected="$2" actual="$3"
    if [ "$actual" = "$expected" ]; then
        echo "  PASS: $desc ($actual)"
        PASS=$((PASS + 1))
    else
        echo "  FAIL: $desc (expected $expected, got '$actual')"
        FAIL=$((FAIL + 1))
    fi
}

strip_ansi() {
    sed 's/\x1b\[[0-9;]*[A-Za-z]//g' | tr -d '\r'
}

RTC_BASE=0x120e0000

run_test() {
    local machine="$1"
    local label="$2"

    echo "--- $label (machine=$machine, base=$RTC_BASE) ---"

    local addrs=()
    local names=()
    local expected=()

    # SPI_CLK_DIV resets to 0
    addrs+=("$(printf '0x%x' $((RTC_BASE + 0x000)))"); names+=("SPI_CLK_DIV reset");     expected+=("0x00000000")
    # SPI_RW resets to 0
    addrs+=("$(printf '0x%x' $((RTC_BASE + 0x004)))"); names+=("SPI_RW reset");           expected+=("0x00000000")

    local cmds=""
    for addr in "${addrs[@]}"; do
        cmds+="xp/1xw ${addr}"$'\n'
    done
    cmds+="quit"$'\n'

    local output
    output=$(echo "$cmds" | timeout 10 "$QEMU" -M "$machine" -nodefaults -nographic -S \
        -chardev stdio,id=mon,mux=off -mon chardev=mon,mode=readline 2>/dev/null \
        | strip_ansi) || true

    local i=0
    for addr in "${addrs[@]}"; do
        local val
        val=$(echo "$output" | grep -oP '[0-9a-f]+:\s+\K0x[0-9a-fA-F]+' | sed -n "$((i + 1))p")
        check "${names[$i]}" "${expected[$i]}" "$val"
        i=$((i + 1))
    done
}

run_test "hi3516ev300" "EV300 RTC"
echo ""
run_test "gk7605v100" "GK7605V100 RTC"

echo ""
echo "=== Results: $PASS passed, $FAIL failed ==="
[ "$FAIL" -eq 0 ] && exit 0 || exit 1
