#!/bin/bash
#
# Test SP805 watchdog register access on HiSilicon machines.
#
# Usage:  bash qemu/tests/test-hisi-wdt.sh
#
# Tests CV300 (0x12080000) and EV300 (0x12030000) watchdog registers.
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

# Strip ANSI escape sequences
strip_ansi() {
    sed 's/\x1b\[[0-9;]*[A-Za-z]//g' | tr -d '\r'
}

run_test() {
    local machine="$1"
    local wdt_base="$2"
    local label="$3"

    echo "--- $label (machine=$machine, base=$wdt_base) ---"

    local addrs=()
    local names=()
    local expected=()

    addrs+=("$(printf '0x%x' $((wdt_base + 0x000)))"); names+=("LOAD reset value");    expected+=("0xffffffff")
    addrs+=("$(printf '0x%x' $((wdt_base + 0x004)))"); names+=("VALUE reset value");   expected+=("0xffffffff")
    addrs+=("$(printf '0x%x' $((wdt_base + 0x008)))"); names+=("CTRL reset value");    expected+=("0x00000000")
    addrs+=("$(printf '0x%x' $((wdt_base + 0x010)))"); names+=("RIS reset value");     expected+=("0x00000000")
    addrs+=("$(printf '0x%x' $((wdt_base + 0x014)))"); names+=("MIS reset value");     expected+=("0x00000000")
    addrs+=("$(printf '0x%x' $((wdt_base + 0xFE0)))"); names+=("PeriphID0 (CMSDK WDT)"); expected+=("0x00000024")

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
        # Output lines look like: "0000000012080000: 0xffffffff"
        val=$(echo "$output" | grep -oP '[0-9a-f]+:\s+\K0x[0-9a-fA-F]+' | sed -n "$((i + 1))p")
        check "${names[$i]}" "${expected[$i]}" "$val"
        i=$((i + 1))
    done
}

run_test "hi3516cv300" "0x12080000" "CV300 Watchdog"
run_test "hi3516ev300" "0x12030000" "EV300 Watchdog"

echo ""
echo "=== Results: $PASS passed, $FAIL failed ==="
[ "$FAIL" -eq 0 ] && exit 0 || exit 1
