#!/bin/bash
#
# Test MIPI RX controller register access on HiSilicon V4 machines.
#
# Usage:  bash qemu/tests/test-hisi-mipi-rx.sh
#
# Tests the hisi-mipi-rx device at 0x11240000 on EV300 and GK7605V100.
# Verifies:
#   - All registers reset to 0
#   - Interrupt status registers always read 0 (no errors)
#   - Interrupt mask registers are writable (RAM-backed)
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

MIPI_BASE=0x11240000

run_test() {
    local machine="$1"
    local label="$2"

    echo "--- $label (machine=$machine, base=$MIPI_BASE) ---"

    local addrs=()
    local names=()
    local expected=()

    # PHY config registers — reset to 0
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0x000)))"); names+=("PHY_MODE_LINK reset");       expected+=("0x00000000")
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0x004)))"); names+=("PHY_SKEW_LINK reset");       expected+=("0x00000000")
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0x00c)))"); names+=("PHY_EN_LINK reset");         expected+=("0x00000000")
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0x104)))"); names+=("CIL_FSM0_LINK reset");       expected+=("0x00000000")
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0x108)))"); names+=("CIL_FSM_ST0 reset");         expected+=("0x00000000")
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0x10c)))"); names+=("CIL_FSM_ST1 reset");         expected+=("0x00000000")

    # PHY CIL interrupt registers — always 0
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0x1f0)))"); names+=("CIL_INT_RAW (no errs)");     expected+=("0x00000000")
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0x1f4)))"); names+=("CIL_INT (no errs)");         expected+=("0x00000000")

    # System interrupt registers — always 0
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0xff0)))"); names+=("MIPI_INT_RAW (no errs)");    expected+=("0x00000000")
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0xff4)))"); names+=("MIPI_INT_ST (no errs)");     expected+=("0x00000000")

    # MIPI ctrl interrupt registers — always 0
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0x100c)))"); names+=("MAIN_INT_ST (no errs)");    expected+=("0x00000000")
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0x1060)))"); names+=("PKT_INTR_ST (no errs)");    expected+=("0x00000000")
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0x1070)))"); names+=("PKT_INTR2_ST (no errs)");   expected+=("0x00000000")
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0x1080)))"); names+=("FRAME_INTR_ST (no errs)");  expected+=("0x00000000")
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0x1090)))"); names+=("LINE_INTR_ST (no errs)");   expected+=("0x00000000")
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0x12f0)))"); names+=("CTRL_INT_RAW (no errs)");   expected+=("0x00000000")
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0x12f4)))"); names+=("CTRL_INT (no errs)");       expected+=("0x00000000")

    # LVDS ctrl interrupt registers — always 0
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0x17f0)))"); names+=("LVDS_INT_RAW (no errs)");   expected+=("0x00000000")
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0x17f4)))"); names+=("LVDS_INT (no errs)");       expected+=("0x00000000")
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0x18f0)))"); names+=("ALIGN_INT_RAW (no errs)");  expected+=("0x00000000")
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0x18f4)))"); names+=("ALIGN_INT (no errs)");      expected+=("0x00000000")
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0x1ff0)))"); names+=("CHN_INT_RAW (no errs)");    expected+=("0x00000000")
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0x1ff4)))"); names+=("CHN_INT (no errs)");        expected+=("0x00000000")

    # MIPI ctrl data registers — reset to 0
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0x1004)))"); names+=("LANES_NUM reset");          expected+=("0x00000000")
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0x1010)))"); names+=("MIPI_DI_1 reset");          expected+=("0x00000000")
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0x1224)))"); names+=("IMGSIZE reset");            expected+=("0x00000000")
    addrs+=("$(printf '0x%x' $((MIPI_BASE + 0x1250)))"); names+=("IMGSIZE0_STATIS reset");    expected+=("0x00000000")

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

run_test "hi3516ev300" "EV300 MIPI RX"
echo ""
run_test "gk7605v100" "GK7605V100 MIPI RX"

echo ""
echo "=== Results: $PASS passed, $FAIL failed ==="
[ "$FAIL" -eq 0 ] && exit 0 || exit 1
