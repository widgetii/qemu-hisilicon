#!/bin/bash
#
# Tear down QEMU guest network created by net-setup.sh.
#
# Must be run as root (or via sudo).
#
set -euo pipefail

STATE_FILE="/run/qemu-net/state"

# Load state (or use defaults)
if [ -f "$STATE_FILE" ]; then
    source "$STATE_FILE"
else
    BRIDGE="${BRIDGE:-qbr0}"
    TAP="${TAP:-tap0}"
    HOST_IF="${HOST_IF:-enp12s0}"
fi

# Stop dnsmasq
pkill -f "dnsmasq.*${BRIDGE}" 2>/dev/null || true
rm -f /run/qemu-net/dnsmasq.pid

# Remove iptables rules (ignore errors if already gone)
iptables -D INPUT -i "$BRIDGE" -j ACCEPT 2>/dev/null || true
iptables -D FORWARD -i "$BRIDGE" -o "$HOST_IF" -j ACCEPT 2>/dev/null || true
iptables -D FORWARD -i "$HOST_IF" -o "$BRIDGE" -j ACCEPT 2>/dev/null || true
iptables -t nat -D POSTROUTING -s 192.168.0.0/24 -o "$HOST_IF" -j MASQUERADE 2>/dev/null || true

# Remove TAP
if ip link show "$TAP" &>/dev/null; then
    ip link set "$TAP" down 2>/dev/null || true
    ip tuntap del dev "$TAP" mode tap
    echo "Removed $TAP."
fi

# Remove bridge
if ip link show "$BRIDGE" &>/dev/null; then
    ip link set "$BRIDGE" down 2>/dev/null || true
    ip link del "$BRIDGE"
    echo "Removed $BRIDGE."
fi

rm -rf /run/qemu-net
echo "Network teardown complete."
