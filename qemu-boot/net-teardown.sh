#!/bin/bash
#
# Tear down QEMU guest network created by net-setup.sh.
#
# Must be run as root (or via sudo).
#
set -euo pipefail

STATE_FILE="/run/qemu-net/state"

if [ ! -f "$STATE_FILE" ]; then
    echo "No state file found at $STATE_FILE; nothing to tear down." >&2
    exit 1
fi

source "$STATE_FILE"

# Stop dnsmasq
if [ -f /run/qemu-net/dnsmasq.pid ]; then
    kill "$(cat /run/qemu-net/dnsmasq.pid)" 2>/dev/null || true
    rm -f /run/qemu-net/dnsmasq.pid
fi
pkill -f "dnsmasq.*${BRIDGE}" 2>/dev/null || true

# Remove iptables rules
iptables -t nat -D POSTROUTING -s 192.168.0.0/24 -o "$HOST_IF" -j MASQUERADE 2>/dev/null || true
iptables -D FORWARD -i "$BRIDGE" -o "$HOST_IF" -j ACCEPT 2>/dev/null || true
iptables -D FORWARD -i "$HOST_IF" -o "$BRIDGE" -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || true

# Remove TAP
if ip link show "$TAP" &>/dev/null; then
    ip link set "$TAP" down
    ip tuntap del dev "$TAP" mode tap
    echo "Removed $TAP."
fi

# Remove bridge
if ip link show "$BRIDGE" &>/dev/null; then
    ip link set "$BRIDGE" down
    ip link del "$BRIDGE"
    echo "Removed $BRIDGE."
fi

rm -f "$STATE_FILE"
echo "Network teardown complete."
