#!/bin/bash
#
# Create a routed TAP network for QEMU guests.
#
# Sets up qbr0 bridge (192.168.0.1/24) with tap0 and a dnsmasq DHCP
# server.  The host forwards packets between qbr0 and the lab network
# (enp12s0).  No NAT — the lab router is expected to route 192.168.0.0/24
# back to this host.
#
# Must be run as root (or via sudo).
# Run net-teardown.sh to reverse.
#
set -euo pipefail

BRIDGE="${BRIDGE:-qbr0}"
TAP="${TAP:-tap0}"
TAP_USER="${TAP_USER:-$(logname 2>/dev/null || echo ${SUDO_USER:-$USER})}"
SUBNET="192.168.0"
BRIDGE_IP="${SUBNET}.1"
DHCP_START="${SUBNET}.50"
DHCP_END="${SUBNET}.99"
HOST_IF="${HOST_IF:-enp12s0}"
LAB_GW="${LAB_GW:-10.216.128.1}"

# ── Clean stale state ────────────────────────────────────────────────
pkill -f "dnsmasq.*${BRIDGE}" 2>/dev/null || true
if ip link show "$BRIDGE" &>/dev/null; then
    ip link del "$BRIDGE" 2>/dev/null || true
fi
if ip link show "$TAP" &>/dev/null; then
    ip tuntap del dev "$TAP" mode tap 2>/dev/null || true
fi

# ── Create bridge + TAP ──────────────────────────────────────────────
ip tuntap add dev "$TAP" mode tap user "$TAP_USER"
ip link add name "$BRIDGE" type bridge
ip link set "$BRIDGE" type bridge stp_state 0
ip link set "$TAP" master "$BRIDGE"
ip link set "$BRIDGE" up
ip link set "$TAP" up
ip addr add "${BRIDGE_IP}/24" dev "$BRIDGE"

# ── Enable forwarding ─────────────────────────────────────────────────
sysctl -q -w net.ipv4.ip_forward=1
sysctl -q -w net.ipv4.conf."$BRIDGE".accept_local=1

# Insert rules at the TOP of chains (before Docker/libvirt rules)
iptables -I FORWARD 1 -i "$BRIDGE" -o "$HOST_IF" -j ACCEPT
iptables -I FORWARD 2 -i "$HOST_IF" -o "$BRIDGE" -j ACCEPT
iptables -I INPUT 1 -i "$BRIDGE" -j ACCEPT

# NAT: guest traffic appears as host IP on the lab network.
# Required because lab firewalls may block traffic from unknown subnets.
iptables -t nat -I POSTROUTING 1 -s "${SUBNET}.0/24" -o "$HOST_IF" -j MASQUERADE

# ── Start dnsmasq (DHCP only, DNS via lab gateway) ───────────────────
# Use --port=0 to skip DNS entirely — avoids conflict with
# systemd-resolved on port 53.  Guests get the lab gateway as
# their DNS server via DHCP option 6.
dnsmasq \
    --interface="$BRIDGE" --bind-interfaces \
    --port=0 \
    --dhcp-range="${DHCP_START},${DHCP_END},255.255.255.0,12h" \
    --dhcp-option=3,"${BRIDGE_IP}" \
    --dhcp-option=6,"${LAB_GW}" \
    --no-daemon --log-dhcp \
    --pid-file=/run/qemu-net/dnsmasq.pid &
disown

# ── Save state for teardown ──────────────────────────────────────────
mkdir -p /run/qemu-net
cat > /run/qemu-net/state <<EOF
BRIDGE=$BRIDGE
TAP=$TAP
HOST_IF=$HOST_IF
EOF

echo "Bridge $BRIDGE ready (${BRIDGE_IP}/24). TAP $TAP for user $TAP_USER."
echo "DHCP: ${DHCP_START}-${DHCP_END}, gateway ${BRIDGE_IP}, DNS ${LAB_GW}."
echo "Routing via $HOST_IF (no NAT)."
