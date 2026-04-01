#!/bin/bash
#
# Create a routable network for QEMU guests with DHCP via dnsmasq.
#
# Sets up qbr0 (10.216.128.1/24) with tap0 and a dnsmasq DHCP server.
# The host acts as a NAT gateway so guests can reach the lab network.
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

# Check if bridge already exists
if ip link show "$BRIDGE" &>/dev/null; then
    echo "Bridge $BRIDGE already exists."
    if ! ip link show "$TAP" &>/dev/null; then
        ip tuntap add dev "$TAP" mode tap user "$TAP_USER"
        ip link set "$TAP" master "$BRIDGE"
        ip link set "$TAP" up
        echo "Created $TAP on existing bridge."
    fi
    exit 0
fi

# Save state for teardown
mkdir -p /run/qemu-net
cat > /run/qemu-net/state <<EOF
BRIDGE=$BRIDGE
TAP=$TAP
HOST_IF=$HOST_IF
EOF

# Create bridge (STP off, no forwarding delay)
ip link add name "$BRIDGE" type bridge
ip link set "$BRIDGE" type bridge stp_state 0
ip link set "$BRIDGE" up
ip addr add "${BRIDGE_IP}/24" dev "$BRIDGE"

# Create TAP for QEMU
ip tuntap add dev "$TAP" mode tap user "$TAP_USER"
ip link set "$TAP" master "$BRIDGE"
ip link set "$TAP" up

# Enable IP forwarding and NAT so guests can reach the lab network
sysctl -q -w net.ipv4.ip_forward=1
iptables -t nat -A POSTROUTING -s "${SUBNET}.0/24" -o "$HOST_IF" -j MASQUERADE
iptables -A FORWARD -i "$BRIDGE" -o "$HOST_IF" -j ACCEPT
iptables -A FORWARD -i "$HOST_IF" -o "$BRIDGE" -m state --state RELATED,ESTABLISHED -j ACCEPT

# Start dnsmasq as DHCP server for guests
dnsmasq --interface="$BRIDGE" \
    --bind-interfaces \
    --dhcp-range="${DHCP_START},${DHCP_END},255.255.255.0,12h" \
    --dhcp-option=3,"${BRIDGE_IP}" \
    --dhcp-option=6,"${BRIDGE_IP}" \
    --no-daemon --log-dhcp \
    --pid-file=/run/qemu-net/dnsmasq.pid &
disown

echo "Bridge $BRIDGE ready (${BRIDGE_IP}/24). TAP $TAP for user $TAP_USER."
echo "DHCP: ${DHCP_START}-${DHCP_END}, gateway ${BRIDGE_IP}."
echo "NAT via $HOST_IF to lab network."
