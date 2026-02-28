#!/usr/bin/env bash
# setup-adhoc-ap.sh
# Configure Raspberry Pi Zero 2W as an AP (infrastructure-mode hotspot) using hostapd + dnsmasq.
# This is the most compatible approach; Linux "ad-hoc" (IBSS) is widely unsupported by phones/laptops now.
#
# Usage:
#   sudo ./setup-adhoc-ap.sh [SSID] [PASSPHRASE] [AP_IP_CIDR] [CHANNEL]
# Example:
#   sudo ./setup-adhoc-ap.sh "PiZeroAP" "correct-horse-battery-staple" "10.42.0.1/24" "6"

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run as root: sudo $0 [SSID] [PASSPHRASE] [AP_IP_CIDR] [CHANNEL]" >&2
  exit 1
fi

SSID="${1:-PiZeroAP}"
PASSPHRASE="${2:-changeme12345}"
AP_IP_CIDR="${3:-10.42.0.1/24}"
CHANNEL="${4:-6}"

if [[ "${#PASSPHRASE}" -lt 8 ]]; then
  echo "Passphrase must be at least 8 characters." >&2
  exit 1
fi

WLAN_IF="wlan0"
STATE_DIR="/var/lib/piap_state"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$STATE_DIR"
touch "$STATE_DIR/.created_${STAMP}"

need_cmd() { command -v "$1" >/dev/null 2>&1 || { echo "Missing command: $1" >&2; exit 1; }; }
need_cmd ip
need_cmd systemctl
need_cmd sed
need_cmd awk

echo "[1/8] Installing packages..."
export DEBIAN_FRONTEND=noninteractive
apt-get update -y
apt-get install -y hostapd dnsmasq iptables iptables-persistent

echo "[2/8] Stopping services before config..."
systemctl stop hostapd || true
systemctl stop dnsmasq || true

echo "[3/8] Backing up current network config..."
# NetworkManager vs dhcpcd presence varies; we support common Raspberry Pi OS defaults.
BACKUP_DIR="$STATE_DIR/backup_${STAMP}"
mkdir -p "$BACKUP_DIR"

# Back up files if they exist
for f in \
  /etc/dhcpcd.conf \
  /etc/network/interfaces \
  /etc/NetworkManager/NetworkManager.conf \
  /etc/NetworkManager/system-connections \
  /etc/dnsmasq.conf \
  /etc/default/hostapd \
  /etc/hostapd/hostapd.conf \
  /etc/sysctl.conf \
  /etc/iptables/rules.v4 \
  /etc/iptables/rules.v6 \
  /etc/resolv.conf
do
  if [[ -e "$f" ]]; then
    mkdir -p "$BACKUP_DIR$(dirname "$f")"
    # For dirs: copy recursively; for files: preserve
    if [[ -d "$f" ]]; then
      cp -a "$f" "$BACKUP_DIR$(dirname "$f")/"
    else
      cp -a "$f" "$BACKUP_DIR$f"
    fi
  fi
done

# Record which network stack seems active
NETMGR_ACTIVE="no"
if systemctl is-enabled NetworkManager >/dev/null 2>&1; then
  NETMGR_ACTIVE="yes"
fi
echo "$NETMGR_ACTIVE" > "$BACKUP_DIR/netmgr_enabled.flag"

echo "[4/8] Choosing uplink interface for NAT (best-effort)..."
# Prefer eth0, otherwise any default route dev other than wlan0, otherwise eth0.
UPLINK_IF="eth0"
DEFAULT_DEV="$(ip route show default 0.0.0.0/0 | awk '{for(i=1;i<=NF;i++) if($i=="dev"){print $(i+1); exit}}' || true)"
if [[ -n "${DEFAULT_DEV}" && "${DEFAULT_DEV}" != "${WLAN_IF}" ]]; then
  UPLINK_IF="${DEFAULT_DEV}"
fi
echo "${UPLINK_IF}" > "$BACKUP_DIR/uplink_if.txt"
echo "Uplink interface: ${UPLINK_IF}"

echo "[5/8] Writing hostapd config..."
cat > /etc/hostapd/hostapd.conf <<EOF
interface=${WLAN_IF}
driver=nl80211
ssid=${SSID}
hw_mode=g
channel=${CHANNEL}
ieee80211n=1
wmm_enabled=1
auth_algs=1
ignore_broadcast_ssid=0

wpa=2
wpa_passphrase=${PASSPHRASE}
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
EOF

# Point /etc/default/hostapd at config
if [[ -f /etc/default/hostapd ]]; then
  sed -i 's|^#\?DAEMON_CONF=.*|DAEMON_CONF="/etc/hostapd/hostapd.conf"|' /etc/default/hostapd
else
  cat > /etc/default/hostapd <<'EOF'
DAEMON_CONF="/etc/hostapd/hostapd.conf"
EOF
fi

echo "[6/8] Configuring dnsmasq for DHCP/DNS on ${WLAN_IF}..."
# Keep original /etc/dnsmasq.conf as backup already taken; write minimal drop-in behavior.
cat > /etc/dnsmasq.conf <<EOF
interface=${WLAN_IF}
bind-interfaces
domain-needed
bogus-priv
dhcp-range=10.42.0.50,10.42.0.150,255.255.255.0,12h
dhcp-option=option:router,10.42.0.1
dhcp-option=option:dns-server,10.42.0.1
EOF

echo "[7/8] Assigning static IP to ${WLAN_IF} and enabling forwarding..."
# dhcpcd is common on Raspberry Pi OS; if present, use it.
if systemctl list-unit-files | grep -q '^dhcpcd\.service'; then
  # Remove any previous block from our tool
  sed -i '/^# PIAP-BEGIN$/,/^# PIAP-END$/d' /etc/dhcpcd.conf || true
  cat >> /etc/dhcpcd.conf <<EOF

# PIAP-BEGIN
interface ${WLAN_IF}
  static ip_address=${AP_IP_CIDR}
  nohook wpa_supplicant
# PIAP-END
EOF
  systemctl restart dhcpcd
else
  # Fallback: set IP directly (non-persistent if no dhcpcd/NetworkManager).
  ip link set "${WLAN_IF}" up
  ip addr flush dev "${WLAN_IF}" || true
  ip addr add "${AP_IP_CIDR}" dev "${WLAN_IF}"
fi

# Enable IPv4 forwarding
if grep -q '^\s*net.ipv4.ip_forward\s*=\s*1\s*$' /etc/sysctl.conf; then
  true
else
  # Remove any old line then add ours
  sed -i '/^\s*net\.ipv4\.ip_forward\s*=/d' /etc/sysctl.conf
  echo 'net.ipv4.ip_forward=1' >> /etc/sysctl.conf
fi
sysctl -w net.ipv4.ip_forward=1 >/dev/null

echo "[8/8] Setting up NAT with iptables..."
# Clean old rules we own (best-effort)
iptables -t nat -D POSTROUTING -o "${UPLINK_IF}" -j MASQUERADE 2>/dev/null || true
iptables -D FORWARD -i "${UPLINK_IF}" -o "${WLAN_IF}" -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || true
iptables -D FORWARD -i "${WLAN_IF}" -o "${UPLINK_IF}" -j ACCEPT 2>/dev/null || true

iptables -t nat -A POSTROUTING -o "${UPLINK_IF}" -j MASQUERADE
iptables -A FORWARD -i "${UPLINK_IF}" -o "${WLAN_IF}" -m state --state RELATED,ESTABLISHED -j ACCEPT
iptables -A FORWARD -i "${WLAN_IF}" -o "${UPLINK_IF}" -j ACCEPT

# Persist rules if service present
if systemctl list-unit-files | grep -q '^netfilter-persistent\.service'; then
  netfilter-persistent save
fi

systemctl unmask hostapd || true
systemctl enable hostapd
systemctl enable dnsmasq
systemctl restart dnsmasq
systemctl restart hostapd

echo "DONE.
AP SSID: ${SSID}
AP IP:   ${AP_IP_CIDR%/*}
DHCP:    10.42.0.50-10.42.0.150
NAT via: ${UPLINK_IF}

To undo: sudo ./restore-original-network.sh"