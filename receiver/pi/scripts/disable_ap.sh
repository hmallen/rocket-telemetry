#!/usr/bin/env bash
# restore-original-network.sh
# Revert changes from setup-adhoc-ap.sh by restoring the most recent backup snapshot.

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run as root: sudo $0" >&2
  exit 1
fi

STATE_DIR="/var/lib/piap_state"

if [[ ! -d "${STATE_DIR}" ]]; then
  echo "No state directory found at ${STATE_DIR}. Nothing to restore." >&2
  exit 1
fi

# Pick latest backup_*
LATEST_BACKUP="$(ls -1dt "${STATE_DIR}"/backup_* 2>/dev/null | head -n 1 || true)"
if [[ -z "${LATEST_BACKUP}" ]]; then
  echo "No backup snapshots found in ${STATE_DIR}." >&2
  exit 1
fi

WLAN_IF="wlan0"

echo "[1/7] Stopping AP services..."
systemctl stop hostapd 2>/dev/null || true
systemctl stop dnsmasq 2>/dev/null || true
systemctl disable hostapd 2>/dev/null || true
systemctl disable dnsmasq 2>/dev/null || true

echo "[2/7] Restoring config files from ${LATEST_BACKUP}..."
# Restore backed-up paths if present in snapshot
restore_path() {
  local p="$1"
  local src="${LATEST_BACKUP}${p}"
  if [[ -e "${src}" ]]; then
    mkdir -p "$(dirname "${p}")"
    # Directories were stored as .../etc/NetworkManager/system-connections (dir) inside snapshot's parent
    if [[ -d "${src}" ]]; then
      rm -rf "${p}"
      cp -a "${src}" "$(dirname "${p}")/"
    else
      cp -a "${src}" "${p}"
    fi
  else
    # If file didn't exist originally, remove it if we created it
    if [[ -e "${p}" ]]; then
      rm -rf "${p}"
    fi
  fi
}

for p in \
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
  restore_path "${p}"
done

echo "[3/7] Removing any PIAP block from dhcpcd.conf (safety pass)..."
if [[ -f /etc/dhcpcd.conf ]]; then
  sed -i '/^# PIAP-BEGIN$/,/^# PIAP-END$/d' /etc/dhcpcd.conf || true
fi

echo "[4/7] Disabling forwarding if we enabled it (best-effort)..."
# If original sysctl.conf existed, it's already restored. Apply runtime sysctl based on restored value.
IPFWD="$(sysctl -n net.ipv4.ip_forward 2>/dev/null || echo 0)"
# Re-read from sysctl.conf if present, else default 0
DESIRED="0"
if [[ -f /etc/sysctl.conf ]]; then
  if grep -q '^\s*net\.ipv4\.ip_forward\s*=\s*1\s*$' /etc/sysctl.conf; then
    DESIRED="1"
  fi
fi
sysctl -w net.ipv4.ip_forward="${DESIRED}" >/dev/null || true

echo "[5/7] Removing NAT rules we likely added (best-effort)..."
UPLINK_IF="eth0"
if [[ -f "${LATEST_BACKUP}/uplink_if.txt" ]]; then
  UPLINK_IF="$(cat "${LATEST_BACKUP}/uplink_if.txt" || echo eth0)"
fi

iptables -t nat -D POSTROUTING -o "${UPLINK_IF}" -j MASQUERADE 2>/dev/null || true
iptables -D FORWARD -i "${UPLINK_IF}" -o "${WLAN_IF}" -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || true
iptables -D FORWARD -i "${WLAN_IF}" -o "${UPLINK_IF}" -j ACCEPT 2>/dev/null || true

if systemctl list-unit-files | grep -q '^netfilter-persistent\.service'; then
  netfilter-persistent save 2>/dev/null || true
fi

echo "[6/7] Restarting networking services..."
if systemctl list-unit-files | grep -q '^dhcpcd\.service'; then
  systemctl restart dhcpcd || true
fi

# If NetworkManager was enabled originally, re-enable and restart it.
if [[ -f "${LATEST_BACKUP}/netmgr_enabled.flag" ]] && [[ "$(cat "${LATEST_BACKUP}/netmgr_enabled.flag")" == "yes" ]]; then
  systemctl enable NetworkManager 2>/dev/null || true
  systemctl restart NetworkManager 2>/dev/null || true
fi

# Bring wlan0 up; wpa_supplicant will handle joining if configured.
ip link set "${WLAN_IF}" up 2>/dev/null || true

echo "[7/7] Done. Reboot recommended for a clean state."