#!/usr/bin/env bash
# nm-ap-enable.sh
# Creates + enables a NetworkManager hotspot on wlan0.
# Stores previous active Wi-Fi connection name for clean revert.

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run as root: sudo $0 [SSID] [PASS] [BAND] [CHANNEL]"
  exit 1
fi

SSID="${1:-PiZeroAP}"
PASS="${2:-changeme12345}"
BAND="${3:-bg}"      # bg (2.4GHz) or a (5GHz if supported)
CHANNEL="${4:-6}"

if [[ "${#PASS}" -lt 8 ]]; then
  echo "Passphrase must be at least 8 characters."
  exit 1
fi

WIFI_DEV="wlan0"
AP_CON="pi-ap-hotspot"
STATE_DIR="/var/lib/piap_state_nm"
mkdir -p "${STATE_DIR}"

# Sanity: ensure NetworkManager is present/active
command -v nmcli >/dev/null 2>&1 || { echo "nmcli not found. Install NetworkManager."; exit 1; }
systemctl is-active --quiet NetworkManager || systemctl start NetworkManager

# Record the currently active Wi-Fi connection (if any)
ACTIVE_WIFI_CON="$(nmcli -t -f NAME,TYPE,DEVICE con show --active | awk -F: '$2=="wifi" && $3=="'"${WIFI_DEV}"'" {print $1; exit}' || true)"
echo -n "${ACTIVE_WIFI_CON}" > "${STATE_DIR}/prev_wifi_con.txt"

# If the AP connection already exists, just bring it up
if nmcli -t -f NAME con show | grep -qx "${AP_CON}"; then
  nmcli con up "${AP_CON}"
  exit 0
fi

# Create hotspot connection
# NetworkManager will provide DHCP/NAT internally (no hostapd/dnsmasq/iptables needed).
nmcli dev wifi hotspot ifname "${WIFI_DEV}" con-name "${AP_CON}" ssid "${SSID}" password "${PASS}"

# Optional band/channel settings (best-effort; some chips/drivers may ignore)
nmcli con modify "${AP_CON}" 802-11-wireless.band "${BAND}" || true
nmcli con modify "${AP_CON}" 802-11-wireless.channel "${CHANNEL}" || true

# Make hotspot deterministic: autoconnect off so it doesn't surprise you on boot
nmcli con modify "${AP_CON}" connection.autoconnect no

# Bring it up (usually already up after creation, but be explicit)
nmcli con up "${AP_CON}"
