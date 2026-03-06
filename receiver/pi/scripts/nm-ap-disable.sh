#!/usr/bin/env bash
# nm-ap-disable.sh
# Stops + removes the hotspot connection and restores the previously active Wi-Fi connection.

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run as root: sudo $0"
  exit 1
fi

WIFI_DEV="wlan0"
AP_CON="pi-ap-hotspot"
STATE_DIR="/var/lib/piap_state_nm"

command -v nmcli >/dev/null 2>&1 || { echo "nmcli not found."; exit 1; }
systemctl is-active --quiet NetworkManager || systemctl start NetworkManager

# Bring down hotspot if active
if nmcli -t -f NAME con show --active | grep -qx "${AP_CON}"; then
  nmcli con down "${AP_CON}" || true
fi

# Delete hotspot profile to fully revert to pre-hotspot config
if nmcli -t -f NAME con show | grep -qx "${AP_CON}"; then
  nmcli con delete "${AP_CON}" || true
fi

# Restore previous Wi-Fi connection if recorded and still exists
PREV_CON=""
if [[ -f "${STATE_DIR}/prev_wifi_con.txt" ]]; then
  PREV_CON="$(cat "${STATE_DIR}/prev_wifi_con.txt" || true)"
fi

if [[ -n "${PREV_CON}" ]] && nmcli -t -f NAME con show | grep -qx "${PREV_CON}"; then
  nmcli con up "${PREV_CON}" || true
else
  # Otherwise just re-enable Wi-Fi radio and let NM autoconnect to its normal preferred network
  nmcli radio wifi on || true
  nmcli dev disconnect "${WIFI_DEV}" || true
  nmcli dev connect "${WIFI_DEV}" || true
fi
