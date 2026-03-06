#!/bin/bash

sudo systemd-run \
  --unit=nm-ap-enable \
  --description="Enable NM AP" \
  --collect \
  --remain-after-exit \
  /bin/bash -lc 'cd /home/rndmzd/rocket-telemetry/receiver/pi/scripts && ./nm-ap-enable.sh "GroundStationAP" "GroundStationAP" bg 6 > /var/log/nm-ap-enable.log 2>&1'
