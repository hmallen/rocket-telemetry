#!/bin/bash

sudo systemd-run \
  --unit=nm-ap-disable \
  --description="Disable NM AP" \
  --collect \
  --remain-after-exit \
  /bin/bash -lc 'cd /home/rndmzd/rocket-telemetry/receiver/pi/scripts && ./nm-ap-disable.sh bg 6 > /var/log/nm-ap-disable.log 2>&1'
