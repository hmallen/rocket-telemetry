#!/usr/bin/env bash

# Stop immediately if any command fails
set -e

cd ./receiver/esp32-companion

pio run -e esp32dev-ota -t clean
pio run -e esp32dev-ota -t upload

echo "Done."