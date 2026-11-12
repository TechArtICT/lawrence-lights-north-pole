#!/usr/bin/env bash
# One-time setup on the Rock64: arduino-cli, the ESP32 core and NeoPixelBus,
# so the ESP32 firmware can be compiled here (see flash.sh).
#
#   bash rock64/setup-arduino.sh
#
# Downloads roughly 1 GB the first time; check free space with  df -h ~  first.
set -euo pipefail

CORE="esp32:esp32@3.3.7"      # same core version as the Arduino IDE build
ESP32_URL="https://espressif.github.io/arduino-esp32/package_esp32_index.json"
BIN="$HOME/bin"

mkdir -p "$BIN"
export PATH="$BIN:$PATH"

if ! command -v arduino-cli >/dev/null; then
  echo "== installing arduino-cli into $BIN"
  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR="$BIN" sh
fi
arduino-cli version

echo "== ESP32 core $CORE"
arduino-cli core update-index --additional-urls "$ESP32_URL"
arduino-cli core install "$CORE" --additional-urls "$ESP32_URL"

echo "== NeoPixelBus library"
arduino-cli lib update-index
arduino-cli lib install "NeoPixelBus by Makuna"

echo "== done. Next: bash rock64/flash.sh compile"
