#!/usr/bin/env bash
# Compile the ESP32 firmware on the Rock64 and flash it over the USB link.
#
#   bash rock64/flash.sh            compile + flash
#   bash rock64/flash.sh compile    compile only
#
# Flashing needs the serial port, so height.py is stopped first; start it again afterwards.
set -euo pipefail

export PATH="$HOME/bin:$PATH"
HERE="$(cd "$(dirname "$0")" && pwd)"
SKETCH="$HERE/../north_pole"
OUT="$HERE/build"
FQBN="esp32:esp32:esp32"                       # "ESP32 Dev Module"
CORE_DIR="$HOME/.arduino15/packages/esp32/hardware/esp32/3.3.7"
ESPTOOL="$HOME/rs/bin/esptool.py"

echo "== compiling $(cd "$SKETCH" && pwd)"
arduino-cli compile --fqbn "$FQBN" --output-dir "$OUT" "$SKETCH"
[ "${1:-}" = "compile" ] && { echo "== compiled to $OUT"; exit 0; }

PORT="$(ls /dev/serial/by-id/*CP210* 2>/dev/null | head -n1 || true)"
[ -n "$PORT" ] || PORT=/dev/ttyUSB0

if pgrep -f "height.py" >/dev/null; then
  echo "== stopping height.py (it holds the serial port)"
  pkill -f "height.py" || true
  sleep 1
fi

echo "== flashing $PORT"
"$ESPTOOL" --chip esp32 --port "$PORT" --baud 921600 \
  --before default_reset --after hard_reset write_flash -z \
  --flash_mode keep --flash_freq keep --flash_size keep \
  0x1000  "$OUT/north_pole.ino.bootloader.bin" \
  0x8000  "$OUT/north_pole.ino.partitions.bin" \
  0xe000  "$CORE_DIR/tools/partitions/boot_app0.bin" \
  0x10000 "$OUT/north_pole.ino.bin"

echo "== done. Start height.py again:  ~/rs/bin/python $HERE/height.py"
