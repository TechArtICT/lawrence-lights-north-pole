# Lawrence Lights North Pole

"How tall this winter?" — a candy-cane pole for Lawrence Lights 2026 that measures your height when you stand next to it, then shows it on a 146-LED strip running up the printed measuring scale.

An Intel RealSense depth camera looks down from the arm above the pole; a Rock64 works out the height and sends it to the ESP32 that runs the LEDs. When someone stands underneath, a random "dazzle" animation plays while the height is measured. Then a column rises like a thermometer to their height, topped with a gold star. The pole shimmers dimly when nobody is there.

## Hardware

| Part | Notes |
|---|---|
| ESP32-WROOM-32 dev board | Arduino board: **ESP32 Dev Module** |
| Intel RealSense D435i + Rock64 | Depth camera on the Rock64's USB 2 port; `rock64/height.py` |
| 146 × WS2812-type 5V pixels | Data through a 3.3 V → 5 V level shifter |
| 5V 12A supply | All grounds common |

### Wiring

| From | To |
|---|---|
| ESP32 USB (VBUS disconnected) | Rock64 USB |
| IO18 | level shifter → 300–500 Ω → strip DIN |
| ESP32 5V / GND | supply + / − |

Add a 500–1000 µF capacitor across the strip's power input. Program with USB VBUS disconnected when the board is also fed from the supply.

## Building

1. Arduino IDE with the **esp32 by Espressif Systems** board package.
2. Library Manager: **NeoPixelBus by Makuna**. (BluetoothSerial and Preferences come with the ESP32 core.)
3. Open `north_pole/north_pole.ino`, board **ESP32 Dev Module**, upload.

Or on the Rock64: `bash rock64/setup-arduino.sh` once, then `bash rock64/flash.sh` to compile and flash over the USB link.

Rock64 (Ubuntu 20.04, Python 3.9 venv at `~/rs`): `~/rs/bin/pip install pyrealsense2 numpy scipy esptool`, then `~/rs/bin/python rock64/height.py calibrate` with nobody in view, and `~/rs/bin/python rock64/height.py` to run.

## Using it

Connect over USB serial (115200, line ending "Newline") or Bluetooth Classic serial (pair with `NorthPoleLidar`, e.g. the Android app "Serial Bluetooth Terminal"). Type `help`.

Over Bluetooth, commands that change anything need `unlock ll` first. USB is always unlocked.

The Bluetooth password is committed to this public repo on purpose, for simplicity. It only keeps passersby from casually changing settings; it isn't meant as real security.

### Field setup

1. Power on. The Rock64 starts `height.py`, which links to the ESP32.
2. `watch on` to see live readings.
3. Find the LEDs that line up with the 1 ft and 6 ft marks with `led <n>`, then:
   `set callowled <n>`, `set calhighled <n>`.
4. Set the visible range: `set firstled <n>`, `set lastled <n>`.
5. `calibrate camera` with nobody in view (starts after 5 s), and set the standing box with `boxw`, `boxd`, `boxx`, `boxy`.
6. Tune `brightness`, `idlelevel`, `mintrigger` as needed. Everything set this way is saved to flash.

### Bench testing (no real measurements needed)

- `demo on` — runs the full cycle with random heights.
- `sim height 120` / `sim leave` — simulate one person.
- `play list`, `play dazzle comets`, `play stop` — loop a single animation.
- `next show breathing` — force the next random pick.

Simulated runs don't count as visitors.

## Docs

- [`docs/software-spec.md`](docs/software-spec.md) — full spec: settings, states, animations, commands.

## License

MIT — see [LICENSE](LICENSE).
