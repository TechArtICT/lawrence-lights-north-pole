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

## Setup

### Rock64

Ubuntu 20.04. Plug the camera into a **USB 2** port: on this Rock64 the USB 3 port failed to start the camera when it was plugged in at boot. The ESP32's USB (VBUS disconnected) goes to another USB port.

```bash
sudo apt install python3.9 python3.9-venv git
python3.9 -m venv ~/rs                     # pyrealsense2 for 64-bit ARM needs Python 3.9, 3.10 or 3.12
~/rs/bin/pip install pyrealsense2 numpy scipy esptool
sudo usermod -aG dialout $USER             # serial port access; log out and back in
git clone https://github.com/TechArtICT/lawrence-lights-north-pole.git
cd lawrence-lights-north-pole
bash rock64/install-service.sh             # height.py starts at every boot
```

Service:

| Command | Does |
|---|---|
| `journalctl -u northpole-height -f` | Live output, including the ESP32's lines (`esp32\| …`). Ctrl+C stops watching. |
| `sudo systemctl restart northpole-height` | Restart, e.g. after `git pull`. |
| `sudo systemctl stop northpole-height` | Stop, e.g. to run `height.py` by hand. |

`height.py` by hand (service stopped), from the repo folder:

| Command | Does |
|---|---|
| `~/rs/bin/python rock64/height.py` | Normal run, linked to the ESP32. |
| `~/rs/bin/python rock64/height.py noserial` | Same, terminal only, default box (50 × 50). |
| `~/rs/bin/python rock64/height.py nobox` | Terminal only, whole view counts. For finding the standing spot. |
| `~/rs/bin/python rock64/height.py calibrate` | Record the empty scene and exit. |

Every run records the empty scene 2 s after it starts, so keep the area clear for ~5 s after power-on or a restart.

### ESP32 firmware

On the Rock64 (compiles and flashes over the USB link; stops and restarts the service):

```bash
bash rock64/setup-arduino.sh      # once: arduino-cli, ESP32 core 3.3.7, NeoPixelBus (~1 GB download)
~/bin/arduino-cli cache clean     # once afterwards: frees the ~1.5 GB of downloaded archives
bash rock64/flash.sh              # compile + flash
bash rock64/flash.sh compile      # compile only
```

Or from a laptop: Arduino IDE with **esp32 by Espressif Systems** and the library **NeoPixelBus by Makuna**; open `north_pole/north_pole.ino`, board **ESP32 Dev Module**, upload.

Settings saved with `set` survive flashing. To pick up new code defaults after flashing, run `defaults` (resets every setting, keeps the visitor count).

## Using it

Connect over Bluetooth Classic serial (pair with `NorthPoleLidar`, e.g. the Android app "Serial Bluetooth Terminal") or USB serial (115200, line ending "Newline"). Type `help`.

Over Bluetooth, commands that change anything need `unlock ll` first. USB is always unlocked.

The Bluetooth password is committed to this public repo on purpose, for simplicity. It only keeps passersby from casually changing settings; it isn't meant as real security.

### Field setup

1. Power on and keep the area clear for ~10 s while the Rock64 records the empty scene (`camera: calibrated` over Bluetooth).
2. `watch on` to see live heights (`camera=` is the Rock64's number, `h=` the displayed one).
3. LED marks: defaults are `callowled` 120 (1 ft) and `calhighled` 28 (6 ft). Check with `led <n>` / `led off`, change with `set callowled <n>`, `set calhighled <n>`. Visible range: `set firstled <n>`, `set lastled <n>`.
4. Standing box (default 50 × 50 cm under the camera): stand on the spot and read "head at x / y" in `journalctl -u northpole-height -f`, then `set boxx <x>`, `set boxy <y>`. Size with `boxw` (camera's wide direction) and `boxd` (narrow direction).
5. Check against a known height. Off by a fixed amount: `set caloffset <cm>`; off by a percentage: `set calfactor <x>`.
6. Small children not detected: lower `minpixels` (default 150).
7. `calibrate camera` (starts after 5 s; step out of view) after moving the camera or when something permanent in view changes, e.g. snow.
8. Tune `brightness`, `idlelevel`, `mintrigger` as needed. Everything set this way is saved to flash.

All settings and commands: [`docs/software-spec.md`](docs/software-spec.md).

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
