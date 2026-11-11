# Lawrence Lights North Pole

"How tall this winter?" — a candy-cane pole for Lawrence Lights 2026 that measures your height when you stand next to it, then shows it on a 146-LED strip running up the printed measuring scale.

A TFmini Plus lidar looks down from the top of the pole. When someone stands underneath, a random "dazzle" animation plays while the height is measured. Then a column rises like a thermometer to their height, topped with a gold star. The pole shimmers dimly when nobody is there.

## Hardware

| Part | Notes |
|---|---|
| ESP32-WROOM-32 dev board | Arduino board: **ESP32 Dev Module** |
| Benewake TFmini Plus | IP65 ToF lidar, UART 115200 |
| 146 × WS2812-type 5V pixels | Data through a 3.3 V → 5 V level shifter |
| 5V 12A supply | All grounds common |

### Wiring

| From | To |
|---|---|
| TFmini green (sensor TX) | IO16 |
| TFmini white (sensor RX) | IO17 |
| TFmini red / black | 5V / GND |
| IO18 | level shifter → 300–500 Ω → strip DIN |
| ESP32 5V / GND | supply + / − |

Add a 500–1000 µF capacitor across the strip's power input. Program with USB VBUS disconnected when the board is also fed from the supply.

## Building

1. Arduino IDE with the **esp32 by Espressif Systems** board package.
2. Library Manager: **NeoPixelBus by Makuna**. (BluetoothSerial and Preferences come with the ESP32 core; TFmini frames are parsed directly, no TFmini library needed.)
3. Open `north_pole/north_pole.ino`, board **ESP32 Dev Module**, upload.

## Using it

Connect over USB serial (115200, line ending "Newline") or Bluetooth Classic serial (pair with `NorthPoleLidar`, e.g. the Android app "Serial Bluetooth Terminal"). Type `help`.

Over Bluetooth, commands that change anything need `unlock ll` first. USB is always unlocked.

### Field setup

1. Power on with nobody under the sensor — it calibrates the ground automatically (~1 s).
2. `watch on` to see live readings.
3. Find the LEDs that line up with the 1 ft and 6 ft marks with `led <n>`, then:
   `set cal_low_led <n>`, `set cal_high_led <n>`.
4. Set the visible range: `set first_view_led <n>`, `set last_view_led <n>`.
5. Optional: `calibrate ground` to save the lens height as the fallback.
6. Tune `brightness_max`, `idle_level_pct`, `min_trigger_cm` as needed. Everything set this way is saved to flash.

### Bench testing (no real measurements needed)

- `demo on` — runs the full cycle with random heights.
- `sim height 120` / `sim leave` — simulate one person.
- `play list`, `play dazzle comets`, `play stop` — loop a single animation.
- `next show breathing` — force the next random pick.

Simulated runs don't count as visitors.

## Docs

- [`docs/software-spec.txt`](docs/software-spec.txt) — full spec (DokuWiki format): settings, states, animations, commands.

## License

MIT — see [LICENSE](LICENSE).
