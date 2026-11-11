# North Pole Height Display — Software Spec

ESP32-WROOM-32 (board: `ESP32 Dev Module`), TFmini Plus, 146-pixel 5V strip, Bluetooth Classic serial.

## Hardware assignments

| Item | Value |
|---|---|
| TFmini green (sensor TX) | `IO16` (Serial2 RX) |
| TFmini white (sensor RX) | `IO17` (Serial2 TX) |
| TFmini baud | 115200 (sensor default) |
| Pixel data | `IO18` → level shifter → 300–500 Ω → DIN |
| Pixel count | 146 |
| Power | 12A 5V supply, all grounds common; program with VBUS disconnected |

## Field configuration

Defaults live at the top of the code (`DEF_*`). Every value can also be changed over USB or Bluetooth with `set <name> <value>` and is saved to flash, so field adjustments survive the nightly power-off without reflashing.

| Setting | Default | Notes |
|---|---|---|
| `sensor_height_cm` | 230 | Lens to ground. Fallback if boot calibration fails; `calibrate ground` sets it. |
| `ground_clamp_cm` | 30 | Ongoing ground learning may drift at most this far from the calibrated ground. |
| `min_trigger_cm` | 70 | Shortest height that starts the show. Set with toddlers in mind. |
| `min_flux` | 100 | Below this (or dist = 0, or flux 65535) a reading is "unknown", not "gone". |
| `presence_stable_ms` | 500 | Height must hold this long to trigger. |
| `absent_margin_cm` | 15 | Reading within this of ground = nobody there. |
| `hold_after_leave_ms` | 1000 | Show continues this long after the person steps away. |
| `remeasure_delta_cm` | 10 | Height change that counts as a different person. |
| `remeasure_stable_ms` | 500 | New height must hold this long before restarting. |
| `first_view_led` | 0 | Raw index of lowest-numbered visible LED. |
| `last_view_led` | 145 | Raw index of highest-numbered visible LED. |
| `cal_low_led` / `cal_low_cm` | 20 / 30.48 | Raw LED index lined up with the 1 ft mark. Placeholder index; set in field. |
| `cal_high_led` / `cal_high_cm` | 130 / 182.88 | Raw LED index lined up with the 6 ft mark. Placeholder index; set in field. |
| `brightness_max` | 128 (of 255) | Global cap. |
| `dazzle_ms` | 3000 | Also the measurement window. |
| `rise_speed` | 60 LEDs/s | Average speed; ease-out (fast start, slows near the top). |
| `drain_speed` | 40 LEDs/s | Average speed of the drain outro. |
| `idle_fade_in_ms` | 1000 | Idle fades in after the outro. |
| `idle_level_pct` | 15 | Idle brightness as % of `brightness_max`. Everything else runs at 100%, so activation "comes to life". |
| `idle_speed_pct` | 100 | Idle shimmer speed (10–500). |

Compile-time only: `SIM_AT_BOOT` (false), `BT_NAME` (`NorthPoleLidar`), `BT_PASSWORD` (`ll`).

### Height → LED mapping

- Linear interpolation between the two calibration points; covers LED spacing, strip start offset and stretch in one step.
- Strip direction is derived from the calibration points: if `cal_high_led` < `cal_low_led`, the strip is reversed. No separate reverse flag.
- All animations work in "visible position" (0 = lowest visible LED), mapped to raw indices through `first_view_led` / `last_view_led` and the direction.
- Height at or above the height of the top visible LED → "off the charts".

## States

- **CALIBRATE** — at boot (nobody is under the sensor at power-on): strip dark ~1 s, median of readings = ground. Not saved to flash. Fewer than 20 valid readings in 3 s → fall back to `sensor_height_cm`. Also entered by `calibrate ground` (which saves) and `sim off`.
- **IDLE** — attract animation at `idle_level_pct`. Ground keeps learning from readings with nobody present (median of ~1 s batches, clamped to ±`ground_clamp_cm` of the calibrated ground). Trigger when height ≥ `min_trigger_cm` holds for `presence_stable_ms` → DAZZLE.
- **DAZZLE** — random dazzle animation for `dazzle_ms`. Collect every valid sample; height = **median**. Person leaves → dissolve outro → IDLE.
- **RISE** — column grows from the bottom to the measured height with ease-out.
- **SHOW** — random show mode with a gold star at the height (off-the-charts variant if above the top LED); loops until the person leaves.
  - Height shifts by `remeasure_delta_cm` for `remeasure_stable_ms` → back to DAZZLE (different person).
  - Gone for `hold_after_leave_ms` → OUTRO.
- **OUTRO** — random outro animation → IDLE (idle fades in, never a cut). A new person arriving during the outro starts a new dazzle.
- **FAULT** — no frames from the sensor for ~1 s: idle animation only, report the fault, return to IDLE when frames resume.

Height = ground distance − measured distance. Gone = no person reading for `hold_after_leave_ms` and a ground-level reading seen since (or 5 s with no person reading at all). Unknown readings (low flux, dist 0) never count as gone.

## Animations

Palette: Christmas red, green, white, gold. Gamma-corrected (2.2). All non-blocking (no `delay()`), so the sensor is read every frame.

### Idle / attract

- **Brook** — three ripple waves of different sizes and speeds (two flowing up, one down) interfere into an irregular shimmer, red/green drifting up, with brief soft white glints.

### Dazzle (random pick)

- **candycane** — red/white bands scrolling up.
- **comets** — bright heads with fading tails shooting up, overlapping.
- **twinkle** — random pixels flash white/gold and fade.
- **scanner** — single bright bar sweeping up and down, speeding up.
- **chase** — red/green marquee.
- **filldrain** — green fills and drains repeatedly.

### Show (random pick)

- **crawlers** — red fill, green pulses travel up to the star and repeat.
- **candyfill** — red/white stripes scrolling slowly upward inside the filled column.
- **breathing** — green fill pulsing, red waves moving up through it.
- In all modes: gold star (white flash on arrival, then gold with sparkles) at the measured height; very dim white twinkles above it.

### Off the charts

- Whole column red/green chase, sparkles bursting at the top.

### Outro (random pick)

- **drain** — column drains down, eased; the star falls last as a falling star with a short tail.
- **dissolve** — lit pixels fade out one by one, roughly top down.

## USB serial and Bluetooth

Both channels are identical: all status output goes to both, and every command is accepted from either. USB serial (115200, line ending "Newline") needs no `unlock` — physical access to the board is treated as trusted. Bluetooth requires `unlock ll` for anything that changes state; the unlock lasts until the client disconnects.

### Status output (always on)

- State changes.
- Each measurement: height in cm and ft/in, sample count, spread, which dazzle/show was picked.
- Ground calibration and learning, faults, boot info, visitor counts.

### Commands

| Command | Unlock needed | Action |
|---|---|---|
| `help` | no | List commands. |
| `status` | no | Current state, last reading, ground distance, mapping, counts. |
| `get` | no | List all settings. |
| `count` | no | Season total and tonight's count. |
| `watch on` / `watch off` | no | Stream live readings 5×/s (distance, flux, height, class, ground). |
| `play list` | no | List animation names by type. |
| `unlock <pw>` | — | Unlock (Bluetooth only). |
| `set <name> <value>` | yes | Apply immediately and save to flash. |
| `defaults` | yes | Restore code defaults (keeps the visitor count). |
| `calibrate ground` | yes | Nobody under the sensor: ~1 s of readings, median saved as `sensor_height_cm`. |
| `led <n>` / `led off` | yes | Light one raw LED (for calibration). Suspends the normal show until `led off`. |
| `count reset` | yes | Zero the counts. |
| `reboot` | yes | Restart. |
| `sim on` / `sim off` | yes | Ignore the TFmini; use simulated readings. Never saved to flash. `sim off` recalibrates against the real sensor. |
| `sim height <cm>` | yes | Simulated person of that height steps in (turns sim on). Sent again with a different height during SHOW = sibling swap. |
| `sim leave` | yes | Simulated person steps away. |
| `sim noise <cm>` | yes | Random jitter on simulated readings. `0` = off. |
| `sim dropout on/off` | yes | Random "unknown" readings mixed in. |
| `sim fault` | yes | Simulated sensor loss → FAULT. `sim height` or `sim leave` ends it. |
| `demo on` / `demo off` | yes | Loop the full cycle automatically with random heights and occasional swaps. |
| `play <type> [name]` / `play stop` | yes | Loop one animation outside the state machine. Types: `idle`, `dazzle`, `rise`, `show`, `outro`, `offcharts`. Uses the last simulated height (or 120 cm). |
| `next <dazzle\|show\|outro> <name>` | yes | Force the next random pick for that type (one time). |

## Bench testing

- Simulation replaces only the sensor input; everything downstream (presence, median, remeasure, hold, state machine, animations) runs unchanged.
- Simulated measurements do **not** increment the visitor count.
- Simulated output is tagged `[SIM]`.

## Visitor count

- Counted when a real measurement completes (not on abort, not in sim).
- Season total saved to flash on every measurement; tonight's count in RAM only.
- Flash wear: ESP32 NVS is wear-leveled and uses 32-byte entries, 126 per 4KB page, so one sector erase per ~126 writes (Espressif NVS docs).

## Reliability

- Task watchdog (8 s) reboots on a hung loop. The core already starts the watchdog, so a harmless `TWDT already initialized` line prints at boot before the timeout is reconfigured.
- Cold boot every evening (external timer); no Bluetooth interaction needed to run.
