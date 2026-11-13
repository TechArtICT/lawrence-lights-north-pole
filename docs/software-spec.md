# North Pole Height Display — Software Spec

ESP32-WROOM-32 (board: `ESP32 Dev Module`), Intel RealSense D435i on a Rock64 (`rock64/height.py`), 146-pixel 5V strip, Bluetooth Classic serial.

## Hardware assignments

| Item | Value |
|---|---|
| Camera | RealSense D435i on the Rock64's USB 2 port (USB 3 failed at boot on this Rock64) |
| Rock64 ↔ ESP32 | USB serial, 115200, VBUS disconnected |
| Pixel data | `IO18` → level shifter → 300–500 Ω → DIN |
| Pixel count | 146 |
| Power | 12A 5V supply, all grounds common; program with VBUS disconnected |

## Field configuration

Defaults live at the top of the code (`DEF_*`). Every value can also be changed over USB or Bluetooth with `set <name> <value>` and is saved to flash, so field adjustments survive the nightly power-off without reflashing.

| Setting | Default | Notes |
|---|---|---|
| `calfactor` | 1.0 | Camera height × this … |
| `caloffset` | 0 | … + this (cm) = displayed height. |
| `boxw`, `boxd` | 50, 50 | Standing box size in cm on the floor (image x, y). A person counts when the top of their head is inside it. Sent to the Rock64. |
| `boxx`, `boxy` | 0, 0 | Box center in cm from the point directly under the camera. |
| `headpatch` | 3 | Rock64: height = median of the person's pixels within this radius (cm) of the top of the head. |
| `minpixels` | 150 | Rock64: blobs smaller than this are noise, not people. Lower it if small children aren't detected. |
| `topk` | 40 | Rock64: this many highest pixels of a person locate the top of the head. |
| `mintrigger` | 70 | Shortest height that starts the show. Set with toddlers in mind. |
| `presencetime` | 500 | Height must hold this long to trigger. |
| `absentmargin` | 15 | Reading within this of ground = nobody there. |
| `holdtime` | 1000 | Show continues this long after the person steps away. |
| `remeasuredelta` | 10 | Height change that counts as a different person. |
| `remeasuretime` | 500 | New height must hold this long before restarting. |
| `firstled` | 0 | Raw index of lowest-numbered visible LED. |
| `lastled` | 145 | Raw index of highest-numbered visible LED. |
| `callowled` | 120 | Raw LED index lined up with the low calibration mark. |
| `callowcm` | 30.48 | Height of that mark in cm (30.48 cm = 1 ft). |
| `calhighled` | 28 | Raw LED index lined up with the high calibration mark. |
| `calhighcm` | 182.88 | Height of that mark in cm (182.88 cm = 6 ft). |
| `brightness` | 128 (of 255) | Global cap. |
| `dazzletime` | 3000 | The height must hold steady this long to be measured; the dazzle runs until it does. |
| `dazzlemax` | 10000 | The dazzle never runs longer than this; if the height never settled, the last second is used. |
| `risespeed` | 60 LEDs/s | Average speed; ease-out (fast start, slows near the top). |
| `drainspeed` | 40 LEDs/s | Average speed of the drain outro. |
| `idlefade` | 1000 | Idle fades in after the outro. |
| `idlelevel` | 15 | Idle brightness as % of `brightness`. Everything else runs at 100%, so activation "comes to life". |
| `idlespeed` | 100 | Idle shimmer speed (10–500). |

Compile-time only: `SIM_AT_BOOT` (false), `BT_NAME` (`NorthPoleLidar`), `BT_PASSWORD` (`ll`).

### Height → LED mapping

- Linear interpolation between the two calibration points; covers LED spacing, strip start offset and stretch in one step.
- Strip direction is derived from the calibration points: if `calhighled` < `callowled`, the strip is reversed. No separate reverse flag.
- All animations work in "visible position" (0 = lowest visible LED), mapped to raw indices through `firstled` / `lastled` and the direction.
- Height at or above the height of the top visible LED → "off the charts".

## States

- **IDLE** — attract animation at `idlelevel`. Trigger when height ≥ `mintrigger` holds for `presencetime` → DAZZLE.
- **DAZZLE** — random dazzle animation while the height is measured. Every 0.1 s the newest 0.5 s of samples is compared with the older ones; if they differ by more than `remeasuredelta` (stepped in, stood up, sibling swap) the older samples are dropped. Once `dazzletime` of samples agree (10–90% spread within `remeasuredelta`), height = their **median**. Never longer than `dazzlemax`: then the median of the last second. Person leaves → dissolve outro → IDLE.
- **RISE** — column grows from the bottom to the measured height with ease-out.
- **SHOW** — random show mode with a gold star at the height (off-the-charts variant if above the top LED); loops until the person leaves.
  - Height shifts by `remeasuredelta` for `remeasuretime` → back to DAZZLE (different person).
  - Gone for `holdtime` → OUTRO.
- **OUTRO** — random outro animation → IDLE (idle fades in, never a cut). A new person arriving during the outro starts a new dazzle.
- **FAULT** — no frames from the sensor for ~1 s: idle animation only, report the fault, return to IDLE when frames resume.

Height comes from the Rock64 (height above the recorded empty scene) × `calfactor` + `caloffset`. Gone = no person reading for `holdtime` and a nobody reading seen since (or 5 s with no person reading at all). Unknown readings (head cut off at the edge of the view, someone reaching into the box, calibrating) never count as gone.

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

Both channels are identical: all status output goes to both, and every command is accepted from either. Commands and setting names are all lowercase with no underscores, for easy phone typing (input is lowercased anyway); `get` shows each setting's unit. USB serial (115200, line ending "Newline") needs no `unlock` — physical access to the board is treated as trusted. Bluetooth requires `unlock ll` for anything that changes state; the unlock lasts until the client disconnects.

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
| `watch on` / `watch off` | no | Stream live readings 5×/s (camera height, displayed height, class). |
| `play list` | no | List animation names by type. |
| `unlock <pw>` | — | Unlock (Bluetooth only). |
| `set <name> <value>` | yes | Apply immediately and save to flash. |
| `defaults` | yes | Restore code defaults (keeps the visitor count). |
| `calibrate camera` | yes | The Rock64 waits 5 s, then records the empty scene for 3 s (saved on the Rock64). |
| `led <n>` / `led off` | yes | Light one raw LED (for calibration). Suspends the normal show until `led off`. |
| `count reset` | yes | Zero the counts. |
| `reboot` | yes | Restart. |
| `sim on` / `sim off` | yes | Ignore the camera; use simulated readings. Never saved to flash. |
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
