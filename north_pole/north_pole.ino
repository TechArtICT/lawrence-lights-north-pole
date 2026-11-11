/*
  North Pole height display — Lawrence Lights 2026

  Board:     ESP32-WROOM-32  (Arduino IDE board: "ESP32 Dev Module")
  Libraries: NeoPixelBus by Makuna (Library Manager).
             BluetoothSerial and Preferences ship with the ESP32 core.
  Sensor:    TFmini Plus frames are parsed directly (no TFMPlus library).

  Wiring:    TFmini green (TX) -> IO16, white (RX) -> IO17, red -> 5V, black -> GND
             IO18 -> level shifter -> 300-500 ohm -> strip DIN
             All grounds common.

  Control:   USB serial (115200, line ending "Newline") or Bluetooth "NorthPoleLidar".
             Type "help". Bluetooth needs "unlock ll" before config/sim/play commands.
*/

#include <Arduino.h>
#include <NeoPixelBus.h>
#include <BluetoothSerial.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <esp_arduino_version.h>
#include <algorithm>
#include <math.h>

// =====================================================================
//  FIELD DEFAULTS
//  Change here, or live with "set <name> <value>" (saved to flash and
//  survives power-off; "defaults" restores these values).
// =====================================================================
#define DEF_SENSOR_HEIGHT_CM      230.0f  // lens to ground. Fallback/clamp for auto-learned ground
#define DEF_GROUND_CLAMP_CM        30.0f  // learned ground may differ from sensor height by at most this
#define DEF_MIN_TRIGGER_CM         70.0f  // shortest height that starts the show
#define DEF_MIN_FLUX              100     // below this (or dist 0) a reading is "unknown"
#define DEF_PRESENCE_STABLE_MS    500     // person must be there this long to trigger
#define DEF_ABSENT_MARGIN_CM       15.0f  // height <= this = nobody there
#define DEF_HOLD_AFTER_LEAVE_MS  1000     // keep showing this long after they step away
#define DEF_REMEASURE_DELTA_CM     10.0f  // height change that means a different person
#define DEF_REMEASURE_STABLE_MS   500     // ...held this long before re-measuring
#define DEF_FIRST_VIEW_LED          0     // raw index of the lowest-numbered visible LED
#define DEF_LAST_VIEW_LED         145     // raw index of the highest-numbered visible LED
#define DEF_CAL_LOW_LED            20     // raw LED index at the 1 ft mark (placeholder)
#define DEF_CAL_LOW_CM             30.48f
#define DEF_CAL_HIGH_LED          130     // raw LED index at the 6 ft mark (placeholder)
#define DEF_CAL_HIGH_CM           182.88f
#define DEF_BRIGHTNESS_MAX        128     // 1-255 global cap
#define DEF_DAZZLE_MS            3000     // dazzle length = measurement window
#define DEF_RISE_SPEED             60.0f  // average LEDs/second for the thermometer rise
#define DEF_DRAIN_SPEED            40.0f  // average LEDs/second for the drain outro
#define DEF_IDLE_FADE_IN_MS      1000
#define DEF_IDLE_LEVEL_PCT         15     // idle brightness as % of brightness_max; show runs at 100%
#define DEF_IDLE_SPEED_PCT        100     // idle shimmer speed, % of designed speed

// Compile-time only
#define SIM_AT_BOOT        false          // true = boot into simulation (bench work)
#define BT_NAME            "NorthPoleLidar"
#define BT_PASSWORD        "ll"

// Hardware
#define NUM_PIXELS   146
#define PIXEL_PIN     18
#define LIDAR_RX      16
#define LIDAR_TX      17

// Internal tuning
#define FAULT_MS          1000   // no sensor frames this long -> FAULT
#define STALE_GONE_MS     5000   // no person reading this long -> gone, even without a ground reading
#define MIN_SAMPLES         20   // fewer valid samples in the dazzle -> measurement fails
#define MAX_DZ            2000   // sample buffer (20 s at 100 Hz)
#define G_N                101   // ground-learning batch size (~1 s)
#define WDT_TIMEOUT_S        8
#define FRAME_MS            16   // ~60 fps

// =====================================================================
//  Types
// =====================================================================
struct Col { float r, g, b; };
static const Col C_RED   = {1.0f, 0.0f, 0.0f};
static const Col C_GREEN = {0.0f, 1.0f, 0.0f};
static const Col C_WHITE = {1.0f, 1.0f, 1.0f};
static const Col C_GOLD  = {1.0f, 0.55f, 0.0f};

enum State { ST_IDLE, ST_DAZZLE, ST_RISE, ST_SHOW, ST_OUTRO, ST_FAULT, ST_PLAY, ST_LEDTEST, ST_CALIB };
static const char* STATE_NAMES[] = {"idle","dazzle","rise","show","outro","fault","play","ledtest","calibrate"};

enum ReadClass { RC_UNKNOWN, RC_ABSENT, RC_LOW, RC_PERSON };
static const char* RC_NAMES[] = {"unknown","absent","low","person"};

enum Src { SRC_USB, SRC_BT };
enum PType { PT_F, PT_I };
struct Param { const char* name; const char* key; PType type; void* ptr; float minV; float maxV; };

struct Comet { float pos; float speed; Col c; bool on; };

enum PlayType { PL_IDLE, PL_DAZZLE, PL_RISE, PL_SHOW, PL_OUTRO, PL_OFFCHARTS, PL_NONE };
static const char* PLAY_TYPES[] = {"idle","dazzle","rise","show","outro","offcharts"};

static const char* IDLE_NAMES[]   = {"wave"};
static const char* DAZZLE_NAMES[] = {"candycane","comets","twinkle","scanner","chase","filldrain"};
static const char* SHOW_NAMES[]   = {"crawlers","candyfill","breathing"};
static const char* OUTRO_NAMES[]  = {"drain","dissolve"};
static const int N_DAZZLE = 6, N_SHOW = 3, N_OUTRO = 2;

struct Settings {
  float   sensorHeightCm, groundClampCm, minTriggerCm, absentMarginCm, remeasureDeltaCm;
  float   calLowCm, calHighCm, riseSpeed, drainSpeed;
  int32_t minFlux, presenceStableMs, holdAfterLeaveMs, remeasureStableMs;
  int32_t firstViewLed, lastViewLed, calLowLed, calHighLed, brightnessMax, dazzleMs, idleFadeInMs;
  int32_t idleLevelPct, idleSpeedPct;
};

// =====================================================================
//  Globals
// =====================================================================
Settings S;

Param params[] = {
  {"sensor_height_cm",    "sens_h",     PT_F, &S.sensorHeightCm,    50, 1000},
  {"ground_clamp_cm",     "gclamp",     PT_F, &S.groundClampCm,      0, 200},
  {"min_trigger_cm",      "min_trig",   PT_F, &S.minTriggerCm,      20, 250},
  {"min_flux",            "min_flux",   PT_I, &S.minFlux,            0, 65535},
  {"presence_stable_ms",  "pres_ms",    PT_I, &S.presenceStableMs,   0, 10000},
  {"absent_margin_cm",    "abs_cm",     PT_F, &S.absentMarginCm,     1, 100},
  {"hold_after_leave_ms", "hold_ms",    PT_I, &S.holdAfterLeaveMs,   0, 30000},
  {"remeasure_delta_cm",  "rem_cm",     PT_F, &S.remeasureDeltaCm,   1, 100},
  {"remeasure_stable_ms", "rem_ms",     PT_I, &S.remeasureStableMs,  0, 10000},
  {"first_view_led",      "first_led",  PT_I, &S.firstViewLed,       0, NUM_PIXELS - 1},
  {"last_view_led",       "last_led",   PT_I, &S.lastViewLed,        0, NUM_PIXELS - 1},
  {"cal_low_led",         "cal_lo_led", PT_I, &S.calLowLed,          0, NUM_PIXELS - 1},
  {"cal_low_cm",          "cal_lo_cm",  PT_F, &S.calLowCm,           0, 400},
  {"cal_high_led",        "cal_hi_led", PT_I, &S.calHighLed,         0, NUM_PIXELS - 1},
  {"cal_high_cm",         "cal_hi_cm",  PT_F, &S.calHighCm,          0, 400},
  {"brightness_max",      "bright",     PT_I, &S.brightnessMax,      1, 255},
  {"dazzle_ms",           "dazzle_ms",  PT_I, &S.dazzleMs,         500, 20000},
  {"rise_speed",          "rise_spd",   PT_F, &S.riseSpeed,          1, 1000},
  {"drain_speed",         "drain_spd",  PT_F, &S.drainSpeed,         1, 1000},
  {"idle_fade_in_ms",     "idle_fade",  PT_I, &S.idleFadeInMs,       0, 10000},
  {"idle_level_pct",      "idle_pct",   PT_I, &S.idleLevelPct,       0, 100},
  {"idle_speed_pct",      "idle_spd",   PT_I, &S.idleSpeedPct,      10, 500},
};
static const int N_PARAMS = sizeof(params) / sizeof(params[0]);

NeoPixelBus<NeoGrbFeature, NeoWs2812xMethod> strip(NUM_PIXELS, PIXEL_PIN);
BluetoothSerial SerialBT;
Preferences prefs;

// Mapping
int  nVis = NUM_PIXELS, visLo = 0, visHi = NUM_PIXELS - 1;
bool reversed = false;

// Frame buffers (unscaled colors, indexed by visible position 0 = bottom)
Col  buf[NUM_PIXELS];
Col  snap[NUM_PIXELS];
uint8_t gammaT[256];

// Time / state
uint32_t nowMs = 0, stateStart = 0, idleStart = 0, lastRenderMs = 0, playStart = 0, playDoneMs = 0;
State state = ST_IDLE;
bool  animInit = true;

// Sensor
uint8_t  fb[9]; int fbi = 0;
uint32_t lastFrameMs = 0, lastPersonMs = 0, lastAbsentMs = 0, personStreakStart = 0, remStart = 0;
uint16_t lastDist = 0, lastFlux = 0;
float    lastHeight = 0, lastTempC = 0, groundCm = DEF_SENSOR_HEIGHT_CM;
float    groundRefCm = DEF_SENSOR_HEIGHT_CM;   // center of the ground-learning clamp (set by calibration)
ReadClass lastClass = RC_UNKNOWN;
uint16_t gBuf[G_N]; int gN = 0;
uint16_t calBuf[G_N]; int calN = 0; bool calSave = false;

// Measurement
float dz[MAX_DZ]; int nDz = 0;
float measuredCm = 0, targetVis = 0;
int   starPix = 0;
bool  offCharts = false;
float riseDurMs = 1000;
int   dazzleIdx = 0, showIdx = 0, outroIdx = 0;
int   forcedDazzle = -1, forcedShow = -1, forcedOutro = -1;

// Outro
bool  outroHasStar = true, outroDone = false;
float drainPhase1Ms = 1000;
uint16_t dissStart[NUM_PIXELS];

// Animation state
Comet   comets[6];
float   cometSpawn = 0;
float   tw[NUM_PIXELS];  uint8_t twc[NUM_PIXELS];
float   dimTw[NUM_PIXELS];
float   scanPhase = 0;

// Play / LED test
PlayType playType = PL_NONE; int playIdx = 0;
int ledTestIdx = -1;

// Simulation / demo
bool  simOn = SIM_AT_BOOT, simPerson = false, simDropout = false, simFault = false;
float simHeightCm = 0, simNoiseCm = 0;
uint32_t simLastMs = 0;
bool  demoOn = false; int demoPhase = 0; uint32_t demoNext = 0, demoSwapAt = 0;

// Counts / IO
uint32_t seasonCount = 0, tonightCount = 0;
bool  btUnlocked = false, btHadClient = false, watchOn = false;
uint32_t lastWatchMs = 0;
String usbLine, btLine;

// =====================================================================
//  Output helpers
// =====================================================================
void out(const char* fmt, ...) {
  char b[320];
  va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof(b), fmt, ap); va_end(ap);
  Serial.println(b);
  if (SerialBT.hasClient()) SerialBT.println(b);
}
const char* simTag() { return simOn ? "[SIM] " : ""; }

// =====================================================================
//  Color / math helpers
// =====================================================================
float clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }
float fmodPos(float a, float m) { float r = fmodf(a, m); return r < 0 ? r + m : r; }
float easeOutCubic(float t) { t = clamp01(t); float u = 1 - t; return 1 - u * u * u; }
float easeInOutCubic(float t) { t = clamp01(t); return t < 0.5f ? 4 * t * t * t : 1 - powf(-2 * t + 2, 3) / 2; }
Col   sc(Col c, float a) { Col r = {c.r * a, c.g * a, c.b * a}; return r; }
Col   mixc(Col a, Col b, float w) { w = clamp01(w); Col r = {a.r + (b.r - a.r) * w, a.g + (b.g - a.g) * w, a.b + (b.b - a.b) * w}; return r; }
void  setPix(int p, Col c, float a) { if (p < 0 || p >= nVis) return; buf[p] = sc(c, a); }
void  addPix(int p, Col c, float a) {
  if (p < 0 || p >= nVis) return;
  buf[p].r = fminf(1, buf[p].r + c.r * a); buf[p].g = fminf(1, buf[p].g + c.g * a); buf[p].b = fminf(1, buf[p].b + c.b * a);
}
void  clearBuf() { for (int i = 0; i < NUM_PIXELS; i++) buf[i] = Col{0, 0, 0}; }
uint8_t to8(float x) { return (uint8_t)(clamp01(x) * 255.0f + 0.5f); }

// =====================================================================
//  Mapping: visible position (0 = bottom) <-> raw LED index <-> height
// =====================================================================
bool validMapping() {
  return S.calLowLed != S.calHighLed && S.calHighCm - S.calLowCm > 1.0f;
}
void recomputeMapping() {
  visLo = constrain(min(S.firstViewLed, S.lastViewLed), 0, NUM_PIXELS - 1);
  visHi = constrain(max(S.firstViewLed, S.lastViewLed), 0, NUM_PIXELS - 1);
  nVis = visHi - visLo + 1;
  reversed = S.calHighLed < S.calLowLed;
}
int visToRaw(int p) { return reversed ? visHi - p : visLo + p; }
float heightToVis(float h) {
  float raw = S.calLowLed + (h - S.calLowCm) * (float)(S.calHighLed - S.calLowLed) / (S.calHighCm - S.calLowCm);
  return reversed ? (visHi - raw) : (raw - visLo);
}
float visToHeight(float v) {
  float raw = reversed ? visHi - v : visLo + v;
  return S.calLowCm + (raw - S.calLowLed) * (S.calHighCm - S.calLowCm) / (float)(S.calHighLed - S.calLowLed);
}
void setTarget(float heightCm) {
  float tv = heightToVis(heightCm);
  offCharts = tv >= (float)(nVis - 1);
  targetVis = fminf(fmaxf(tv, 0.0f), (float)(nVis - 1));
  starPix = (int)roundf(targetVis);
  riseDurMs = fmaxf(700.0f, targetVis / S.riseSpeed * 1000.0f);
}

// =====================================================================
//  Settings
// =====================================================================
void setDefaults() {
  S.sensorHeightCm = DEF_SENSOR_HEIGHT_CM;   S.groundClampCm = DEF_GROUND_CLAMP_CM;
  S.minTriggerCm = DEF_MIN_TRIGGER_CM;       S.absentMarginCm = DEF_ABSENT_MARGIN_CM;
  S.remeasureDeltaCm = DEF_REMEASURE_DELTA_CM;
  S.calLowCm = DEF_CAL_LOW_CM;               S.calHighCm = DEF_CAL_HIGH_CM;
  S.riseSpeed = DEF_RISE_SPEED;              S.drainSpeed = DEF_DRAIN_SPEED;
  S.minFlux = DEF_MIN_FLUX;                  S.presenceStableMs = DEF_PRESENCE_STABLE_MS;
  S.holdAfterLeaveMs = DEF_HOLD_AFTER_LEAVE_MS; S.remeasureStableMs = DEF_REMEASURE_STABLE_MS;
  S.firstViewLed = DEF_FIRST_VIEW_LED;       S.lastViewLed = DEF_LAST_VIEW_LED;
  S.calLowLed = DEF_CAL_LOW_LED;             S.calHighLed = DEF_CAL_HIGH_LED;
  S.brightnessMax = DEF_BRIGHTNESS_MAX;      S.dazzleMs = DEF_DAZZLE_MS;
  S.idleFadeInMs = DEF_IDLE_FADE_IN_MS;      S.idleLevelPct = DEF_IDLE_LEVEL_PCT;
  S.idleSpeedPct = DEF_IDLE_SPEED_PCT;
}
void loadSettings() {
  for (int i = 0; i < N_PARAMS; i++) {
    Param& p = params[i];
    if (p.type == PT_F) *(float*)p.ptr = prefs.getFloat(p.key, *(float*)p.ptr);
    else                *(int32_t*)p.ptr = prefs.getInt(p.key, *(int32_t*)p.ptr);
  }
}
void printParam(Param& p) {
  if (p.type == PT_F) out("  %-20s %.2f", p.name, *(float*)p.ptr);
  else                out("  %-20s %ld", p.name, (long)*(int32_t*)p.ptr);
}

// =====================================================================
//  State changes
// =====================================================================
void setState(State s) {
  state = s; stateStart = nowMs; animInit = true;
  out("STATE %s", STATE_NAMES[s]);
}
void enterIdle() { setState(ST_IDLE); idleStart = nowMs; personStreakStart = 0; gN = 0; }

void setGround(float g) { groundCm = g; groundRefCm = g; }

// Ground calibration: ~1 s of readings with nobody under the sensor.
// save=false at boot (runtime only); save=true from "calibrate ground" (writes sensor_height_cm).
void startCalib(bool save) { calN = 0; calSave = save; setState(ST_CALIB); }
void finishCalib() {
  std::sort(calBuf, calBuf + calN);
  float med = calBuf[calN / 2];
  setGround(med);
  out("%sground calibrated: %.1f cm (%d readings)", simTag(), med, calN);
  if (calSave) {
    if (med >= 50 && med <= 1000) {
      S.sensorHeightCm = med; prefs.putFloat("sens_h", med);
      out("saved sensor_height_cm = %.1f", med);
    } else out("not saved: %.1f cm is outside 50..1000", med);
  }
  enterIdle();
}

void startDazzle() {
  nDz = 0; remStart = 0;
  dazzleIdx = forcedDazzle >= 0 ? forcedDazzle : (int)random(N_DAZZLE);
  forcedDazzle = -1;
  setState(ST_DAZZLE);
  out("%sdazzle: %s", simTag(), DAZZLE_NAMES[dazzleIdx]);
}

void initDissolve() {
  for (int p = 0; p < nVis; p++) dissStart[p] = (uint16_t)((1.0f - (float)p / nVis) * 900.0f + random(0, 400));
}
void startOutro(bool forceDissolve, bool hasStar) {
  memcpy(snap, buf, sizeof(buf));
  outroHasStar = hasStar; outroDone = false;
  if (forceDissolve) outroIdx = 1;
  else { outroIdx = forcedOutro >= 0 ? forcedOutro : (int)random(N_OUTRO); forcedOutro = -1; }
  float h0 = hasStar ? (float)starPix : (float)nVis;
  drainPhase1Ms = fmaxf(600.0f, h0 / S.drainSpeed * 1000.0f);
  initDissolve();
  setState(ST_OUTRO);
  out("outro: %s", OUTRO_NAMES[outroIdx]);
}

void finishMeasurement() {
  if (nDz < MIN_SAMPLES) {
    out("%smeasure failed: only %d valid samples", simTag(), nDz);
    startOutro(true, false);
    return;
  }
  std::sort(dz, dz + nDz);
  float med = (nDz % 2) ? dz[nDz / 2] : 0.5f * (dz[nDz / 2 - 1] + dz[nDz / 2]);
  float spread = dz[(nDz * 9) / 10] - dz[nDz / 10];
  measuredCm = med;
  setTarget(med);
  if (!simOn) { seasonCount++; tonightCount++; prefs.putUInt("visits", seasonCount); }
  showIdx = forcedShow >= 0 ? forcedShow : (int)random(N_SHOW);
  forcedShow = -1;
  float inches = med / 2.54f; int ft = (int)(inches / 12.0f); float rem = inches - ft * 12.0f;
  out("%sMEASURED %.1f cm (%d' %.1f\") samples=%d spread=%.1f cm  show=%s",
      simTag(), med, ft, rem, nDz, spread,
      offCharts ? "offcharts" : SHOW_NAMES[showIdx]);
  remStart = 0;
  setState(ST_RISE);
}

// =====================================================================
//  Sensor input
// =====================================================================
void learnGround(uint16_t dist) {
  gBuf[gN++] = dist;
  if (gN < G_N) return;
  gN = 0;
  uint16_t tmp[G_N];
  memcpy(tmp, gBuf, sizeof(tmp));
  std::sort(tmp, tmp + G_N);
  float med = tmp[G_N / 2];
  float g = constrain(med, groundRefCm - S.groundClampCm, groundRefCm + S.groundClampCm);
  if (fabsf(g - groundCm) >= 2.0f) out("%sground learned: %.1f cm (was %.1f)", simTag(), g, groundCm);
  groundCm = g;
}

void onReading(uint16_t dist, uint16_t flux) {
  lastFrameMs = nowMs; lastDist = dist; lastFlux = flux;
  bool valid = dist > 0 && flux >= S.minFlux && flux != 65535;
  if (!valid) { lastClass = RC_UNKNOWN; return; }
  if (state == ST_CALIB) { if (calN < G_N) calBuf[calN++] = dist; return; }

  float h = groundCm - (float)dist;
  lastHeight = h;
  ReadClass rc = (h >= S.minTriggerCm) ? RC_PERSON : (h <= S.absentMarginCm ? RC_ABSENT : RC_LOW);
  lastClass = rc;

  if (rc == RC_PERSON) {
    lastPersonMs = nowMs;
    if (!personStreakStart) personStreakStart = nowMs;
    if (state == ST_DAZZLE && nDz < MAX_DZ) dz[nDz++] = h;
    if (state == ST_RISE || state == ST_SHOW) {
      if (fabsf(h - measuredCm) > S.remeasureDeltaCm) { if (!remStart) remStart = nowMs; }
      else remStart = 0;
    }
  } else {
    personStreakStart = 0;
    if (rc == RC_ABSENT) { lastAbsentMs = nowMs; remStart = 0; }
    if (state == ST_IDLE) learnGround(dist);
  }
}

void pollLidar() {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (fbi == 0) { if (b == 0x59) fb[fbi++] = b; continue; }
    if (fbi == 1) { if (b == 0x59) fb[fbi++] = b; else fbi = 0; continue; }
    fb[fbi++] = b;
    if (fbi < 9) continue;
    fbi = 0;
    uint8_t sum = 0;
    for (int i = 0; i < 8; i++) sum += fb[i];
    if (sum != fb[8]) continue;
    uint16_t dist = fb[2] | (fb[3] << 8);
    uint16_t flux = fb[4] | (fb[5] << 8);
    uint16_t tr   = fb[6] | (fb[7] << 8);
    lastTempC = tr / 8.0f - 256.0f;          // Benewake formula: deg C = Temp/8 - 256
    if (!simOn) onReading(dist, flux);
  }
}

// =====================================================================
//  Simulation and demo
// =====================================================================
void simTick() {
  if (!simOn) return;
  if (nowMs - simLastMs > 200) simLastMs = nowMs - 10;    // don't burst after a pause
  while (nowMs - simLastMs >= 10) {                        // 100 Hz, like the sensor
    simLastMs += 10;
    if (simFault) continue;
    if (simDropout && random(100) < 15) { onReading(0, 0); continue; }
    float d = simPerson ? S.sensorHeightCm - simHeightCm : S.sensorHeightCm;
    d += (random(-1000, 1001) / 1000.0f) * simNoiseCm;
    if (d < 1) d = 1;
    onReading((uint16_t)(d + 0.5f), 1500);
  }
}

void demoTick() {
  if (!demoOn) return;
  if (demoPhase == 0) {
    if (nowMs >= demoNext) {
      simPerson = true; simHeightCm = random(80, 201);
      out("[SIM] demo: person %.0f cm steps in", simHeightCm);
      demoPhase = 1;
      demoNext = nowMs + S.dazzleMs + 6000 + random(0, 3000);
      demoSwapAt = (random(100) < 30) ? nowMs + S.dazzleMs + 3500 : 0;
    }
  } else {
    if (demoSwapAt && nowMs >= demoSwapAt) {
      float h; do { h = random(80, 201); } while (fabsf(h - simHeightCm) < 20);
      simHeightCm = h; demoSwapAt = 0;
      out("[SIM] demo: swap to %.0f cm", simHeightCm);
      demoNext = nowMs + S.dazzleMs + 6000;
    }
    if (nowMs >= demoNext) {
      simPerson = false; demoPhase = 0;
      demoNext = nowMs + 5000 + random(0, 3000);
      out("[SIM] demo: person leaves");
    }
  }
}

// =====================================================================
//  Presence logic
// =====================================================================
bool triggered() {
  if (!personStreakStart) return false;
  if (nowMs - lastPersonMs > 200) { personStreakStart = 0; return false; }
  return nowMs - personStreakStart >= (uint32_t)S.presenceStableMs;
}
bool personGone() {
  uint32_t since = nowMs - lastPersonMs;
  if (since < (uint32_t)S.holdAfterLeaveMs) return false;
  if (lastAbsentMs > lastPersonMs) return true;      // saw ground after the person
  return since >= STALE_GONE_MS;                      // only unknown/low readings for a long time
}
bool remeasureNow() {
  return remStart && nowMs - remStart >= (uint32_t)S.remeasureStableMs && nowMs - lastPersonMs < 200;
}

void updateState() {
  if (state == ST_PLAY || state == ST_LEDTEST) return;

  if (state == ST_CALIB) {
    uint32_t el = nowMs - stateStart;
    if (calN >= G_N || (el >= 1000 && calN >= MIN_SAMPLES)) finishCalib();
    else if (el >= 3000) {
      out("%sground calibration failed (%d valid readings); using sensor_height_cm %.1f", simTag(), calN, S.sensorHeightCm);
      setGround(S.sensorHeightCm);
      enterIdle();
    }
    return;
  }

  if (state == ST_FAULT) {
    if (nowMs - lastFrameMs < 200) { out("sensor data resumed"); enterIdle(); }
    return;
  }
  if (nowMs - lastFrameMs > FAULT_MS) {
    out("%sFAULT: no data from TFmini", simTag());
    setState(ST_FAULT); idleStart = nowMs;
    return;
  }

  switch (state) {
    case ST_IDLE:
      if (triggered()) startDazzle();
      break;
    case ST_DAZZLE:
      if (personGone()) { out("%sabort: person left during dazzle", simTag()); startOutro(true, false); }
      else if (nowMs - stateStart >= (uint32_t)S.dazzleMs) finishMeasurement();
      break;
    case ST_RISE:
    case ST_SHOW:
      if (remeasureNow()) { out("%sdifferent person: re-measuring", simTag()); startDazzle(); }
      else if (personGone()) startOutro(false, true);
      else if (state == ST_RISE && nowMs - stateStart >= (uint32_t)riseDurMs) setState(ST_SHOW);
      break;
    case ST_OUTRO:
      if (triggered()) startDazzle();
      else if (outroDone) enterIdle();
      break;
    default: break;
  }
}

// =====================================================================
//  Animations
// =====================================================================
// Idle "babbling brook": three ripple waves at different sizes and speeds
// (two flowing up, one flowing down) interfere into an irregular shimmer,
// with brief soft glints on top. Drawn at full scale; renderFrame scales it
// down to idle_level_pct. idle_speed_pct scales the motion.
float idleGlint[NUM_PIXELS];
void renderIdle(float t, float dt) {
  float k = S.idleSpeedPct / 100.0f;
  float tt = t * k;
  float decay = expf(-dt * 6.0f * k);
  for (int p = 0; p < nVis; p++) {
    float a = sinf(p * 0.45f - tt * 3.1f);
    float b = sinf(p * 0.27f + tt * 1.9f);
    float c = sinf(p * 0.83f - tt * 5.3f);
    float shimmer = (a + 0.8f * b + 0.6f * c) / 2.4f;           // -1..1, irregular
    float lvl = 0.45f + 0.35f * shimmer;                         // 0.1..0.8
    float w = 0.5f + 0.5f * sinf(p * 0.06f - tt * 0.9f);         // red/green drifting up
    setPix(p, mixc(C_RED, C_GREEN, w), lvl);

    idleGlint[p] *= decay;                                       // glints: ~0.15/s per pixel
    if (random(100000) < (long)(dt * k * 0.15f * 100000.0f)) idleGlint[p] = 1.0f;
    if (idleGlint[p] > 0.01f) addPix(p, C_WHITE, idleGlint[p] * 0.5f);
  }
}

// ---- Dazzle ----
void dzCandyCane(float t) {
  for (int p = 0; p < nVis; p++) {
    int s = (int)floorf(p - t * 25.0f);
    bool red = (((s % 10) + 10) % 10) < 5;
    setPix(p, red ? C_RED : C_WHITE, red ? 0.9f : 0.7f);
  }
}
void dzComets(float dt, bool init) {
  if (init) { for (auto& c : comets) c.on = false; cometSpawn = 0; }
  cometSpawn -= dt;
  if (cometSpawn <= 0) {
    for (auto& c : comets) if (!c.on) {
      c.on = true; c.pos = 0; c.speed = random(40, 91);
      int k = random(4); c.c = k == 0 ? C_GOLD : k == 1 ? C_WHITE : k == 2 ? C_GREEN : C_RED;
      break;
    }
    cometSpawn = random(150, 400) / 1000.0f;
  }
  for (auto& c : comets) {
    if (!c.on) continue;
    c.pos += c.speed * dt;
    if (c.pos - 12 > nVis) { c.on = false; continue; }
    for (int k = 0; k < 12; k++) { float a = 1.0f - k / 12.0f; addPix((int)c.pos - k, c.c, a * a); }
  }
}
void twinkleField(float dt, bool init, int from, float ratePerPixel) {
  if (init) for (int p = 0; p < NUM_PIXELS; p++) tw[p] = 0;
  float decay = expf(-dt * 4.0f);
  for (int p = from; p < nVis; p++) {
    tw[p] *= decay;
    if (random(100000) < (long)(dt * ratePerPixel * 100000.0f)) { tw[p] = 1; twc[p] = random(2); }
    if (tw[p] > 0.01f) addPix(p, twc[p] ? C_WHITE : C_GOLD, tw[p]);
  }
}
void dzScanner(float st, float dt, bool init) {
  if (init) scanPhase = 0;
  float freq = 0.4f + 2.6f * clamp01(st / (float)S.dazzleMs);   // speeds up
  scanPhase += freq * dt;
  float tri = fabsf(fmodPos(scanPhase, 1.0f) * 2.0f - 1.0f);
  float pos = (1.0f - tri) * (nVis - 1);
  for (int p = 0; p < nVis; p++) setPix(p, C_RED, 0.08f);
  for (int d = -4; d <= 4; d++) { float a = 1.0f - fabsf(d) / 5.0f; setPix((int)roundf(pos) + d, mixc(C_RED, C_WHITE, a), a); }
}
void dzChase(float t) {
  int step = (int)(t * 12.0f);
  for (int p = 0; p < nVis; p++) {
    int q = p - step + 6000;
    if (q % 3 == 0) setPix(p, ((q / 3) % 2) ? C_RED : C_GREEN, 1.0f);
  }
}
void dzFillDrain(float t) {
  float ph = fmodPos(t, 1.4f) / 1.4f;
  float lvl = ph < 0.5f ? easeInOutCubic(ph * 2) : easeInOutCubic(2 - ph * 2);
  float top = lvl * nVis; int ti = (int)top;
  for (int p = 0; p < ti && p < nVis; p++) setPix(p, C_GREEN, 0.8f);
  setPix(ti, C_WHITE, 0.4f + 0.6f * (top - ti));
}
void renderDazzle(int idx, float stMs, float dt, bool init) {
  float t = stMs / 1000.0f;
  switch (idx) {
    case 0: dzCandyCane(t); break;
    case 1: dzComets(dt, init); break;
    case 2: twinkleField(dt, init, 0, 1.2f); break;
    case 3: dzScanner(stMs, dt, init); break;
    case 4: dzChase(t); break;
    case 5: dzFillDrain(t); break;
  }
}

// ---- Rise / show ----
void renderRise(float stMs) {
  float cur = targetVis * easeOutCubic(stMs / riseDurMs);
  int ci = (int)cur;
  for (int p = 0; p < ci; p++) setPix(p, C_RED, 0.6f);
  setPix(ci, C_WHITE, 0.4f + 0.6f * (cur - ci));
}
void renderStar(float t) {
  if (t < 0.35f) { setPix(starPix, C_WHITE, 1.0f); return; }       // arrival flash
  setPix(starPix, C_GOLD, 0.7f + 0.3f * sinf(t * 6.0f));
  if (random(1000) < 12) setPix(starPix, C_WHITE, 1.0f);
  addPix(starPix - 1, C_GOLD, 0.12f);
  addPix(starPix + 1, C_GOLD, 0.12f);
}
void dimAbove(float dt) {
  float decay = expf(-dt * 2.0f);
  for (int p = starPix + 2; p < nVis; p++) {
    dimTw[p] *= decay;
    if (random(100000) < (long)(dt * 5000.0f)) dimTw[p] = 1;
    if (dimTw[p] > 0.01f) setPix(p, C_WHITE, dimTw[p] * 0.08f);
  }
}
void renderShow(int idx, float t, float dt) {
  int top = starPix;
  switch (idx) {
    case 0:  // crawlers: red column, green pulses travel up
      for (int p = 0; p < top; p++) {
        float x = fmodPos(p - t * 30.0f, 18.0f); float d = fminf(x, 18.0f - x);
        buf[p] = mixc(sc(C_RED, 0.6f), C_GREEN, expf(-d * d / 3.0f));
      }
      break;
    case 1:  // candy cane stripes scrolling up inside the column
      for (int p = 0; p < top; p++) {
        int s = (int)floorf(p - t * 8.0f);
        buf[p] = ((((s % 8) + 8) % 8) < 4) ? sc(C_RED, 0.7f) : sc(C_WHITE, 0.55f);
      }
      break;
    case 2: {  // breathing green with red waves rising
      float g = 0.3f + 0.35f * (0.5f + 0.5f * sinf(2.0f * PI * t / 2.6f));
      for (int p = 0; p < top; p++) {
        float r = sinf(p * 0.35f - t * 3.0f); r = r > 0 ? powf(r, 6) : 0;
        buf[p] = mixc(sc(C_GREEN, g), sc(C_RED, 0.8f), r);
      }
      break;
    }
  }
  renderStar(t);
  dimAbove(dt);
}
void renderOffCharts(float t, float dt, bool init) {
  for (int p = 0; p < nVis; p++) {
    int q = (int)floorf(p - t * 20.0f);
    buf[p] = ((((q % 4) + 4) % 4) < 2) ? sc(C_RED, 0.7f) : sc(C_GREEN, 0.7f);
  }
  twinkleField(dt, init, max(0, nVis - 15), 4.0f);
}

// ---- Outro ----
bool renderDrain(float stMs) {
  float h0 = outroHasStar ? (float)starPix : (float)nVis;
  if (stMs < drainPhase1Ms) {
    float top = h0 * (1.0f - easeInOutCubic(stMs / drainPhase1Ms));
    int ti = (int)top;
    for (int p = 0; p < ti && p < nVis; p++) buf[p] = snap[p];
    if (ti < nVis && ti < (int)h0) buf[ti] = sc(snap[ti], top - ti);
    if (outroHasStar) setPix(starPix, C_GOLD, 0.9f);
    return false;
  }
  if (!outroHasStar) return true;
  float tt = (stMs - drainPhase1Ms) / 1000.0f;
  float pos = starPix - 0.5f * 400.0f * tt * tt;                 // falling star
  if (pos <= 0) return true;
  for (int k = 0; k < 7; k++) {
    float a = 1.0f - k / 7.0f; a *= a;
    addPix((int)(pos + k), k == 0 ? C_GOLD : C_WHITE, k == 0 ? a : a * 0.6f);
  }
  return false;
}
bool renderDissolve(float stMs) {
  for (int p = 0; p < nVis; p++) {
    float s = dissStart[p];
    float a = stMs < s ? 1.0f : 1.0f - (stMs - s) / 300.0f;
    if (a > 0) buf[p] = sc(snap[p], a);
  }
  return stMs > 1800;
}
bool renderOutro(float stMs) { return outroIdx == 0 ? renderDrain(stMs) : renderDissolve(stMs); }

// ---- Play (test one animation in a loop) ----
void renderPlay(float stMs, float dt, bool init) {
  float t = stMs / 1000.0f;
  switch (playType) {
    case PL_IDLE: renderIdle(t, dt); break;     // scaled to idle level in renderFrame
    case PL_DAZZLE:
      if (stMs >= S.dazzleMs) { playStart = nowMs; animInit = true; }
      renderDazzle(playIdx, stMs, dt, init);
      break;
    case PL_RISE:
      if (stMs >= riseDurMs + 1500) { playStart = nowMs; animInit = true; }
      if (stMs < riseDurMs) renderRise(stMs); else renderShow(0, (stMs - riseDurMs) / 1000.0f, dt);
      break;
    case PL_SHOW: renderShow(playIdx, t, dt); break;
    case PL_OFFCHARTS: renderOffCharts(t, dt, init); break;
    case PL_OUTRO:
      if (init) {
        clearBuf(); renderShow(0, 1.0f, 0.016f); memcpy(snap, buf, sizeof(buf));
        outroHasStar = true; outroIdx = playIdx; initDissolve(); playDoneMs = 0;
        drainPhase1Ms = fmaxf(600.0f, (float)starPix / S.drainSpeed * 1000.0f);
        clearBuf();
      }
      if (renderOutro(stMs)) {
        if (!playDoneMs) playDoneMs = nowMs;
        else if (nowMs - playDoneMs > 700) { playStart = nowMs; animInit = true; }
      }
      break;
    default: break;
  }
}

// =====================================================================
//  Frame output
// =====================================================================
void output(float master) {
  float k = (S.brightnessMax / 255.0f) * master;
  strip.ClearTo(RgbColor(0, 0, 0));
  for (int p = 0; p < nVis; p++) {
    Col c = buf[p];
    strip.SetPixelColor(visToRaw(p), RgbColor((uint8_t)(gammaT[to8(c.r)] * k + 0.5f),
                                              (uint8_t)(gammaT[to8(c.g)] * k + 0.5f),
                                              (uint8_t)(gammaT[to8(c.b)] * k + 0.5f)));
  }
  strip.Show();
}

void renderFrame(float dt) {
  bool init = animInit; animInit = false;

  if (state == ST_LEDTEST) {                     // raw index, bypasses mapping
    strip.ClearTo(RgbColor(0, 0, 0));
    if (ledTestIdx >= 0 && ledTestIdx < NUM_PIXELS) strip.SetPixelColor(ledTestIdx, RgbColor(80, 80, 80));
    strip.Show();
    return;
  }

  clearBuf();
  float st = (float)(nowMs - stateStart);
  float master = 1.0f;
  switch (state) {
    case ST_IDLE:
    case ST_FAULT:
      renderIdle((nowMs - idleStart) / 1000.0f, dt);
      master = S.idleLevelPct / 100.0f;
      if (state == ST_IDLE && S.idleFadeInMs > 0) master *= clamp01((nowMs - idleStart) / (float)S.idleFadeInMs);
      break;
    case ST_DAZZLE: renderDazzle(dazzleIdx, st, dt, init); break;
    case ST_RISE:   renderRise(st); break;
    case ST_SHOW:
      if (offCharts) renderOffCharts(st / 1000.0f, dt, init);
      else           renderShow(showIdx, st / 1000.0f, dt);
      break;
    case ST_OUTRO:  outroDone = renderOutro(st); break;
    case ST_PLAY:
      renderPlay((float)(nowMs - playStart), dt, init);
      if (playType == PL_IDLE) master = S.idleLevelPct / 100.0f;
      break;
    default: break;   // ST_CALIB: dark for ~1 s, then idle fades in
  }
  output(master);
}

// =====================================================================
//  Commands (USB serial and Bluetooth)
// =====================================================================
int findName(const char* const* names, int n, const char* s) {
  for (int i = 0; i < n; i++) if (!strcmp(names[i], s)) return i;
  return -1;
}

void printStatus() {
  out("--- status ---");
  out("state=%s sim=%s demo=%s watch=%s", STATE_NAMES[state], simOn ? "on" : "off", demoOn ? "on" : "off", watchOn ? "on" : "off");
  out("last: dist=%u cm flux=%u height=%.1f cm (%s) temp=%.1f C", lastDist, lastFlux, lastHeight, RC_NAMES[lastClass], lastTempC);
  out("ground=%.1f cm (calibrated=%.1f, sensor_height_cm=%.1f)", groundCm, groundRefCm, S.sensorHeightCm);
  out("last measured=%.1f cm  visible LEDs=%d (raw %d..%d) reversed=%s", measuredCm, nVis, visLo, visHi, reversed ? "yes" : "no");
  out("top visible LED = %.1f cm", visToHeight(nVis - 1));
  out("visitors: season=%lu tonight=%lu  uptime=%lus", (unsigned long)seasonCount, (unsigned long)tonightCount, (unsigned long)(nowMs / 1000));
}

void printHelp() {
  out("--- commands ---");
  out("help | status | get | count | play list | watch on|off");
  out("unlock <pw>                  (Bluetooth only; USB is always unlocked)");
  out("set <name> <value> | defaults | count reset | reboot");
  out("calibrate ground             nobody under the sensor; saves sensor_height_cm");
  out("led <n> | led off            light one raw LED for calibration");
  out("sim on|off | sim height <cm> | sim leave | sim noise <cm>");
  out("sim dropout on|off | sim fault | demo on|off");
  out("play <type> [name] | play stop   types: idle dazzle rise show outro offcharts");
  out("next <dazzle|show|outro> <name>  force the next random pick");
}

void handleCmd(String line, Src src) {
  line.trim(); line.toLowerCase();
  if (!line.length()) return;
  char b[128]; line.toCharArray(b, sizeof(b));
  char* tok[4] = {0}; int nt = 0; char* sv = nullptr;
  for (char* t = strtok_r(b, " \t", &sv); t && nt < 4; t = strtok_r(nullptr, " \t", &sv)) tok[nt++] = t;
  const char* c  = tok[0];
  const char* a1 = nt > 1 ? tok[1] : "";
  const char* a2 = nt > 2 ? tok[2] : "";
  bool unlocked = (src == SRC_USB) || btUnlocked;

  if (!strcmp(c, "help"))   { printHelp(); return; }
  if (!strcmp(c, "status")) { printStatus(); return; }
  if (!strcmp(c, "get"))    { out("--- settings ---"); for (int i = 0; i < N_PARAMS; i++) printParam(params[i]); return; }
  if (!strcmp(c, "watch"))  { watchOn = !strcmp(a1, "on"); out("watch %s", watchOn ? "on" : "off"); return; }
  if (!strcmp(c, "count") && strcmp(a1, "reset")) {
    out("visitors: season=%lu tonight=%lu", (unsigned long)seasonCount, (unsigned long)tonightCount); return;
  }
  if (!strcmp(c, "play") && !strcmp(a1, "list")) {
    out("idle: wave");
    out("dazzle: candycane comets twinkle scanner chase filldrain");
    out("rise: rise   offcharts: offcharts");
    out("show: crawlers candyfill breathing");
    out("outro: drain dissolve");
    return;
  }
  if (!strcmp(c, "unlock")) {
    if (src == SRC_USB) { out("USB is always unlocked"); return; }
    if (!strcmp(a1, BT_PASSWORD)) { btUnlocked = true; out("unlocked"); }
    else out("wrong password");
    return;
  }

  // Everything below changes something
  if (!unlocked) { out("locked: send 'unlock <password>' first"); return; }

  if (!strcmp(c, "set")) {
    for (int i = 0; i < N_PARAMS; i++) {
      Param& p = params[i];
      if (strcmp(p.name, a1)) continue;
      if (!*a2) { out("usage: set %s <value>", p.name); return; }
      float v = atof(a2);
      if (v < p.minV || v > p.maxV) { out("out of range (%.0f..%.0f)", p.minV, p.maxV); return; }
      float oldF = 0; int32_t oldI = 0;
      if (p.type == PT_F) { oldF = *(float*)p.ptr; *(float*)p.ptr = v; }
      else                { oldI = *(int32_t*)p.ptr; *(int32_t*)p.ptr = (int32_t)lroundf(v); }
      if (!validMapping()) {
        if (p.type == PT_F) *(float*)p.ptr = oldF; else *(int32_t*)p.ptr = oldI;
        out("rejected: calibration points must differ (cal_high_cm > cal_low_cm, LEDs not equal)");
        return;
      }
      if (p.type == PT_F) prefs.putFloat(p.key, *(float*)p.ptr); else prefs.putInt(p.key, *(int32_t*)p.ptr);
      recomputeMapping();
      if (!strcmp(p.name, "sensor_height_cm")) setGround(S.sensorHeightCm);
      printParam(p);
      return;
    }
    out("unknown setting '%s' (try 'get')", a1);
    return;
  }
  if (!strcmp(c, "defaults")) {
    prefs.clear(); prefs.putUInt("visits", seasonCount);
    setDefaults(); recomputeMapping(); setGround(S.sensorHeightCm);
    out("defaults restored");
    return;
  }
  if (!strcmp(c, "count") && !strcmp(a1, "reset")) {
    seasonCount = 0; tonightCount = 0; prefs.putUInt("visits", 0); out("counts reset"); return;
  }
  if (!strcmp(c, "reboot")) { out("rebooting"); delay(100); ESP.restart(); return; }

  if (!strcmp(c, "calibrate")) {
    if (strcmp(a1, "ground")) { out("usage: calibrate ground"); return; }
    out("%scalibrating ground: keep clear of the sensor for 1 s", simTag());
    startCalib(true);
    return;
  }

  if (!strcmp(c, "led")) {
    if (!strcmp(a1, "off")) { ledTestIdx = -1; enterIdle(); return; }
    ledTestIdx = atoi(a1);
    if (ledTestIdx < 0 || ledTestIdx >= NUM_PIXELS) { out("LED must be 0..%d", NUM_PIXELS - 1); ledTestIdx = -1; return; }
    setState(ST_LEDTEST);
    out("raw LED %d lit; 'led off' to resume", ledTestIdx);
    return;
  }

  if (!strcmp(c, "sim")) {
    if (!strcmp(a1, "on"))  { simOn = true; simLastMs = nowMs; setGround(S.sensorHeightCm); out("[SIM] simulation on (sensor ignored)"); return; }
    if (!strcmp(a1, "off")) {
      simOn = false; demoOn = false; simPerson = false; simFault = false;
      out("simulation off (real sensor)");
      startCalib(false);                       // re-find the real ground
      return;
    }
    if (!strcmp(a1, "height")) {
      float h = atof(a2);
      if (h <= 0) { out("usage: sim height <cm>"); return; }
      if (!simOn) { simOn = true; simLastMs = nowMs; setGround(S.sensorHeightCm); out("[SIM] simulation on"); }
      simHeightCm = h; simPerson = true; simFault = false;
      out("[SIM] person %.1f cm steps in", h); return;
    }
    if (!strcmp(a1, "leave"))   { simPerson = false; simFault = false; out("[SIM] person leaves"); return; }
    if (!strcmp(a1, "noise"))   { simNoiseCm = fmaxf(0, atof(a2)); out("[SIM] noise %.1f cm", simNoiseCm); return; }
    if (!strcmp(a1, "dropout")) { simDropout = !strcmp(a2, "on"); out("[SIM] dropout %s", simDropout ? "on" : "off"); return; }
    if (!strcmp(a1, "fault"))   { if (!simOn) { simOn = true; simLastMs = nowMs; } simFault = true; out("[SIM] sensor fault"); return; }
    out("usage: sim on|off|height <cm>|leave|noise <cm>|dropout on|off|fault");
    return;
  }

  if (!strcmp(c, "demo")) {
    if (!strcmp(a1, "on")) {
      demoOn = true; simOn = true; simLastMs = nowMs; setGround(S.sensorHeightCm);
      simPerson = false; simFault = false; demoPhase = 0; demoNext = nowMs + 2000;
      out("[SIM] demo on");
    } else { demoOn = false; simPerson = false; out("[SIM] demo off (sim still on; 'sim off' to stop)"); }
    return;
  }

  if (!strcmp(c, "play")) {
    if (!strcmp(a1, "stop")) { playType = PL_NONE; enterIdle(); return; }
    int pt = findName(PLAY_TYPES, 6, a1);
    if (pt < 0) { out("usage: play <idle|dazzle|rise|show|outro|offcharts> [name] | play stop | play list"); return; }
    int idx = 0;
    if (pt == PL_DAZZLE) idx = *a2 ? findName(DAZZLE_NAMES, N_DAZZLE, a2) : 0;
    if (pt == PL_SHOW)   idx = *a2 ? findName(SHOW_NAMES, N_SHOW, a2) : 0;
    if (pt == PL_OUTRO)  idx = *a2 ? findName(OUTRO_NAMES, N_OUTRO, a2) : 0;
    if (idx < 0) { out("unknown name (try 'play list')"); return; }
    playType = (PlayType)pt; playIdx = idx;
    setTarget(simHeightCm > 0 ? simHeightCm : 120.0f);
    if (pt == PL_OFFCHARTS) { targetVis = nVis - 1; starPix = nVis - 1; }
    playStart = nowMs;
    setState(ST_PLAY);
    out("playing %s %s; 'play stop' to end", a1, *a2 ? a2 : "");
    return;
  }

  if (!strcmp(c, "next")) {
    int idx = -1;
    if (!strcmp(a1, "dazzle")) { idx = findName(DAZZLE_NAMES, N_DAZZLE, a2); if (idx >= 0) forcedDazzle = idx; }
    else if (!strcmp(a1, "show")) { idx = findName(SHOW_NAMES, N_SHOW, a2); if (idx >= 0) forcedShow = idx; }
    else if (!strcmp(a1, "outro")) { idx = findName(OUTRO_NAMES, N_OUTRO, a2); if (idx >= 0) forcedOutro = idx; }
    if (idx < 0) out("usage: next <dazzle|show|outro> <name> (try 'play list')");
    else out("next %s will be %s", a1, a2);
    return;
  }

  out("unknown command '%s' (try 'help')", c);
}

void pollCmds() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { if (usbLine.length()) handleCmd(usbLine, SRC_USB); usbLine = ""; }
    else if (usbLine.length() < 120) usbLine += ch;
  }
  while (SerialBT.available()) {
    char ch = SerialBT.read();
    if (ch == '\n' || ch == '\r') { if (btLine.length()) handleCmd(btLine, SRC_BT); btLine = ""; }
    else if (btLine.length() < 120) btLine += ch;
  }
  bool hc = SerialBT.hasClient();
  if (hc && !btHadClient) out("Bluetooth client connected");
  if (!hc && btHadClient) { btUnlocked = false; btLine = ""; out("Bluetooth client disconnected (locked)"); }
  btHadClient = hc;
}

void watchTick() {
  if (!watchOn || nowMs - lastWatchMs < 200) return;
  lastWatchMs = nowMs;
  out("%sd=%u flux=%u h=%.1f %s ground=%.1f", simTag(), lastDist, lastFlux, lastHeight, RC_NAMES[lastClass], groundCm);
}

// =====================================================================
//  Setup / loop
// =====================================================================
void setupWatchdog() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t cfg = {};
  cfg.timeout_ms = WDT_TIMEOUT_S * 1000;
  cfg.idle_core_mask = 0;
  cfg.trigger_panic = true;
  if (esp_task_wdt_init(&cfg) != ESP_OK) esp_task_wdt_reconfigure(&cfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, LIDAR_RX, LIDAR_TX);

  for (int i = 0; i < 256; i++) gammaT[i] = (uint8_t)(powf(i / 255.0f, 2.2f) * 255.0f + 0.5f);

  setDefaults();
  prefs.begin("northpole", false);
  loadSettings();
  seasonCount = prefs.getUInt("visits", 0);
  if (!validMapping()) { setDefaults(); Serial.println("saved calibration invalid; using defaults"); }
  recomputeMapping();
  setGround(S.sensorHeightCm);              // fallback until boot calibration finishes

  strip.Begin();
  strip.Show();

  SerialBT.begin(BT_NAME);
  setupWatchdog();

  nowMs = millis();
  lastFrameMs = nowMs; simLastMs = nowMs; lastRenderMs = nowMs;
  out("North Pole ready. season visitors=%lu. Type 'help'.%s", (unsigned long)seasonCount, simOn ? " [SIM at boot]" : "");
  startCalib(false);                        // nobody is under the sensor at power-on
}

void loop() {
  nowMs = millis();
  esp_task_wdt_reset();

  pollLidar();
  simTick();
  demoTick();
  pollCmds();
  updateState();
  watchTick();

  if (nowMs - lastRenderMs >= FRAME_MS) {
    float dt = (nowMs - lastRenderMs) / 1000.0f;
    if (dt > 0.1f) dt = 0.1f;
    lastRenderMs = nowMs;
    renderFrame(dt);
  }
}
