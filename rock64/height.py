#!/usr/bin/env python3
"""North Pole height sensor: Rock64 + Intel RealSense D435i, camera looking straight down.

  ~/rs/bin/python height.py calibrate   # nobody in view: records the empty scene
  ~/rs/bin/python height.py             # run, linked to the ESP32 over USB serial
  ~/rs/bin/python height.py noserial    # run without the ESP32 (terminal only, default box)
  ~/rs/bin/python height.py nobox       # terminal only, whole view counts (finding the spot)

How it works:
  calibrate: ~3 s of depth frames, per-pixel median = empty scene (floor, pole, sign).
             Saved to background.npy next to this script.
             Done automatically at every start (anyone in view then becomes part of the floor
             until the next start or 'calibrate camera'), and on request from the ESP32.
  run:       a pixel is "person" if it is more than FG_MARGIN_CM closer than the empty scene.
             Touching person pixels form blobs (one per person, unless people touch); a depth
             step of more than JUMP_CM between neighbors separates blobs (one person in front
             of another). Blobs smaller than MIN_PIXELS are noise. For each blob the TOP_K highest pixels (their densest group) locate the
             top of the head; height = median of the blob's pixels within HEAD_PATCH_CM of that
             spot (height above the empty scene).
             A blob counts if its top of head is inside the standing box (a rectangle on the
             floor, measured from the point directly under the camera); the tallest one wins.
             Someone outside the box reaching into it is ignored, but reported as unknown.
             RealSense depth is distance from the camera plane, not along the ray, so a head
             off-center reads the same as one dead center.
             If the top of the head is at the edge of the camera's view it may be cut off:
             reported as unknown, not as a (too low) height.

Serial protocol with the ESP32 (115200, lines; '#' lines are sensor traffic):
  to ESP32:   "#H <cm>" height (0 = nobody), "#U" unknown, "#I <text>" info, "#HELLO" on connect
  from ESP32: "#BOX <w> <d> <x> <y>" standing box in cm,
              "#CFG <headpatch cm> <minpixels> <topk>" measuring settings, "#CAL" record the empty scene
  Other lines from the ESP32 are its status output; they are printed here with "esp32|".
"""
import glob
import os
import sys
import time

import numpy as np
import pyrealsense2 as rs
from scipy import ndimage

# ---- defaults (when linked, the ESP32 sends the box and the three marked * below) ----
BOX_W_CM = 50.0        # standing box size along the image x axis
BOX_D_CM = 50.0        # standing box size along the image y axis
BOX_X_CM = 0.0         # box center, cm from the point directly under the camera (image x)
BOX_Y_CM = 0.0         # box center, cm from the point directly under the camera (image y)
FG_MARGIN_CM = 10.0    # must be this far above the empty scene to count as person
MIN_PIXELS = 150       # * blobs smaller than this are noise, not people
TOP_K = 40             # * the highest this-many pixels of a blob locate the top of the head
TOP_CLUSTER_CM = 10.0  # ... using only the densest group of them within this radius
HEAD_PATCH_CM = 3.0    # * height = median of the blob's pixels within this radius of that spot
EDGE_PX = 6            # head top this close to the image edge = maybe cut off -> unknown
JUMP_CM = 25.0         # depth step between neighboring pixels that separates two blobs
PRINT_EVERY_S = 0.2

# ---- camera ----
WIDTH, HEIGHT, FPS = 640, 480, 30   # works on USB 2
DECIMATE = 2                        # 640x480 -> 320x240 (less CPU, fills small holes)
SETTLE_FRAMES = 30                  # let auto-exposure settle before calibrating
CAL_SECONDS = 3.0
CAL_EVERY = 3                       # use every 3rd frame while calibrating (~30 frames)
CAL_DELAY_S = 5.0                   # "calibrate camera" from the ESP32 waits this long first
START_CAL_DELAY_S = 2.0             # at every start: record the empty scene after this long

# ---- serial link ----
SERIAL_GLOB = "/dev/serial/by-id/*CP210*"
SERIAL_FALLBACK = "/dev/ttyUSB0"
BAUD = 115200

BG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "background.npy")


def log(msg):
    print(time.strftime("%H:%M:%S ") + msg, flush=True)


# ---------------------------------------------------------------- camera

def start_camera():
    """Start the depth stream; keep retrying if the camera isn't there yet."""
    while True:
        try:
            pipe = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
            prof = pipe.start(cfg)
            scale = prof.get_device().first_depth_sensor().get_depth_scale()
            dec = rs.decimation_filter()
            dec.set_option(rs.option.filter_magnitude, DECIMATE)
            log(f"camera started, depth unit {scale * 1000:.3f} mm")
            return pipe, dec, scale
        except RuntimeError as e:
            log(f"camera not available ({e}), retrying in 2 s")
            time.sleep(2)


def stop_camera(pipe):
    try:
        pipe.stop()
    except RuntimeError:
        pass


def grab(pipe, dec, scale):
    """One depth frame in cm (0 = no reading), plus the original frame for intrinsics."""
    raw = pipe.wait_for_frames().get_depth_frame()
    small = dec.process(raw)
    d = np.asanyarray(small.get_data()).astype(np.float32) * (scale * 100.0)
    return d, raw


def intrinsics_for(shape, raw):
    """Intrinsics of the original frame, scaled to the decimated size."""
    i = raw.profile.as_video_stream_profile().get_intrinsics()
    k = shape[1] / float(i.width)
    return i.fx * k, i.fy * k, i.ppx * k, i.ppy * k


def background_from(frames):
    # Per-pixel median of the valid (non-zero) readings. Sorting with the zeros pushed to
    # the end is ~5x faster than np.nanmedian, which matters on the Rock64: while this runs
    # nothing goes to the ESP32, and after 1 s it reports FAULT.
    stack = np.stack(frames)
    valid = stack > 0
    cnt = valid.sum(axis=0)
    srt = np.where(valid, stack, np.inf)
    srt.sort(axis=0)
    lo = np.take_along_axis(srt, np.clip((cnt - 1) // 2, 0, None)[None], 0)[0]
    hi = np.take_along_axis(srt, np.clip(cnt // 2, 0, None)[None], 0)[0]
    bg = np.where(cnt >= 0.5 * len(frames), 0.5 * (lo + hi), 0).astype(np.float32)
    np.save(BG_FILE, bg)
    h, w = bg.shape
    return bg, (f"{len(frames)} frames, {w}x{h}, center {bg[h // 2, w // 2]:.1f} cm, "
                f"{100 * np.mean(bg > 0):.0f}% of pixels have a reading")


def calibrate_standalone():
    pipe, dec, scale = start_camera()
    try:
        for _ in range(SETTLE_FRAMES):
            grab(pipe, dec, scale)
        log(f"calibrating: keep the area empty for {CAL_SECONDS:.0f} s")
        frames, n = [], 0
        t0 = time.time()
        while time.time() - t0 < CAL_SECONDS:
            d, _ = grab(pipe, dec, scale)
            n += 1
            if n % CAL_EVERY == 0:
                frames.append(d)
    finally:
        stop_camera(pipe)
    _, summary = background_from(frames)
    log(f"saved {BG_FILE}")
    log(summary)


# ---------------------------------------------------------------- serial link

class Link:
    """USB serial to the ESP32. Reconnects on its own; never blocks the camera loop."""

    def __init__(self, enabled):
        self.enabled = enabled
        self.s = None
        self.buf = b""
        self.next_try = 0.0

    def ensure(self):
        if not self.enabled or self.s:
            return
        now = time.time()
        if now < self.next_try:
            return
        self.next_try = now + 2.0
        import serial
        ports = sorted(glob.glob(SERIAL_GLOB))
        port = ports[0] if ports else (SERIAL_FALLBACK if os.path.exists(SERIAL_FALLBACK) else None)
        if not port:
            return
        try:
            s = serial.Serial()
            s.port, s.baudrate, s.timeout, s.write_timeout = port, BAUD, 0, 0.2
            s.exclusive = True          # flashing tools get "busy" instead of fighting over the port
            s.dtr = False               # don't reset the ESP32 when opening
            s.rts = False
            s.open()
            self.s, self.buf = s, b""
            log(f"serial: connected to {port}")
            self.send("#HELLO")
        except Exception as e:          # noqa: BLE001  (missing port, busy, permissions)
            log(f"serial: {port}: {e}")

    def drop(self, e):
        log(f"serial: lost ({e}), retrying")
        try:
            self.s.close()
        except Exception:               # noqa: BLE001
            pass
        self.s = None

    def send(self, line):
        if not self.s:
            return
        try:
            self.s.write((line + "\n").encode())
        except Exception as e:          # noqa: BLE001
            self.drop(e)

    def lines(self):
        if not self.s:
            return []
        try:
            data = self.s.read(4096)
        except Exception as e:          # noqa: BLE001
            self.drop(e)
            return []
        self.buf += data
        parts = self.buf.split(b"\n")
        self.buf = parts.pop()[-1000:]
        return [p.decode("utf-8", "replace").strip() for p in parts]


# ---------------------------------------------------------------- measuring

class Measurer:
    def __init__(self, use_box):
        self.use_box = use_box
        self.box = [BOX_W_CM, BOX_D_CM, BOX_X_CM, BOX_Y_CM]
        self.head_patch_cm, self.min_pixels, self.top_k = HEAD_PATCH_CM, MIN_PIXELS, TOP_K
        self.bg = np.load(BG_FILE) if os.path.exists(BG_FILE) else None
        self.rays = None          # per-column / per-row lateral cm per cm of depth

    def set_cfg(self, head_patch_cm, min_pixels, top_k):
        self.head_patch_cm, self.min_pixels, self.top_k = head_patch_cm, int(min_pixels), int(top_k)
        log(f"headpatch {self.head_patch_cm:.1f} cm, minpixels {self.min_pixels}, topk {self.top_k}")

    def set_box(self, w, d, x, y):
        self.box = [w, d, x, y]
        log(f"standing box {w:.0f} x {d:.0f} cm centered at x {x:+.0f} y {y:+.0f} cm")

    def measure(self, d, raw):
        """Returns ("none"|"nobody"|"cut"|"person", height_cm, hx, hy, n_pixels)."""
        bg = self.bg
        if bg is None:
            return "none", 0.0, 0.0, 0.0, 0
        if d.shape != bg.shape:
            return "none", 0.0, 0.0, 0.0, 0
        if self.rays is None:
            fx, fy, ppx, ppy = intrinsics_for(bg.shape, raw)
            h, w = bg.shape
            self.rays = ((np.arange(w, dtype=np.float32) - ppx) / fx,
                         (np.arange(h, dtype=np.float32) - ppy) / fy,
                         fx, ppx, ppy, fy)
        rx, ry, fx, ppx, ppy, fy = self.rays

        above = bg - d                                  # cm above the empty scene
        cand = (d > 0) & (bg > 0) & (above > FG_MARGIN_CM)
        # Where one surface is in front of another (a child's shoulder in front of a parent's
        # side, seen from above) the depth jumps; cut the blobs apart there.
        jump = np.zeros(d.shape, bool)
        jx = np.abs(np.diff(d, axis=1)) > JUMP_CM
        jy = np.abs(np.diff(d, axis=0)) > JUMP_CM
        jump[:, 1:] |= jx
        jump[:, :-1] |= jx
        jump[1:, :] |= jy
        jump[:-1, :] |= jy
        labels, n_lab = ndimage.label(cand & ~jump, structure=np.ones((3, 3), bool))
        if n_lab == 0:
            return "nobody", 0.0, 0.0, 0.0, 0
        sizes = np.bincount(labels.ravel(), minlength=n_lab + 1)
        h_img, w_img = d.shape
        bw, bd, bx, by = self.box

        # Each blob is one person (or a few touching). Measure each blob's top of head from
        # the whole blob; the blob counts if that top is inside the box.
        best = None            # (height, hx, hy, n, u0, v0)
        reaching_in = False    # a blob whose head is outside the box but whose body is inside
        for lab in np.flatnonzero(sizes >= self.min_pixels):
            if lab == 0:
                continue
            ys, xs = np.nonzero(labels == lab)
            dd = d[ys, xs]
            vals = above[ys, xs]
            n = len(ys)
            k = min(self.top_k, n)
            top = np.argpartition(vals, n - k)[n - k:]
            # The highest pixels can sit in two places (a crawler's hips and shoulders). Use
            # the densest group of them, so their middle can't land on the lower part between.
            tu, tv = xs[top].astype(np.float32), ys[top].astype(np.float32)
            r_cl = TOP_CLUSTER_CM * fx / float(np.median(dd[top]))
            near = (tu[:, None] - tu[None, :]) ** 2 + (tv[:, None] - tv[None, :]) ** 2 <= r_cl ** 2
            seed = int(np.argmax(near.sum(axis=1) + vals[top] * 1e-6))   # most neighbors (tie: highest)
            top = top[near[seed]]
            u0, v0 = float(np.median(xs[top])), float(np.median(ys[top]))
            head_d = float(np.median(dd[top]))
            r_px = max(1.5, self.head_patch_cm * fx / head_d)
            patch = (xs - u0) ** 2 + (ys - v0) ** 2 <= r_px ** 2
            height = float(np.median(vals[patch])) if patch.sum() >= 5 else float(np.median(vals[top]))
            hx = (u0 - ppx) / fx * head_d
            hy = (v0 - ppy) / fy * head_d
            if self.use_box:
                head_in = abs(hx - bx) <= bw / 2 and abs(hy - by) <= bd / 2
                if not head_in:
                    px = rx[xs] * dd                    # where each point really is, laterally
                    py = ry[ys] * dd
                    inside = (np.abs(px - bx) <= bw / 2) & (np.abs(py - by) <= bd / 2)
                    if int(inside.sum()) >= self.min_pixels:
                        reaching_in = True
                    continue
            if best is None or height > best[0]:
                best = (height, hx, hy, n, u0, v0)

        if best is None:
            return ("cut" if reaching_in else "nobody"), 0.0, 0.0, 0.0, 0
        height, hx, hy, n, u0, v0 = best
        # top of the head at the edge of the camera's view: may be cut off
        if (u0 < EDGE_PX or u0 > w_img - 1 - EDGE_PX or
                v0 < EDGE_PX or v0 > h_img - 1 - EDGE_PX):
            return "cut", height, hx, hy, n
        return "person", height, hx, hy, n


def ft_in(cm):
    inches = cm / 2.54
    ft = int(inches // 12)
    return f"{ft}'{inches - 12 * ft:.1f}\""


# ---------------------------------------------------------------- main loop

def run(use_box, use_serial):
    m = Measurer(use_box)
    link = Link(use_serial)
    if use_box:
        m.set_box(*m.box)
    else:
        log("no box: whole view")

    pipe, dec, scale = start_camera()
    cal_at = time.time() + START_CAL_DELAY_S   # fresh empty scene at every start
    log(f"recording the empty scene in {START_CAL_DELAY_S:.0f} s: keep the area clear")
    cal_frames, cal_n, cal_end = None, 0, 0.0
    last_print = 0.0
    told_no_bg = False
    n_frames, t_rate, rate = 0, time.time(), 0.0
    try:
        while True:
            link.ensure()
            for line in link.lines():
                if line.startswith("#BOX"):
                    try:
                        w, dpt, x, y = (float(v) for v in line.split()[1:5])
                        m.set_box(w, dpt, x, y)
                    except ValueError:
                        log(f"bad box line: {line!r}")
                elif line.startswith("#CFG"):
                    try:
                        hp, mp, tk = (float(v) for v in line.split()[1:4])
                        m.set_cfg(hp, mp, tk)
                    except ValueError:
                        log(f"bad cfg line: {line!r}")
                elif line.startswith("#CAL"):
                    cal_at = time.time() + CAL_DELAY_S
                    log(f"calibrate requested: starting in {CAL_DELAY_S:.0f} s")
                    link.send(f"#I calibrating in {CAL_DELAY_S:.0f} s: step out of view")
                elif line:
                    print("esp32| " + line, flush=True)

            try:
                d, raw = grab(pipe, dec, scale)
            except RuntimeError as e:            # frame timeout / camera dropped
                log(f"camera error ({e}), restarting")
                stop_camera(pipe)
                pipe, dec, scale = start_camera()
                continue

            now = time.time()
            n_frames += 1
            if now - t_rate >= 1.0:
                rate = n_frames / (now - t_rate)
                n_frames, t_rate = 0, now

            # calibration requested by the ESP32 (keeps sending "unknown" so it doesn't FAULT)
            if cal_at is not None and now >= cal_at:
                cal_at, cal_frames, cal_n, cal_end = None, [], 0, now + CAL_SECONDS
                log("calibrating: keep the area empty")
                link.send("#I calibrating now")
            if cal_frames is not None:
                cal_n += 1
                if cal_n % CAL_EVERY == 0:
                    cal_frames.append(d)
                link.send("#U")
                if now >= cal_end:
                    link.send("#U")                       # fresh "unknown" before the slow part
                    m.bg, summary = background_from(cal_frames)
                    link.send("#U")
                    m.rays = None
                    cal_frames = None
                    log("calibrated: " + summary)
                    link.send("#I calibrated: " + summary)
                    told_no_bg = False
                continue
            if cal_at is not None:
                link.send("#U")
                continue

            status, height, hx, hy, n = m.measure(d, raw)
            if status == "person":
                link.send(f"#H {height:.1f}")
            elif status == "nobody":
                link.send("#H 0")
            else:
                link.send("#U")
                if status == "none" and not told_no_bg and link.s:
                    link.send("#I no background yet: send 'calibrate camera'")
                    told_no_bg = True

            if now - last_print < PRINT_EVERY_S:
                continue
            last_print = now
            if status == "person":
                log(f"height {height:6.1f} cm {ft_in(height)}  head at x {hx:+5.0f} y {hy:+5.0f} cm  "
                    f"(person pixels {n:5d}, {rate:4.1f} fps)")
            elif status == "cut" and n == 0:
                log(f"unknown: someone outside the box reaching in  ({rate:4.1f} fps)")
            elif status == "cut":
                log(f"unknown: head at the edge of the view ({height:5.1f}) at x {hx:+5.0f} "
                    f"y {hy:+5.0f} cm  (person pixels {n:5d}, {rate:4.1f} fps)")
            elif status == "nobody":
                log(f"nobody          (person pixels {n:5d}, {rate:4.1f} fps)")
            elif m.bg is not None:
                log(f"frame {d.shape} != background {m.bg.shape}: recalibrate")
    except KeyboardInterrupt:
        pass
    finally:
        stop_camera(pipe)


if __name__ == "__main__":
    arg = sys.argv[1] if len(sys.argv) > 1 else ""
    if arg == "calibrate":
        calibrate_standalone()
    elif arg == "":
        run(use_box=True, use_serial=True)
    elif arg == "noserial":
        run(use_box=True, use_serial=False)
    elif arg == "nobox":
        run(use_box=False, use_serial=False)
    else:
        sys.exit(__doc__)
