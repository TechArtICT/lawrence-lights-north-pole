#!/usr/bin/env python3
"""North Pole height sensor: Rock64 + Intel RealSense D435i, camera looking straight down.

Step 4 (terminal only, nothing sent to the ESP32 yet).

  ~/rs/bin/python height.py calibrate   # nobody in view: records the empty scene
  ~/rs/bin/python height.py             # prints the height of a person inside the standing box
  ~/rs/bin/python height.py nobox       # same, but the whole view counts (for finding the spot)

How it works:
  calibrate: ~3 s of depth frames, per-pixel median = empty scene (floor, pole, sign).
             Saved to background.npy next to this script.
  run:       a pixel is "person" if it is more than FG_MARGIN_CM closer than the empty scene
             and inside the standing box. Fewer than MIN_PIXELS person pixels = nobody.
             The TOP_K highest person pixels locate the top of the head; height = median of the
             person pixels within HEAD_PATCH_CM of that spot (height above the empty scene).
             RealSense depth is distance from the camera plane, not along the ray, so a head
             off-center reads the same as one dead center.
"""
import os
import sys
import time
import warnings

import numpy as np
import pyrealsense2 as rs

# ---- settings (later these come from the ESP32) ----
BOX_W_CM = 100.0       # standing box size along the image x axis
BOX_D_CM = 100.0       # standing box size along the image y axis
BOX_X_CM = 0.0         # box center, cm from the point directly under the camera (image x)
BOX_Y_CM = 0.0         # box center, cm from the point directly under the camera (image y)
FG_MARGIN_CM = 10.0    # must be this far above the empty scene to count as person
MIN_PIXELS = 150       # fewer person pixels than this = nobody
TOP_K = 40             # the highest this-many person pixels locate the top of the head
HEAD_PATCH_CM = 3.0    # height = median of person pixels within this radius of that spot
PRINT_EVERY_S = 0.2

# ---- camera ----
WIDTH, HEIGHT, FPS = 640, 480, 30   # works on USB 2
DECIMATE = 2                        # 640x480 -> 320x240 (less CPU, fills small holes)
SETTLE_FRAMES = 30                  # let auto-exposure settle before calibrating
CAL_SECONDS = 3.0

BG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "background.npy")


def log(msg):
    print(time.strftime("%H:%M:%S ") + msg, flush=True)


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


def calibrate():
    pipe, dec, scale = start_camera()
    try:
        for _ in range(SETTLE_FRAMES):
            grab(pipe, dec, scale)
        log(f"calibrating: keep the area empty for {CAL_SECONDS:.0f} s")
        frames = []
        t0 = time.time()
        while time.time() - t0 < CAL_SECONDS:
            d, _ = grab(pipe, dec, scale)
            frames.append(d)
    finally:
        pipe.stop()

    stack = np.stack(frames)
    stack[stack == 0] = np.nan
    valid_frac = np.mean(~np.isnan(stack), axis=0)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")          # all-empty pixels give a harmless warning
        bg = np.nanmedian(stack, axis=0)
    bg[(valid_frac < 0.5) | np.isnan(bg)] = 0
    np.save(BG_FILE, bg.astype(np.float32))

    h, w = bg.shape
    log(f"saved {BG_FILE}")
    log(f"{len(frames)} frames, {w}x{h}, center {bg[h // 2, w // 2]:.1f} cm, "
        f"{100 * np.mean(bg > 0):.0f}% of pixels have a reading")


def build_mask(bg, raw, use_box):
    fx, fy, ppx, ppy = intrinsics_for(bg.shape, raw)
    h, w = bg.shape
    u = np.arange(w, dtype=np.float32)[None, :]
    v = np.arange(h, dtype=np.float32)[:, None]
    x = (u - ppx) / fx * bg          # cm on the floor from the point under the camera
    y = (v - ppy) / fy * bg
    m = bg > 0
    if use_box:
        m &= (np.abs(x - BOX_X_CM) <= BOX_W_CM / 2) & (np.abs(y - BOX_Y_CM) <= BOX_D_CM / 2)
        log(f"standing box {BOX_W_CM:.0f} x {BOX_D_CM:.0f} cm centered at "
            f"x {BOX_X_CM:+.0f} y {BOX_Y_CM:+.0f} cm: {int(m.sum())} pixels")
    else:
        log(f"no box: whole view, {int(m.sum())} pixels")
    return m, (fx, fy, ppx, ppy)


def ft_in(cm):
    inches = cm / 2.54
    ft = int(inches // 12)
    return f"{ft}'{inches - 12 * ft:.1f}\""


def run(use_box):
    if not os.path.exists(BG_FILE):
        sys.exit("no background yet: run  ~/rs/bin/python height.py calibrate")
    bg = np.load(BG_FILE)
    pipe, dec, scale = start_camera()
    mask = None
    last_print = 0.0
    n_frames, t_rate, rate = 0, time.time(), 0.0
    try:
        while True:
            try:
                d, raw = grab(pipe, dec, scale)
            except RuntimeError as e:            # frame timeout / camera dropped
                log(f"camera error ({e}), restarting")
                try:
                    pipe.stop()
                except RuntimeError:
                    pass
                pipe, dec, scale = start_camera()
                continue

            if d.shape != bg.shape:
                sys.exit(f"frame {d.shape} != background {bg.shape}: recalibrate")
            if mask is None:
                mask, (fx, fy, ppx, ppy) = build_mask(bg, raw, use_box)

            n_frames += 1
            now = time.time()
            if now - t_rate >= 1.0:
                rate = n_frames / (now - t_rate)
                n_frames, t_rate = 0, now
            if now - last_print < PRINT_EVERY_S:
                continue
            last_print = now

            above = bg - d                        # cm above the empty scene
            person = mask & (d > 0) & (above > FG_MARGIN_CM)
            n = int(person.sum())
            if n < MIN_PIXELS:
                log(f"nobody          (person pixels {n:5d}, {rate:4.1f} fps)")
                continue

            vals = above[person]
            ys, xs = np.nonzero(person)
            # find the top of the head: the highest TOP_K pixels
            k = min(TOP_K, n)
            top = np.argpartition(vals, n - k)[n - k:]
            u0, v0 = float(np.median(xs[top])), float(np.median(ys[top]))
            head_d = float(np.median(d[ys[top], xs[top]]))
            # measure it: median of all person pixels within HEAD_PATCH_CM of that spot.
            # (the median of only the highest pixels would read high by the noise)
            r_px = max(1.5, HEAD_PATCH_CM * fx / head_d)
            patch = (xs - u0) ** 2 + (ys - v0) ** 2 <= r_px ** 2
            if patch.sum() >= 5:
                height = float(np.median(vals[patch]))
            else:
                height = float(np.median(vals[top]))
            hx = (u0 - ppx) / fx * head_d
            hy = (v0 - ppy) / fy * head_d
            log(f"height {height:6.1f} cm {ft_in(height)}  head at x {hx:+5.0f} y {hy:+5.0f} cm  "
                f"(person pixels {n:5d}, {rate:4.1f} fps)")
    except KeyboardInterrupt:
        pass
    finally:
        try:
            pipe.stop()
        except RuntimeError:
            pass


if __name__ == "__main__":
    arg = sys.argv[1] if len(sys.argv) > 1 else ""
    if arg == "calibrate":
        calibrate()
    elif arg in ("", "nobox"):
        run(use_box=(arg != "nobox"))
    else:
        sys.exit(__doc__)
