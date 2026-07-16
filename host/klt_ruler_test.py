#!/usr/bin/env python3
"""
KLT visual-odometry ground-truth check against a ruler in the field of view.

Straight-line motion only (pure translation): stream N frames, run KLT frame-to-frame,
log per-frame displacement, and accumulate the total displacement first->last frame.
Saves the FIRST and LAST frames as viewable RGB images so you can read the ruler and
compare the measured total against the real-world distance.

Units are made explicit:
  - KLT runs on a 2x2 Bayer->gray image (half resolution), so raw shifts are in
    half-res pixels; everything reported here is scaled to FULL-RES sensor pixels (x2).
  - Set MM_PER_PIXEL (mm per full-res pixel, from your ruler) to also get millimetres.

Sign: displacement is IMAGE-space feature motion. Camera motion is the opposite sign
(camera pans right -> features move left -> dx negative).

Requires (vimbax env): vmbpy, numpy, opencv-python.
Run (Teensy triggering, Vimba X Viewer closed):  python3 host/klt_ruler_test.py [num_frames]
"""

import queue
import statistics
import sys
import threading
import time

import numpy as np
import cv2

# ============================================================================
#  CONFIGURATION
# ============================================================================
TRIGGER_SOURCE       = 'Line0'
EXPOSURE_US          = 2000          # tune for a sharp, readable ruler (shorter = less motion blur)
GAIN                 = 'max'
PIXEL_FORMAT         = 'BayerRG8'
THROUGHPUT_LIMIT_BPS = 450_000_000
BUFFER_COUNT         = 50
NUM_FRAMES           = 500
FRAME_TIMEOUT_MS     = 2000

MM_PER_PIXEL         = None          # <-- FILL IN: mm per FULL-RES pixel (from the ruler). None -> pixels only

MAX_CORNERS          = 300
QUALITY_LEVEL        = 0.01
MIN_DISTANCE         = 7
RESEED_MIN_FEATURES  = 80
LK_WIN               = (21, 21)
LK_MAXLEVEL          = 3

FULLRES_SCALE        = 2             # 2x2 Bayer->gray halves resolution; x2 back to full-res px
BAYER_TO_BGR         = cv2.COLOR_BayerBG2BGR  # validated correct for this Alvium (GenICam BayerRG8)
OUTPUT_CSV           = 'klt_ruler.csv'
FIRST_PNG            = 'frame_first.png'
LAST_PNG             = 'frame_last.png'
# ============================================================================


def load_vmbpy():
    try:
        from vmbpy import VmbSystem, VmbFeatureError, FrameStatus
    except (ImportError, OSError) as exc:
        sys.exit(f"Could not load vmbpy ({exc}). See host/SETUP.md.")
    globals().update(VmbSystem=VmbSystem, VmbFeatureError=VmbFeatureError,
                     FrameStatus=FrameStatus)


def try_set(cam, name, value):
    try:
        cam.get_feature_by_name(name).set(value)
        print(f"  {name} = {value}")
        return True
    except (VmbFeatureError, AttributeError):
        print(f"  {name}: not available (skipped)")
        return False


def try_range(cam, name):
    try:
        return cam.get_feature_by_name(name).get_range()
    except (VmbFeatureError, AttributeError):
        return None


def try_run(cam, *names):
    for name in names:
        try:
            cam.get_feature_by_name(name).run()
            return
        except (VmbFeatureError, AttributeError):
            continue


def is_simulator(cam):
    return 'Simulator' in (cam.get_model() or '')


def describe(cam):
    return f"{cam.get_id()}  {cam.get_model()}  (serial {cam.get_serial()})"


def select_camera(cams):
    real = [c for c in cams if not is_simulator(c)]
    if len(real) == 1:
        return real[0]
    if not real:
        sys.exit("Only simulator cameras detected — no real Alvium found.")
    sys.exit(f"Multiple real cameras: {', '.join(c.get_id() for c in real)}")


def apply_gain(cam):
    if GAIN is None:
        return
    try_set(cam, 'GainAuto', 'Off')
    try_set(cam, 'GainSelector', 'All')
    if GAIN == 'max':
        rng = try_range(cam, 'Gain')
        if rng is not None:
            try_set(cam, 'Gain', rng[1])
    else:
        try_set(cam, 'Gain', float(GAIN))


def configure(cam):
    print("Configuring external trigger (FrameStart / Timed) + payload:")
    try_set(cam, 'PixelFormat', PIXEL_FORMAT)
    try_set(cam, 'DeviceLinkThroughputLimitMode', 'On')
    try_set(cam, 'DeviceLinkThroughputLimit', int(THROUGHPUT_LIMIT_BPS))
    try_set(cam, 'AcquisitionMode', 'Continuous')
    try_set(cam, 'TriggerSelector', 'FrameStart')
    try_set(cam, 'TriggerSource', TRIGGER_SOURCE)
    try_set(cam, 'TriggerActivation', 'RisingEdge')
    try_set(cam, 'ExposureMode', 'Timed')
    try_set(cam, 'ExposureTime', float(EXPOSURE_US))
    apply_gain(cam)
    try_set(cam, 'AcquisitionFrameRateEnable', False)
    try_set(cam, 'TriggerMode', 'On')


def bayer2gray(raw):
    """2x2 Bayer-cell average -> half-res grayscale (mosaic-free)."""
    a = raw[0::2, 0::2].astype(np.uint16)
    a += raw[0::2, 1::2]
    a += raw[1::2, 0::2]
    a += raw[1::2, 1::2]
    return (a >> 2).astype(np.uint8)


def main():
    num_frames = NUM_FRAMES
    if len(sys.argv) > 1:
        try:
            num_frames = int(sys.argv[1])
        except ValueError:
            sys.exit(f"usage: {sys.argv[0]} [num_frames]")

    load_vmbpy()

    q = queue.Queue(maxsize=256)
    stop = threading.Event()
    stats = {'recv': 0, 'incomplete': 0, 'backlog_drops': 0}

    def handler(cam, stream, frame):
        stats['recv'] += 1
        if frame.get_status() != FrameStatus.Complete:
            stats['incomplete'] += 1
        raw = np.squeeze(frame.as_numpy_ndarray()).copy()
        try:
            q.put_nowait((frame.get_id(), raw))
        except queue.Full:
            stats['backlog_drops'] += 1
        cam.queue_frame(frame)

    records = []          # (idx, frame_id, dx_full, dy_full, n_tracked, cum_dx, cum_dy)
    klt_ms = []
    hold = {'first_raw': None, 'last_raw': None}
    lk = dict(winSize=LK_WIN, maxLevel=LK_MAXLEVEL,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    gf = dict(maxCorners=MAX_CORNERS, qualityLevel=QUALITY_LEVEL,
              minDistance=MIN_DISTANCE, blockSize=7)

    def worker():
        prev_gray = prev_pts = None
        cum_dx = cum_dy = 0.0
        idx = 0
        while len(records) < num_frames:
            try:
                item = q.get(timeout=0.2)
            except queue.Empty:
                if stop.is_set():
                    break
                continue
            if item is None:
                break
            fid, raw = item
            if hold['first_raw'] is None:
                hold['first_raw'] = raw
            hold['last_raw'] = raw

            t0 = time.perf_counter()
            gray = bayer2gray(raw)
            dxf = dyf = 0.0
            n_tracked = 0
            if prev_gray is not None and prev_pts is not None and len(prev_pts) > 0:
                nxt, st, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_pts, None, **lk)
                if nxt is not None:
                    st = st.reshape(-1).astype(bool)
                    gn = nxt.reshape(-1, 2)[st]
                    go = prev_pts.reshape(-1, 2)[st]
                    n_tracked = int(len(gn))
                    if n_tracked > 0:
                        d = np.median(gn - go, axis=0)
                        dxf = float(d[0]) * FULLRES_SCALE      # half-res px -> full-res px
                        dyf = float(d[1]) * FULLRES_SCALE
                    prev_pts = gn.reshape(-1, 1, 2)
                else:
                    prev_pts = None
            if prev_pts is None or len(prev_pts) < RESEED_MIN_FEATURES:
                p = cv2.goodFeaturesToTrack(gray, **gf)
                if p is not None:
                    prev_pts = p
            prev_gray = gray

            cum_dx += dxf
            cum_dy += dyf
            records.append((idx, fid, dxf, dyf, n_tracked, cum_dx, cum_dy))
            klt_ms.append((time.perf_counter() - t0) * 1e3)
            idx += 1

    w = threading.Thread(target=worker, name="klt", daemon=True)

    with VmbSystem.get_instance() as vmb:
        cams = vmb.get_all_cameras()
        if not cams:
            sys.exit("No camera found. See host/SETUP.md.")
        cam = select_camera(cams)
        with cam:
            print(f"Camera: {describe(cam)}")
            configure(cam)
            try_run(cam, 'TimestampReset', 'GevTimestampControlReset')
            w.start()
            print(f"\nCapturing {num_frames} frames, KLT straight-line displacement. "
                  "Ctrl-C to stop early...\n")
            cam.start_streaming(handler=handler, buffer_count=BUFFER_COUNT)
            try:
                last_n, last_t = 0, time.monotonic()
                while len(records) < num_frames:
                    time.sleep(0.02)
                    n = len(records)
                    if n > last_n:
                        last_n, last_t = n, time.monotonic()
                    elif (time.monotonic() - last_t) * 1000 > FRAME_TIMEOUT_MS:
                        print("Timed out waiting for frames — are triggers arriving?")
                        break
            except KeyboardInterrupt:
                print("(stopped early)")
            finally:
                cam.stop_streaming()

    stop.set()
    q.put(None)
    w.join(timeout=5)

    save_frame(hold['first_raw'], FIRST_PNG, "first")
    save_frame(hold['last_raw'], LAST_PNG, "last")
    report(records, klt_ms, stats)


def save_frame(raw, path, label):
    if raw is None:
        print(f"({label} frame not captured)")
        return
    bgr = cv2.cvtColor(raw, BAYER_TO_BGR)          # full-res demosaic for viewing
    cv2.imwrite(path, bgr)
    print(f"saved {label} frame -> {path}  ({bgr.shape[1]}x{bgr.shape[0]})")


def report(records, klt_ms, stats):
    n = len(records)
    if not n:
        print("\nNo frames processed — did any triggers arrive?")
        return
    cum_dx, cum_dy = records[-1][5], records[-1][6]
    tot_mag = (cum_dx**2 + cum_dy**2) ** 0.5

    print("\n==== KLT ruler test ====")
    print(f"frames processed    : {n}   received {stats['recv']}, "
          f"incomplete {stats['incomplete']}, backlog {stats['backlog_drops']}")
    if klt_ms:
        s = sorted(klt_ms)
        print(f"KLT time (ms)       : mean {statistics.mean(s):.2f}  "
              f"p95 {s[int(0.95*len(s))]:.2f}  max {max(s):.2f}")
    print("\n---- TOTAL displacement, first -> last frame (full-res sensor pixels) ----")
    print(f"  total dx = {cum_dx:+.2f} px   total dy = {cum_dy:+.2f} px   "
          f"|total| = {tot_mag:.2f} px")
    if MM_PER_PIXEL is not None:
        print(f"  total dx = {cum_dx*MM_PER_PIXEL:+.3f} mm   "
              f"total dy = {cum_dy*MM_PER_PIXEL:+.3f} mm   "
              f"|total| = {tot_mag*MM_PER_PIXEL:.3f} mm   (MM_PER_PIXEL={MM_PER_PIXEL})")
    else:
        print("  (set MM_PER_PIXEL to also report millimetres)")
    print("  NOTE: this is image-space feature motion; camera motion is the opposite sign.")

    # clean per-frame log to CSV (all frames)
    with open(OUTPUT_CSV, 'w') as f:
        hdr = "idx,frame_id,dx_px,dy_px,n_tracked,cum_dx_px,cum_dy_px"
        if MM_PER_PIXEL is not None:
            hdr += ",dx_mm,dy_mm,cum_dx_mm,cum_dy_mm"
        f.write(hdr + "\n")
        for idx, fid, dxf, dyf, ntr, cdx, cdy in records:
            row = f"{idx},{fid},{dxf:.4f},{dyf:.4f},{ntr},{cdx:.4f},{cdy:.4f}"
            if MM_PER_PIXEL is not None:
                m = MM_PER_PIXEL
                row += f",{dxf*m:.5f},{dyf*m:.5f},{cdx*m:.5f},{cdy*m:.5f}"
            f.write(row + "\n")
    print(f"\nwrote per-frame log -> {OUTPUT_CSV}  ({n} rows)")
    print(f"open {FIRST_PNG} and {LAST_PNG} and read the ruler to check the total above.")


if __name__ == '__main__':
    main()
