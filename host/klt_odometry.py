#!/usr/bin/env python3
"""
100 Hz KLT visual odometry, stamped with the Teensy exposure timestamp.

Fast-loop tracking primitive for the laser-weeding pipeline: stream BayerRG8 under
external trigger, run KLT (good-features + pyramidal Lucas-Kanade) frame-to-frame to
measure pixel displacement, and stamp every measurement with the TEENSY's per-trigger
exposure time — the system's master clock (Teensy owns trigger/gyro/fire; all time is
Teensy time). Emits one fast-loop record per frame: (frame_id, teensy_t_exposure_us,
dx, dy, n_tracked).

Time join (robust to USB drops):
  - Teensy firmware prints "T <seq> <t_us>" per trigger (see src/frame_start.cpp).
  - Each camera frame carries frame.get_id() (device frame counter; gaps on USB drop).
  - Bookmark at stream start: id0 = first frame id, seq0 = (latest Teensy seq at arm)+1.
    Per frame: teensy_seq = (get_id() - id0) + seq0  ->  t_exposure = teensy_times[seq].
    A dropped frame leaves an id gap that maps to the right seq, so time can't desync.
  NOTE: the absolute id<->seq bootstrap can be off by +-1 frame (arm/trigger phase). That
  does NOT affect displacement or dt (both relative); it only matters once fusing with the
  gyro clock, which is a later step. Reported as a caveat, not fixed here.

Requires (vimbax env): vmbpy, numpy, opencv-python, pyserial (all installed).
Run (Teensy flashed + triggering at 100 fps, Vimba X Viewer closed):
    python3 host/klt_odometry.py [duration_seconds]
"""

import queue
import statistics
import sys
import threading
import time

import numpy as np
import cv2
import serial

# ============================================================================
#  CONFIGURATION
# ============================================================================
TRIGGER_SOURCE       = 'Line0'
EXPOSURE_US          = 2000          # brighter than 200 us so KLT has visible texture; tune to your light
GAIN                 = 'max'         # 'max' -> Gain range maximum; a float -> dB; None -> leave
PIXEL_FORMAT         = 'BayerRG8'
THROUGHPUT_LIMIT_BPS = 450_000_000
BUFFER_COUNT         = 50
DURATION_S           = 30
QUEUE_MAX            = 256

SERIAL_PORT          = '/dev/ttyACM0'  # Teensy USB serial ("T <seq> <t_us>" lines)
SERIAL_BAUD          = 115200

MAX_CORNERS          = 300           # goodFeaturesToTrack cap
QUALITY_LEVEL        = 0.01
MIN_DISTANCE         = 7
RESEED_MIN_FEATURES  = 80            # re-seed features when inliers fall below this
LK_WIN               = (21, 21)
LK_MAXLEVEL          = 3

FULLRES_SCALE        = 2             # 2x2 Bayer->gray halves resolution; x2 -> full-res sensor px
MM_PER_PIXEL         = None          # <-- FILL IN: mm per FULL-RES pixel (from calibration). None -> px only
OUTPUT_CSV           = 'klt.csv'
# ============================================================================


def load_vmbpy():
    try:
        from vmbpy import VmbSystem, VmbFeatureError, FrameStatus
    except (ImportError, OSError) as exc:
        sys.exit(f"Could not load vmbpy ({exc}). See host/SETUP.md.")
    globals().update(VmbSystem=VmbSystem, VmbFeatureError=VmbFeatureError,
                     FrameStatus=FrameStatus)


# ---- small camera helpers (self-contained) ----------------------------------
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


# ---- Teensy serial reader ---------------------------------------------------
class TeensyClock:
    """Reads 'T <seq> <t_us>' lines in a thread; maps seq -> exposure micros."""
    def __init__(self, port, baud):
        self.times = {}
        self.latest_seq = None
        self._stop = threading.Event()
        self._ser = None
        try:
            self._ser = serial.Serial(port, baud, timeout=0.5)
        except Exception as exc:  # noqa: BLE001
            print(f"  WARNING: could not open {port} ({exc}). "
                  "Frames will NOT be Teensy-stamped — flash the firmware and check the port.")

    def start(self):
        self._t = threading.Thread(target=self._run, name="teensy-serial", daemon=True)
        self._t.start()

    def _run(self):
        if self._ser is None:
            return
        while not self._stop.is_set():
            try:
                line = self._ser.readline().decode('ascii', 'replace').strip()
            except Exception:  # noqa: BLE001
                continue
            if not line.startswith('T '):
                continue
            parts = line.split()
            if len(parts) == 3:
                try:
                    seq, t_us = int(parts[1]), int(parts[2])
                except ValueError:
                    continue
                self.times[seq] = t_us          # dict set is GIL-atomic
                self.latest_seq = seq

    def stop(self):
        self._stop.set()
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass


def bayer2gray(raw):
    """2x2 Bayer-cell average -> half-res grayscale (mosaic-free, cheap)."""
    a = raw[0::2, 0::2].astype(np.uint16)
    a += raw[0::2, 1::2]
    a += raw[1::2, 0::2]
    a += raw[1::2, 1::2]
    return (a >> 2).astype(np.uint8)


def main():
    duration = DURATION_S
    if len(sys.argv) > 1:
        try:
            duration = float(sys.argv[1])
        except ValueError:
            sys.exit(f"usage: {sys.argv[0]} [duration_seconds]")

    load_vmbpy()

    teensy = TeensyClock(SERIAL_PORT, SERIAL_BAUD)
    teensy.start()

    q = queue.Queue(maxsize=QUEUE_MAX)
    stop = threading.Event()
    stats = {'recv': 0, 'incomplete': 0, 'backlog_drops': 0, 'qhw': 0}
    bookmark = {'arm_seq': None}

    def handler(cam, stream, frame):
        ok = frame.get_status() == FrameStatus.Complete
        stats['recv'] += 1
        if not ok:
            stats['incomplete'] += 1
        raw = np.squeeze(frame.as_numpy_ndarray()).copy()
        try:
            q.put_nowait((frame.get_id(), raw))
            stats['qhw'] = max(stats['qhw'], q.qsize())
        except queue.Full:
            stats['backlog_drops'] += 1
        cam.queue_frame(frame)

    # ---- KLT consumer (single, serial chain frame N vs N-1) ----
    records = []          # (fid, teensy_seq, t_us_or_None, dx, dy, n_tracked)
    klt_ms = []
    lk_params = dict(winSize=LK_WIN, maxLevel=LK_MAXLEVEL,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    gf_params = dict(maxCorners=MAX_CORNERS, qualityLevel=QUALITY_LEVEL,
                     minDistance=MIN_DISTANCE, blockSize=7)

    def klt_worker():
        prev_gray = None
        prev_pts = None
        id0 = None
        seq0 = None
        while True:
            try:
                item = q.get(timeout=0.2)
            except queue.Empty:
                if stop.is_set():
                    break
                continue
            if item is None:
                break
            fid, raw = item
            t0 = time.perf_counter()
            gray = bayer2gray(raw)

            if id0 is None:
                id0 = fid
                seq0 = (bookmark['arm_seq'] + 1) if bookmark['arm_seq'] is not None else None
            teensy_seq = ((fid - id0) + seq0) if seq0 is not None else None
            t_us = teensy.times.get(teensy_seq) if teensy_seq is not None else None

            dx = dy = 0.0
            n_tracked = 0
            if prev_gray is not None and prev_pts is not None and len(prev_pts) > 0:
                next_pts, st, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_pts,
                                                           None, **lk_params)
                if next_pts is not None:
                    st = st.reshape(-1).astype(bool)
                    good_new = next_pts.reshape(-1, 2)[st]
                    good_old = prev_pts.reshape(-1, 2)[st]
                    n_tracked = int(len(good_new))
                    if n_tracked > 0:
                        d = np.median(good_new - good_old, axis=0)
                        dx = float(d[0]) * FULLRES_SCALE   # half-res px -> full-res sensor px
                        dy = float(d[1]) * FULLRES_SCALE
                    prev_pts = good_new.reshape(-1, 1, 2)
                else:
                    prev_pts = None

            # (re)seed features when sparse
            if prev_pts is None or len(prev_pts) < RESEED_MIN_FEATURES:
                p = cv2.goodFeaturesToTrack(gray, **gf_params)
                if p is not None:
                    prev_pts = p
            prev_gray = gray

            records.append((fid, teensy_seq, t_us, dx, dy, n_tracked))
            klt_ms.append((time.perf_counter() - t0) * 1e3)

    worker = threading.Thread(target=klt_worker, name="klt", daemon=True)

    with VmbSystem.get_instance() as vmb:
        cams = vmb.get_all_cameras()
        if not cams:
            sys.exit("No camera found. See host/SETUP.md.")
        cam = select_camera(cams)
        with cam:
            print(f"Camera: {describe(cam)}")
            configure(cam)
            try_run(cam, 'TimestampReset', 'GevTimestampControlReset')
            worker.start()
            print(f"\nStreaming {duration:g}s @ trigger rate; KLT + Teensy-time join. "
                  "Ctrl-C to stop early...\n")
            cam.start_streaming(handler=handler, buffer_count=BUFFER_COUNT)
            bookmark['arm_seq'] = teensy.latest_seq   # Teensy seq at arm -> first frame = +1
            t_start = time.monotonic()
            try:
                while time.monotonic() - t_start < duration:
                    time.sleep(0.25)
            except KeyboardInterrupt:
                print("(stopped early)")
            finally:
                cam.stop_streaming()
                elapsed = time.monotonic() - t_start

    stop.set()
    q.put(None)
    worker.join(timeout=5)
    teensy.stop()

    report(stats, records, klt_ms, elapsed)


def report(stats, records, klt_ms, elapsed):
    n = len(records)
    joined = sum(1 for r in records if r[2] is not None)
    tracked = [r[5] for r in records]
    dxs = [r[3] for r in records]
    dys = [r[4] for r in records]
    # id-contiguity anomalies
    fids = [r[0] for r in records]
    gaps = sum(1 for a, b in zip(fids, fids[1:]) if b - a != 1) if n > 1 else 0

    print("\n==== KLT odometry report ====")
    print(f"duration (wall)     : {elapsed:.2f} s")
    print(f"received frames     : {stats['recv']}")
    print(f"KLT-processed       : {n}")
    print(f"incomplete frames   : {stats['incomplete']}")
    print(f"backlog drops       : {stats['backlog_drops']}   (KLT-behind, queue full)")
    print(f"queue high-water    : {stats['qhw']} / {QUEUE_MAX}")
    print(f"processed fps       : {n / elapsed if elapsed > 0 else 0:.3f} Hz")
    print(f"Teensy-time joined  : {joined}/{n}  ({100*joined/n if n else 0:.1f}%)")
    print(f"frame-id gaps       : {gaps}   (USB drops; time-join handles them)")
    if tracked:
        print(f"features tracked    : mean {statistics.mean(tracked):.0f}  min {min(tracked)}")
    if dxs:
        unit = "mm" if MM_PER_PIXEL is not None else "full-res px"
        sc = MM_PER_PIXEL if MM_PER_PIXEL is not None else 1.0
        print(f"per-frame disp ({unit}): |dx| mean {statistics.mean(map(abs,dxs))*sc:.4f}  "
              f"|dy| mean {statistics.mean(map(abs,dys))*sc:.4f}")
    if klt_ms:
        s = sorted(klt_ms)
        p = lambda qq: s[min(len(s)-1, int(qq*len(s)))]
        print(f"KLT time (ms)       : mean {statistics.mean(s):.2f}  p50 {p(.5):.2f}  "
              f"p95 {p(.95):.2f}  max {max(s):.2f}   (budget 10.0 ms @100 fps)")

    with open(OUTPUT_CSV, 'w') as f:
        # dx_px/dy_px are FULL-RES sensor pixels; dx_mm/dy_mm present only if MM_PER_PIXEL set
        hdr = "frame_id,teensy_seq,teensy_t_us,dx_px,dy_px,n_tracked"
        if MM_PER_PIXEL is not None:
            hdr += ",dx_mm,dy_mm"
        f.write(hdr + "\n")
        for fid, seq, t_us, dx, dy, ntr in records:
            row = (f"{fid},{'' if seq is None else seq},{'' if t_us is None else t_us},"
                   f"{dx:.4f},{dy:.4f},{ntr}")
            if MM_PER_PIXEL is not None:
                row += f",{dx*MM_PER_PIXEL:.5f},{dy*MM_PER_PIXEL:.5f}"
            f.write(row + "\n")
    print(f"wrote {OUTPUT_CSV}  ({n} rows)")

    p95 = (sorted(klt_ms)[int(0.95*len(klt_ms))] if klt_ms else 0)
    ok = (stats['backlog_drops'] == 0 and n >= stats['recv'] - 1
          and p95 < 10.0 and (joined / n if n else 0) >= 0.98)
    print(f"\nVERDICT: {'PASS' if ok else 'CHECK'} — {n} processed / {stats['recv']} received, "
          f"KLT p95 {p95:.2f} ms, {100*joined/n if n else 0:.0f}% Teensy-stamped.")


if __name__ == '__main__':
    main()
