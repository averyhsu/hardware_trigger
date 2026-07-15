#!/usr/bin/env python3
"""
Sustained 100 fps stream + real-time BayerRG8->RGB8 conversion throughput test.

Step 1 of the perception pipeline: prove the front end holds up under a *continuous*
external-trigger stream where **every** frame is demosaiced to RGB8 in real time, for a
long run — with zero dropped frames and conversion keeping pace at >=100 Hz. This is not a
burst-and-save; it mirrors the real design (light acquisition callback -> bounded queue ->
parallel convert worker) and discards converted frames after timing them (an inference feed
does not archive raw frames).

`cv2.cvtColor` releases the GIL, so the convert worker thread runs genuinely parallel to the
vmbpy acquisition thread.

Requires (in the vimbax env):  pip install numpy opencv-python
Run (with the Teensy triggering at 100 fps):  python3 host/stream_convert_test.py
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
TRIGGER_SOURCE       = 'Line0'       # camera input wired to the Teensy TRIG_PIN
EXPOSURE_US          = 200           # at 100 fps the period is 10 ms -> can raise toward ~9000 for light
GAIN                 = 'max'         # 'max' -> Gain range maximum; a float -> that dB; None -> leave as-is
PIXEL_FORMAT         = 'BayerRG8'    # capture raw mosaic; demosaic to RGB8 on the host
THROUGHPUT_LIMIT_BPS = 450_000_000   # DeviceLinkThroughputLimit (18e6..450e6 on the U-240c)
BUFFER_COUNT         = 50            # deep async buffer pool for a long continuous run
DURATION_S           = 60            # sustained test length; bump (e.g. 1800) for a soak test
QUEUE_MAX            = 256           # bounded hand-off; Full == convert falling behind
NUM_WORKERS          = 1            # convert worker threads (raise if 1 can't hold 100 Hz)
SAVE_SAMPLE          = 'sample_rgb.png'  # save ONE converted frame to eyeball color/gain; None to skip
# ============================================================================


def load_vmbpy():
    """Import vmbpy into module globals, or exit with a helpful message."""
    try:
        from vmbpy import (VmbSystem, VmbFeatureError, FrameStatus,
                           PixelFormat)
    except (ImportError, OSError) as exc:
        sys.exit(
            f"Could not load vmbpy ({exc}).\n"
            "  - Is the Vimba X SDK/runtime installed system-wide?\n"
            "  - Is the matching vmbpy wheel installed in this environment?\n"
            "See host/SETUP.md for the Linux install steps."
        )
    globals().update(VmbSystem=VmbSystem, VmbFeatureError=VmbFeatureError,
                     FrameStatus=FrameStatus, PixelFormat=PixelFormat)


# ---- small camera helpers (kept self-contained on purpose) ------------------
def try_set(cam, name, value):
    try:
        cam.get_feature_by_name(name).set(value)
        print(f"  {name} = {value}")
        return True
    except (VmbFeatureError, AttributeError):
        print(f"  {name}: not available (skipped)")
        return False


def try_get(cam, name):
    try:
        return cam.get_feature_by_name(name).get()
    except (VmbFeatureError, AttributeError):
        return None


def try_range(cam, name):
    try:
        return cam.get_feature_by_name(name).get_range()
    except (VmbFeatureError, AttributeError):
        return None


def try_run(cam, *names):
    for name in names:
        try:
            cam.get_feature_by_name(name).run()
            print(f"  ran {name}")
            return
        except (VmbFeatureError, AttributeError):
            continue
    print(f"  none of {names} available (timestamp not reset)")


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


def timestamp_hz(cam):
    for name in ('DeviceTimestampFrequency', 'GevTimestampTickFrequency'):
        v = try_get(cam, name)
        if v is not None:
            return float(v)
    return 1e9  # U3V default: nanoseconds


def apply_gain(cam):
    """Maximize (or set) analog gain — for low-light bring-up. Adds noise."""
    if GAIN is None:
        return
    try_set(cam, 'GainAuto', 'Off')
    try_set(cam, 'GainSelector', 'All')
    if GAIN == 'max':
        rng = try_range(cam, 'Gain')
        if rng is not None:
            try_set(cam, 'Gain', rng[1])
            print(f"  (Gain range was {rng[0]:.2f}..{rng[1]:.2f})")
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
    try_set(cam, 'AcquisitionFrameRateEnable', False)  # external trigger drives FrameStart
    try_set(cam, 'TriggerMode', 'On')                  # enable last


# ---- Bayer color-code calibration (GenICam vs OpenCV naming differ) ---------
# Candidate OpenCV codes that demosaic an 8-bit Bayer mosaic to RGB.
_BAYER_CANDIDATES = [
    ('COLOR_BayerRG2RGB', cv2.COLOR_BayerRG2RGB),
    ('COLOR_BayerBG2RGB', cv2.COLOR_BayerBG2RGB),
    ('COLOR_BayerGR2RGB', cv2.COLOR_BayerGR2RGB),
    ('COLOR_BayerGB2RGB', cv2.COLOR_BayerGB2RGB),
]


def calibrate_bayer_code(raw, sdk_rgb_reference):
    """Pick the OpenCV Bayer code whose RGB output best matches the SDK's own demosaic."""
    best_name, best_code, best_err = None, None, None
    for name, code in _BAYER_CANDIDATES:
        try:
            got = cv2.cvtColor(raw, code)
        except cv2.error:
            continue
        if got.shape != sdk_rgb_reference.shape:
            continue
        err = float(np.mean(np.abs(got.astype(np.int16)
                                   - sdk_rgb_reference.astype(np.int16))))
        if best_err is None or err < best_err:
            best_name, best_code, best_err = name, code, err
    return best_name, best_code, best_err


def main():
    load_vmbpy()

    duration = DURATION_S
    if len(sys.argv) > 1:                 # optional: python stream_convert_test.py <seconds>
        try:
            duration = float(sys.argv[1])
        except ValueError:
            sys.exit(f"usage: {sys.argv[0]} [duration_seconds]")

    q = queue.Queue(maxsize=QUEUE_MAX)
    stop = threading.Event()

    # callback-thread state (only the acquisition callback touches these -> no lock)
    stats = {'recv': 0, 'incomplete': 0, 'backlog_drops': 0, 'qhw': 0,
             'first_id': None, 'last_id': None, 'first_ts': None, 'last_ts': None}
    calib = {'code': None, 'name': None, 'err': None, 'raw0': None, 'ref0': None}

    def handler(cam, stream, frame):
        fid = frame.get_id()
        ts = frame.get_timestamp()
        ok = frame.get_status() == FrameStatus.Complete
        if stats['first_id'] is None:
            stats['first_id'], stats['first_ts'] = fid, ts
            # one-time: SDK-correct RGB reference for color calibration
            try:
                ref = frame.convert_pixel_format(PixelFormat.Rgb8).as_numpy_ndarray()
                calib['ref0'] = np.ascontiguousarray(ref).copy()
            except Exception as exc:  # noqa: BLE001 - best effort
                print(f"  (SDK reference convert failed: {exc}; will use default code)")
        stats['last_id'], stats['last_ts'] = fid, ts
        stats['recv'] += 1
        if not ok:
            stats['incomplete'] += 1
        raw = np.squeeze(frame.as_numpy_ndarray()).copy()   # copy BEFORE requeue
        if calib['raw0'] is None:
            calib['raw0'] = raw
            _resolve_calibration(calib)   # sets calib['code'] before frame 0 is dequeued
        try:
            q.put_nowait((fid, raw))
            stats['qhw'] = max(stats['qhw'], q.qsize())
        except queue.Full:
            stats['backlog_drops'] += 1
        cam.queue_frame(frame)

    # convert workers
    convert_ms = []          # combined per-frame convert times
    convert_count = [0]
    sample_holder = {'rgb': None}
    lock = threading.Lock()

    def worker():
        local_ms = []
        n = 0
        while True:
            try:
                item = q.get(timeout=0.2)
            except queue.Empty:
                if stop.is_set():
                    break
                continue
            if item is None:
                break
            _fid, raw = item
            code = calib['code'] or cv2.COLOR_BayerRG2RGB
            t0 = time.perf_counter()
            rgb = cv2.cvtColor(raw, code)          # BayerRG8 -> RGB8, the work under test
            local_ms.append((time.perf_counter() - t0) * 1e3)
            n += 1
            if SAVE_SAMPLE and sample_holder['rgb'] is None:
                with lock:
                    if sample_holder['rgb'] is None:
                        sample_holder['rgb'] = rgb
        with lock:
            convert_ms.extend(local_ms)
            convert_count[0] += n

    workers = [threading.Thread(target=worker, name=f"convert-{i}")
               for i in range(NUM_WORKERS)]

    with VmbSystem.get_instance() as vmb:
        cams = vmb.get_all_cameras()
        if not cams:
            sys.exit("No camera found. See host/SETUP.md.")
        cam = select_camera(cams)
        with cam:
            print(f"Camera: {describe(cam)}")
            configure(cam)
            print("Resetting camera timestamp clock:")
            try_run(cam, 'TimestampReset', 'GevTimestampControlReset')
            hz = timestamp_hz(cam)

            for w in workers:
                w.start()

            print(f"\nStreaming for {duration:g}s at the trigger rate, converting every "
                  f"frame BayerRG8->RGB8 ({NUM_WORKERS} worker(s), {BUFFER_COUNT} buffers).")
            print("Ctrl-C to stop early...\n")
            cam.start_streaming(handler=handler, buffer_count=BUFFER_COUNT)
            t_start = time.monotonic()
            try:
                while time.monotonic() - t_start < duration:
                    time.sleep(0.25)
            except KeyboardInterrupt:
                print("(stopped early)")
            finally:
                cam.stop_streaming()
                elapsed = time.monotonic() - t_start

    # drain + stop workers
    stop.set()
    for _ in workers:
        q.put(None)
    for w in workers:
        w.join()

    report(stats, calib, convert_ms, convert_count[0], elapsed, hz, sample_holder)


def _resolve_calibration(calib):
    if calib['ref0'] is not None and calib['raw0'] is not None:
        name, code, err = calibrate_bayer_code(calib['raw0'], calib['ref0'])
        calib['name'], calib['code'], calib['err'] = name, code, err
    if calib['code'] is None:
        calib['code'] = cv2.COLOR_BayerRG2RGB
        calib['name'] = 'COLOR_BayerRG2RGB (default; SDK ref unavailable)'


def report(stats, calib, convert_ms, converted, elapsed, hz, sample_holder):
    recv = stats['recv']
    span_s = ((stats['last_ts'] - stats['first_ts']) / hz) if recv > 1 else 0.0
    recv_fps = (recv - 1) / span_s if span_s > 0 else 0.0
    id_span = (stats['last_id'] - stats['first_id'] + 1) if recv else 0
    dropped = id_span - recv if recv else 0
    conv_fps = converted / elapsed if elapsed > 0 else 0.0

    print("\n==== stream + convert report ====")
    print(f"duration (wall)     : {elapsed:.2f} s")
    print(f"Bayer code          : {calib['name']}"
          + (f"  (match err {calib['err']:.2f})" if calib.get('err') is not None else ""))
    print(f"received frames     : {recv}   (ids {stats['first_id']}..{stats['last_id']})")
    print(f"dropped (id gaps)   : {dropped}")
    print(f"incomplete frames   : {stats['incomplete']}")
    print(f"backlog drops       : {stats['backlog_drops']}   (convert-behind, queue full)")
    print(f"queue high-water    : {stats['qhw']} / {QUEUE_MAX}")
    print(f"received fps        : {recv_fps:.3f} Hz   (camera timestamps)")
    print(f"converted frames    : {converted}")
    print(f"converted fps       : {conv_fps:.3f} Hz")
    if convert_ms:
        s = sorted(convert_ms)
        p = lambda q: s[min(len(s) - 1, int(q * len(s)))]
        print(f"convert time (ms)   : mean {statistics.mean(s):.2f}  p50 {p(0.50):.2f}  "
              f"p95 {p(0.95):.2f}  max {max(s):.2f}   (budget 10.0 ms @100 fps)")

    ok = (dropped == 0 and stats['incomplete'] == 0 and stats['backlog_drops'] == 0
          and converted >= recv - NUM_WORKERS and conv_fps >= 0.98 * recv_fps
          and recv_fps > 0)
    verdict = "PASS" if ok else "CHECK"
    print(f"\nVERDICT: {verdict} — {converted} converted / {recv} received, "
          f"{dropped} dropped, convert p95 "
          f"{(sorted(convert_ms)[int(0.95*len(convert_ms))] if convert_ms else 0):.2f} ms "
          f"-> {'sustains' if ok else 'does NOT cleanly sustain'} the stream rate.")

    if SAVE_SAMPLE and sample_holder['rgb'] is not None:
        # our arrays are RGB; cv2.imwrite expects BGR
        cv2.imwrite(SAVE_SAMPLE, cv2.cvtColor(sample_holder['rgb'], cv2.COLOR_RGB2BGR))
        print(f"\nsaved one converted frame -> {SAVE_SAMPLE} (check color/brightness)")


if __name__ == '__main__':
    main()
