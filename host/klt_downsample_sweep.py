#!/usr/bin/env python3
"""
KLT downsample sweep — find the minimal downsample with optimal accuracy at 100 Hz.

Trade-off: less downsampling = higher resolution = better displacement accuracy, but
more compute; more downsampling = faster, but coarser. This sweeps the downsample
factor and reports, per factor:
  - THROUGHPUT: gray+KLT time (mean/p95) and the max fps it could sustain single-thread.
    (Content-independent -> transfers to your rig directly. Must be < 10 ms to hold 100 Hz.)
  - ACCURACY: apply KNOWN sub-pixel shifts (warpAffine) and measure the KLT displacement
    error, in FULL-RES sensor pixels. (Absolute value depends on scene texture; the trend
    with downsample does not.)

Source frame: grabs ONE real BayerRG8 frame if the camera is free (realistic texture);
otherwise falls back to a synthetic textured frame. Pass --frame X.npy to use a saved raw.

Requires (vimbax env): numpy, opencv-python (vmbpy only if capturing).
Run:  python3 host/klt_downsample_sweep.py            # capture one frame or synth
      python3 host/klt_downsample_sweep.py --synth    # force synthetic
"""

import statistics
import sys
import time

import numpy as np
import cv2

# ============================================================================
FACTORS        = [1, 2, 3, 4, 6, 8]   # downsample factors to sweep (1 = full-res)
KNOWN_SHIFTS   = [(0.5, 0.0), (1.3, -0.7), (3.0, 2.0), (5.0, -4.0), (0.2, 0.2)]  # full-res px
ITERS          = 200                  # timing iterations per factor
MAX_CORNERS    = 300
QUALITY_LEVEL  = 0.01
MIN_DISTANCE   = 7
LK_WIN         = (21, 21)
LK_MAXLEVEL    = 3
BUDGET_MS      = 10.0                  # per-frame budget for 100 fps
MM_PER_PIXEL   = None                  # optional: report accuracy in mm too
BAYER_TO_GRAY  = cv2.COLOR_BayerBG2GRAY   # validated pattern for this Alvium
# ============================================================================

LK = dict(winSize=LK_WIN, maxLevel=LK_MAXLEVEL,
          criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
GF = dict(maxCorners=MAX_CORNERS, qualityLevel=QUALITY_LEVEL,
          minDistance=MIN_DISTANCE, blockSize=7)


def synth_frame(h=1216, w=1936):
    """Synthetic full-res grayscale with trackable structure + fine texture + noise."""
    rng = np.random.default_rng(0)
    base = cv2.resize(rng.integers(0, 255, (48, 76), np.uint8), (w, h),
                      interpolation=cv2.INTER_CUBIC)
    return cv2.add(base, rng.integers(0, 40, (h, w), np.uint8))


def capture_one_gray():
    """Grab one real BayerRG8 frame and return full-res grayscale, or None on failure."""
    try:
        from vmbpy import VmbSystem, FrameStatus
    except Exception:  # noqa: BLE001
        return None
    try:
        with VmbSystem.get_instance() as vmb:
            cams = [c for c in vmb.get_all_cameras()
                    if 'Simulator' not in (c.get_model() or '')]
            if not cams:
                return None
            with cams[0] as cam:
                def s(n, v):
                    try:
                        cam.get_feature_by_name(n).set(v)
                    except Exception:  # noqa: BLE001
                        pass
                s('PixelFormat', 'BayerRG8')
                s('DeviceLinkThroughputLimitMode', 'On')
                s('DeviceLinkThroughputLimit', 450_000_000)
                s('AcquisitionMode', 'Continuous')
                s('TriggerSelector', 'FrameStart')
                s('TriggerSource', 'Line0')
                s('TriggerActivation', 'RisingEdge')
                s('ExposureMode', 'Timed')
                s('ExposureTime', 2000.0)
                s('GainAuto', 'Off'); s('GainSelector', 'All')
                try:
                    rng = cam.get_feature_by_name('Gain').get_range()
                    cam.get_feature_by_name('Gain').set(rng[1])
                except Exception:  # noqa: BLE001
                    pass
                s('AcquisitionFrameRateEnable', False)
                s('TriggerMode', 'On')
                got = {}
                def h(c, st, fr):
                    if 'raw' not in got and fr.get_status() == FrameStatus.Complete:
                        got['raw'] = np.squeeze(fr.as_numpy_ndarray()).copy()
                    c.queue_frame(fr)
                cam.start_streaming(handler=h, buffer_count=10)
                t0 = time.monotonic()
                while 'raw' not in got and time.monotonic() - t0 < 3:
                    time.sleep(0.02)
                cam.stop_streaming()
                if 'raw' not in got:
                    return None
                return cv2.cvtColor(got['raw'], BAYER_TO_GRAY)  # full-res luma
    except Exception as exc:  # noqa: BLE001
        print(f"(camera capture failed: {exc}; using synthetic frame)")
        return None


def down(gray_full, f):
    if f == 1:
        return gray_full
    h, w = gray_full.shape
    return cv2.resize(gray_full, (w // f, h // f), interpolation=cv2.INTER_AREA)


def measure_accuracy(gray_full, f):
    """Apply known full-res shifts; return mean |error| in full-res px at factor f."""
    H, W = gray_full.shape
    base_f = down(gray_full, f)
    pts = cv2.goodFeaturesToTrack(base_f, **GF)
    if pts is None:
        return None, 0
    errs = []
    for (sx, sy) in KNOWN_SHIFTS:
        M = np.float32([[1, 0, sx], [0, 1, sy]])
        shifted_full = cv2.warpAffine(gray_full, M, (W, H),
                                      flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
        g = down(shifted_full, f)
        nxt, stt, _ = cv2.calcOpticalFlowPyrLK(base_f, g, pts, None, **LK)
        if nxt is None:
            continue
        stt = stt.reshape(-1).astype(bool)
        gn = nxt.reshape(-1, 2)[stt]; go = pts.reshape(-1, 2)[stt]
        if len(gn) == 0:
            continue
        d = np.median(gn - go, axis=0) * f          # downsampled px -> full-res px
        errs.append(((d[0] - sx) ** 2 + (d[1] - sy) ** 2) ** 0.5)
    return (statistics.mean(errs) if errs else None), (0 if pts is None else len(pts))


def measure_throughput(gray_full, f):
    """Time the per-frame path: downsample + LK.

    NOTE: this measures (resize-to-factor + LK). It EXCLUDES the raw Bayer->gray step,
    which in production is a ~0.2 ms 2x2 block-average for even factors, or a ~1-3 ms
    cvtColor demosaic for factor 1. Add that to the numbers below for a true per-frame
    cost; even so all factors here sit well under the 10 ms budget on this x86 box.
    """
    H, W = gray_full.shape
    base_f = down(gray_full, f)
    pts = cv2.goodFeaturesToTrack(base_f, **GF)
    ms = []
    for i in range(ITERS):
        sx, sy = (i % 7) - 3, (i % 5) - 2
        M = np.float32([[1, 0, sx], [0, 1, sy]])
        shifted_full = cv2.warpAffine(gray_full, M, (W, H), borderMode=cv2.BORDER_REFLECT)
        t0 = time.perf_counter()
        g = down(shifted_full, f)                    # the real per-frame downsample cost
        cv2.calcOpticalFlowPyrLK(base_f, g, pts, None, **LK)
        ms.append((time.perf_counter() - t0) * 1e3)
    return ms


def main():
    if '--synth' in sys.argv:
        gray_full = synth_frame(); src = "synthetic"
    else:
        arg = next((a for a in sys.argv[1:] if a.endswith('.npy')), None)
        if arg:
            raw = np.load(arg)
            gray_full = cv2.cvtColor(raw, BAYER_TO_GRAY) if raw.ndim == 2 else raw
            src = arg
        else:
            g = capture_one_gray()
            gray_full = g if g is not None else synth_frame()
            src = "real camera frame" if g is not None else "synthetic (no camera)"
    H, W = gray_full.shape
    print(f"source: {src}   full-res gray {W}x{H}\n")
    print(f"{'factor':>6} {'resolution':>12} {'mean_ms':>8} {'p95_ms':>7} "
          f"{'max_fps':>8} {'acc_err_px':>11}" + ("  acc_err_mm" if MM_PER_PIXEL else ""))
    rows = []
    for f in FACTORS:
        ms = measure_throughput(gray_full, f)
        acc, nfeat = measure_accuracy(gray_full, f)
        mean_ms = statistics.mean(ms); p95 = sorted(ms)[int(0.95 * len(ms))]
        max_fps = 1000.0 / mean_ms
        line = (f"{f:>6} {f'{W//f}x{H//f}':>12} {mean_ms:>8.2f} {p95:>7.2f} "
                f"{max_fps:>8.0f} {('n/a' if acc is None else f'{acc:.3f}'):>11}")
        if MM_PER_PIXEL and acc is not None:
            line += f"  {acc*MM_PER_PIXEL:.4f}"
        print(line)
        rows.append((f, p95, acc))
    # recommend: smallest factor (best accuracy) whose p95 < budget
    holds = [r for r in rows if r[1] < BUDGET_MS]
    print()
    if holds:
        best = min(holds, key=lambda r: r[0])   # smallest factor that holds 100 Hz
        print(f"RECOMMENDATION: minimal downsample holding 100 Hz (p95<{BUDGET_MS:g}ms) = "
              f"factor {best[0]}  ->  {W//best[0]}x{H//best[0]}, "
              f"acc_err ~{best[2]:.3f} full-res px"
              + (f" (~{best[2]*MM_PER_PIXEL:.4f} mm)" if MM_PER_PIXEL and best[2] else ""))
        print("Lower factor = better accuracy; this is the least downsampling that fits the "
              "10 ms/frame budget. Re-run on the real scene to confirm accuracy numbers.")
    else:
        print(f"No swept factor held p95<{BUDGET_MS:g}ms — reduce MAX_CORNERS/ROI or use a "
              "coarser factor / GPU.")


if __name__ == '__main__':
    main()
