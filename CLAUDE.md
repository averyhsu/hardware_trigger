# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

The camera-trigger + fast-loop-tracking front end of a laser-weeding targeting
pipeline. Two halves that must stay in lock-step:

- **`src/` — Teensy 4.1 firmware** (PlatformIO/Arduino). Emits the hardware
  camera trigger and stamps every trigger with `micros()` on the Teensy crystal.
  **The Teensy is the master clock**: all time in the system is Teensy time.
- **`host/` — Linux Python** (conda env `vimbax`). Drives an Allied Vision
  Alvium 1800 U-240c over USB3 via Vimba X / `vmbpy`, verifies trigger timing,
  and runs the fast-loop KLT visual odometry.

`README.md` is the full pipeline architecture (detection/tracking loops, error
budget, why it only ever measures deltas). Read it before touching timing or
tracking logic — the delta-not-absolute design is load-bearing.

## Build & flash firmware

Two mutually exclusive PlatformIO environments select which `.cpp` is the trigger
firmware (`build_src_filter` picks one file; both define the same symbols, so
only one can build at a time):

- `frame_start` → `src/frame_start.cpp` — **the current design.** TIMED mode: one
  clean rising edge per frame, camera owns exposure length. Streams per-trigger
  `T <seq> <t_us>` stamps over serial for the host time-join.
- `pule_width` → `src/pulse_width.cpp` — older trigger-width mode (line-high
  duration *is* the exposure). Kept for reference.

```bash
pio run -e frame_start                 # build
pio run -e frame_start -t upload       # flash (Teensy on /dev/ttyACM0)
pio device monitor -b 115200           # watch trigger heartbeat / T-stamps
```

Change the trigger rate by editing `fps` at the top of the `.cpp`, then reflash.
The `IntervalTimer` period is kept as a float on purpose — truncating to whole
microseconds introduces accumulating fps drift.

## Run host scripts

All host scripts run in the `vimbax` conda env and require the Vimba X SDK
installed system-wide (the `vmbpy` wheel is **not on PyPI** — it ships inside the
SDK). Full one-time Linux setup — SDK install, `GENICAM_GENTL64_PATH`, udev
rules, `usbfs_memory_mb` — is in **`host/SETUP.md`**. There is no test suite; each
script is a standalone verification/bring-up tool.

```bash
conda activate vimbax
python3 host/read_timestamps.py --list-cameras   # sanity: is the Alvium seen?
python3 host/read_timestamps.py 200               # verify trigger fps/jitter (camera clock)
python3 host/stream_convert_test.py 60            # sustained stream + BayerRG8->RGB8 soak
python3 host/klt_ruler_test.py 500                # KLT vs. a physical ruler (ground truth)
python3 host/klt_odometry.py 30                   # fast-loop KLT stamped with Teensy time
```

Rig settings (trigger line, exposure, pixel format, throughput limit) are
**constants at the top of each script**, not CLI flags — edit them there. The
Vimba X Viewer must be **closed** while these run (it holds the camera).

## Conventions that matter

- **Every script is self-contained by design.** The `try_set` / `try_get` /
  `try_run` camera-feature helpers are deliberately duplicated across host
  scripts rather than shared — do not refactor them into a common module; each
  tool is meant to run alone.
- **Timing is verified against an independent oscillator.** Never trust the
  Vimba X viewer's fps readout (host-side estimate). The real checks are the
  camera's embedded frame timestamps (`read_timestamps.py`) and a scope on
  `TRIG_PIN`. See `host/SETUP.md` §6.
- **BayerRG8 is captured raw and demosaiced on the host**, not RGB8 on-camera —
  RGB8's 3× payload caps fps well below the sensor's ~126 fps ceiling. Keep
  per-frame work off the acquisition callback (it back-pressures the camera and
  silently caps the rate); hand frames to a worker via a bounded queue.
- **GenICam vs. OpenCV Bayer naming differ.** `stream_convert_test.py`
  auto-calibrates the OpenCV Bayer code against the SDK's own demosaic; the KLT
  scripts hard-code `COLOR_BayerBG2BGR`, validated for this Alvium's `BayerRG8`.
- **Firmware ISR discipline:** trigger edges run at `IntervalTimer` priority 0 so
  USB/serial can't jitter them; all printing/serial happens in `loop()`, never in
  the ISR. The stamp ring buffer is lock-free SPSC (ISR writes head, loop writes
  tail) — preserve that invariant.
