# hardware_trigger

Hardware frame-trigger for an **Allied Vision Alvium** camera, driven by a
**Teensy 4.1**. The MCU emits a precise trigger edge per frame; the camera
timestamps each captured frame on its own clock. Part of the Kyron
DataCollection effort.

**Goal:** exact, stable frame rate and **per-frame timestamps** we can trust
for future data collection — not the smoothed fps estimate the Vimba X viewer
shows.

---

## Handoff context (read this first)

This section is a handoff note so work can continue on a **Linux** machine
(Claude Code doesn't sync between devices). It captures where things stand as of
**2026-07-14**.

### What we're trying to achieve
1. Trigger the Alvium at an exact, drift-free frame rate from the Teensy.
2. Record a reliable per-frame timestamp for each image.
3. **Verify** the real frame rate independently of the Vimba X viewer, whose
   fps readout is a host-side rolling estimate (unreliable) even though its
   frame *count* is exact.

### Why the acquisition host must be Linux
The Allied Vision **Vimba X SDK / VmbPy has no macOS build** — only Windows x64,
Linux x64, and Linux ARM64. On macOS, `pip install vmbpy` fails with "no matching
distribution." Development started on an Apple Silicon Mac; the camera-acquisition
host (running `host/read_timestamps.py`) is moving to Linux. The Teensy firmware
still builds fine on macOS via PlatformIO — only the host-side Python is affected.

### State of things
- **Firmware builds and works** — hardware is wired, pictures are taken.
- **Timer accuracy fix applied** (`src/frame_start.cpp`): the frame period is now
  passed to `IntervalTimer` as a **float** (`1000000.0f / fps`) instead of being
  truncated to whole microseconds. On Teensy 4.x the PIT is clocked from the
  24 MHz crystal (~41.7 ns/tick), so 11/20/30/60 fps land on an exact tick count.
  The old integer cast added an accumulating drift (e.g. 60 fps → 16666 µs →
  60.0024 Hz).
- Current rate: `fps = 11.0` in `src/frame_start.cpp`.
- **Not yet done:** run `host/read_timestamps.py` on Linux to confirm the measured
  inter-frame interval matches the commanded fps.

### Immediate next steps on the Linux box
1. Install VmbPy (see [Host setup](#host-side-timestamp-reader-linux)).
2. Set `TRIGGER_SOURCE` in `host/read_timestamps.py` to the camera input line
   the Teensy is wired to (the same `TriggerSource` used in the Vimba X viewer).
3. Run `python3 host/read_timestamps.py 200` and check `measured fps` ≈ 11.000
   and that the interval std-dev (jitter) is small.

---

## Hardware / wiring
- **Board:** Teensy 4.1 (PlatformIO `board = teensy41`).
- **Trigger pin:** `TRIG_PIN = 2`, idle **low**, one clean **rising edge** per
  frame = "start exposing now" (~10 µs pulse; width does not set exposure).
- **To the camera:** Teensy GPIO is **3.3 V push-pull**. Wire it to an Alvium
  **non-isolated** GPIO input (3.3 V logic, 33–63 kΩ internal pull-ups, absolute
  max +5.5 V). **Do not** wire it to an opto-isolated input (those expect a
  higher-voltage, current-driven signal).
- **Share ground** between the Teensy and the camera — a floating ground is the
  usual "trigger does nothing" cause.

## Firmware (`src/`)
Two mutually exclusive PlatformIO environments (`platformio.ini`):

| File | Mode | Camera `ExposureMode` | Notes |
|------|------|----------------------|-------|
| `frame_start.cpp` | **Timed** | `Timed` | One rising edge per frame; camera owns exposure via `ExposureTime`. **This is the primary/current file.** |
| `pulse_width.cpp` | **TriggerWidth** | `TriggerWidth` | Line-HIGH duration sets exposure (`EXPOSURE_US`, `TriggerActivation = LevelHigh`). |

Build/upload:
```bash
pio run -e frame_start -t upload      # Timed mode (primary)
# pio run -e pulse_width -t upload    # TriggerWidth mode
```
Serial heartbeat (115200 baud) prints a live count and exact long-run average:
`triggers/sec: 11  (total 330, avg 11.0000 Hz)`. This validates the period math,
but rides the same crystal — the camera timestamps are the independent check.

## Camera-side config (Vimba X, Timed mode)
Set these on the camera (the `host/read_timestamps.py` script also applies them):
```
AcquisitionMode   = Continuous
TriggerSelector   = FrameStart
TriggerMode       = On
TriggerSource     = Line0        # <-- the non-isolated input wired to TRIG_PIN
TriggerActivation = RisingEdge
ExposureMode      = Timed
ExposureTime      = <us>         # camera enforces exposure length
```

## Host-side timestamp reader (Linux)
`host/read_timestamps.py` opens the first camera, applies the trigger config,
resets the camera timestamp clock, then prints each frame's timestamp, the
interval to the previous frame, and a summary (mean interval, **measured fps**,
jitter). Because the timestamp comes from the camera's own oscillator, agreement
with the commanded fps is an independent cross-check of the Teensy.

Install VmbPy (no PyPI wheel — use the Vimba X install or the GitHub release):
```bash
conda create -n vimbax python=3.11 && conda activate vimbax   # VmbPy supports 3.10–3.14
# Linux x64:
pip install vmbpy-1.2.2-py3-none-manylinux_2_27_x86_64.whl
# Linux ARM64 (e.g. Jetson):
# pip install vmbpy-1.2.2-py3-none-manylinux_2_27_aarch64.whl
python -c "import vmbpy; print(vmbpy.__version__)"            # sanity check
```
The Vimba X **runtime/SDK must be installed system-wide** (it provides the native
transport layers VmbPy loads). Then:
```bash
python3 host/read_timestamps.py 200      # capture 200 frames
```

## Verifying fps — from good to definitive
1. **Camera frame timestamps** (`read_timestamps.py`) — independent oscillator; the
   main check.
2. **Logic analyzer / scope on `TRIG_PIN`** — the ultimate arbiter; rules out both
   crystals sharing an error.
3. Not the Vimba X viewer fps stat (host estimate) — its frame count is exact, its
   rate readout is not.

## Future: absolute (UTC) timestamps
The crystal floor is ~±10–30 ppm (~1–2.6 s/day). For absolute time, either feed a
**GPS PPS** into the Teensy (also disciplines out the ppm), or move to a **GigE**
Alvium and use **PTP**. USB3 can't do PTP.
