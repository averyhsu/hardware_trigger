# Host setup — Alvium timestamp reader (Linux)

Setup for running `host/read_timestamps.py`, which reads per-frame timestamps
from an Allied Vision Alvium under external trigger and reports the measured
frame rate + jitter. This is the independent cross-check of the Teensy's
commanded fps (camera oscillator vs. Teensy crystal).

> **Why Linux:** Vimba X / VmbPy has no macOS build (only Windows x64, Linux
> x64, Linux ARM64). The Teensy firmware still builds on macOS via PlatformIO;
> only this host-side acquisition must run on Linux.

Target host verified for these notes: **Ubuntu 22.04.5 LTS, x86_64**, miniconda.

---

## 1. Install the Vimba X SDK / runtime (system-wide)

VmbPy is a thin wrapper over the native **VmbC** library. It loads the Vimba X
transport layers (GenTL producers) at import time, so the SDK/runtime must be
installed system-wide *before* the Python wheel is of any use. **The wheel is
not on PyPI** — `pip install vmbpy` fails with "no matching distribution." The
matching wheel ships *inside* the SDK.

1. Download the **Vimba X SDK for Linux x64** from Allied Vision
   (https://www.alliedvision.com/en/products/software/vimba-x-sdk/). The
   download is license-gated (portal/registration), so it must be fetched
   manually — it can't be scripted unattended.
2. Unpack and install. Typical layout unpacks to `VimbaX_<version>/` containing
   `cti/` (the GenTL transport layers), `api/`, and installer scripts under
   `install/` or `bin/`.
3. Register the transport layers. The SDK ships a script for this
   (e.g. `VimbaX_<version>/cti/Install_GenTL_Path.sh`) which sets
   `GENICAM_GENTL64_PATH` for your shell. Confirm it is set:
   ```bash
   echo "$GENICAM_GENTL64_PATH"      # must point at the SDK's cti/ dir
   ```
   If empty, `VmbSystem` will find **no cameras** even when one is plugged in.

## 2. USB permissions (udev)

By default a non-root user can't claim the USB3 camera. The SDK ships a udev
rule installer — run it once, then replug the camera:

```bash
sudo <VimbaX_dir>/cti/Vimba_USB/Install_USB_rules.sh   # exact path varies by version
# or, if the SDK provides it:
sudo <VimbaX_dir>/install.sh
```

Also raise the USB-FS buffer size so high-rate USB3 transfers don't drop frames:

```bash
# temporary (until reboot):
sudo sh -c 'echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb'
# permanent: add usbcore.usbfs_memory_mb=1000 to GRUB_CMDLINE_LINUX and update-grub
```

## 3. Python environment + wheel

A dedicated conda env is already created on this host (`vimbax`, Python 3.11).
VmbPy supports Python 3.10–3.14.

```bash
conda activate vimbax
# install the wheel that SHIPPED WITH THE SDK (version-matched to the runtime):
pip install <VimbaX_dir>/api/python/vmbpy-*-py3-none-manylinux_2_27_x86_64.whl
python -c "import vmbpy; print(vmbpy.__version__)"   # sanity check
```

> Prefer the SDK's bundled wheel over a GitHub-release wheel — a wheel whose
> version doesn't match the installed runtime can throw API-mismatch errors.

## 4. Connect the hardware

- **Alvium**: connect via USB3, confirm it enumerates:
  ```bash
  lsusb | grep -i allied        # Allied Vision vendor id 1ab2
  ```
- **Teensy**: flashed with the trigger firmware, powered. It appears as
  `/dev/ttyACM0`. (Only needed to *emit* triggers / read the serial heartbeat;
  the timestamp reader talks only to the camera.)
- **Wiring** (from the project handoff): Teensy `TRIG_PIN = 2` (3.3 V push-pull)
  → an Alvium **non-isolated** GPIO input. **Share ground** between Teensy and
  camera. Do **not** use an opto-isolated input.

## 5. Run

Rig settings live as **constants at the top of `host/read_timestamps.py`** —
edit them there (no command-line flags for these):

```python
TRIGGER_SOURCE = 'Line0'          # camera input wired to the Teensy TRIG_PIN
EXPOSURE_US    = 200              # camera ExposureTime (Timed mode)
OUTPUT_CSV     = 'timestamps.csv' # per-frame log, written every run (cwd-relative)
DEFAULT_FRAMES = 100
```

If unsure which line the Teensy is wired to, list what the camera exposes:

```bash
conda activate vimbax
python3 host/read_timestamps.py --list-cameras        # detected cameras (skips simulators at capture time)
python3 host/read_timestamps.py --list-lines          # show TriggerSource / LineSelector options
python3 host/read_timestamps.py 200                   # capture 200 frames -> timestamps.csv
```

Every run writes `OUTPUT_CSV` (default `timestamps.csv` in the current
directory) with columns `frame_id, timestamp_ticks, dt_s, status`. The
`timestamp_ticks` value is the **camera's** on-board clock at frame
acquisition (embedded in the frame), not the host USB-receive time. No image
files are saved — this is a timing/verification tool; record images with the
Vimba X viewer if you need them.

Expected with firmware at `fps = 11.0`:

```
measured fps    : ~11.000 Hz
interval std dev: small (microseconds) = low trigger jitter
```

## 6. Verifying fps — good → definitive

1. **Camera frame timestamps** (this script) — independent oscillator; the main
   check.
2. **Logic analyzer / scope on `TRIG_PIN`** — the ultimate arbiter; rules out
   both crystals sharing an error.
3. **Not** the Vimba X viewer fps readout — its frame *count* is exact, its
   *rate* is a host-side rolling estimate.

## 7. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `Could not load vmbpy` | SDK/runtime not installed, or wheel not in this env. §1, §3. |
| `No camera found` | `GENICAM_GENTL64_PATH` unset (§1), USB permissions (§2), or camera not powered/enumerated (`lsusb`). |
| `TriggerSource: not available` + list of options | Wrong line name — set `TRIGGER_SOURCE` (top of the script) to one of the listed values. |
| No intervals / hangs then times out | No trigger edges arriving: check wiring, **shared ground**, and that the Teensy is running (serial heartbeat at 115200). |
| Many `[INCOMPLETE]` frames | USB bandwidth / dropped transfers — raise `usbfs_memory_mb` (§2). |

## 8. Future: absolute (UTC) timestamps

The crystal floor is ~±10–30 ppm (~1–2.6 s/day). For absolute time either feed
a GPS PPS into the Teensy (also disciplines out the ppm) or move to a GigE
Alvium and use PTP. USB3 cannot do PTP.
