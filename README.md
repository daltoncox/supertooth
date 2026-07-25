<div align="center">
  <h1>Supertooth</h1>

  <picture style="display: inline-block;">
    <source media="(prefers-color-scheme: dark)" srcset="imgs/Behold_Supertooth.png">
    <source media="(prefers-color-scheme: light)" srcset="imgs/Behold_Supertooth.png">
    <img alt="Supertooth logo" src="imgs/Behold_Supertooth.png" width="180">
  </picture>
</div>

Supertooth is a C-based software-defined radio (SDR) project for receiving and decoding Bluetooth traffic with a HackRF.

It includes four runtime binaries:

1. `supertooth`: Qt GUI application with live BR/EDR + BLE capture, spectrum view, and packet log.
2. `supertooth-bredr`: BR/EDR multichannel receiver with piconet tracking.
3. `supertooth-ble`: BLE advertising capture/decoder on a selected advertising channel (37/38/39), channelized from a wideband capture.
4. `supertooth-hybrid`: simultaneous BR/EDR multichannel + BLE advertising processing from a shared stream.

## Install

Pre-built `.deb` packages are available on the [releases page](https://github.com/daltoncox/supertooth/releases). Download the latest package for your architecture and install it:

```bash
sudo apt install ./supertooth_*.deb
supertooth          # GUI
supertooth-bredr --help    # CLI
```

The package bundles Qt 6.8, radio libs, and QML modules — no extra runtime dependencies beyond glibc/libstdc++. The GUI binary is wrapped in a thin launcher at `/usr/bin/supertooth` that sets plugin and QML import paths before exec'ing the real binary in `/usr/lib/supertooth/`.

## Building from source

### Prerequisites

| Dependency | Purpose |
|---|---|
| `libhackrf` | HackRF device API |
| `liquid-dsp` | Channelization, filtering, NCO mixing, GFSK/CPFSK demodulation |
| `libbtbb` | BR/EDR access-code workflows and piconet UAP/clock recovery |

### CLI-only build (no GUI)

Linux (Debian-based):

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config \
  hackrf libhackrf-dev libbtbb-dev libliquid-dev
```

macOS (Homebrew):

```bash
brew install cmake pkg-config hackrf liquid-dsp libbtbb
```

### GUI build (adds Qt 6.8)

The GUI requires **Qt 6.8+** with Quick, QuickLayouts, and Graphs.  Because most distro Qt packages are too old, either use the **official Qt Installer** (gui installer or `aqtinstall`) or the project's convenience download helper:

```bash
python3 -m venv /tmp/qt-venv
/tmp/qt-venv/bin/pip install requests py7zr
sudo /tmp/qt-venv/bin/python packaging/install-qt.py \
  --version 6.8.0 \
  --modules qtgraphs qtquick3d qtshadertools \
  --output /opt/Qt/6.8.0/gcc_64
```

The helper fetches prebuilt Qt archives from `download.qt.io` and is meant for CI/quick-setup — run it with `sudo` if writing to a system path like `/opt`.

macOS users can install Qt via Homebrew (6.8+):

```bash
brew install qt@6
```

## Build

```bash
cd supertooth
mkdir build
cd build
cmake ..
make

# With GUI (point CMAKE_PREFIX_PATH at the Qt installation)
cmake .. \
  -DCMAKE_PREFIX_PATH=/opt/Qt/6.8.0/gcc_64 \
  -DBUILD_GUI=ON
make
```

Output binaries are in `build/src/apps/cli/` (CLI) and `build/src/apps/gui/` (GUI).

## Run

All binaries require a HackRF:

```bash
./build/src/apps/cli/supertooth-ble --view full
./build/src/apps/cli/supertooth-bredr --view full
./build/src/apps/cli/supertooth-hybrid --view full
./build/src/apps/gui/supertooth
```

`supertooth-bredr`, `supertooth-ble`, and `supertooth-hybrid` accept `--help` for runtime flags.  The GUI needs a Wayland or X11 display.

## Architecture

### Source layout

```text
src/
  apps/cli/        CLI binaries (supertooth-bredr, -ble, -hybrid) + shared cli_common
  apps/gui/        Qt GUI application
  core/
    dsp/           Shared DSP utilities (RSSI measurement helpers)
    models/        Shared packet and receive metadata types
    radio/         HackRF integration, sample dispatcher, block pool
    service/       Session API, channel processors, hybrid orchestration
    protocol/
      ble/         BLE bitstream decoder, codec, display utilities
      bredr/       BR/EDR bitstream decoder, codec, piconet tracking, UAP recovery
```

### Key design points

- **Core library** (`libsupertooth_core.a`): all DSP, radio I/O, protocol decoding, and session logic live here.  CLI apps and the GUI link against this library — no code duplication.
- **Decoder state machines**: per-channel/thread processor owns its decoder context.  Caller pulls decoded packets immediately after push-status signals readiness.
- **Frame-vs-packet split**: bitstream decoders emit raw frames (`ble_frame_t` / `bredr_frame_t`); codec layer decodes into clean semantic packet models.  Service callbacks carry the frame (not the decoded packet), keeping layers decoupled.
- **BR/EDR channel layout** avoids DC by centering LO at `-(N/2 - 0.5) × channel_bw`.
- **Piconet tracking** is centralized through `bredr_piconet_store_add_packet()` — runtime binaries never duplicate UAP/clock logic.
