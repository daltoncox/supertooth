<div align="center">
  <h1>Supertooth</h1>

  <picture style="display: inline-block;">
    <source media="(prefers-color-scheme: dark)" srcset="imgs/Behold_Supertooth.png">
    <source media="(prefers-color-scheme: light)" srcset="imgs/Behold_Supertooth.png">
    <img alt="Supertooth logo" src="imgs/Behold_Supertooth.png" width="180">
  </picture>
</div>

Supertooth is a C-based software-defined radio (SDR) project for receiving and decoding Bluetooth traffic with a HackRF.

It includes two runtime binaries:

1. `supertooth-desktop`: Qt GUI application with live BR/EDR + BLE capture, spectrum view, and packet log.
2. `supertooth`: multiplexed CLI with one subcommand per operating mode:
   - `supertooth hybrid`: simultaneous BR/EDR multichannel + BLE advertising processing from a shared stream.
   - `supertooth ble`: BLE advertising capture/decoder over a window of LE RF channels (37/38/39), channelized from a wideband capture.
   - `supertooth bredr`: BR/EDR multichannel receiver with piconet tracking.
   - `supertooth record`: record raw IQ to a WAV file (no decoding).

## Install

### Linux

Pre-built `.deb` packages are available on the [releases page](https://github.com/daltoncox/supertooth/releases). Download the latest package for your architecture and install it:

```bash
sudo apt install ./supertooth_*.deb
supertooth-desktop  # GUI
supertooth -h       # CLI mode list
```

The package bundles Qt 6.8, radio libs, and QML modules — no extra runtime dependencies beyond glibc/libstdc++. Both binaries live in `/opt/supertooth/bin/` and are symlinked into `/usr/bin/` (`supertooth` is the multiplexed CLI, `supertooth-desktop` the GUI; plugin/QML paths come from `qt.conf` next to the GUI binary, libraries via RPATH). Desktop entry and icons install under `/usr/share/applications` and `/usr/share/icons/`.

### macOS (Apple Silicon)

Download the latest `supertooth_macos_arm64.dmg` from the [releases page](https://github.com/daltoncox/supertooth/releases), open it, and drag `Supertooth.app` to `/Applications`. The disk image is self-contained: Qt frameworks, QML imports, and the radio libraries (`hackrf`, `liquid-dsp` and their transitive dependencies) are all bundled inside the `.app` — no Homebrew packages required at runtime. Requires macOS 14+ on `arm64`.

The multiplexed CLI ships inside the bundle alongside the GUI:

```bash
/Applications/Supertooth.app/Contents/MacOS/supertooth -h
/Applications/Supertooth.app/Contents/MacOS/supertooth hybrid -h
/Applications/Supertooth.app/Contents/MacOS/supertooth ble -h
/Applications/Supertooth.app/Contents/MacOS/supertooth bredr -h
```

> **Gatekeeper note:** release builds are ad-hoc signed but not notarized, so macOS may refuse to open the app on first launch. Right-click `Supertooth.app` → Open, or run `xattr -d com.apple.quarantine /Applications/Supertooth.app`.

## Building from source

### Prerequisites

| Dependency | Purpose |
|---|---|
| `libhackrf` | HackRF device API (default backend, always in releases) |
| `libbladeRF` (`libbladerf`) | bladeRF device API (opt-in via `-DENABLE_BLADERF=ON`, not in releases) |
| `liquid-dsp` | Channelization, filtering, NCO mixing, GFSK/CPFSK demodulation |

BR/EDR UAP and CLK1-6 recovery is implemented in-tree (no external
dependency); see `src/core/protocol/bredr/bredr_clock_recovery.c`,
`src/core/protocol/bredr/bredr_registry.c`,
`src/core/protocol/ble/ble_registry.c`, and the shared helpers in
`src/core/models/rssi_tracker.c` and `src/core/service/collector.c`.

### CLI-only build (no GUI)

Linux (Debian-based):

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config \
  hackrf libhackrf-dev libliquid-dev
```

Optional bladeRF support (off by default, not shipped in releases):

```bash
sudo apt install -y libbladerf-dev
```

macOS (Homebrew):

```bash
brew install cmake pkg-config hackrf autoconf automake libtool
```

Optional bladeRF support:

```bash
brew install bladerf
```

> **Do NOT `brew install liquid-dsp`.** Its bottle is built with liquid-dsp's
> default `LIQUID_LOG_LEVEL_COMPILE=0`, which compiles a `liquid_log_trace()`
> call into every SIMD dot-product dispatcher. On Apple Silicon that costs
> ~2000% CPU and drops RF blocks in `supertooth-hybrid`. Build it from
> source with TRACE logging compiled out instead:
>
> ```bash
> bash packaging/build-liquid-macos.sh
> ```

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

Optional: `-DBUILD_TESTS=ON` enables the unit/integration suite (`make tests`,
then `ctest --output-on-failure`). Version strings come from
`-DSUPERTOOTH_VERSION=<ver>` or `git describe` at configure time. The
differential `libbtbb` oracle tests additionally need a `libbtbb/` checkout at
the repo root (it is gitignored); without it those oracle tests are skipped.

### CMake options

| Option | Build Default | In Release |
|---|---|---|
| `ENABLE_HACKRF` | `ON` | yes |
| `ENABLE_BLADERF` | `OFF` | no |
| `BUILD_GUI` | `OFF` | yes |
| `BUILD_TESTS` | `OFF` | no |

Enable bladeRF (requires `libbladeRF` installed, see Prerequisites):

```bash
cmake .. -DENABLE_BLADERF=ON
```

Probe for it: `supertooth ble -d` lists one `bladerf:<serial>` line per
connected device (serials are also shown by `bladeRF-cli -p`); select one
with `-d bladerf:<serial>`. The bladeRF 2.0 Micro sustains 61.44 Msps, so
BR/EDR defaults to 60 channels on bladeRF (20 on HackRF). A single
`-g/--gain` flag sets RX gain per radio: HackRF takes `LNA,VGA[,AMP]`
(e.g. `-g 24,18,0`; AMP defaults to off) while bladeRF takes an overall
gain in dB (e.g. `-g 30`, default 30 dB, manual gain control). A malformed
or out-of-range value prints that radio's gain help.

Output binaries are in `build/src/apps/cli/supertooth` (CLI) and `build/src/apps/gui/supertooth-desktop` (GUI, plus `Supertooth.app` on macOS).

## Run

All binaries require a HackRF by default (or a bladeRF with
`-DENABLE_BLADERF=ON` builds):

```bash
./build/src/apps/cli/supertooth ble
./build/src/apps/cli/supertooth bredr
./build/src/apps/cli/supertooth hybrid
./build/src/apps/cli/supertooth record
./build/src/apps/gui/supertooth-desktop
```

Running `supertooth` with no arguments (or `-h`) prints the mode list.
Each mode has its own help with `General Options` plus the sections that
apply to it (`BREDR Options`, `LE Options`):

```bash
./build/src/apps/cli/supertooth hybrid -h
```

CLI output defaults to `--view summary`. All decode modes also accept
`--view full|summary|devices`:

- `supertooth bredr` / `supertooth hybrid`: `--ac-errors N` sets the max
  BR/EDR access-code bit errors (default: 0, strict).
- `supertooth ble` / `supertooth hybrid`: `--enforce-crc on|off` drops BLE
  frames with bad CRC (default: on); `-c/-b` select the LE channel window.
- All decode modes: `--debug` prints a Debug Summary with per-stage drop breakdown
  (rf / sub / out with per-lane detail) plus BLE/BR-EDR frame counters.

`supertooth record` captures raw IQ straight to a WAV file in the current
directory (no decoding); it takes the BR/EDR-style `-c/-b` window plus
`-d/-g`. The GUI needs a Wayland or X11 display.

## Architecture

### Source layout

```text
src/
  apps/cli/        Multiplexed `supertooth` CLI (hybrid, ble, bredr, record modes) + shared app_common, app_summary_view, app_device_view
  apps/gui/        Qt GUI application
  core/
    dsp/           Shared DSP utilities (channelizer_bank, rssi_measurements)
    models/        Shared packet and receive metadata types (device_models, receive_event_models, phy, rssi_tracker)
    radio/         Radio integration (hackrf, bladerf, radio_common) and sample dispatcher
    service/       Session API, channel processors, channelizer_service, event collector
    protocol/
      ble/         BLE bitstream decoder, codec, registry (devices + connections), display utilities, BT assigned numbers
      bredr/       BR/EDR bitstream decoder, codec, piconet link + registry, clock recovery, display utilities
```

### Key design points

- **Core library** (`libsupertooth_core.a`): all DSP, radio I/O, protocol decoding, and session logic live here.  CLI apps and the GUI link against this library — no code duplication.
- **Decoder state machines**: per-channel/thread processor owns its decoder context.  Caller pulls decoded packets immediately after push-status signals readiness.
- **Frame-vs-packet split**: bitstream decoders emit raw frames (`ble_frame_t` / `bredr_frame_t`); codec layer decodes into clean semantic packet models.  Service callbacks carry the frame (not the decoded packet), keeping layers decoupled.
- **BR/EDR channel layout** avoids DC by centering LO at `-(N/2 - 0.5) × channel_bw`.
- **Connection tracking** is centralized through the registries (`bredr_registry_submit()` / `ble_registry_submit()`) — runtime binaries never duplicate UAP/clock/CRCInit logic. Each registry owns dynamic connection + device tables (single lock) plus a static bounded pending tally so decoder junk never allocates.
