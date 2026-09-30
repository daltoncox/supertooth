<div align="center">
  <h1>Supertooth</h1>

  <picture style="display: inline-block;">
    <source media="(prefers-color-scheme: dark)" srcset="imgs/Behold_Supertooth.png">
    <source media="(prefers-color-scheme: light)" srcset="imgs/Behold_Supertooth.png">
    <img alt="Supertooth logo" src="imgs/Behold_Supertooth.png" width="180">
  </picture>
</div>

Supertooth is a C-based software-defined radio (SDR) project for observing Bluetooth traffic. It offers:

- Passive, wideband multi-channel monitoring for BR/EDR + LE
- Packet decoding for active Bluetooth connections as an outside party
- Pure host-side processing for using a HackRF or bladeRF off-the-shelf

It includes two runtime binaries:

1. `supertooth`: CLI with various modes (bredr, le, and hybrid)
2. `supertooth-desktop`: Qt GUI application with live BR/EDR + LE capture, spectrum view, and packet log.

## Install

### Linux

Pre-built `.deb` packages are available on the [releases page](https://github.com/daltoncox/supertooth/releases). Download the latest package for your architecture and install it:

```bash
sudo apt install ./supertooth_*.deb
supertooth          # CLI
supertooth-desktop  # GUI
```

The package bundles dependencies such as Qt 6.8 and the radio libraries to make use of newer features than those offered by the apt sources. As such, binaries are placed in `/opt/supertooth` and are symlinked into `bin`.

### macOS (Apple Silicon)

Download the latest `supertooth_macos_arm64.dmg` from the [releases page](https://github.com/daltoncox/supertooth/releases), open it, and drag `Supertooth.app` to `/Applications`.

The CLI ships inside the bundle alongside the GUI:

```bash
/Applications/Supertooth.app/Contents/MacOS/supertooth -h
/Applications/Supertooth.app/Contents/MacOS/supertooth-desktop
```

> **Gatekeeper note:** macOS will likely refuse to open the app on first launch. To resolve this, open Settings -> Privacy & Security, scroll to the bottom, and under Security, click `Open Anyway`.

## Building from source

### Dependencies and Build Options

| Dependency | CMake Option | Build Default | In Release |
|---|---|---|---|
| `libhackrf` | ENABLE_HACKRF | ON | yes |
| `libbladerf` | ENABLE_BLADERF | OFF | yes |
| `Qt 6.8+` | BUILD_GUI | OFF | yes |
| - | BUILD_TESTS | OFF | no |
| - | SUPERTOOTH_PORTABLE (no AVX512/AMX, keeps AVX2) | OFF | yes |

> **NOTE:** liquid-dsp is also required to build this project. However, it is automatically pulled and statically compiled by CMake into the binaries, so there is no need to install it from a package manager. 

### Minimal Dependency Install (no GUI, only HackRF)

Linux (Debian-based):

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config git \
  libhackrf-dev
```

macOS (Homebrew):

```bash
brew install cmake pkg-config hackrf
```

### GUI Dependency (needs Qt 6.8+)

The GUI requires **Qt 6.8+** with Quick, QuickLayouts, and Graphs. On certain distros, such as Ubuntu Noble (24.04) and earlier, it is not available through `apt` and must be installed from elsewhere. This could affect other applications that use Qt if you are not careful, so discretion is advised.

macOS users can use the Homebrew package:

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
```

To enable certain features, pass their corresponding arguments from the above table to CMake. For example, `cmake -DENABLE_BLADERF=ON ..`.



Output binaries are in `build/src/apps/cli/supertooth` (CLI) and `build/src/apps/gui/supertooth-desktop` (GUI, plus `Supertooth.app` on macOS).

## Run

All binaries require a HackRF or bladeRF by default:

```bash
./build/src/apps/cli/supertooth le
./build/src/apps/cli/supertooth bredr
./build/src/apps/cli/supertooth hybrid
./build/src/apps/gui/supertooth-desktop
```

Running `supertooth` with no arguments (or `-h`) prints the mode list.
Each mode has its own help with `General Options` plus the sections that
apply to it (`BREDR Options`, `LE Options`):

```bash
./build/src/apps/cli/supertooth hybrid -h
```

CLI output defaults to `--view summary`. All decode modes also accept
`--view full|summary|devices`.

## Architecture

### Source layout

```text
src/
  apps/
    cli/        CLI application code for various modes and views
    gui/        Qt GUI application
  core/
    dsp/           Shared DSP utilities 
    models/        Shared packet and receive metadata types
    radio/         Radio integration and sample dispatcher
    service/       Session API, channel processors, channelizer_service
    protocol/
      ble/         BLE bitstream decoder, codec, registry, display utilities, BT assigned numbers
      bredr/       BR/EDR bitstream decoder, codec, registry, clock recovery, display utilities
```

## References
- [Ubertooth One](https://greatscottgadgets.com/ubertoothone/) - The primary inspiration for this project (hence the name).
- [libbtbb](https://github.com/greatscottgadgets/libbtbb) - Reference for many of the algorithms.
- [liquid-dsp](https://liquidsdr.org/) - A fantastic DSP library.
