<div align="center">
  <h1>Supertooth</h1>

  <picture style="display: inline-block;">
    <source media="(prefers-color-scheme: dark)" srcset="imgs/Behold_Supertooth.png">
    <source media="(prefers-color-scheme: light)" srcset="imgs/Behold_Supertooth.png">
    <img alt="Supertooth logo" src="imgs/Behold_Supertooth.png" width="180">
  </picture>
</div>

Supertooth is a C-based software-defined radio (SDR) project for receiving and decoding Bluetooth traffic with a HackRF.

It includes three runtime binaries:

1. `supertooth-rx`: BR/EDR multichannel receiver with piconet tracking.
2. `supertooth-btle`: BLE advertising capture/decoder on channel 37 (2.402 GHz).
3. `supertooth-hybrid`: simultaneous BR/EDR multichannel + BLE channel 37 processing from a shared stream.

## Prerequisites

- CMake 3.10+
- C compiler (C11)
- `libhackrf`
- `liquid-dsp`
- `libbtbb`
- pthreads (system)

On macOS (Homebrew), the CMake files prioritize `/opt/homebrew` and `/usr/local`.

### Core dependencies (what they do)

- `liquid-dsp`: DSP primitives used for channelization, filtering, NCO mixing, and GFSK/CPFSK demodulation.
- `libhackrf`: HackRF device API used by runtime binaries and live examples to configure and stream SDR samples.
- `libbtbb`: Bluetooth baseband helpers used for BR/EDR access-code workflows and piconet UAP/clock recovery.

### Install dependencies

Linux (Debian-Based):

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config \
  hackrf libhackrf-dev libbtbb-dev libliquid-dev
```

If your distro package name for Liquid-DSP differs, install the equivalent dev package that provides `liquid/liquid.h` and `libliquid`.

macOS (Homebrew):

```bash
brew update
brew install cmake pkg-config hackrf liquid-dsp libbtbb
```

## Build

From repository root:

```bash
mkdir build
cmake ..
make
```

To build the examples run:

```bash
make examples
```

Output binaries are in:

- `build/src/` (main executables)
- `build/examples/` (example tools)

## Run

Main binaries (require HackRF hardware):

```bash
./build/src/supertooth-btle --view full
./build/src/supertooth-rx --view full
./build/src/supertooth-hybrid --view full
```

`supertooth-rx` supports runtime options:

```bash
./build/src/supertooth-rx --help
```

`supertooth-btle` and `supertooth-hybrid` also support:

```bash
--view full|summary
--debug
```

## Basic architecture

### Signal processing runtimes (`src/`)

- `supertooth-btle.c`: HackRF callback → I/Q normalization → Liquid-DSP GFSK demodulation → `ble_push_bit()`.
- `supertooth-rx.c`: wideband capture across configurable BR/EDR channel span; per-channel NCO mixing + FIR decimation + CPFSK demod; packet/piconet handling.
- `supertooth-hybrid.c`: combined BR/EDR channel workers plus BLE channel 37 worker using shared buffer generations.

### Protocol decoders

- `ble_phy.c/h`: per-channel BLE push-bit processor (preamble/AA detect, collect, dewhiten, CRC verify, packet API).
- `bredr_phy.c/h`: BR/EDR push-bit processor (access-code detect, FEC header decode, payload collection).
- `bredr_piconet.c/h`: per-piconet tracking state and packet ring.
- `bredr_piconet_store.c/h`: LAP-keyed piconet store with libbtbb-assisted UAP/clock recovery.

### Hardware abstraction

- `hackrf.c/h`: wraps HackRF lifecycle and common RF parameter configuration.

### Examples and tools

- `examples/`: standalone tools for recording, demodulation experiments, BR/EDR scanning/detection, CSV export, and channel utilities.
