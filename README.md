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
2. `supertooth-ble`: BLE advertising capture/decoder on a selected advertising channel (37/38/39), channelized from a wideband capture.
3. `supertooth-hybrid`: simultaneous BR/EDR multichannel + BLE advertising processing from a shared stream (advertising channel derived from the capture window).

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
- `libhackrf`: HackRF device API used by runtime binaries to configure and stream SDR samples.
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
cmake -S . -B build
cmake --build build
```

Output binaries are in:

- `build/src/` (main executables)

## Run

Main binaries (require HackRF hardware):

```bash
./build/src/supertooth-ble --view full
./build/src/supertooth-rx --view full
./build/src/supertooth-hybrid --view full
```

`supertooth-rx` supports runtime options:

```bash
./build/src/supertooth-rx --help
```

`supertooth-ble` and `supertooth-hybrid` also support:

```bash
--view full|summary
--debug
```

`supertooth-ble` additionally supports:

```bash
--ble-channel 37|38|39
```

The BLE session tunes a whole-MHz LO at the channel center and channelizes a
4 Msps capture down to 2 Msps (the same pipeline the hybrid session uses).

`supertooth-hybrid` additionally supports the same channel-window options as
`supertooth-rx` (the BLE advertising channel is derived from the window: the
single advertising channel whose center lies inside it, or the BLE worker
stays idle when the window covers none):

```bash
--channels N          # even 2-20, default 20
--bottom-channel CH   # 0-78, default 0
```

## Architecture

Supertooth is organized as a layered core library with thin CLI entrypoints.

### Source layout

```text
src/
  apps/            CLI binaries and presentation
  service/         session API and runtime orchestration
  dsp/             shared DSP utilities (RSSI measurement helpers)
  radio/           HackRF integration
  models/          shared packet and receive metadata types
  protocol/
    ble/           BLE bitstream decoder, codec helpers, assigned-number helpers
    bredr/         BR/EDR bitstream decoder, codec helpers, tracking, recovery
```

### Main layers

- `src/apps/`: `supertooth-rx`, `supertooth-ble`, and `supertooth-hybrid` user-facing binaries.
- `src/service/`: reusable library boundary built around `receiver_session`.
- `src/service/*_channel_processor.*`: per-mode sample processing, demodulation flow, and callback emission.
- `src/dsp/`: shared DSP helper utilities (currently RSSI measurement primitives).
- `src/radio/`: HackRF lifecycle and configuration wrapper.
- `src/models/`: shared receive-event wrappers such as `ble_event_t`, `bredr_event_t`, and `rx_metadata_t`.
- `src/protocol/ble/`: BLE bitstream decoding and decode support.
- `src/protocol/bredr/`: BR/EDR bitstream decoding, measurement, tracking, and recovery support.

### Frame And Packet Split

The protocol pipeline now separates captured frames from decoded packets:

- BLE bitstream decoder (`ble_bitstream_decoder.*`) produces `ble_frame_t`.
- BLE codec owns the clean decoded `ble_packet_t` model and `ble_decode_frame()`.
- Service callbacks carry `ble_event_t`, which pairs `rx_metadata_t` with a captured `ble_frame_t`.
- BR/EDR bitstream decoder (`bredr_bitstream_decoder.*`) follows the same high-level shape with `bredr_event_t` carrying `bredr_frame_t`.

This keeps capture-stage data in the bitstream decoder layer, decoded semantic packet fields in the codec layer, and display formatting in the display layer.

For a broader description of the current architecture, see `docs/architecture.md`.
