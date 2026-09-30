#!/usr/bin/env bash
# Build and install libbladeRF from source into an isolated prefix (macOS).
#
# Why: Homebrew's libbladerf bottle predates libbladeRF 2.6.0, which FX3
# firmware v2.6.0's larger USB framing (2048 samples/message, up from 512)
# requires. The old host stack then mis-frames the stream and delivers 4x
# the configured sample rate (all cores saturated, no packets decoded).
#
# Mirrors packaging/build-liquid-macos.sh: same --version/--prefix/--jobs
# flags, -headerpad + install_name_tool -id fixup so bundle-macos.sh can
# vendor the dylib into the .dmg. Release CI builds this before configuring
# supertooth (see .github/workflows/release.yml):
#   sudo bash packaging/build-bladerf-macos.sh --prefix /opt/bladerf
#   PKG_CONFIG_PATH=/opt/bladerf/lib/pkgconfig:$PKG_CONFIG_PATH \
#     cmake -DCMAKE_PREFIX_PATH=/opt/bladerf:... ...
#
# Usage:
#   bash packaging/build-bladerf-macos.sh [--version 2025.10] [--prefix DIR] [--jobs N]
#
# Defaults:
#   --version  2025.10  (upstream release containing libbladeRF v2.6.0,
#              FX3 firmware v2.6.0, FPGA bitstream v0.16.0)
#   --prefix   /opt/bladerf
#   --jobs     sysctl -n hw.ncpu
#
# Requirements: git, cmake, pkg-config, cc, make, libusb
#   (brew install git cmake pkg-config libusb ncurses)
#
# If the Homebrew libbladerf formula is installed, uninstall it first --
# this script installs the same library version family and the two can
# shadow each other via pkg-config / CMAKE_PREFIX_PATH:
#   brew uninstall libbladerf
# To restore stock Homebrew libbladerf afterwards:
#   brew install libbladerf
set -euo pipefail

VERSION="2025.10"
PREFIX="/opt/bladerf"
JOBS="$(sysctl -n hw.ncpu 2>/dev/null || echo 4)"
REPO_URL="https://github.com/Nuand/bladeRF.git"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --version) VERSION="$2"; shift 2 ;;
        --prefix) PREFIX="$2"; shift 2 ;;
        --jobs) JOBS="$2"; shift 2 ;;
        -h|--help)
            sed -n '2,34p' "$0"; exit 0 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

for tool in git cmake cc make; do
    command -v "$tool" >/dev/null || { echo "Error: missing required tool: $tool"; exit 1; }
done
if ! command -v pkg-config >/dev/null && ! command -v pkgconf >/dev/null; then
    echo "Error: missing required tool: pkg-config (install with: brew install pkg-config)"
    exit 1
fi

# Refuse to collide with a Homebrew-installed bottle of the same library.
if [[ -L "$PREFIX/lib/libbladeRF.dylib" ]] && [[ "$(readlink "$PREFIX/lib/libbladeRF.dylib")" == *Cellar* ]]; then
    echo "Error: Homebrew's libbladerf bottle owns $PREFIX/lib/libbladeRF.dylib."
    echo "Uninstall it first: brew uninstall libbladerf"
    exit 1
fi
if [[ -e "$PREFIX/lib/libbladeRF.dylib" ]]; then
    echo "Warning: $PREFIX already contains libbladeRF; reinstalling over it."
fi

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

echo "=== Cloning bladeRF $VERSION ==="
git clone --depth 1 --branch "$VERSION" "$REPO_URL" "$WORK/bladeRF"

# Patch upstream -Werror failure on newer Clang (>=17):
# host/utilities/bladeRF-cli/src/cmd/flash_image.c uses
#   if (val[i] >= 'a' || val[i] <= 'f')
# which is always true (overlapping comparison) and fails the build with
# -Werror,-Wtautological-overlap-compare. The intent is clearly a lowercase
# hex range check, i.e. &&. Fix it in place.
python3 - "$WORK/bladeRF/host/utilities/bladeRF-cli/src/cmd/flash_image.c" <<'PY'
import sys
path = sys.argv[1]
with open(path) as f:
    src = f.read()
old = "if (val[i] >= 'a' || val[i] <= 'f')"
new = "if (val[i] >= 'a' && val[i] <= 'f')"
if src.count(old) == 1:
    with open(path, "w") as f:
        f.write(src.replace(old, new))
    print("Patched flash_image.c tautological compare (|| -> &&)")
elif src.count(new) >= 1:
    print("flash_image.c already fixed upstream, skipping patch")
else:
    raise SystemExit(f"patch target not found in {path}")
PY

echo "=== Building (prefix=$PREFIX, jobs=$JOBS) ==="
cmake -S "$WORK/bladeRF/host" -B "$WORK/bladeRF/host/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$PREFIX" \
    -DCMAKE_INSTALL_RPATH="$PREFIX/lib"
# -headerpad keeps install_name_tool -id working on the installed dylib.
cmake --build "$WORK/bladeRF/host/build" -j"$JOBS" -- -headerpad_max_install_names 2>/dev/null || \
    cmake --build "$WORK/bladeRF/host/build" -j"$JOBS"
cmake --install "$WORK/bladeRF/host/build"

DYLIB="$PREFIX/lib/libbladeRF.dylib"
if [[ -f "$DYLIB" ]]; then
    install_name_tool -id "$DYLIB" "$DYLIB"
    codesign --force --sign - "$DYLIB" 2>/dev/null || true
fi

echo ""
echo "Custom libbladeRF installed:"
ls -l "$PREFIX/lib"/libbladeRF* 2>/dev/null || true
if command -v pkg-config >/dev/null; then
    PKG_CONFIG_PATH="$PREFIX/lib/pkgconfig:${PKG_CONFIG_PATH:-}" \
        pkg-config --modversion libbladeRF 2>/dev/null || true
fi
echo "(restore stock with: brew install libbladerf)"
