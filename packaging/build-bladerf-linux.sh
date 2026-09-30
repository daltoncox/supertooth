#!/usr/bin/env bash
# Build and install libbladeRF from source into an isolated prefix (Linux).
#
# Why: neither Ubuntu's libbladerf-dev (2.5.0) nor the Nuand PPA's 2024.07
# snapshot reaches libbladeRF 2.6.0, which FX3 firmware v2.6.0's larger USB
# framing (2048 samples/message, up from 512) requires. The old host stack
# then mis-frames the stream and delivers 4x the configured sample rate
# (all cores saturated, no packets decoded).
#
# Isolated --prefix (default /opt/bladerf), same --version/--prefix/--jobs
# flags. Release CI builds this before configuring supertooth
# (see .github/workflows/release.yml):
#   sudo bash packaging/build-bladerf-linux.sh --prefix /opt/bladerf
#   PKG_CONFIG_PATH=/opt/bladerf/lib/pkgconfig:$PKG_CONFIG_PATH \
#     cmake -DCMAKE_PREFIX_PATH=/opt/bladerf:... ...
#
# Usage:
#   sudo bash packaging/build-bladerf-linux.sh [--version 2025.10] [--prefix DIR] [--jobs N]
#
# Defaults:
#   --version  2025.10  (upstream release containing libbladeRF v2.6.0,
#              FX3 firmware v2.6.0, FPGA bitstream v0.16.0)
#   --prefix   /opt/bladerf
#   --jobs     nproc
#
# Requirements: git, cmake, cc, make, pkg-config, libusb-1.0 dev headers,
#   ncurses dev headers
#   (sudo apt install -y git cmake build-essential pkg-config
#    libusb-1.0-0-dev libncurses-dev libcurl4-openssl-dev)
set -euo pipefail

VERSION="2025.10"
PREFIX="/opt/bladerf"
JOBS="$(nproc 2>/dev/null || echo 4)"
REPO_URL="https://github.com/Nuand/bladeRF.git"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --version) VERSION="$2"; shift 2 ;;
        --prefix) PREFIX="$2"; shift 2 ;;
        --jobs) JOBS="$2"; shift 2 ;;
        -h|--help)
            sed -n '2,29p' "$0"; exit 0 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

for tool in git cmake cc make; do
    command -v "$tool" >/dev/null || { echo "Error: missing required tool: $tool"; exit 1; }
done
if ! command -v pkg-config >/dev/null && ! command -v pkgconf >/dev/null; then
    echo "Error: missing required tool: pkg-config (install package: pkg-config)"
    exit 1
fi

# Refuse to collide with a distro-installed libbladeRF in the same prefix.
if [[ -e "$PREFIX/lib/libbladeRF.so" || -e "$PREFIX/lib64/libbladeRF.so" ]]; then
    echo "Warning: $PREFIX already contains libbladeRF; reinstalling over it."
fi
if ldconfig -p 2>/dev/null | grep -q "libbladeRF.so" && [[ "$PREFIX" == "/usr"* ]]; then
    echo "Error: a system libbladeRF is installed and --prefix is under /usr."
    echo "Use the default --prefix /opt/bladerf."
    exit 1
fi

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

echo "=== Cloning bladeRF $VERSION ==="
git clone --depth 1 --branch "$VERSION" "$REPO_URL" "$WORK/bladeRF"

# Patch upstream -Werror failure on newer Clang (>=17):
# host/utilities/bladeRF-cli/src/cmd/flash_image.c uses
#   if (val[i] >= 'a' || val[i] <= 'f')
# which is always true (overlapping comparison). The intent is clearly a
# lowercase hex range check, i.e. &&. Fix it in place so -Werror builds
# (macOS) succeed and serial validation works as intended.
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
    -DCMAKE_INSTALL_PREFIX="$PREFIX"
cmake --build "$WORK/bladeRF/host/build" -j"$JOBS"
cmake --install "$WORK/bladeRF/host/build"

if command -v ldconfig >/dev/null; then
    ldconfig "$PREFIX/lib" 2>/dev/null || ldconfig 2>/dev/null || true
fi

echo ""
echo "Custom libbladeRF installed under $PREFIX:"
ls -l "$PREFIX/lib"/libbladeRF.so* 2>/dev/null || ls -l "$PREFIX/lib64"/libbladeRF.so* 2>/dev/null || true
if command -v pkg-config >/dev/null; then
    PKG_CONFIG_PATH="$PREFIX/lib/pkgconfig:$PREFIX/lib64/pkgconfig:${PKG_CONFIG_PATH:-}" \
        pkg-config --modversion libbladeRF 2>/dev/null || true
fi
echo "Configure supertooth with:"
echo "  PKG_CONFIG_PATH=$PREFIX/lib/pkgconfig:\$PKG_CONFIG_PATH \\"
echo "  cmake -DCMAKE_PREFIX_PATH=$PREFIX:... ..."
