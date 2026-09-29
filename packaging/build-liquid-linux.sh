#!/usr/bin/env bash
# Build and install liquid-dsp from source with optimizations enabled and
# hot-path logging compiled out (Linux).
#
# Mirrors packaging/build-liquid-macos.sh: same v1.8.2 tarball, same
# -DLIQUID_LOG_LEVEL_COMPILE=1 (TRACE off, DEBUG and above kept) and same
# -O2 -ffast-math optimization. Stock distro liquid-dsp (e.g. apt
# libliquid-dev, built with the default LIQUID_LOG_LEVEL_COMPILE=0)
# compiles a liquid_log_trace() call into every SIMD dotprod execute
# dispatcher (dotprod_{cccf,crcf,rrrf}_execute_{neon,sse,avx,avx512f}) --
# a function call plus shared-counter increment on EVERY FIR tap
# evaluation, tens of millions of times per second across the channelizer
# and every demodulator thread.
#
# Usage:
#   sudo bash packaging/build-liquid-linux.sh [--version 1.8.2] [--prefix DIR] [--jobs N]
#
# Defaults:
#   --version  1.8.2
#   --prefix   /opt/liquid-dsp  (isolated from distro libliquid-dev; add
#              /opt/liquid-dsp/lib/pkgconfig to PKG_CONFIG_PATH and
#              /opt/liquid-dsp to CMAKE_PREFIX_PATH when configuring
#              supertooth -- see .github/workflows/release.yml)
#   --jobs     nproc
#
# Requirements: curl, autoconf, automake, libtool, cc, make
#   (sudo apt install -y curl autoconf automake libtool build-essential
#    pkg-config libfftw3-dev)
#
# Do NOT install apt libliquid-dev alongside this -- remove it first so
# pkg-config / CMake cannot pick up the stock library:
#   sudo apt remove libliquid-dev
set -euo pipefail

VERSION="1.8.2"
PREFIX="/opt/liquid-dsp"
JOBS="$(nproc 2>/dev/null || echo 4)"
TARBALL_URL="https://github.com/jgaeddert/liquid-dsp/archive/refs/tags/v${VERSION}.tar.gz"
# sha256 of v1.8.2.tar.gz (matches the Homebrew liquid-dsp formula).
TARBALL_SHA256="a0adbc3ec5630d620a55351285573f59948153a14c703fb64b4ad58989bd6e2f"

# TRACE (level 0) off, everything else on. This is the entire fix: the only
# per-sample logging in liquid-dsp's DSP hot path is TRACE-level, inside the
# dotprod execute dispatchers.
LOG_LEVEL_COMPILE="1"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --version) VERSION="$2"; shift 2 ;;
        --prefix) PREFIX="$2"; shift 2 ;;
        --jobs) JOBS="$2"; shift 2 ;;
        -h|--help)
            sed -n '2,30p' "$0"; exit 0 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

# Re-derive URL in case --version was overridden.
TARBALL_URL="https://github.com/jgaeddert/liquid-dsp/archive/refs/tags/v${VERSION}.tar.gz"

for tool in curl autoconf automake cc make; do
    command -v "$tool" >/dev/null || { echo "Error: missing required tool: $tool"; exit 1; }
done
# Debian/Ubuntu's libtool package provides libtoolize; Homebrew's provides
# glibtoolize. bootstrap.sh needs one of them. (Note: bare `libtool` on
# macOS is Apple's cctools linker wrapper, NOT GNU libtool, so it must not
# satisfy this check.)
if ! command -v libtoolize >/dev/null && ! command -v glibtoolize >/dev/null; then
    echo "Error: missing required tool: libtoolize (install package: libtool)"
    exit 1
fi

# Refuse to collide with a distro-installed libliquid in the same prefix.
if [[ -e "$PREFIX/lib/libliquid.so" || -e "$PREFIX/lib64/libliquid.so" ]]; then
    echo "Warning: $PREFIX already contains libliquid; reinstalling over it."
fi
if ldconfig -p 2>/dev/null | grep -q "libliquid.so" && [[ "$PREFIX" == "/usr"* ]]; then
    echo "Error: a system libliquid is installed and --prefix is under /usr."
    echo "Use the default --prefix /opt/liquid-dsp (and remove libliquid-dev)."
    exit 1
fi

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

echo "=== Downloading liquid-dsp v$VERSION ==="
curl -sL "$TARBALL_URL" -o "$WORK/liquid.tar.gz"
if command -v sha256sum >/dev/null; then
    echo "$TARBALL_SHA256  $WORK/liquid.tar.gz" | sha256sum -c - \
        || { echo "Error: tarball checksum mismatch"; exit 1; }
elif command -v shasum >/dev/null; then
    echo "$TARBALL_SHA256  $WORK/liquid.tar.gz" | shasum -a 256 -c - \
        || { echo "Error: tarball checksum mismatch"; exit 1; }
fi
tar xzf "$WORK/liquid.tar.gz" -C "$WORK"
SRC="$WORK/liquid-dsp-$VERSION"

echo "=== Building (prefix=$PREFIX, jobs=$JOBS, LIQUID_LOG_LEVEL_COMPILE=$LOG_LEVEL_COMPILE) ==="
(
    cd "$SRC"
    ./bootstrap.sh
    ./configure --prefix="$PREFIX" \
        "CFLAGS=-g -O2 -ffast-math -DLIQUID_LOG_LEVEL_COMPILE=$LOG_LEVEL_COMPILE"
    make -j"$JOBS"
    make install
)

if command -v ldconfig >/dev/null; then
    ldconfig "$PREFIX/lib" 2>/dev/null || ldconfig 2>/dev/null || true
fi

echo ""
echo "Custom libliquid installed under $PREFIX:"
ls -l "$PREFIX/lib"/libliquid.so* 2>/dev/null || ls -l "$PREFIX/lib64"/libliquid.so* 2>/dev/null || true
echo "Configure supertooth with:"
echo "  PKG_CONFIG_PATH=$PREFIX/lib/pkgconfig:\$PKG_CONFIG_PATH \\"
echo "  cmake -DCMAKE_PREFIX_PATH=$PREFIX:... ..."
