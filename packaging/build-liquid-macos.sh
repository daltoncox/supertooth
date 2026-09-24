#!/usr/bin/env bash
# Build and install liquid-dsp from source with the hot-path logging
# compiled out.
#
# Why: stock liquid-dsp (e.g. the Homebrew bottle, built with the default
# LIQUID_LOG_LEVEL_COMPILE=0) compiles a liquid_log_trace() call into every
# SIMD dotprod execute dispatcher
# (dotprod_{cccf,crcf,rrrf}_execute_{neon,sse,avx,avx512f}). That is a
# function call plus shared-counter increment on EVERY FIR tap evaluation --
# tens of millions of times per second across the channelizer and every
# demodulator thread. On Apple Silicon this shows up as ~2000% CPU and RF
# block drops in supertooth-hybrid; building with
# -DLIQUID_LOG_LEVEL_COMPILE=1 drops it to ~300% CPU with zero drops.
# Level 1 disables TRACE only; DEBUG/INFO/WARN/ERROR/FATAL diagnostics
# (including every error path in the DSP hot path) are unaffected -- the
# per-sample paths contain no DEBUG-or-higher logging.
#
# Usage:
#   bash packaging/build-liquid-macos.sh [--version 1.8.2] [--prefix DIR] [--jobs N]
#
# Defaults:
#   --version  1.8.2
#   --prefix   $(brew --prefix)  (i.e. /opt/homebrew on Apple Silicon;
#              headers -> $PREFIX/include, lib -> $PREFIX/lib, which is
#              where supertooth's CMake lookup already finds liquid-dsp)
#   --jobs     sysctl -n hw.ncpu
#
# Requirements: curl (or shasum for verification), autoconf, automake,
# libtool, cc, make (brew install autoconf automake libtool).
#
# If the Homebrew liquid-dsp formula is installed, uninstall it first --
# this script installs the same files and would collide with the bottle:
#   brew uninstall liquid-dsp
# To restore stock Homebrew liquid-dsp afterwards:
#   brew install liquid-dsp
set -euo pipefail

VERSION="1.8.2"
PREFIX=""
JOBS="$(sysctl -n hw.ncpu 2>/dev/null || echo 4)"
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

if [[ -z "$PREFIX" ]]; then
    PREFIX="$(brew --prefix 2>/dev/null || echo /opt/homebrew)"
fi

for tool in curl autoconf automake libtool cc make; do
    command -v "$tool" >/dev/null || { echo "Error: missing required tool: $tool"; exit 1; }
done

# Refuse to collide with a Homebrew-installed bottle of the same library.
if [[ -L "$PREFIX/lib/libliquid.dylib" ]] && [[ "$(readlink "$PREFIX/lib/libliquid.dylib")" == *Cellar* ]]; then
    echo "Error: Homebrew's liquid-dsp bottle owns $PREFIX/lib/libliquid.dylib."
    echo "Uninstall it first: brew uninstall liquid-dsp"
    exit 1
fi

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

echo "=== Downloading liquid-dsp v$VERSION ==="
curl -sL "$TARBALL_URL" -o "$WORK/liquid.tar.gz"
if command -v shasum >/dev/null; then
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
    # -headerpad keeps install_name_tool -id working on the installed dylib.
    make -j"$JOBS" LDFLAGS="-headerpad_max_install_names"
    make install
)

DYLIB="$PREFIX/lib/libliquid.dylib"
install_name_tool -id "$DYLIB" "$DYLIB"
codesign --force --sign - "$DYLIB" 2>/dev/null || true

echo ""
echo "Fixed libliquid installed:"
otool -D "$DYLIB"
echo "(restore stock with: brew install liquid-dsp)"
