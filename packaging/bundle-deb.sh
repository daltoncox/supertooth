#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<EOF
Usage: $0 <build-dir> --version <ver> --qt-prefix <path> --output <deb-path>

Build a vendor .deb for supertooth.  Bundles both binaries plus Qt 6.8
and radio library dependencies (hackrf, bladeRF) inside the
package under /opt/supertooth. liquid-dsp is statically linked and is
not bundled.

Layout:
  /opt/supertooth/bin/      supertooth (CLI) + supertooth-desktop (GUI), qt.conf
  /opt/supertooth/lib/      bundled shared libraries (Qt, ICU, radio)
  /opt/supertooth/plugins/  Qt platform/imageformat/... plugins
  /opt/supertooth/qml/      QML modules
  /usr/bin/supertooth       CLI (multiplexed: hybrid, ble, bredr, record)
  /usr/bin/supertooth-desktop  Qt GUI application
  /usr/share/applications/  supertooth.desktop (via cmake --install, left in place)
  /usr/share/icons/...      supertooth-app.png (via cmake --install, left in place)

All linkage is private: RPATH (\$ORIGIN/../lib) on every binary, \$ORIGIN
on bundled libraries, and qt.conf for plugin/QML paths.  The bundled
libraries are NEVER registered with the system-wide linker search path
(/etc/ld.so.conf.d), so no other application on the host can pick up our
vendored Qt.

Steps:
  1. cmake --install to a staging directory
  2. Move binaries into /opt/supertooth/bin, symlinks into /usr/bin
  3. Copy needed shared libraries from Qt and system paths into lib/
  4. Copy Qt platform plugins and QML modules
  5. Set RPATH on all binaries and bundled Qt libraries
  6. Verify every linkage resolves inside /opt/supertooth/lib
  7. Create DEBIAN/control + postinst
  8. Build .deb with dpkg-deb
EOF
    exit 1
}

BUILD_DIR=""
VERSION="0.0.0"
QT_PREFIX=""
OUTPUT_DEB=""
# Custom libbladeRF install prefix (packaging/build-bladerf-linux.sh
# default). The release builds libbladeRF >= 2.6.0 from source here because
# distro packages predate the FX3 firmware v2.6.0 framing change.
BLADERF_PREFIX="${BLADERF_PREFIX:-/opt/bladerf}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --version) VERSION="$2"; shift 2 ;;
        --qt-prefix) QT_PREFIX="$2"; shift 2 ;;
        --liquid-prefix)
            echo "Warning: --liquid-prefix is deprecated and ignored (liquid-dsp is now statically linked)."
            shift 2 ;;
        --bladerf-prefix) BLADERF_PREFIX="$2"; shift 2 ;;
        --output) OUTPUT_DEB="$2"; shift 2 ;;
        --arch) ARCH="$2"; shift 2 ;;
        -*)
            if [[ -z "$BUILD_DIR" ]]; then
                BUILD_DIR="$1"; shift
            else
                echo "Unknown option: $1"; exit 1
            fi
            ;;
        *)
            if [[ -z "$BUILD_DIR" ]]; then
                BUILD_DIR="$1"; shift
            else
                echo "Unexpected argument: $1"; exit 1
            fi
    esac
done

if [[ -z "$BUILD_DIR" || -z "$OUTPUT_DEB" ]]; then
    usage
fi

if [[ -z "$QT_PREFIX" ]]; then
    echo "Error: --qt-prefix is required"
    exit 1
fi

# Architecture: prefer explicit --arch, else the host's Debian arch, else
# derive from uname -m.  This drives the multiarch lib directory and the
# DEBIAN/control Architecture field so the same script produces amd64 or
# arm64 packages unchanged in behavior.
if [[ -z "${ARCH:-}" ]]; then
    ARCH=$(dpkg --print-architecture 2>/dev/null || true)
    if [[ -z "$ARCH" ]]; then
        case "$(uname -m)" in
            x86_64)  ARCH=amd64 ;;
            aarch64) ARCH=arm64 ;;
            *)       ARCH=amd64 ;;
        esac
    fi
fi

case "$ARCH" in
    amd64) MULTIARCH="x86_64-linux-gnu" ;;
    arm64) MULTIARCH="aarch64-linux-gnu" ;;
    *)
        echo "Error: unsupported architecture '$ARCH' (expected amd64 or arm64)"
        exit 1
        ;;
esac

# Check for patchelf.  RPATH is the ONLY mechanism by which the bundled
# libraries are found, so this is mandatory.
PATCHELF=""
if command -v patchelf &>/dev/null; then
    PATCHELF="patchelf"
else
    # Try to find it in the nix store or other common locations
    for p in /usr/bin/patchelf /usr/local/bin/patchelf /nix/var/nix/profiles/default/bin/patchelf; do
        if [[ -x "$p" ]]; then
            PATCHELF="$p"
            break
        fi
    done
fi
if [[ -z "$PATCHELF" ]]; then
    echo "Error: patchelf not found. It is required to set the RPATH on"
    echo "bundled binaries and libraries. Install with: sudo apt install patchelf"
    exit 1
fi

# Strip 'v' prefix from version if present
VERSION="${VERSION#v}"

# dpkg requires the Version field to start with a digit.  Real releases pass
# a semantic version (e.g. 1.2.3 from a v1.2.3 tag).  For non-release / local
# test builds that pass a non-numeric value, fall back to a valid placeholder.
if [[ ! "$VERSION" =~ ^[0-9] ]]; then
    VERSION="0.0.0"
fi

STAGING=$(mktemp -d)
trap 'rm -rf "$STAGING"' EXIT

echo "=== Installing to staging ==="
DESTDIR="$STAGING" cmake --install "$BUILD_DIR" --prefix /usr

# liquid-dsp is statically linked, but its install() rules still stage
# libliquid.a + headers + cmake/pc files. Purge them so the .deb ships
# only supertooth's own files (dpkg-deb packages the whole staging tree).
echo "=== Purging vendored liquid-dsp install artifacts (statically linked) ==="
find "$STAGING" -depth -iname '*liquid*' -exec rm -rf {} +
if find "$STAGING" -iname '*liquid*' | grep -q .; then
    echo "  ERROR: liquid artifacts remain in staging"
    exit 1
fi

# ------------------------------------------------------------------
# Layout: everything self-contained under /opt/supertooth
# ------------------------------------------------------------------
OPT_ROOT="$STAGING/opt/supertooth"
BIN_DIR="$OPT_ROOT/bin"
LIB_DIR="$OPT_ROOT/lib"
PLUGIN_DIR="$OPT_ROOT/plugins"
QML_DIR="$OPT_ROOT/qml"
mkdir -p "$BIN_DIR" "$LIB_DIR"

if [[ ! -f "$STAGING/usr/bin/supertooth-desktop" ]]; then
    echo "Error: GUI binary not found in staging. Was the project configured with -DBUILD_GUI=ON?"
    exit 1
fi
if [[ ! -f "$STAGING/usr/bin/supertooth" ]]; then
    echo "Error: CLI binary not found in staging."
    exit 1
fi

# Move both binaries into /opt/supertooth/bin and symlink them back into
# /usr/bin. The kernel resolves the symlink before exec, so $ORIGIN still
# points at /opt/supertooth/bin.
mv "$STAGING/usr/bin/supertooth-desktop" "$BIN_DIR/supertooth-desktop"
mv "$STAGING/usr/bin/supertooth" "$BIN_DIR/supertooth"

# CLI + GUI: plain symlinks into /usr/bin.
ln -s /opt/supertooth/bin/supertooth "$STAGING/usr/bin/supertooth"
ln -s /opt/supertooth/bin/supertooth-desktop "$STAGING/usr/bin/supertooth-desktop"

# qt.conf sits next to supertooth-desktop.  Qt resolves relative entries
# against the directory containing qt.conf, so plugin/QML paths need no
# environment variables in the wrapper.
cat > "$BIN_DIR/qt.conf" << 'QTCONF'
[Paths]
Plugins = ../plugins
Qml2Imports = ../qml
QTCONF

# ------------------------------------------------------------------
# Collect shared library dependencies
# ------------------------------------------------------------------
echo "=== Collecting shared library dependencies ==="

# Scan build-directory binaries for dependencies (they have correct RPATH
# pointing to the Qt prefix).  Staged binaries lose this RPATH after install.
BUILD_BIN_DIR="$BUILD_DIR/src/apps/cli"
BUILD_GUI_DIR="$BUILD_DIR/src/apps/gui"

BINARIES=(
    "$BUILD_BIN_DIR/supertooth"
    "$BUILD_GUI_DIR/supertooth-desktop"
)

# System library directories to search (bundled only if not standard)
# Standard libs (glibc, libstdc++, libpthread, libm, librt, libdl, libz)
# are NOT bundled — they come from the target system.
# NOTE: liquid-dsp is statically linked, never bundled.
SYSTEM_LIB_DIRS=(
    /usr/lib/$MULTIARCH
    /usr/lib
    /lib/$MULTIARCH
    /lib
)
if [[ -d "$BLADERF_PREFIX/lib" ]]; then
    SYSTEM_LIB_DIRS+=("$BLADERF_PREFIX/lib")
fi
if [[ -d "$BLADERF_PREFIX/lib64" ]]; then
    SYSTEM_LIB_DIRS+=("$BLADERF_PREFIX/lib64")
fi

resolve_lib() {
    local soname="$1"
    for dir in "${SYSTEM_LIB_DIRS[@]}"; do
        local candidate="$dir/$soname"
        if [[ -f "$candidate" ]]; then
            echo "$candidate"
            return 0
        fi
    done
    return 1
}

is_standard_lib() {
    local name="$1"
    # Standard system libraries that must NOT be bundled.
    for std in libc.so libc- libpthread.so libpthread- libm.so libm- \
               librt.so librt- libdl.so libdl- libstdc++.so libgcc_s.so \
               ld-linux libz.so libz- libresolv.so libresolv- \
               libEGL.so libEGL- libGL.so libGL- libGLX.so libGLdispatch.so \
               libOpenGL.so libdrm.so libdrm- libX11.so libXau.so libxcb.so \
               libXfixes.so libXext.so libXrender.so libXrandr.so libXcursor.so \
               libXinerama.so libXi.so libXxf86vm.so libXdamage.so libXcomposite.so \
               libX11-xcb.so libxcb-shm.so libxcb-shape.so libxcb-xfixes.so \
               libxcb-render.so libxcb-sync.so libxcb-xkb.so libxcb-keysyms.so \
               libxcb-image.so libxcb-icccm.so libxcb-randr.so libxcb-xinerama.so \
               libxcb-xinput.so libxcb-util.so libxcb-present.so libxcb-composite.so \
               libwayland.so libwayland- libxkbcommon.so libxkbcommon- \
               libgbm.so libgbm- libglapi.so libglapi- libxshmfence.so libxshmfence-; do
        if [[ "$name" == "$std"* ]]; then
            return 0
        fi
    done
    return 1
}

is_system_qt_lib() {
    local libpath="$1"
    # Skip Qt libs from system paths (we bundle from our Qt prefix instead)
    if [[ "$libpath" == /usr/lib/*/libQt6* || "$libpath" == /usr/lib/libQt6* ]]; then
        return 0
    fi
    return 1
}

declare -A COPIED

copy_lib_with_symlinks() {
    local lib_path="$1"
    [[ -f "$lib_path" ]] || return 0
    local lib_name
    lib_name=$(basename "$lib_path")
    [[ -n "${COPIED[$lib_name]:-}" ]] && return 0
    COPIED["$lib_name"]=1

    local target="$LIB_DIR/$lib_name"
    if [[ ! -f "$target" ]]; then
        cp -a "$lib_path" "$target"
    fi

    # Follow symlinks and copy their targets too
    if [[ -L "$lib_path" ]]; then
        local real_path
        real_path=$(readlink -f "$lib_path")
        copy_lib_with_symlinks "$real_path"
    fi
}

# For each binary, check what shared libs it needs (transitively)
for binary in "${BINARIES[@]}"; do
    if [[ ! -f "$binary" ]]; then
        echo "  Warning: $binary not found, skipping"
        continue
    fi
    echo "  Scanning: $(basename "$binary")"
    while IFS= read -r line; do
        # Parse ldd output: "libfoo.so.6 => /path/to/libfoo.so.6 (0x...)"
        soname=""
        libpath=""
        if [[ "$line" =~ ^[[:space:]]*([^ ]+)' => '([^ ]+)' '[\(0x] ]]; then
            soname="${BASH_REMATCH[1]}"
            libpath="${BASH_REMATCH[2]}"
        elif [[ "$line" =~ ^[[:space:]]*([^ ]+)' => not found' ]]; then
            echo "  ERROR: Missing dependency: ${BASH_REMATCH[1]}"
            exit 1
        else
            continue
        fi

        # Skip standard system libs (glibc, libstdc++, X11, GL, etc.)
        is_standard_lib "$soname" && continue

        # Skip system Qt libs (we bundle Qt from our prefix)
        is_system_qt_lib "$libpath" && continue

        # Skip GPU driver-specific libs (EGL, GL, etc.)
        if [[ "$libpath" == /usr/lib/$MULTIARCH/libEGL* || \
              "$libpath" == /usr/lib/$MULTIARCH/libGL* || \
              "$libpath" == /usr/lib/$MULTIARCH/libOpenGL* || \
              "$libpath" == /usr/lib/$MULTIARCH/libGLX* || \
              "$libpath" == /usr/lib/$MULTIARCH/libGLdispatch* || \
              "$libpath" == /usr/lib/$MULTIARCH/libdrm* ]]; then
            continue
        fi

        # Bundle remaining system libraries (+ custom bladeRF prefix)
        if [[ "$libpath" == /lib/* || "$libpath" == /usr/lib/* ||
              "$libpath" == "$QT_PREFIX"/* ||
              "$libpath" == "$BLADERF_PREFIX"/* ]]; then
            copy_lib_with_symlinks "$libpath"
        fi
    done < <(ldd "$binary" 2>/dev/null || true)
done

# ------------------------------------------------------------------
# Copy Qt shared libraries (those not already copied via ldd scan)
# ------------------------------------------------------------------
echo "=== Copying Qt shared libraries ==="
if [[ -d "$QT_PREFIX/lib" ]]; then
    for lib in "$QT_PREFIX/lib"/libQt6*.so* "$QT_PREFIX/lib"/libicudata.so* \
               "$QT_PREFIX/lib"/libicui18n.so* "$QT_PREFIX/lib"/libicuuc.so*; do
        if [[ -f "$lib" ]]; then
            copy_lib_with_symlinks "$lib"
        fi
    done
fi

# Convert real .so.6 files to symlinks so ldconfig doesn't warn.
# The Qt SDK installs hard copies instead of symlinks for versioned
# filenames like libQt6Core.so.6 — ldconfig expects a symlink here.
pushd "$LIB_DIR" >/dev/null
for lib in libQt6*.so.?; do
    if [[ -f "$lib" && ! -L "$lib" ]]; then
        # Find the full versioned file (e.g. libQt6Core.so.6.8.0)
        target="${lib}.0"
        # Try to find any more-specific version match
        for candidate in "${lib}".*.*; do
            if [[ -f "$candidate" && "$candidate" != "$lib" ]]; then
                target="$candidate"
                break
            fi
        done
        if [[ -f "$target" ]]; then
            rm "$lib"
            ln -s "$(basename "$target")" "$lib"
        fi
    fi
done
# Same for libicu*.so.?
for lib in libicu*.so.??; do
    if [[ -f "$lib" && ! -L "$lib" ]]; then
        target="${lib}.2"
        if [[ -f "$target" ]]; then
            rm "$lib"
            ln -s "$(basename "$target")" "$lib"
        fi
    fi
done
popd >/dev/null

# ------------------------------------------------------------------
# Copy Qt plugins
# ------------------------------------------------------------------
echo "=== Copying Qt plugins ==="
if [[ -d "$QT_PREFIX/plugins" ]]; then
    mkdir -p "$PLUGIN_DIR"
    # Copy platforms, imageformats, styles, xcbglintegrations, tls, etc.
    for plugin_subdir in platforms imageformats styles xcbglintegrations tls sqldrivers \
                         wayland-decoration-client wayland-graphics-integration-client wayland-shell-integration; do
        src="$QT_PREFIX/plugins/$plugin_subdir"
        if [[ -d "$src" ]]; then
            cp -a "$src" "$PLUGIN_DIR/"
        fi
    done
fi

# ------------------------------------------------------------------
# Copy QML modules
# ------------------------------------------------------------------
echo "=== Copying QML modules ==="
if [[ -d "$QT_PREFIX/qml" ]]; then
    mkdir -p "$QML_DIR"
    for qml_mod in QtQuick QtQuick.2 QtQml QtQml.Models QtGraphs QtQuick3D; do
        src="$QT_PREFIX/qml/$qml_mod"
        if [[ -d "$src" ]]; then
            cp -a "$src" "$QML_DIR/"
        fi
    done
    # Also copy builtins.qmltypes and plugins.qmltypes if they exist
    for f in builtins.qmltypes plugins.qmltypes jsroot.qmltypes; do
        if [[ -f "$QT_PREFIX/qml/$f" ]]; then
            cp -a "$QT_PREFIX/qml/$f" "$QML_DIR/"
        fi
    done
fi

# ------------------------------------------------------------------
# Also bundle radio libs: hackrf from the system, bladeRF from the custom
# source prefix (/opt/bladerf) or system fallback.
# (liquid-dsp is statically linked into the binaries, never bundled.)
# NOTE: bladeRF soname uses capital RF (libbladeRF.so.2).
# ------------------------------------------------------------------
echo "=== Bundling radio library dependencies ==="
for libpattern in libhackrf.so* 'libbladeRF.so*' libbladeRf.so* libbladerf.so*; do
    for dir in "${SYSTEM_LIB_DIRS[@]}"; do
        for libpath in "$dir/$libpattern"; do
            [[ -f "$libpath" ]] && copy_lib_with_symlinks "$libpath"
        done
    done
done

# liquid-dsp must never appear as a shared dependency (static link check).
if ls "$LIB_DIR"/libliquid.so* 2>/dev/null | grep -q .; then
    echo "  ERROR: libliquid found in bundle but liquid-dsp must be statically linked"
    exit 1
fi

# ------------------------------------------------------------------
# Second dependency pass: scan plugin + QML .so files for additional
# shared libs not linked directly by the main binaries
# (e.g. libQt6WaylandClient needed by Wayland platform plugins)
# ------------------------------------------------------------------
echo "=== Scanning plugin + QML dependencies ==="
while IFS= read -r -d '' sofile; do
    while IFS= read -r line; do
        soname=""
        libpath=""
        if [[ "$line" =~ ^[[:space:]]*([^ ]+)' => '([^ ]+)' '[\(0x] ]]; then
            soname="${BASH_REMATCH[1]}"
            libpath="${BASH_REMATCH[2]}"
        else
            continue
        fi
        is_standard_lib "$soname" && continue
        is_system_qt_lib "$libpath" && continue
        if [[ "$libpath" == /lib/* || "$libpath" == /usr/lib/* ||
              "$libpath" == "$QT_PREFIX"/* ||
              "$libpath" == "$BLADERF_PREFIX"/* ||
              "$libpath" == "$OPT_ROOT"/* ]]; then
            copy_lib_with_symlinks "$libpath"
        fi
    done < <(ldd "$sofile" 2>/dev/null || true)
done < <(find "$OPT_ROOT" \( -name '*.so' -o -name '*.so.*' \) -type f -print0)

# ------------------------------------------------------------------
# Set RPATH on all binaries
# ------------------------------------------------------------------
STAGED_BINARIES=(
    "$BIN_DIR/supertooth"
    "$BIN_DIR/supertooth-desktop"
)

echo "=== Setting RPATH ==="
# Every binary lives in /opt/supertooth/bin, so one uniform RPATH works.
RPATH='$ORIGIN/../lib'
for binary in "${STAGED_BINARIES[@]}"; do
    echo "  patchelf: $(basename "$binary")"
    "$PATCHELF" --set-rpath "$RPATH" "$binary"
done

# Bundled Qt libs find each other via $ORIGIN (same directory)
for lib in "$LIB_DIR"/libQt6*.so*; do
    if [[ -f "$lib" && ! -L "$lib" ]]; then
        "$PATCHELF" --set-rpath '$ORIGIN' "$lib" 2>/dev/null || true
    fi
done

# ------------------------------------------------------------------
# Verify linkages: everything must resolve, and Qt must resolve into
# our bundle — never from system paths.  Catches RPATH regressions at
# build time instead of on user machines.
# ------------------------------------------------------------------
echo "=== Verifying linkages ==="
for binary in "${STAGED_BINARIES[@]}"; do
    while IFS= read -r line; do
        if [[ "$line" =~ '=> not found' ]]; then
            echo "  ERROR: $(basename "$binary") has unresolved dependency:"
            echo "    $line"
            exit 1
        fi
        if [[ "$line" =~ ^[[:space:]]*(libQt6[^ ]+)' => '([^ ]+)' '[\(0x] ]]; then
            dep="${BASH_REMATCH[2]}"
            # Normalize: ldd prints $ORIGIN as a literal 'bin/../lib' path
            dep=$(readlink -f "$dep")
            if [[ "$dep" != "$LIB_DIR/"* ]]; then
                echo "  ERROR: $(basename "$binary") resolves ${BASH_REMATCH[1]} outside the bundle:"
                echo "    $dep"
                exit 1
            fi
        fi
    done < <(ldd "$binary" 2>/dev/null || true)
    echo "  OK: $(basename "$binary")"
done

# Bundled Qt libs themselves must also resolve (ICU, zstd, etc. inside lib/)
for lib in "$LIB_DIR"/libQt6*.so.6.* "$LIB_DIR"/libicu*.so.??.*; do
    [[ -f "$lib" ]] || continue
    while IFS= read -r line; do
        if [[ "$line" =~ '=> not found' ]]; then
            echo "  ERROR: $(basename "$lib") has unresolved dependency:"
            echo "    $line"
            exit 1
        fi
    done < <(ldd "$lib" 2>/dev/null || true)
done
echo "  All bundle libraries resolve"

# Plugin and QML paths are handled by qt.conf next to supertooth-desktop.
# No environment variables or global linker configuration are needed.

# ------------------------------------------------------------------
# Create DEBIAN/ control files
# ------------------------------------------------------------------
echo "=== Creating DEBIAN/control ==="
DEBIAN_DIR="$STAGING/DEBIAN"
mkdir -p "$DEBIAN_DIR"

cat > "$DEBIAN_DIR/control" << CONTROL
Package: supertooth
Version: ${VERSION}
Architecture: ${ARCH}
Maintainer: daltoncox <dalton@skinnyrd.com>
Depends: libc6 (>= 2.35), libstdc++6 (>= 12), libgl1, libglx0
Section: comm
Priority: optional
Homepage: https://github.com/daltoncox/supertooth
 Description: Supertooth is a C-based software-defined Bluetooth receiver that
  captures and decodes BR/EDR and BLE packets using a HackRF SDR.
  .
  This package includes both applications:
   - supertooth: multiplexed CLI (hybrid, ble, bredr, record)
   - supertooth-desktop: Qt GUI application
CONTROL

cat > "$DEBIAN_DIR/postinst" << 'POSTINST'
#!/bin/sh
set -e
# Clean up any registration left behind by versions that added
# /usr/lib/supertooth to the system-wide linker search path.  Bundled
# libraries are private to supertooth and found via RPATH only.
rm -f /etc/ld.so.conf.d/supertooth.conf
ldconfig
POSTINST
chmod 755 "$DEBIAN_DIR/postinst"

cat > "$DEBIAN_DIR/prerm" << 'PRERM'
#!/bin/sh
set -e
rm -f /etc/ld.so.conf.d/supertooth.conf
ldconfig
PRERM
chmod 755 "$DEBIAN_DIR/prerm"

# ------------------------------------------------------------------
# Build .deb
# ------------------------------------------------------------------
echo "=== Building .deb ==="
mkdir -p "$(dirname "$OUTPUT_DEB")"
fakeroot dpkg-deb --build "$STAGING" "$OUTPUT_DEB"

echo ""
echo "Package built: $OUTPUT_DEB"
dpkg-deb --info "$OUTPUT_DEB" 2>/dev/null | grep -E "^ Package|^ Version|^ Depends|^ Architecture" || true
echo ""
echo "Contents:"
dpkg-deb --contents "$OUTPUT_DEB" 2>/dev/null | head -30 || true
