#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<EOF
Usage: $0 <build-dir> --version <ver> --qt-prefix <path> --output <deb-path>

Build a vendor .deb for supertooth.  Bundles all 4 binaries plus Qt 6.8
and radio library dependencies (hackrf, liquid-dsp, libbtbb) inside the
package.

Steps:
  1. cmake --install to a staging directory
  2. Copy needed shared libraries from Qt and system paths into staging
  3. Copy Qt platform plugins and QML modules
  4. Set RPATH on all binaries
  5. Create DEBIAN/control + postinst
  6. Build .deb with dpkg-deb
EOF
    exit 1
}

BUILD_DIR=""
VERSION="0.0.0"
QT_PREFIX=""
OUTPUT_DEB=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --version) VERSION="$2"; shift 2 ;;
        --qt-prefix) QT_PREFIX="$2"; shift 2 ;;
        --output) OUTPUT_DEB="$2"; shift 2 ;;
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

# Check for patchelf
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
    echo "Warning: patchelf not found. RPATH will not be set on binaries."
    echo "Install with: sudo apt install patchelf"
fi

# Strip 'v' prefix from version if present
VERSION="${VERSION#v}"

STAGING=$(mktemp -d)
trap 'rm -rf "$STAGING"' EXIT

echo "=== Installing to staging ==="
DESTDIR="$STAGING" cmake --install "$BUILD_DIR" --prefix /usr

# Move the GUI binary into the bundle directory so it can ship with a
# qt.conf alongside it.  Replace it at /usr/bin/supertooth with a thin
# wrapper that sets plugin/QML import paths before exec'ing the real one.
GUI_REAL="$STAGING/usr/lib/supertooth/supertooth"
mkdir -p "$(dirname "$GUI_REAL")"
mv "$STAGING/usr/bin/supertooth" "$GUI_REAL"

cat > "$STAGING/usr/bin/supertooth" << 'WRAPPER'
#!/bin/sh
appdir=/usr/lib/supertooth
export QT_PLUGIN_PATH="$appdir/qtplugins"
export QML2_IMPORT_PATH="$appdir/qml"
exec "$appdir/supertooth" "$@"
WRAPPER
chmod 755 "$STAGING/usr/bin/supertooth"

cat > "$(dirname "$GUI_REAL")/qt.conf" << 'QTCONF'
[Paths]
Plugins = qtplugins
Qml2Imports = qml
QTCONF

BUNDLE_DIR="$STAGING/usr/lib/supertooth"
mkdir -p "$BUNDLE_DIR"

# ------------------------------------------------------------------
# Collect shared library dependencies
# ------------------------------------------------------------------
echo "=== Collecting shared library dependencies ==="

# Scan build-directory binaries for dependencies (they have correct RPATH
# pointing to the Qt prefix).  Staged binaries lose this RPATH after install.
BUILD_BIN_DIR="$BUILD_DIR/src/apps/cli"
BUILD_GUI_DIR="$BUILD_DIR/src/apps/gui"

BINARIES=(
    "$BUILD_BIN_DIR/supertooth-bredr"
    "$BUILD_BIN_DIR/supertooth-ble"
    "$BUILD_BIN_DIR/supertooth-hybrid"
    "$BUILD_GUI_DIR/supertooth"
)

# System library directories to search (bundled only if not standard)
# Standard libs (glibc, libstdc++, libpthread, libm, librt, libdl, libz)
# are NOT bundled — they come from the target system.
SYSTEM_LIB_DIRS=(
    /usr/lib/x86_64-linux-gnu
    /usr/lib
    /lib/x86_64-linux-gnu
    /lib
)

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
               libxcb-xinput.so libxcb-util.so libxcb-present.so libxcb-composite.so; do
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

    local target="$BUNDLE_DIR/$lib_name"
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
        if [[ "$libpath" == /usr/lib/x86_64-linux-gnu/libEGL* || \
              "$libpath" == /usr/lib/x86_64-linux-gnu/libGL* || \
              "$libpath" == /usr/lib/x86_64-linux-gnu/libOpenGL* || \
              "$libpath" == /usr/lib/x86_64-linux-gnu/libGLX* || \
              "$libpath" == /usr/lib/x86_64-linux-gnu/libGLdispatch* || \
              "$libpath" == /usr/lib/x86_64-linux-gnu/libdrm* ]]; then
            continue
        fi

        # Bundle remaining system libraries
        if [[ "$libpath" == /lib/* || "$libpath" == /usr/lib/* ]]; then
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
pushd "$BUNDLE_DIR" >/dev/null
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
    PLUGIN_DEST="$BUNDLE_DIR/qtplugins"
    mkdir -p "$PLUGIN_DEST"
    # Copy platforms, imageformats, styles, xcbglintegrations, tls, etc.
    for plugin_subdir in platforms imageformats styles xcbglintegrations tls sqldrivers \
                         wayland-decoration-client wayland-graphics-integration-client wayland-shell-integration; do
        src="$QT_PREFIX/plugins/$plugin_subdir"
        if [[ -d "$src" ]]; then
            cp -a "$src" "$PLUGIN_DEST/"
        fi
    done
fi

# ------------------------------------------------------------------
# Copy QML modules
# ------------------------------------------------------------------
echo "=== Copying QML modules ==="
if [[ -d "$QT_PREFIX/qml" ]]; then
    QML_DEST="$BUNDLE_DIR/qml"
    mkdir -p "$QML_DEST"
    for qml_mod in QtQuick QtQuick.2 QtQml QtQml.Models QtGraphs QtQuick3D; do
        src="$QT_PREFIX/qml/$qml_mod"
        if [[ -d "$src" ]]; then
            cp -a "$src" "$QML_DEST/"
        fi
    done
    # Also copy builtins.qmltypes and plugins.qmltypes if they exist
    for f in builtins.qmltypes plugins.qmltypes jsroot.qmltypes; do
        if [[ -f "$QT_PREFIX/qml/$f" ]]; then
            cp -a "$QT_PREFIX/qml/$f" "$QML_DEST/"
        fi
    done
fi

# ------------------------------------------------------------------
# Also bundle radio libs from the system (hackrf, liquid, btbb)
# ------------------------------------------------------------------
echo "=== Bundling radio library dependencies ==="
for libpattern in libhackrf.so* libliquid.so* libbtbb.so*; do
    for dir in "${SYSTEM_LIB_DIRS[@]}"; do
        for libpath in "$dir/$libpattern"; do
            [[ -f "$libpath" ]] && copy_lib_with_symlinks "$libpath"
        done
    done
done

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
              "$libpath" == "$BUNDLE_DIR"/* ]]; then
            copy_lib_with_symlinks "$libpath"
        fi
    done < <(ldd "$sofile" 2>/dev/null || true)
done < <(find "$BUNDLE_DIR" -name '*.so' -o -name '*.so.*' -type f -print0)

# ------------------------------------------------------------------
# Set RPATH on all binaries
# ------------------------------------------------------------------
STAGED_BINARIES=(
    "$STAGING/usr/bin/supertooth-bredr"
    "$STAGING/usr/bin/supertooth-ble"
    "$STAGING/usr/bin/supertooth-hybrid"
    "$STAGING/usr/lib/supertooth/supertooth"
)

echo "=== Setting RPATH ==="
if [[ -n "$PATCHELF" ]]; then
    RPATH='$ORIGIN/../lib/supertooth'
    for binary in "${STAGED_BINARIES[@]}"; do
        if [[ -f "$binary" ]]; then
            echo "  patchelf: $(basename "$binary")"
            "$PATCHELF" --set-rpath "$RPATH" "$binary" || true
        fi
    done

    # Also set RPATH on the bundled Qt libs so they find each other at install time
    if [[ -d "$BUNDLE_DIR" ]]; then
        for lib in "$BUNDLE_DIR"/libQt6*.so*; do
            if [[ -f "$lib" && ! -L "$lib" ]]; then
                "$PATCHELF" --set-rpath "/usr/lib/supertooth" "$lib" 2>/dev/null || true
            fi
        done
    fi
else
    echo "  Skipped (patchelf not available)"
fi

# ------------------------------------------------------------------
# Plugin and QML paths are set by the wrapper script at
# /usr/bin/supertooth.  The real binary is at /usr/lib/supertooth/
# with a qt.conf alongside it.  No further config needed.

# ------------------------------------------------------------------
# Create DEBIAN/ control files
# ------------------------------------------------------------------
echo "=== Creating DEBIAN/control ==="
DEBIAN_DIR="$STAGING/DEBIAN"
mkdir -p "$DEBIAN_DIR"

cat > "$DEBIAN_DIR/control" << CONTROL
Package: supertooth
Version: ${VERSION}
Architecture: amd64
Maintainer: Supertooth Developers <dev@supertooth.local>
Depends: libc6 (>= 2.35), libstdc++6 (>= 12), libgl1, libglx0
Section: comm
Priority: optional
Homepage: https://github.com/supertooth/supertooth
Description: Software-defined Bluetooth packet sniffer for HackRF
 Supertooth is a C-based software-defined Bluetooth receiver that
 captures and decodes BR/EDR and BLE packets using a HackRF SDR.
 .
 This package includes all four applications:
  - supertooth-bredr:  BR/EDR multichannel receiver
  - supertooth-ble: BLE advertising channel scanner
  - supertooth-hybrid: simultaneous BR/EDR + BLE receiver
  - supertooth:     Qt GUI application
CONTROL

cat > "$DEBIAN_DIR/postinst" << 'POSTINST'
#!/bin/sh
set -e
# Register the bundled library path so ldconfig finds it
echo "/usr/lib/supertooth" > /etc/ld.so.conf.d/supertooth.conf
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
