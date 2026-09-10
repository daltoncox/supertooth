#!/usr/bin/env bash
# Build a self-contained macOS Apple Silicon .dmg for supertooth.
#
# Usage:
#   bash packaging/bundle-macos.sh <build-dir> \
#     --version <ver> --output <dmg-path> \
#     [--qt-prefix <path>] [--qml-dir <path>] [--volname <name>]
#
# Layout (DMG contents):
#   /Supertooth.app                 Qt GUI (macdeployqt vendors Qt frameworks,
#                                   plugins and QML imports)
#   /Supertooth.app/Contents/MacOS/
#     Supertooth                    GUI executable
#     supertooth-bredr              CLI: BR/EDR multichannel receiver
#     supertooth-ble                CLI: BLE advertising scanner
#     supertooth-hybrid             CLI: simultaneous BR/EDR + BLE receiver
#   /Supertooth.app/Contents/Frameworks/
#     *.dylib                       Homebrew radio libs (hackrf, liquid-dsp)
#                                   plus transitive deps (libusb, fftw), with
#                                   install names rewritten to
#                                   @executable_path/../Frameworks/...
#   /Applications -> /Applications  symlink for drag-to-install
#
# The bundle is ad-hoc signed only (`codesign -s -`); it is NOT notarized,
# so first launch requires right-click > Open or clearing the quarantine
# attribute. See README.md.
#
# Steps:
#   1. Copy Supertooth.app to a staging dir (keeps the build tree clean)
#   2. macdeployqt: vendor Qt frameworks / plugins / QML imports
#   3. Copy the 3 CLI tools into Contents/MacOS
#   4. Recursively bundle non-system dylibs into Contents/Frameworks
#      (anything under /opt/homebrew, /usr/local, /opt/local)
#   5. Verify no absolute Homebrew/local refs remain (otool -L)
#   6. Ad-hoc codesign
#   7. hdiutil create -> .dmg
set -euo pipefail

usage() {
    cat <<EOF
Usage: $0 <build-dir> --version <ver> --output <dmg-path> [options]

Options:
  --qt-prefix <path>   Qt installation prefix (default: \$QT_PREFIX or
                       \$(brew --prefix qt@6), else macdeployqt from PATH)
  --qml-dir <path>     QML source dir for macdeployqt -qmldir
                       (default: <repo>/src/apps/gui/qml)
  --volname <name>     DMG volume name (default: Supertooth)
EOF
    exit 1
}

BUILD_DIR=""
VERSION="0.0.0"
OUTPUT_DMG=""
QT_PREFIX="${QT_PREFIX:-}"
QML_DIR=""
VOLNAME="Supertooth"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --version) VERSION="$2"; shift 2 ;;
        --qt-prefix) QT_PREFIX="$2"; shift 2 ;;
        --qml-dir) QML_DIR="$2"; shift 2 ;;
        --volname) VOLNAME="$2"; shift 2 ;;
        --output) OUTPUT_DMG="$2"; shift 2 ;;
        -h|--help) usage ;;
        -*)
            echo "Unknown option: $1"; exit 1 ;;
        *)
            if [[ -z "$BUILD_DIR" ]]; then
                BUILD_DIR="$1"; shift
            else
                echo "Unexpected argument: $1"; exit 1
            fi
            ;;
    esac
done

if [[ -z "$BUILD_DIR" || -z "$OUTPUT_DMG" ]]; then
    usage
fi

if [[ -z "$QML_DIR" ]]; then
    QML_DIR="$REPO_ROOT/src/apps/gui/qml"
fi

# Strip 'v' prefix (tags look like v1.2.3).
VERSION="${VERSION#v}"

APP_SRC="$BUILD_DIR/src/apps/gui/Supertooth.app"
CLI_DIR="$BUILD_DIR/src/apps/cli"
CLIS=(supertooth-bredr supertooth-ble supertooth-hybrid)

if [[ ! -d "$APP_SRC" ]]; then
    echo "Error: GUI bundle not found at $APP_SRC."
    echo "Was the project configured with -DBUILD_GUI=ON?"
    exit 1
fi
for cli in "${CLIS[@]}"; do
    if [[ ! -f "$CLI_DIR/$cli" ]]; then
        echo "Error: CLI binary not found at $CLI_DIR/$cli."
        exit 1
    fi
done

# Resolve macdeployqt.
MACDEPLOYQT=""
if [[ -n "$QT_PREFIX" && -x "$QT_PREFIX/bin/macdeployqt" ]]; then
    MACDEPLOYQT="$QT_PREFIX/bin/macdeployqt"
elif command -v macdeployqt &>/dev/null; then
    MACDEPLOYQT="macdeployqt"
elif command -v brew &>/dev/null; then
    BREW_QT_PREFIX="$(brew --prefix qt@6 2>/dev/null || true)"
    if [[ -n "$BREW_QT_PREFIX" && -x "$BREW_QT_PREFIX/bin/macdeployqt" ]]; then
        MACDEPLOYQT="$BREW_QT_PREFIX/bin/macdeployqt"
    fi
fi
if [[ -z "$MACDEPLOYQT" ]]; then
    echo "Error: macdeployqt not found. Install Qt 6 via: brew install qt@6"
    exit 1
fi

STAGING=$(mktemp -d)
trap 'rm -rf "$STAGING"' EXIT

echo "=== Staging Supertooth.app ==="
APP="$STAGING/Supertooth.app"
cp -a "$APP_SRC" "$APP"
chmod -R u+w "$APP"

echo "=== macdeployqt (Qt frameworks, plugins, QML imports) ==="
# macdeployqt prints "Cannot resolve rpath" ERRORs for every plugin whose
# dependencies are not installed. Verified by sweeping the deployed bundle
# (otool -L over every Mach-O): all of these belong to plugins this app
# never loads (qpdf, virtual keyboard, Qt3D scene/physics, state machine,
# timeline blend trees, spatial audio — our QML imports only QtQuick,
# Controls, Layouts and Graphs), so they are benign. Anything else macdeployqt
# prints is left visible. The `2> >(...)` redirection (not a pipe) filters
# stderr while preserving macdeployqt's own exit status for `$?` below.
# Note: this step takes several minutes; the time is the framework copy
# itself, not the failed lookups.
set +e
"$MACDEPLOYQT" "$APP" -qmldir="$QML_DIR" -always-overwrite \
    2> >(grep -v -e 'Cannot resolve rpath' -e 'using QList' >&2)
MACDEPLOYQT_STATUS=$?
set -e
echo "macdeployqt exit code: $MACDEPLOYQT_STATUS"

echo "=== Verifying Qt deployment ==="
FRAMEWORKS_DIR="$APP/Contents/Frameworks"
missing=0
while IFS= read -r -d '' exe; do
    while IFS= read -r dep; do
        case "$dep" in
            @rpath/*.framework/*)
                rel="${dep#@rpath/}"
                if [[ ! -e "$FRAMEWORKS_DIR/$rel" && ! -e "$FRAMEWORKS_DIR/${rel%%/*}" ]]; then
                    echo "  MISSING: $(basename "$exe") needs $dep"
                    missing=1
                fi
                ;;
        esac
    done < <(otool -L "$exe" | tail -n +2 | awk '{print $1}')
done < <(find "$APP/Contents/MacOS" -type f -perm +111 -print0)
if [[ "$missing" -ne 0 ]]; then
    echo "Error: macdeployqt deployment is incomplete (exit $MACDEPLOYQT_STATUS)"
    exit 1
fi
if [[ "$MACDEPLOYQT_STATUS" -ne 0 ]]; then
    echo "Warning: macdeployqt exited $MACDEPLOYQT_STATUS but all referenced Qt frameworks are present; continuing"
fi

echo "=== Copying CLI tools into Contents/MacOS ==="
MACOS_DIR="$APP/Contents/MacOS"
for cli in "${CLIS[@]}"; do
    cp -a "$CLI_DIR/$cli" "$MACOS_DIR/"
done

# ------------------------------------------------------------------
# Bundle non-system shared libraries into Contents/Frameworks.
# Anything a bundled binary (or bundled dylib) references from
# /opt/homebrew, /usr/local or /opt/local is copied in and the
# referencing load command is rewritten to
# @executable_path/../Frameworks/<basename>. Runs to a fixpoint so
# transitive deps (e.g. hackrf -> libusb, liquid -> fftw) are caught.
# System (/usr/lib, /System) and framework (@rpath, Qt .framework)
# references are left alone — macdeployqt already handled Qt.
# ------------------------------------------------------------------
mkdir -p "$FRAMEWORKS_DIR"

realpath_py() {
    python3 -c 'import os,sys; print(os.path.realpath(sys.argv[1]))' "$1"
}

is_bundled_ref() {
    case "$1" in
        /opt/homebrew/*|/usr/local/*|/opt/local/*) return 0 ;;
        *) return 1 ;;
    esac
}

echo "=== Bundling Homebrew/local dylibs ==="
# Normalize the LC_ID of every dylib macdeployqt staged: some Homebrew
# dylibs (e.g. libbrotlicommon) keep their absolute install path as their
# id, which `otool -L` reports like a dependency. Rewriting an id requires
# `install_name_tool -id` (-change only affects LC_LOAD_DYLIB), so fix ids
# up front and distinguish id-vs-load in the loop below.
while IFS= read -r -d '' dylib; do
    id_name="$(otool -D "$dylib" 2>/dev/null | tail -n 1 || true)"
    if is_bundled_ref "$id_name"; then
        install_name_tool -id "@executable_path/../Frameworks/$(basename "$dylib")" "$dylib"
    fi
done < <(find "$FRAMEWORKS_DIR" -type f -name '*.dylib' -print0)

for _pass in 1 2 3 4 5 6; do
    changed=0
    # Every Mach-O we are responsible for: executables + bundled dylibs.
    while IFS= read -r -d '' target; do
        target_id="$(otool -D "$target" 2>/dev/null | tail -n 1 || true)"
        while IFS= read -r dep; do
            is_bundled_ref "$dep" || continue
            base="$(basename "$dep")"
            dest="$FRAMEWORKS_DIR/$base"
            if [[ ! -f "$dest" ]]; then
                # Resolve through any Homebrew versioned symlinks so the
                # bundled file is real (install_name_tool needs that).
                src_real="$(realpath_py "$dep")"
                cp -a "$src_real" "$dest"
                chmod u+w "$dest"
                # Give the dylib itself a bundle-relative identity so
                # anything linking it resolves inside the app.
                install_name_tool -id "@executable_path/../Frameworks/$base" "$dest" 2>/dev/null || true
                changed=1
            fi
            # Rewrite this reference (idempotent if already rewritten).
            # A dylib's own id shows up in `otool -L` and needs -id, not
            # -change (which silently leaves LC_ID_DYLIB untouched and
            # would keep this loop from ever converging).
            if otool -L "$target" | tail -n +2 | awk '{print $1}' | grep -qxF "$dep"; then
                if [[ "$dep" == "$target_id" ]]; then
                    install_name_tool -id "@executable_path/../Frameworks/$base" "$target"
                else
                    install_name_tool -change "$dep" "@executable_path/../Frameworks/$base" "$target"
                fi
                changed=1
            fi
        done < <(otool -L "$target" | tail -n +2 | awk '{print $1}')
    done < <(find "$MACOS_DIR" "$FRAMEWORKS_DIR" -type f \( -perm +111 -o -name '*.dylib' \) -print0)
    if [[ "$changed" -eq 0 ]]; then
        echo "  fixpoint reached after pass $_pass"
        break
    fi
    if [[ "$_pass" -eq 6 ]]; then
        echo "Error: dylib bundling did not converge"
        exit 1
    fi
done

# ------------------------------------------------------------------
# Verify: no absolute Homebrew/local refs may remain.
# ------------------------------------------------------------------
echo "=== Verifying linkage ==="
fail=0
while IFS= read -r -d '' target; do
    while IFS= read -r dep; do
        if is_bundled_ref "$dep"; then
            echo "  ERROR: $target still references $dep"
            fail=1
        fi
    done < <(otool -L "$target" | tail -n +2 | awk '{print $1}')
done < <(find "$MACOS_DIR" "$FRAMEWORKS_DIR" -type f \( -perm +111 -o -name '*.dylib' \) -print0)
if [[ "$fail" -ne 0 ]]; then
    exit 1
fi
echo "  All linkage resolves inside the bundle or to system libraries"

echo "=== Ad-hoc codesign ==="
codesign --force --deep --sign - "$APP"

# ------------------------------------------------------------------
# Build the .dmg with hdiutil (preinstalled, no extra dependency).
# ------------------------------------------------------------------
echo "=== Creating .dmg ==="
DMG_STAGING=$(mktemp -d)
trap 'rm -rf "$STAGING" "$DMG_STAGING"' EXIT
cp -a "$APP" "$DMG_STAGING/"
ln -s /Applications "$DMG_STAGING/Applications"

mkdir -p "$(dirname "$OUTPUT_DMG")"
rm -f "$OUTPUT_DMG"
hdiutil create -volname "$VOLNAME" \
    -srcfolder "$DMG_STAGING" -ov -format UDZO "$OUTPUT_DMG"
hdiutil verify "$OUTPUT_DMG" >/dev/null

echo ""
echo "Disk image built: $OUTPUT_DMG"
hdiutil imageinfo "$OUTPUT_DMG" 2>/dev/null | grep -E '^(Format|Size)' || true
