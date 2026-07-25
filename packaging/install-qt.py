#!/usr/bin/env python3
"""
Download and extract a prebuilt Qt for Linux from the official Qt SDK repository.

Usage:
  python3 install-qt.py --version 6.8.0 --modules qtgraphs --output /opt/Qt

The base package (which includes qtbase, qtdeclarative, qtquicklayouts, etc.)
is always installed. Extra modules are specified via --modules (comma-separated
or repeated).

Requires: requests, py7zr
"""

import argparse
import hashlib
import os
import re
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import requests
import py7zr

QT_MIRROR = "https://download.qt.io/online/qtsdkrepository/linux_x64/desktop"


def version_to_dir(ver: str) -> tuple[str, str]:
    """Convert '6.8.0' to (url_dir='qt6_680', pkg_prefix='qt.qt6.680')."""
    parts = ver.split(".")
    if len(parts) != 3:
        raise ValueError(f"Expected X.Y.Z version, got: {ver}")
    ver_compact = f"{parts[0]}{parts[1]}{parts[2]}"
    return f"qt6_{ver_compact}", f"qt.qt6.{ver_compact}"


def sha1_file(path: str) -> str:
    h = hashlib.sha1()
    with open(path, "rb") as f:
        while True:
            chunk = f.read(65536)
            if not chunk:
                break
            h.update(chunk)
    return h.hexdigest()


def find_package(root: ET.Element, name: str) -> dict:
    for pkg in root.findall(".//PackageUpdate"):
        if pkg.find("Name").text == name:
            archives_str = pkg.find("DownloadableArchives")
            archives = [a.strip() for a in archives_str.text.split(",")] if archives_str is not None and archives_str.text else []
            sha1 = pkg.findtext("SHA1", "")
            ver = pkg.findtext("Version", "")
            # The version prefix for archive filenames (e.g. "6.8.0-0-202410030750")
            # is the version up to the first dash group.
            return {"name": name, "archives": archives, "sha1": sha1, "version": ver}
    raise ValueError(f"Package '{name}' not found in Updates.xml")


def download_archive(session: requests.Session, base_url: str, pkg_dir: str, archive: str, version_prefix: str, dest_dir: str) -> str:
    """Download a single .7z archive to dest_dir. Returns the local path.
    
    The actual filename on the server is prefixed with the package version.
    Archives are located in the package subdirectory.
    """
    actual_name = f"{version_prefix}{archive}"
    url = f"{base_url}/{pkg_dir}/{actual_name}"
    local_path = os.path.join(dest_dir, actual_name)

    if os.path.exists(local_path):
        print(f"  Already cached: {actual_name}")
        return local_path

    print(f"  Downloading: {actual_name}")
    resp = session.get(url, stream=True, timeout=300)
    resp.raise_for_status()
    with open(local_path, "wb") as f:
        for chunk in resp.iter_content(chunk_size=8192):
            f.write(chunk)
    return local_path


def extract_7z(archive_path: str, target_dir: str, strip_prefix: str = ""):
    """Extract a .7z archive. If strip_prefix is set, remove that prefix from paths.
    
    The strip_prefix is a path like '6.8.0/gcc_64/' — files within that subdirectory
    are moved to the root of target_dir.
    """
    print(f"  Extracting: {os.path.basename(archive_path)}")
    tmp = tempfile.mkdtemp()
    try:
        with py7zr.SevenZipFile(archive_path, mode="r") as sz:
            sz.extractall(path=tmp)

        # Determine the actual source root after extraction
        # If strip_prefix is set, walk into it; otherwise use the extraction root
        src_root = tmp
        if strip_prefix:
            candidate = os.path.join(tmp, strip_prefix)
            if os.path.exists(candidate):
                src_root = candidate

        # Walk the source root and copy every entry to target_dir
        for dirpath, dirnames, filenames in os.walk(src_root):
            rel_dir = os.path.relpath(dirpath, src_root)
            if rel_dir == ".":
                rel_dir = ""
            for name in dirnames + filenames:
                src = os.path.join(dirpath, name)
                dst = os.path.join(target_dir, rel_dir, name)
                if os.path.isdir(src):
                    os.makedirs(dst, exist_ok=True)
                else:
                    os.makedirs(os.path.dirname(dst), exist_ok=True)
                    shutil.copy2(src, dst)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def install_qt(version: str, extra_modules: list[str], output_dir: str):
    ver_dir, pkg_prefix = version_to_dir(version)
    base_url = f"{QT_MIRROR}/{ver_dir}/{ver_dir}"
    updates_url = f"{base_url}/Updates.xml"

    print(f"Fetching: {updates_url}")
    resp = requests.get(updates_url, timeout=30)
    resp.raise_for_status()
    root = ET.fromstring(resp.text)

    # Determine package names
    arch = "linux_gcc_64"
    base_pkg_name = f"{pkg_prefix}.{arch}"
    # Also check if the package uses the new naming scheme
    # Older scheme: qt.qt6.680.linux_gcc_64
    # We already verified this works for 6.8.0

    packages_to_download = [base_pkg_name]
    for mod in extra_modules:
        modules_pkg = f"{pkg_prefix}.addons.{mod}.{arch}"
        packages_to_download.append(modules_pkg)

    os.makedirs(output_dir, exist_ok=True)

    # Resolve package info
    pkg_infos = []
    for pkg_name in packages_to_download:
        info = find_package(root, pkg_name)
        pkg_infos.append(info)
        print(f"  Found: {pkg_name} v{info['version']}")
        for a in info["archives"]:
            print(f"    -> {a}")

    # Download all archives
    cache_dir = os.path.join(output_dir, ".cache")
    os.makedirs(cache_dir, exist_ok=True)

    session = requests.Session()
    archive_paths = []
    for info in pkg_infos:
        pkg_dir = info["name"]
        # Version prefix: the version string up to the last '-', then appended to archive names
        ver = info["version"]
        # e.g. "6.8.0-0-202410030750" -> "6.8.0-0-202410030750"
        version_prefix = ver
        for arch_name in info["archives"]:
            path = download_archive(session, base_url, pkg_dir, arch_name, version_prefix, cache_dir)
            archive_paths.append(path)

    # Determine the Qt prefix within the archives by looking for the last
    # directory component that contains all expected entries (lib/, bin/, include/, etc.)
    with py7zr.SevenZipFile(archive_paths[0], mode="r") as sz:
        names = list(sz.getnames())
        # Find the shortest path that contains 'lib' and 'include' as subdirectories
        prefix_parts = None
        for name in names:
            parts = name.split("/")
            for i in range(1, len(parts)):
                sub = parts[i]
                if sub in ("lib", "bin", "include", "plugins", "qml"):
                    candidate = "/".join(parts[:i]) + "/"
                    if prefix_parts is None or len(candidate) < len(prefix_parts):
                        prefix_parts = candidate
        strip_prefix = prefix_parts or ""

    # Extract all archives into the output directory
    for path in archive_paths:
        extract_7z(path, output_dir, strip_prefix=strip_prefix)

    if not os.path.exists(os.path.join(output_dir, "lib")):
        print("Warning: lib/ directory not found in expected location")

    print(f"\nQt {version} installed to {output_dir}")
    print(f"  Set CMAKE_PREFIX_PATH={output_dir} to use it.")


def main():
    parser = argparse.ArgumentParser(description="Download and install Qt for Linux")
    parser.add_argument("--version", required=True, help="Qt version (e.g. 6.8.0)")
    parser.add_argument("--modules", nargs="*", default=[], help="Extra Qt modules (e.g. qtgraphs)")
    parser.add_argument("--output", required=True, help="Installation prefix directory")
    args = parser.parse_args()

    install_qt(args.version, args.modules, args.output)


if __name__ == "__main__":
    main()
