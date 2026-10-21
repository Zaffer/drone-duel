#!/usr/bin/env bash
# One-time setup: Python env, ArduPilot SITL, mavlink-router, 3D viewer libraries.
# QGroundControl is optional (WITH_QGC=1); nothing in the stack needs it.
# Everything lands in external/ (git-ignored). Re-running skips finished steps.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
EXT="$ROOT/external"
AP_TAG="${AP_TAG:-Copter-4.7.1}"
QGC_VERSION="${QGC_VERSION:-v5.1.4}"
WITH_QGC="${WITH_QGC:-0}"
# gcc-14 where installed, else the system compiler.
if [ -z "${CC:-}" ]; then command -v gcc-14 >/dev/null && CC=gcc-14 || CC=gcc; fi
if [ -z "${CXX:-}" ]; then command -v g++-14 >/dev/null && CXX=g++-14 || CXX=g++; fi
export CC CXX
# meson and ninja from PATH, else through uvx, so no system packages are needed.
MESON=(meson)
NINJA=(ninja)
command -v meson >/dev/null || MESON=(uvx --with ninja meson)
command -v ninja >/dev/null || NINJA=(uvx ninja)
mkdir -p "$EXT"

cd "$ROOT"
uv sync --group sitl

if [ ! -d "$EXT/ardupilot/.git" ]; then
    git clone --branch "$AP_TAG" --depth 1 --recurse-submodules --shallow-submodules \
        https://github.com/ArduPilot/ardupilot.git "$EXT/ardupilot"
fi
if [ ! -x "$EXT/ardupilot/build/sitl/bin/arducopter" ]; then
    (cd "$EXT/ardupilot" && uv run --project "$ROOT" ./waf configure --board sitl \
        && uv run --project "$ROOT" ./waf copter)
fi

if [ ! -d "$EXT/mavlink-router/.git" ]; then
    git clone --depth 1 --recurse-submodules --shallow-submodules \
        https://github.com/mavlink-router/mavlink-router.git "$EXT/mavlink-router"
fi
if [ ! -x "$EXT/mavlink-router/build/src/mavlink-routerd" ]; then
    (cd "$EXT/mavlink-router" \
        && "${MESON[@]}" setup build --buildtype=release -Dsystemdsystemunitdir=/tmp/unused \
        && "${NINJA[@]}" -C build)
fi

QGC="$EXT/QGroundControl-x86_64.AppImage"
if [ "$WITH_QGC" = 1 ] && [ ! -x "$QGC" ]; then
    curl -L -o "$QGC" \
        "https://github.com/mavlink/qgroundcontrol/releases/download/$QGC_VERSION/QGroundControl-x86_64.AppImage"
    chmod +x "$QGC"
fi
# Unpacked, so QGC runs without mounting the AppImage: on Ubuntu 26.04 fusermount3 refuses
# the mount from QGC's AppImage runtime ("file descriptor 5 is not a socket").
if [ "$WITH_QGC" = 1 ] && [ ! -x "$EXT/qgc/AppRun" ]; then
    (cd "$EXT" && rm -rf squashfs-root qgc && "$QGC" --appimage-extract >/dev/null && mv squashfs-root qgc)
fi

# Browser libraries for the 3D viewer (`swarm view`), unpacked from npm tarballs so no Node is
# needed. Served from external/web/ by swarm.view.
WEB="$EXT/web"
CESIUM_VERSION="${CESIUM_VERSION:-1.145.0}"
MILSYMBOL_VERSION="${MILSYMBOL_VERSION:-3.0.4}"
MGRS_VERSION="${MGRS_VERSION:-2.2.0}"
RAPIER_VERSION="${RAPIER_VERSION:-0.20.0}"
TRYSTERO_VERSION="${TRYSTERO_VERSION:-0.26.0}"
SECP256K1_VERSION="${SECP256K1_VERSION:-3.2.0}"
fetch_npm() {  # package version [directory under external/web, default the package]
    local dir="$WEB/${3:-$1}"
    if [ ! -f "$dir/.version-$2" ]; then
        rm -rf "${dir:?}"
        mkdir -p "$dir"
        curl -fsSL "https://registry.npmjs.org/$1/-/${1##*/}-$2.tgz" \
            | tar -xz -C "$dir" --strip-components=1
        touch "$dir/.version-$2"
    fi
}
fetch_npm cesium "$CESIUM_VERSION"
fetch_npm milsymbol "$MILSYMBOL_VERSION"
fetch_npm mgrs "$MGRS_VERSION"
# Physics for the drive page (`swarm drive`), its WebAssembly inlined.
fetch_npm @dimforge/rapier3d-compat "$RAPIER_VERSION" rapier
# Peer-to-peer WebRTC for the duel (duel.js): Trystero over Nostr relays, as ES modules that
# index.html and drive.html map by name.
fetch_npm trystero "$TRYSTERO_VERSION"
fetch_npm @trystero-p2p/nostr "$TRYSTERO_VERSION" trystero-nostr
fetch_npm @trystero-p2p/core "$TRYSTERO_VERSION" trystero-core
fetch_npm @noble/secp256k1 "$SECP256K1_VERSION" noble-secp256k1
# The drone model is committed with the viewer (src/swarm/web/models/, built by swarm.fpvmodel).

echo "setup complete"
