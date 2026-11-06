#!/usr/bin/env bash
# One-time setup: the Python env, ArduPilot, INAV and Betaflight SITL, mavlink-router, bun and the
# page.
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

# The page's libraries (Cesium, Rapier, Trystero, milsymbol, alien-signals, analog-vision) are
# bun dependencies, pinned in web/package.json; bun itself is installed here if missing.
BUN="$(command -v bun || echo "$HOME/.bun/bin/bun")"
if [ ! -x "$BUN" ]; then
    curl -fsSL https://bun.sh/install | bash
fi
(cd "$ROOT/web" && "$BUN" install --frozen-lockfile && "$BUN" run build)

# Files the page serves from external/web/ (the server's /vendor/).
WEB="$EXT/web"
# The Chupito's model (content/vehicles/chupito.toml): narenana's model for Nanawing, as sim.narenana.com
# serves it. Not committed: it has no licence, and it carries Team BlackSheep's logos.
CHUPITO_GLB=chupito_v6.glb
mkdir -p "$WEB/chupito"
[ -f "$WEB/chupito/$CHUPITO_GLB" ] ||
    curl -fsSL -o "$WEB/chupito/$CHUPITO_GLB" "https://sim.narenana.com/assets/models/real/$CHUPITO_GLB"

# Betaflight's OSD font, from its configurator (GPL-3.0): what DJI's goggles draw an MSP
# DisplayPort OSD in, so the Chupito's INAV OSD too (drone_duels.firmware.inav).
BETAFLIGHT_FONT_REF=9f4a52d2702ca0ab109d77b31633b749c5198e47
mkdir -p "$WEB/betaflight"
[ -f "$WEB/betaflight/default.mcm" ] ||
    curl -fsSL -o "$WEB/betaflight/default.mcm" \
        "https://raw.githubusercontent.com/betaflight/betaflight-configurator/$BETAFLIGHT_FONT_REF/resources/osd/2/default.mcm"

# INAV SITL, the Chupito's firmware (drone_duels.firmware.inav). Building INAV needs Ruby, so this is
# the binary INAV's Configurator ships for its SITL tab (Configurator 9.1.1: INAV 9.1.0).
INAV_CONFIGURATOR_VERSION=9.1.1
if [ ! -x "$EXT/inav/inav_SITL" ]; then
    mkdir -p "$EXT/inav"
    zip="$(mktemp --suffix=.zip)"
    curl -fsSL -o "$zip" "https://github.com/iNavFlight/inav-configurator/releases/download/$INAV_CONFIGURATOR_VERSION/INAV-Configurator_linux_x64_$INAV_CONFIGURATOR_VERSION.zip"
    unzip -j -o "$zip" '*/resources/sitl/linux/inav_SITL' -d "$EXT/inav"
    chmod +x "$EXT/inav/inav_SITL"
    rm -f "$zip"
fi
# Betaflight SITL, the Betaflight 5" quad's firmware (drone_duels.firmware.betaflight): built from
# source (gcc and make), with scripts/betaflight-sitl.patch (its ports as arguments, the OSD).
# Submodules are fetched first, one at a time: the build's own fetching runs them in parallel,
# and they race on .git/config.
BF_TAG="${BF_TAG:-2026.6.2}"
BF="$EXT/betaflight"
if [ ! -d "$BF/.git" ]; then
    git clone --branch "$BF_TAG" --depth 1 https://github.com/betaflight/betaflight.git "$BF"
    git -C "$BF" submodule update --init --depth 1
fi
if ! grep -q tcpBasePort "$BF/src/main/drivers/serial_tcp.c"; then
    git -C "$BF" apply "$ROOT/scripts/betaflight-sitl.patch"
fi
if [ ! -x "$BF/obj/main/betaflight_SITL.elf" ]; then
    make -C "$BF" TARGET=SITL -j"$(nproc)"
fi

# The quad's and truck's models are committed (web/public/models/, built by tools/models/).

echo "setup complete"
