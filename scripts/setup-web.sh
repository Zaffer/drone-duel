#!/usr/bin/env bash
# The web version's simulator (DESIGN.md §18): Betaflight and the quad's plant as one
# WebAssembly module, external/web/sim/sim.wasm, which the page's worker flies
# (web/src/sim/). Re-running rebuilds what changed.
#
# Betaflight is cudaflight's SITL_LOCKSTEP target (Synetic-Labs/cudaflight: Betaflight on a
# virtual clock, stepped by direct call, no threads or sockets), built with Emscripten and
# scripts/cudaflight-web.patch (change the patch, not the checkout), with web/wasm/bfweb.c
# as its harness. The plant is SkyFlow-Dynamics' generated C (web/wasm/sfd.c) behind
# web/wasm/plant.c. Everything lands in external/ (git-ignored).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
EXT="$ROOT/external"
EMSDK_VERSION="${EMSDK_VERSION:-7.0.0}"
# cudaflight 0.6.0, on Betaflight 2026.6.1.
CUDAFLIGHT_REF="${CUDAFLIGHT_REF:-74dd44a0a5755a0b11626ce29732862126df016c}"
CF="$EXT/cudaflight"
OUT="$EXT/web/sim"
mkdir -p "$EXT" "$OUT"

if [ ! -d "$EXT/emsdk/.git" ]; then
    git clone --depth 1 https://github.com/emscripten-core/emsdk.git "$EXT/emsdk"
fi
if [ ! -x "$EXT/emsdk/upstream/emscripten/emcc" ] ||
    ! "$EXT/emsdk/upstream/emscripten/emcc" --version 2>/dev/null | grep -q " $EMSDK_VERSION "; then
    (cd "$EXT/emsdk" && git pull -q --ff-only || true
        ./emsdk install "$EMSDK_VERSION" && ./emsdk activate "$EMSDK_VERSION")
fi
# shellcheck disable=SC1091
EMSDK_QUIET=1 source "$EXT/emsdk/emsdk_env.sh"

if [ ! -d "$CF/.git" ]; then
    git init -q "$CF"
    git -C "$CF" remote add origin https://github.com/Synetic-Labs/cudaflight.git
fi
if [ "$(git -C "$CF" rev-parse HEAD 2>/dev/null)" != "$CUDAFLIGHT_REF" ]; then
    git -C "$CF" fetch -q --depth 1 origin "$CUDAFLIGHT_REF"
    git -C "$CF" checkout -q -f FETCH_HEAD
fi
# The two submodules this target needs, one at a time: the build's own fetching runs them in
# parallel, and they race on .git/config.
for module in src/config lib/modules/dronecan/libcanard; do
    [ -e "$CF/$module/.git" ] || git -C "$CF" submodule update --init --depth 1 "$module"
done
# A checkout patched with an older patch is put back and patched again.
if ! git -C "$CF" diff --quiet &&
    ! git -C "$CF" apply --reverse --check "$ROOT/scripts/cudaflight-web.patch" 2>/dev/null; then
    git -C "$CF" checkout -- .
fi
if git -C "$CF" diff --quiet; then
    git -C "$CF" apply "$ROOT/scripts/cudaflight-web.patch"
fi
cmp -s "$ROOT/web/wasm/bfweb.c" "$CF/src/platform/SIMULATOR/bfweb.c" ||
    cp "$ROOT/web/wasm/bfweb.c" "$CF/src/platform/SIMULATOR/bfweb.c"

# The firmware's objects, linked into one relocatable object (-r): the last link is below,
# with the plant. SIZE is a no-op (there is no size for WebAssembly objects).
make -C "$CF" TARGET=SITL_LOCKSTEP CROSS_CC=emcc BFL_WEB=yes LD_FLAGS=-r SIZE=true \
    -j"$(nproc)" >"$EXT/cudaflight-web-build.log" 2>&1 || {
    tail -40 "$EXT/cudaflight-web-build.log"
    echo "the firmware's build failed: see $EXT/cudaflight-web-build.log" >&2
    exit 1
}

# Standalone: no JavaScript glue, only the few WASI calls web/src/sim/firmware.ts answers.
# Betaflight has its own strtol (common/strtol.c), which stands in for libc's: the first
# definition wins. The plant is built without fast-math: SkyFlow-Dynamics' golden vectors
# hold to 1e-9 that way.
emcc -O2 -c "$ROOT/web/wasm/sfd.c" -o "$OUT/sfd.o"
emcc -O2 -c "$ROOT/web/wasm/plant.c" -o "$OUT/plant.o"
emcc -O3 -sSTANDALONE_WASM --no-entry -sALLOW_MEMORY_GROWTH=1 -sINITIAL_MEMORY=16MB \
    -sSTACK_SIZE=1MB -Wl,--allow-multiple-definition \
    "$CF/obj/main/betaflight_SITL_LOCKSTEP.elf" "$OUT/sfd.o" "$OUT/plant.o" \
    -o "$OUT/sim.wasm"
rm -f "$OUT/sfd.o" "$OUT/plant.o"
# Betaflight's OSD font, from its configurator (GPL-3.0), as scripts/setup.sh fetches it: the
# web version's data has it as a picture (drone_duels.webdata).
BETAFLIGHT_FONT_REF=9f4a52d2702ca0ab109d77b31633b749c5198e47
mkdir -p "$EXT/web/betaflight"
[ -f "$EXT/web/betaflight/default.mcm" ] ||
    curl -fsSL -o "$EXT/web/betaflight/default.mcm" \
        "https://raw.githubusercontent.com/betaflight/betaflight-configurator/$BETAFLIGHT_FONT_REF/resources/osd/2/default.mcm"

echo "built $OUT/sim.wasm ($(stat -c %s "$OUT/sim.wasm") bytes)"
