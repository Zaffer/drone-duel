"""`duels doctor`: what the game needs, and whether it is there."""

import shutil
from collections.abc import Callable
from pathlib import Path

from drone_duels.content.store import ContentError, load
from drone_duels.server.app import CESIUM, ENV_FILE, map_keys

ROOT = Path(__file__).resolve().parents[2]
EXTERNAL = ROOT / "external"


def _bun() -> bool:
    return bool(shutil.which("bun") or (Path.home() / ".bun/bin/bun").exists())


def checks() -> list[tuple[str, Callable[[], bool], str]]:
    return [
        ("ArduCopter SITL", lambda: (EXTERNAL / "ardupilot/build/sitl/bin/arducopter").exists(),
         "scripts/setup.sh"),
        ("INAV SITL", lambda: (EXTERNAL / "inav/inav_SITL").exists(), "scripts/setup.sh"),
        ("Betaflight SITL",
         lambda: (EXTERNAL / "betaflight/obj/main/betaflight_SITL.elf").exists(),
         "scripts/setup.sh"),
        ("mavlink-router",
         lambda: (EXTERNAL / "mavlink-router/build/src/mavlink-routerd").exists(),
         "scripts/setup.sh"),
        ("bun", _bun, "scripts/setup.sh (installs bun)"),
        ("web dependencies", lambda: (ROOT / "web/node_modules").is_dir(),
         "cd web && bun install"),
        ("Cesium assets", lambda: (CESIUM / "Cesium.js").exists(), "cd web && bun install"),
        ("the page", lambda: (ROOT / "web/dist/index.html").exists(), "cd web && bun run build"),
        ("Chupito model", lambda: (EXTERNAL / "web/chupito/chupito_v6.glb").exists(),
         "scripts/setup.sh"),
        ("OSD fonts", lambda: (EXTERNAL / "ardupilot/libraries/AP_OSD/fonts").is_dir()
         and (EXTERNAL / "web/betaflight/default.mcm").exists(), "scripts/setup.sh"),
        ("the web version's simulator", lambda: (EXTERNAL / "web/sim/sim.wasm").exists(),
         "optional: scripts/setup-web.sh, for `duels web`"),
        ("a map key", lambda: bool(map_keys().google_maps_api_key or map_keys().cesium_ion_token),
         f"optional: GOOGLE_MAPS_API_KEY in {ENV_FILE.name} for photorealistic 3D tiles"),
    ]


def doctor() -> int:
    failed = 0
    for name, ok, fix in checks():
        good = ok()
        failed += 0 if good or fix.startswith("optional") else 1
        print(f"  {'ok ' if good else 'NO '} {name}" + ("" if good else f"  ({fix})"))
    try:
        c = load()
        print(
            f"  ok  content: {len(c.locations)} locations, "
            f"{len(c.vehicles)} vehicles, {len(c.courses)} courses, "
            f"{len(c.controllers)} controllers"
        )
    except ContentError as exc:
        failed += 1
        print(f"  NO  content:\n{exc}")
    return 1 if failed else 0
