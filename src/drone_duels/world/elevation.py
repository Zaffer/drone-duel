"""Ground elevation for the sim's home, looked up online when mission.toml says "auto".

Open-Meteo's elevation API (Copernicus DEM, 90 m grid, no key). Each answer is remembered in
logs/elevation.json, so a field looked up once starts offline afterwards.
"""

import json
import logging
import urllib.error
import urllib.request
from pathlib import Path

log = logging.getLogger(__name__)

ELEVATION_URL = "https://api.open-meteo.com/v1/elevation"
CACHE_PATH = Path(__file__).resolve().parents[3] / "logs" / "elevation.json"
TIMEOUT_S = 10.0


class ElevationError(RuntimeError):
    pass


def ground_elevation_m(lat_deg: float, lon_deg: float, cache_path: Path = CACHE_PATH) -> float:
    """The ground's elevation AMSL at a point, in metres."""
    key = f"{lat_deg:.6f},{lon_deg:.6f}"
    cache = _read_cache(cache_path)
    if key in cache:
        return cache[key]
    url = f"{ELEVATION_URL}?latitude={lat_deg:.6f}&longitude={lon_deg:.6f}"
    try:
        with urllib.request.urlopen(url, timeout=TIMEOUT_S) as response:
            elevation = float(json.load(response)["elevation"][0])
    except (
        urllib.error.URLError,
        TimeoutError,
        KeyError,
        IndexError,
        TypeError,
        ValueError,
    ) as exc:
        raise ElevationError(
            f"could not look up the ground elevation at {key} ({exc}); set [sim] ground_alt_m "
            "in mission.toml to a number"
        ) from None
    log.info("ground at %s: %.0f m AMSL (Open-Meteo)", key, elevation)
    cache[key] = elevation
    cache_path.parent.mkdir(parents=True, exist_ok=True)
    cache_path.write_text(json.dumps(cache, indent=1, sort_keys=True))
    return elevation


def _read_cache(path: Path) -> dict[str, float]:
    try:
        data = json.loads(path.read_text())
    except (OSError, ValueError):
        return {}
    if not isinstance(data, dict):
        return {}
    return {
        str(k): float(v)  # pyright: ignore[reportUnknownArgumentType]
        for k, v in data.items()  # pyright: ignore[reportUnknownVariableType]
        if isinstance(v, int | float)
    }
