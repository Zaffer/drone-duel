"""The mission's settings file, mission.toml: points A and B, the swarm, and the sim's field.

[mission], [swarm] and [fence] hold what the sim and the field share, [sim] what only the sim
uses.
Every key must be there, and no other, so a misspelt one is an error rather than a silent
default.
"""

import tomllib
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from swarm.fence import FENCE_ACTIONS, Fence, contains, self_intersects
from swarm.geo import LocalFrame
from swarm.sim import ROOT, ROUTER_CONFS, SimField

SETTINGS_PATH = ROOT / "mission.toml"

MISSION_KEYS = {"point_a", "point_b", "copters"}
SWARM_KEYS = {
    "follower_distance_m",
    "follower_height_step_m",
    "liftoff_max_leader_distance_m",
    "follow_max_distance_m",
    "liftoff_min_leader_height_m",
    "landing_zone_radius_m",
}
FENCE_KEYS = {"enabled", "corners", "max_height_m", "action"}
FENCE_MAX_HEIGHT_M = (10.0, 1000.0)
"""FENCE_ALT_MAX's range in ArduPilot."""
SIM_KEYS = {
    "home",
    "ground_alt_m",
    "spawn_spacing_m",
    "router",
    "wind_speed_ms",
    "wind_from_deg",
    "wind_turbulence_ms",
}


class SettingsError(ValueError):
    pass


@dataclass(frozen=True)
class SimSettings:
    spawn_spacing_m: float
    """Copter 1 starts at home, the others in a line to the east, this far apart."""
    router: str
    """Which router config the copters connect through (sim.ROUTER_CONFS)."""
    wind_speed_ms: float
    wind_from_deg: float
    wind_turbulence_ms: float

    def sitl_params(self) -> dict[str, float]:
        """The SITL parameters these settings give every copter."""
        return {
            "SIM_WIND_SPD": self.wind_speed_ms,
            "SIM_WIND_DIR": self.wind_from_deg,
            "SIM_WIND_TURB": self.wind_turbulence_ms,
        }


@dataclass(frozen=True)
class SwarmSettings:
    """The formation and its limits: the numbers to tune on the field."""

    follower_distance_m: float
    """Formation radius: how far each follower flies from the leader, horizontally."""
    follower_height_step_m: float
    """Height between one follower and the next, the first one step above the leader."""
    liftoff_max_leader_distance_m: float
    """A follower lifts off only this close to the leader; the viewer's ring round it."""
    follow_max_distance_m: float
    """FOLL_DIST_MAX: a follower farther than this from the leader stops and holds."""
    liftoff_min_leader_height_m: float
    """Followers lift off only once the leader is this high above home."""
    landing_zone_radius_m: float
    """The landing ring round B, in the viewer and `swarm rehearse`."""


@dataclass(frozen=True)
class Settings:
    field: SimField
    """Points A and B, and the sim's home."""
    copters: int
    """Copters in the swarm, the leader included."""
    swarm: SwarmSettings
    fence: Fence
    sim: SimSettings


def load_settings(path: Path = SETTINGS_PATH) -> Settings:
    try:
        data = tomllib.loads(path.read_text())
    except OSError as exc:
        raise SettingsError(f"{path}: {exc.strerror}") from None
    except tomllib.TOMLDecodeError as exc:
        raise SettingsError(f"{path}: {exc}") from None
    mission = _table(data, "mission", MISSION_KEYS, path)
    swarm = _table(data, "swarm", SWARM_KEYS, path)
    fence = _table(data, "fence", FENCE_KEYS, path)
    sim = _table(data, "sim", SIM_KEYS, path)
    home = _point(sim, "home", path)
    router = sim["router"]
    if router not in ROUTER_CONFS:
        choices = sorted(ROUTER_CONFS)
        raise SettingsError(f"{path}: [sim] router is {router!r}, not one of {choices}")
    copters = mission["copters"]
    if not isinstance(copters, int) or isinstance(copters, bool) or copters < 1:
        raise SettingsError(f"{path}: [mission] copters must be a whole number, 1 or more")
    ground_alt_m = None if sim["ground_alt_m"] == "auto" else _number(sim, "ground_alt_m", path)
    settings = Settings(
        field=SimField(
            home=LocalFrame(*home),
            home_alt_m=ground_alt_m,
            point_a=_point(mission, "point_a", path),
            point_b=_point(mission, "point_b", path),
        ),
        copters=copters,
        swarm=_swarm(swarm, path),
        sim=SimSettings(
            spawn_spacing_m=_number(sim, "spawn_spacing_m", path),
            router=router,
            wind_speed_ms=_number(sim, "wind_speed_ms", path),
            wind_from_deg=_number(sim, "wind_from_deg", path),
            wind_turbulence_ms=_number(sim, "wind_turbulence_ms", path),
        ),
        fence=_fence(fence, path),
    )
    _check_inside_fence(settings, path)
    return settings


def _swarm(table: dict[str, Any], path: Path) -> SwarmSettings:
    swarm = SwarmSettings(**{key: _number(table, key, path) for key in sorted(SWARM_KEYS)})
    for key in sorted(SWARM_KEYS):
        if getattr(swarm, key) <= 0:
            raise SettingsError(f"{path}: [swarm] {key} must be more than 0")
    if swarm.follow_max_distance_m <= swarm.liftoff_max_leader_distance_m:
        raise SettingsError(
            f"{path}: [swarm] follow_max_distance_m must be more than "
            "liftoff_max_leader_distance_m, or a follower lifting off at the edge of the control "
            "area would never join"
        )
    return swarm


def _fence(table: dict[str, Any], path: Path) -> Fence:
    enabled, action, corners = table["enabled"], table["action"], table["corners"]
    if not isinstance(enabled, bool):
        raise SettingsError(f"{path}: [fence] enabled must be true or false")
    if action not in FENCE_ACTIONS:
        raise SettingsError(
            f"{path}: [fence] action is {action!r}, not one of {sorted(FENCE_ACTIONS)}"
        )
    if not isinstance(corners, list) or len(corners) < 3:  # pyright: ignore[reportUnknownArgumentType]
        raise SettingsError(f"{path}: [fence] corners must be 3 [latitude, longitude] or more")
    points = tuple(
        _point({"corner": c}, "corner", path)
        for c in corners  # pyright: ignore[reportUnknownVariableType]
    )
    if self_intersects(points):
        raise SettingsError(f"{path}: [fence] corners: two edges cross; list them in order round")
    low, high = FENCE_MAX_HEIGHT_M
    max_height_m = _number(table, "max_height_m", path)
    if not low <= max_height_m <= high:
        raise SettingsError(f"{path}: [fence] max_height_m must be {low:g} to {high:g}")
    return Fence(enabled, points, max_height_m, action)


def _check_inside_fence(settings: Settings, path: Path) -> None:
    """A, B and the sim's start line inside the fence: a drone outside does not arm."""
    fence, field = settings.fence, settings.field
    if not fence.enabled:
        return
    starts = [
        (
            f"[sim] start of copter {i + 1}",
            field.home.to_latlon(0.0, settings.sim.spawn_spacing_m * i),
        )
        for i in range(settings.copters)
    ]
    for name, (lat, lon) in [("point_a", field.point_a), ("point_b", field.point_b), *starts]:
        if not contains(fence.corners, lat, lon):
            raise SettingsError(f"{path}: {name} [{lat:.6f}, {lon:.6f}] is outside the fence")


def _table(data: dict[str, Any], name: str, keys: set[str], path: Path) -> dict[str, Any]:
    table = data.get(name)
    if not isinstance(table, dict):
        raise SettingsError(f"{path}: no [{name}] table")
    missing, unknown = keys - table.keys(), table.keys() - keys
    if missing:
        raise SettingsError(f"{path}: [{name}] is missing {', '.join(sorted(missing))}")
    if unknown:
        raise SettingsError(f"{path}: [{name}] has unknown {', '.join(sorted(unknown))}")
    return table  # pyright: ignore[reportUnknownVariableType]


def _number(table: dict[str, Any], key: str, path: Path) -> float:
    value = table[key]
    if not isinstance(value, int | float) or isinstance(value, bool):
        raise SettingsError(f"{path}: {key} must be a number, not {value!r}")
    return float(value)


def _point(table: dict[str, Any], key: str, path: Path) -> tuple[float, float]:
    value = table[key]
    if (
        not isinstance(value, list)
        or len(value) != 2  # pyright: ignore[reportUnknownArgumentType]
        or not all(isinstance(x, int | float) for x in value)  # pyright: ignore[reportUnknownVariableType]
    ):
        raise SettingsError(f"{path}: {key} must be [latitude, longitude], not {value!r}")
    lat, lon = (float(x) for x in value)  # pyright: ignore[reportUnknownVariableType, reportUnknownArgumentType]
    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
        raise SettingsError(f"{path}: {key} [{lat}, {lon}] is not a latitude and longitude")
    return lat, lon
