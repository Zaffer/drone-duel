"""The server ⇄ page protocol over /ws (DESIGN.md §8), defined once, here.

Text frames carry JSON `{"ch": ..., ...}`, one model per channel, a union discriminated on `ch`.
Binary frames carry only the true pose (`POSE`), the one high-rate stream. tools/gen_protocol.py
writes these models' JSON Schema to web/src/net/protocol.schema.json, and the page's types
(web/src/net/protocol.ts) are generated from that; tests/test_protocol.py fails when the schema on
disk is stale.
"""

import struct
from typing import Annotated, Any, Literal

from pydantic import BaseModel, ConfigDict, Field, TypeAdapter

from drone_duels.content.schema import (
    Controller,
    Course,
    Location,
    Vehicle,
    Wind,
)


class Msg(BaseModel):
    model_config = ConfigDict(extra="forbid", json_schema_serialization_defaults_required=True)


VideoSystem = Literal["analog", "digital"]

# Page → server.


class SessionMsg(Msg):
    """Start a vehicle at a location, switch vehicle (back at the start), reset, destroy (a
    crash: the physics stops dead), throw (a hand-launched wing), or stop."""

    ch: Literal["session"] = "session"
    action: Literal["start", "switch", "reset", "destroy", "throw", "stop"]
    location: str | None = None
    vehicle: str | None = None
    pad: int = Field(default=0, ge=0, description="the start pad: 0 player 1, 1 player 2")


class RcMsg(Msg):
    """The radio's channels, from channel 1, µs (1000-2000)."""

    ch: Literal["rc"] = "rc"
    seq: int
    us: list[int] = Field(min_length=4, max_length=16)


class GroundMsg(Msg):
    """The ground's height above sea level under the vehicle, from the map's 3D tiles."""

    ch: Literal["ground"] = "ground"
    lat: float
    lon: float
    alt: float


class WindMsg(Msg):
    """The wind now (the Wind options tab); `gust_now` starts a gust at once (G)."""

    ch: Literal["wind"] = "wind"
    wind: Wind
    gust_now: bool = False


class VideoMsg(Msg):
    ch: Literal["video"] = "video"
    system: VideoSystem


class DynamicsMsg(Msg):
    """Stream the flight model's internals (the dynamics overlay) or stop."""

    ch: Literal["dynamics"] = "dynamics"
    on: bool


class RadioSourceMsg(Msg):
    """Read the radio in the server (Linux evdev) and stream it here (RadioMsg), for when
    Chrome's Gamepad API cannot see all of it; or stop. `fly`: the server flies it straight to
    the firmware (rc.pilot), and the page sends no `rc` (DESIGN.md §12)."""

    ch: Literal["radio_source"] = "radio_source"
    on: bool
    fly: bool = False


ClientMsg = Annotated[
    SessionMsg | RcMsg | GroundMsg | WindMsg | VideoMsg | DynamicsMsg | RadioSourceMsg,
    Field(discriminator="ch"),
]
client_msg = TypeAdapter[ClientMsg](ClientMsg)

# Server → page.

SessionState = Literal["idle", "starting", "ready", "failed", "destroyed"]


class StatusMsg(Msg):
    ch: Literal["status"] = "status"
    state: SessionState
    location: str | None = None
    vehicle: str | None = None
    pad: int = 0
    step: str = Field(default="", description="what starting is doing now, or why it failed")
    video: VideoSystem = "analog"
    origin: tuple[float, float, float] | None = Field(
        default=None, description="the pad: lat, lon, ground m above sea level"
    )
    heading_deg: float = 0.0


class TelemetryMsg(Msg):
    """What a ground station sees, from MAVLink (ArduPilot) or MSP (INAV): the map and the dials
    read this, never the physics (DESIGN.md §2.2)."""

    ch: Literal["telemetry"] = "telemetry"
    armed: bool
    mode: str
    lat: float | None = None
    lon: float | None = None
    alt_amsl: float | None = None
    alt_rel: float | None = Field(default=None, description="above home, m")
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    groundspeed: float = 0.0
    airspeed: float | None = None
    climb: float = 0.0
    throttle_pct: float = 0.0
    battery_v: float | None = None
    battery_pct: float | None = None
    gps_sats: int = 0
    blockers: list[str] = Field(default_factory=list, description="what stops it arming")


class OsdMsg(Msg):
    """The firmware's own OSD screen, as MSP DisplayPort drew it: character codes row by row."""

    ch: Literal["osd"] = "osd"
    font: str = Field(description="the font's name, served at /osd/font/{font}.png")
    cols: int
    rows: int
    codes: list[int]


class PlanItem(Msg):
    seq: int
    command: int
    lat: float
    lon: float
    alt: float
    frame: int


class PlanMsg(Msg):
    """The vehicle's uploaded mission, fence and rally points (ArduPilot)."""

    ch: Literal["plan"] = "plan"
    mission: list[PlanItem] = Field(default_factory=list)
    fence: list[PlanItem] = Field(default_factory=list)
    rally: list[PlanItem] = Field(default_factory=list)


class DynamicsData(Msg):
    ch: Literal["dynamics"] = "dynamics"
    data: dict[str, Any]


class LogMsg(Msg):
    """A line from the firmware (STATUSTEXT) or the session."""

    ch: Literal["log"] = "log"
    level: Literal["info", "warn", "error"]
    source: str
    text: str


class PhysicsMsg(Msg):
    """What the physics says the vehicle is doing, on each change: "ground", "hand", "flying",
    "crashed"; and whether its firmware is stepping it."""

    ch: Literal["physics"] = "physics"
    phase: str
    connected: bool


class RadioMsg(Msg):
    """The radio as the server reads it: every axis, -1..1, nothing reversed, and the channels
    as the server maps them (its profile, the player's calibration); name None with no radio. At
    60 Hz while a page asks (RadioSourceMsg)."""

    ch: Literal["radio"] = "radio"
    name: str | None
    axes: list[float]
    channels: list[float] = Field(default_factory=list)


ServerMsg = Annotated[
    StatusMsg | TelemetryMsg | OsdMsg | PlanMsg | DynamicsData | LogMsg | RadioMsg | PhysicsMsg,
    Field(discriminator="ch"),
]


# REST.


class ContentMsg(Msg):
    locations: dict[str, Location]
    vehicles: dict[str, Vehicle]
    courses: dict[str, Course]
    controllers: dict[str, Controller]


class MapKeys(Msg):
    google_maps_api_key: str | None = None
    cesium_ion_token: str | None = None


class ConfigMsg(Msg):
    map_keys: MapKeys
    qgc_port: int
    version: str


# The pose: binary, little-endian float64, POSE_FIELDS in order.

POSE_FIELDS = (
    "t",  # physics time, s
    "lat",
    "lon",
    "alt",  # m above sea level
    "qw",
    "qx",
    "qy",
    "qz",  # attitude, body (forward, right, down) to NED
    "vn",
    "ve",
    "vd",  # m/s
    "p",
    "q",
    "r",  # body rates, rad/s
    "a0",
    "a1",
    "a2",
    "a3",  # actuators: quad rotor speeds rad/s; wing elevon L, R rad, prop rad/s, 0
)
POSE = struct.Struct(f"<{len(POSE_FIELDS)}d")


def _for_typescript(node: Any, top: bool = False) -> Any:  # noqa: ANN401 - any JSON
    """What json-schema-to-typescript reads well: tuples as draft-07 `items` lists (it does not
    know 2020-12's `prefixItems`), and titles only on the definitions, so each field is typed
    inline rather than as an alias of its own."""
    if isinstance(node, list):
        return [_for_typescript(n) for n in node]
    if not isinstance(node, dict):
        return node
    out = {k: _for_typescript(v) for k, v in node.items() if k != "title" or top}
    if "prefixItems" in out:
        out["items"] = out.pop("prefixItems")
        out["additionalItems"] = False
    return out


def schema() -> dict[str, Any]:
    """Every model the page uses, as one JSON Schema with a definition each."""
    models: list[type[BaseModel]] = [
        SessionMsg,
        RcMsg,
        GroundMsg,
        WindMsg,
        VideoMsg,
        DynamicsMsg,
        RadioSourceMsg,
        StatusMsg,
        TelemetryMsg,
        OsdMsg,
        PlanMsg,
        DynamicsData,
        LogMsg,
        RadioMsg,
        PhysicsMsg,
        ContentMsg,
        ConfigMsg,
    ]
    defs: dict[str, Any] = {}
    for m in models:
        s = m.model_json_schema(ref_template="#/$defs/{model}", mode="serialization")
        defs.update(s.pop("$defs", {}))
        defs[m.__name__] = s
    defs = {name: _for_typescript(d, top=True) for name, d in defs.items()}
    return {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "title": "Protocol",
        "type": "object",
        "$defs": defs,
        "properties": {name: {"$ref": f"#/$defs/{name}"} for name in sorted(defs)},
        "poseFields": list(POSE_FIELDS),
    }
