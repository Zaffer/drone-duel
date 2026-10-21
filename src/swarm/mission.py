"""Read each vehicle's mission, geofence and rally points, for display.

Whatever QGroundControl (or anyone) uploaded is stored on the vehicle, so the vehicle is
the source of truth. `PlanReader` downloads it with the MAVLink mission protocol:
MISSION_REQUEST_LIST, MISSION_COUNT, one MISSION_REQUEST_INT / MISSION_ITEM_INT per item,
MISSION_ACK. A download only reads; ArduPilot refuses it (MISSION_ACK DENIED) while
another GCS is uploading, and the upload carries on, so we just try again later.

ArduPilot 4.7 does not announce a changed mission, so the reader downloads again when the
item count in MISSION_CURRENT changes and otherwise every REFRESH_S.

`build_plan` turns the raw items into what the viewer draws: positions in lat/lon with
heights above home, takeoff/land/RTL resolved to where they happen, and fence shapes.
"""

import logging
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from typing import TypedDict

from pymavlink.dialects.v20 import ardupilotmega as apm

log = logging.getLogger(__name__)

REFRESH_S = 10.0
"""Download every plan this often, so a same-length edit is seen too."""
RETRY_S = 2.0
"""Wait this long after a refused or failed download."""
STEP_TIMEOUT_S = 1.5
STEP_RETRIES = 3
PLAN_TYPES = (apm.MAV_MISSION_TYPE_MISSION, apm.MAV_MISSION_TYPE_FENCE, apm.MAV_MISSION_TYPE_RALLY)
PLAN_PARAMS = ("FENCE_TYPE", "FENCE_RADIUS", "FENCE_ALT_MAX", "FENCE_ACTION", "RTL_ALT_M")
"""Parameters the viewer needs: the circular fence around home, its ceiling, and RTL height."""

ALT_ABOVE_HOME = frozenset(
    {apm.MAV_FRAME_GLOBAL_RELATIVE_ALT, apm.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT}
)
ALT_AMSL = frozenset({apm.MAV_FRAME_GLOBAL, apm.MAV_FRAME_GLOBAL_INT})
ALT_ABOVE_TERRAIN = frozenset(
    {apm.MAV_FRAME_GLOBAL_TERRAIN_ALT, apm.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT}
)

COMMAND_NAMES: dict[int, str] = {
    apm.MAV_CMD_NAV_WAYPOINT: "WAYPOINT",
    apm.MAV_CMD_NAV_SPLINE_WAYPOINT: "SPLINE",
    apm.MAV_CMD_NAV_LOITER_UNLIM: "LOITER",
    apm.MAV_CMD_NAV_LOITER_TURNS: "LOITER TURNS",
    apm.MAV_CMD_NAV_LOITER_TIME: "LOITER TIME",
    apm.MAV_CMD_NAV_LOITER_TO_ALT: "LOITER TO ALT",
    apm.MAV_CMD_NAV_RETURN_TO_LAUNCH: "RTL",
    apm.MAV_CMD_NAV_LAND: "LAND",
    apm.MAV_CMD_NAV_TAKEOFF: "TAKEOFF",
    apm.MAV_CMD_NAV_DELAY: "DELAY",
    apm.MAV_CMD_NAV_PAYLOAD_PLACE: "PAYLOAD PLACE",
    apm.MAV_CMD_DO_SET_ROI: "ROI",
    apm.MAV_CMD_DO_SET_ROI_LOCATION: "ROI",
    apm.MAV_CMD_DO_SET_ROI_NONE: "ROI NONE",
    apm.MAV_CMD_DO_JUMP: "JUMP",
    apm.MAV_CMD_DO_CHANGE_SPEED: "SPEED",
    apm.MAV_CMD_CONDITION_YAW: "YAW",
    apm.MAV_CMD_CONDITION_DELAY: "WAIT",
    apm.MAV_CMD_DO_SET_SERVO: "SERVO",
    apm.MAV_CMD_DO_DIGICAM_CONTROL: "PHOTO",
    apm.MAV_CMD_IMAGE_START_CAPTURE: "PHOTO",
    apm.MAV_CMD_DO_SET_CAM_TRIGG_DIST: "CAM TRIGGER",
    apm.MAV_CMD_DO_MOUNT_CONTROL: "GIMBAL",
    apm.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW: "GIMBAL",
}

NAV_COMMANDS = frozenset(
    {
        apm.MAV_CMD_NAV_WAYPOINT,
        apm.MAV_CMD_NAV_SPLINE_WAYPOINT,
        apm.MAV_CMD_NAV_LOITER_UNLIM,
        apm.MAV_CMD_NAV_LOITER_TURNS,
        apm.MAV_CMD_NAV_LOITER_TIME,
        apm.MAV_CMD_NAV_LOITER_TO_ALT,
        apm.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        apm.MAV_CMD_NAV_LAND,
        apm.MAV_CMD_NAV_TAKEOFF,
        apm.MAV_CMD_NAV_PAYLOAD_PLACE,
    }
)
"""Commands that move the vehicle somewhere: these make the drawn path."""
ROI_COMMANDS = frozenset({apm.MAV_CMD_DO_SET_ROI, apm.MAV_CMD_DO_SET_ROI_LOCATION})


@dataclass(frozen=True)
class RawItem:
    """One MISSION_ITEM_INT as the vehicle sent it."""

    seq: int
    frame: int
    command: int
    params: tuple[float, float, float, float]
    lat_deg: float
    lon_deg: float
    alt_m: float

    @property
    def has_position(self) -> bool:
        return self.lat_deg != 0.0 or self.lon_deg != 0.0


class PlanItem(TypedDict):
    seq: int
    """Mission sequence number, as MISSION_CURRENT reports it. Home is 0, so items start at 1."""
    cmd: int
    name: str
    nav: bool
    """True for items that move the vehicle; they make the path."""
    lat: float | None
    lon: float | None
    alt: float | None
    """Height above home, or above the terrain under the item when `terrain` is set."""
    terrain: bool
    params: list[float]


class FenceShape(TypedDict):
    kind: str
    """"polygon" or "circle"."""
    inclusion: bool
    points: list[list[float]]
    """Polygon vertices as [lat, lon], or the circle centre as the only point."""
    radius: float | None


class FenceParams(TypedDict):
    enabled: bool
    types: int
    """FENCE_TYPE bitmask: 1 max altitude, 2 circle around home, 4 polygons, 8 min altitude."""
    radius: float | None
    alt_max: float | None
    action: int | None


class Plan(TypedDict):
    kind: str
    """Always "plan", which tells it apart from a telemetry snapshot."""
    t: float
    sysid: int
    home: list[float] | None
    """[lat, lon, ground AMSL] from mission item 0."""
    items: list[PlanItem]
    fence: list[FenceShape]
    fence_return: list[float] | None
    fence_params: FenceParams
    rally: list[list[float]]
    """[lat, lon, height above home] per rally point."""
    rtl_alt: float | None


def build_plan(
    sysid: int,
    t: float,
    mission: Sequence[RawItem],
    fence: Sequence[RawItem],
    rally: Sequence[RawItem],
    params: dict[str, float],
    fence_enabled: bool,
) -> Plan:
    home_item = mission[0] if mission and mission[0].seq == 0 else None
    home = (
        [home_item.lat_deg, home_item.lon_deg, home_item.alt_m]
        if home_item and home_item.has_position
        else None
    )
    home_amsl = home[2] if home else 0.0

    def height(item: RawItem) -> tuple[float, bool]:
        if item.frame in ALT_AMSL:
            return item.alt_m - home_amsl, False
        return item.alt_m, item.frame in ALT_ABOVE_TERRAIN

    items: list[PlanItem] = []
    # Where the vehicle is before each item: home, then the end of the last nav item.
    at: tuple[float, float, float] | None = (home[0], home[1], 0.0) if home else None
    for raw in mission:
        if raw.seq == 0:
            continue
        nav = raw.command in NAV_COMMANDS
        lat: float | None = None
        lon: float | None = None
        alt: float | None = None
        terrain = False
        if raw.command == apm.MAV_CMD_NAV_RETURN_TO_LAUNCH:
            if home:
                lat, lon, alt = home[0], home[1], 0.0
        elif raw.command == apm.MAV_CMD_NAV_TAKEOFF:
            # ArduCopter climbs straight up from where it is; the item's position is unused.
            alt, terrain = height(raw)
            if at:
                lat, lon = at[0], at[1]
        elif raw.has_position:
            lat, lon = raw.lat_deg, raw.lon_deg
            alt, terrain = height(raw)
            if raw.command == apm.MAV_CMD_NAV_LAND:
                alt, terrain = 0.0, False
        elif nav and at:
            # No position: the item acts where the vehicle already is (e.g. LAND here).
            lat, lon = at[0], at[1]
            alt = 0.0 if raw.command == apm.MAV_CMD_NAV_LAND else at[2]
        if nav and lat is not None and lon is not None and alt is not None:
            at = (lat, lon, alt)
        items.append(
            PlanItem(
                seq=raw.seq,
                cmd=raw.command,
                name=COMMAND_NAMES.get(raw.command, f"CMD {raw.command}"),
                nav=nav,
                lat=lat,
                lon=lon,
                alt=alt,
                terrain=terrain,
                params=list(raw.params),
            )
        )

    return Plan(
        kind="plan",
        t=t,
        sysid=sysid,
        home=home,
        items=items,
        fence=_fence_shapes(fence),
        fence_return=next(
            (
                [f.lat_deg, f.lon_deg]
                for f in fence
                if f.command == apm.MAV_CMD_NAV_FENCE_RETURN_POINT
            ),
            None,
        ),
        fence_params=FenceParams(
            enabled=fence_enabled,
            types=int(params.get("FENCE_TYPE", 0)),
            radius=params.get("FENCE_RADIUS"),
            alt_max=params.get("FENCE_ALT_MAX"),
            action=int(params["FENCE_ACTION"]) if "FENCE_ACTION" in params else None,
        ),
        rally=[[r.lat_deg, r.lon_deg, height(r)[0]] for r in rally if r.has_position],
        rtl_alt=params.get("RTL_ALT_M"),
    )


def _fence_shapes(fence: Sequence[RawItem]) -> list[FenceShape]:
    """Polygon vertices arrive as consecutive items, each carrying the vertex count in param1."""
    shapes: list[FenceShape] = []
    polygon: list[list[float]] = []
    polygon_cmd: int | None = None
    for f in fence:
        if f.command in (
            apm.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
            apm.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
        ):
            if polygon_cmd != f.command or len(polygon) >= int(f.params[0]):
                polygon, polygon_cmd = [], f.command
            polygon.append([f.lat_deg, f.lon_deg])
            if len(polygon) == int(f.params[0]):
                shapes.append(
                    FenceShape(
                        kind="polygon",
                        inclusion=f.command == apm.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                        points=polygon,
                        radius=None,
                    )
                )
        elif f.command in (
            apm.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
            apm.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
        ):
            polygon, polygon_cmd = [], None
            shapes.append(
                FenceShape(
                    kind="circle",
                    inclusion=f.command == apm.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
                    points=[[f.lat_deg, f.lon_deg]],
                    radius=f.params[0],
                )
            )
    return shapes


class PlanSender:
    """The three messages a download sends, bound to one vehicle."""

    def __init__(
        self,
        request_list: Callable[[int], None],
        request_item: Callable[[int, int], None],
        ack: Callable[[int], None],
    ) -> None:
        self.request_list = request_list
        self.request_item = request_item
        """(seq, mission_type)"""
        self.ack = ack


@dataclass
class PlanReader:
    """Downloads one vehicle's mission, fence and rally points in turn, again and again."""

    sender: PlanSender
    plans: dict[int, tuple[RawItem, ...]] = field(default_factory=dict)
    """Last complete download per mission type."""
    version: int = 0
    """Bumped whenever a download differs from the previous one."""
    _type_index: int = 0
    _count: int | None = None
    """Items announced by MISSION_COUNT for the transfer in progress; None before it arrives."""
    _items: list[RawItem] = field(default_factory=list)
    _busy: bool = False
    _sent_s: float = 0.0
    _tries: int = 0
    _next_cycle_s: float = 0.0
    _mission_total: int | None = None
    _again: bool = False
    """Download again as soon as this cycle ends: the plan changed under it."""

    @property
    def mission_type(self) -> int:
        return PLAN_TYPES[self._type_index]

    def mission_total_seen(self, total: int, now: float) -> None:
        """MISSION_CURRENT's item count (home excluded). A change means a new mission."""
        if self._mission_total is not None and total != self._mission_total:
            log.debug("mission count changed to %d; downloading", total)
            self._next_cycle_s = min(self._next_cycle_s, now)
        self._mission_total = total

    def refresh(self, now: float) -> None:
        """Download everything again now, as after an upload: once this cycle ends if busy."""
        self._next_cycle_s = now
        self._again = self._busy

    def tick(self, now: float) -> None:
        if not self._busy:
            if now >= self._next_cycle_s:
                self._type_index = 0
                self._again = False
                self._start(now)
            return
        if now - self._sent_s < STEP_TIMEOUT_S:
            return
        if self._tries >= STEP_RETRIES:
            log.debug("plan download timed out (type %d)", self.mission_type)
            self._next_type(now)
            return
        self._send(now)

    def on_count(self, count: int, mission_type: int, now: float) -> None:
        if not self._busy or mission_type != self.mission_type or self._count is not None:
            return
        self._count = count
        self._tries = 0
        if count == 0:
            self._finish(now)
        else:
            self._send(now)

    def on_item(self, item: RawItem, mission_type: int, now: float) -> None:
        if not self._busy or mission_type != self.mission_type or self._count is None:
            return
        if item.seq != len(self._items):
            return
        self._items.append(item)
        self._tries = 0
        if len(self._items) == self._count:
            self.sender.ack(self.mission_type)
            self._finish(now)
        else:
            self._send(now)

    def on_ack(self, result: int, mission_type: int, now: float) -> None:
        """A MISSION_ACK during a download is a refusal. DENIED means another GCS is
        uploading: try the whole cycle again soon. Anything else: skip this type."""
        if not self._busy or mission_type != self.mission_type:
            return
        if result == apm.MAV_MISSION_ACCEPTED:
            return
        log.debug("plan download refused (type %d, result %d)", mission_type, result)
        if result == apm.MAV_MISSION_DENIED:
            self._busy = False
            self._next_cycle_s = now + RETRY_S
        else:
            self._next_type(now)

    def _start(self, now: float) -> None:
        self._busy = True
        self._count = None
        self._items = []
        self._tries = 0
        self._send(now)

    def _send(self, now: float) -> None:
        if self._count is None:
            self.sender.request_list(self.mission_type)
        else:
            self.sender.request_item(len(self._items), self.mission_type)
        self._sent_s = now
        self._tries += 1

    def _finish(self, now: float) -> None:
        items = tuple(self._items)
        if self.plans.get(self.mission_type) != items:
            self.plans[self.mission_type] = items
            self.version += 1
        self._next_type(now)

    def _next_type(self, now: float) -> None:
        self._busy = False
        if self._type_index + 1 < len(PLAN_TYPES):
            self._type_index += 1
            self._start(now)
        else:
            self._next_cycle_s = now if self._again else now + REFRESH_S
