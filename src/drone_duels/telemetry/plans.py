"""Read the vehicle's mission, geofence and rally points, for the map (protocol.PlanMsg).

Whatever QGroundControl (or anyone) uploaded is stored on the vehicle, so the vehicle is
the source of truth. `PlanReader` downloads it with the MAVLink mission protocol:
MISSION_REQUEST_LIST, MISSION_COUNT, one MISSION_REQUEST_INT / MISSION_ITEM_INT per item,
MISSION_ACK. A download only reads; ArduPilot refuses it (MISSION_ACK DENIED) while
another GCS is uploading, and the upload carries on, so we just try again later.

ArduPilot 4.7 does not announce a changed mission, so the reader downloads again when the
item count in MISSION_CURRENT changes and otherwise every REFRESH_S.
"""

import logging
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

from pymavlink.dialects.v20 import ardupilotmega as apm

log = logging.getLogger(__name__)

REFRESH_S = 10.0
"""Download every plan this often, so a same-length edit is seen too."""
RETRY_S = 2.0
"""Wait this long after a refused or failed download."""
STEP_TIMEOUT_S = 1.5
STEP_RETRIES = 3
PLAN_TYPES = (apm.MAV_MISSION_TYPE_MISSION, apm.MAV_MISSION_TYPE_FENCE, apm.MAV_MISSION_TYPE_RALLY)


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


def plan_msg(plans: dict[int, tuple[RawItem, ...]]) -> dict[str, Any]:
    """The downloaded plans as a protocol.PlanMsg: mission (item 0 is home), fence, rally."""

    def items(kind: int) -> list[dict[str, Any]]:
        return [
            {
                "seq": i.seq,
                "command": i.command,
                "lat": i.lat_deg,
                "lon": i.lon_deg,
                "alt": i.alt_m,
                "frame": i.frame,
            }
            for i in plans.get(kind, ())
        ]

    return {
        "ch": "plan",
        "mission": items(apm.MAV_MISSION_TYPE_MISSION),
        "fence": items(apm.MAV_MISSION_TYPE_FENCE),
        "rally": items(apm.MAV_MISSION_TYPE_RALLY),
    }


def raw_item(msg: apm.MAVLink_mission_item_int_message) -> RawItem:
    """A MISSION_ITEM_INT message as a RawItem."""
    return RawItem(
        seq=int(msg.seq),
        frame=int(msg.frame),
        command=int(msg.command),
        params=(float(msg.param1), float(msg.param2), float(msg.param3), float(msg.param4)),
        lat_deg=msg.x * 1e-7,
        lon_deg=msg.y * 1e-7,
        alt_m=float(msg.z),
    )


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
