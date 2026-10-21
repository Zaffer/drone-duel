"""The geofence from mission.toml, uploaded to every drone. Each drone enforces it on board.

ArduPilot keeps an inclusion polygon (FENCE_TYPE bit 2) and a ceiling above home (bit 0) and
checks them at 25 Hz. A drone outside the fence does not arm. On a breach it does FENCE_ACTION;
a mode change during a breach turns the fence off for 10 s, so the pilot can fly back in
(AC_Fence manual recovery).

The polygon goes up with the MAVLink mission protocol, mission type FENCE: we send
MISSION_COUNT, the drone asks for each item with MISSION_REQUEST, we answer MISSION_ITEM_INT,
and it ends with MISSION_ACK. Each corner is one MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
item with the corner count in param1. The upload is checked by downloading the fence again
(swarm.mission.PlanReader), as the 3D viewer draws it.
"""

import logging
from collections.abc import Callable, Sequence
from dataclasses import dataclass

from pymavlink.dialects.v20 import ardupilotmega as apm

from swarm.geo import LocalFrame
from swarm.mission import RawItem

log = logging.getLogger(__name__)

FENCE_ACTIONS = {"report": 0, "rtl": 1, "land": 2, "brake": 4}
"""mission.toml's action -> FENCE_ACTION: report only, RTL or land, land, brake or land."""
TYPE_ALT_MAX = 1
TYPE_POLYGON = 4
VERTEX = apm.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
MATCH_DEG = 2e-7
"""Corners match within this: MISSION_ITEM_INT carries degrees x 1e7 as whole numbers."""
UPLOAD_TIMEOUT_S = 1.5
"""Send MISSION_COUNT again when the drone asks for nothing this long."""
UPLOAD_TRIES = 3


@dataclass(frozen=True)
class Fence:
    enabled: bool
    corners: tuple[tuple[float, float], ...]
    """[lat, lon] in order round the polygon the drones must stay inside."""
    max_height_m: float
    """The ceiling above home."""
    action: str
    """What a drone does on a breach: a key of FENCE_ACTIONS."""

    def params(self) -> dict[str, float]:
        """The FENCE_* parameters, FENCE_ENABLE aside: it goes on once the polygon is there."""
        return {
            "FENCE_TYPE": float(TYPE_ALT_MAX | TYPE_POLYGON),
            "FENCE_ACTION": float(FENCE_ACTIONS[self.action]),
            "FENCE_ALT_MAX": self.max_height_m,
        }

    def matches(self, items: Sequence[RawItem]) -> bool:
        """Whether a drone's downloaded fence is this polygon."""
        return len(items) == len(self.corners) and all(
            item.command == VERTEX
            and int(item.params[0]) == len(self.corners)
            and abs(item.lat_deg - lat) <= MATCH_DEG
            and abs(item.lon_deg - lon) <= MATCH_DEG
            for item, (lat, lon) in zip(items, self.corners, strict=True)
        )


def contains(corners: Sequence[tuple[float, float]], lat: float, lon: float) -> bool:
    """Whether (lat, lon) is inside the polygon (flat earth, ray casting)."""
    frame = LocalFrame(lat, lon)
    points = [frame.to_ne(*c) for c in corners]
    inside = False
    for (n1, e1), (n2, e2) in zip(points, points[1:] + points[:1], strict=True):
        if (n1 > 0) != (n2 > 0) and 0 < e1 + (0 - n1) * (e2 - e1) / (n2 - n1):
            inside = not inside
    return inside


def self_intersects(corners: Sequence[tuple[float, float]]) -> bool:
    """Whether two edges that do not share a corner cross."""
    frame = LocalFrame(*corners[0])
    p = [frame.to_ne(*c) for c in corners]
    edges = [(p[i], p[(i + 1) % len(p)]) for i in range(len(p))]

    def side(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> float:
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    for i in range(len(edges)):
        for j in range(i + 1, len(edges)):
            if j == i + 1 or (i == 0 and j == len(edges) - 1):
                continue  # neighbours share a corner
            (a, b), (c, d) = edges[i], edges[j]
            if side(a, b, c) * side(a, b, d) < 0 and side(c, d, a) * side(c, d, b) < 0:
                return True
    return False


@dataclass
class FenceSender:
    """The two messages an upload sends, bound to one vehicle."""

    count: Callable[[int], None]
    """MISSION_COUNT for the fence."""
    item: Callable[[int, float, float, int], None]
    """MISSION_ITEM_INT: (seq, lat, lon, corner count)."""


class FenceUploader:
    """Uploads one vehicle's fence polygon on request."""

    def __init__(self, sender: FenceSender) -> None:
        self.sender = sender
        self.busy = False
        self._corners: tuple[tuple[float, float], ...] = ()
        self._sent_s = 0.0
        self._tries = 0

    def start(self, corners: tuple[tuple[float, float], ...], now: float) -> None:
        self._corners = corners
        self.busy = True
        self._tries = 0
        self._send_count(now)

    def tick(self, now: float) -> None:
        if not self.busy or now - self._sent_s < UPLOAD_TIMEOUT_S:
            return
        if self._tries >= UPLOAD_TRIES:
            log.warning("fence upload timed out")
            self.busy = False
            return
        self._send_count(now)

    def on_request(self, seq: int, mission_type: int, now: float) -> None:
        if not self.busy or mission_type != apm.MAV_MISSION_TYPE_FENCE:
            return
        if 0 <= seq < len(self._corners):
            lat, lon = self._corners[seq]
            self.sender.item(seq, lat, lon, len(self._corners))
            self._sent_s = now
            self._tries = 0

    def on_ack(self, result: int, mission_type: int) -> bool:
        """The end of an upload. True when the drone took it."""
        if not self.busy or mission_type != apm.MAV_MISSION_TYPE_FENCE:
            return False
        self.busy = False
        if result != apm.MAV_MISSION_ACCEPTED:
            log.warning("fence upload refused (MAV_MISSION_RESULT %d)", result)
        return result == apm.MAV_MISSION_ACCEPTED

    def _send_count(self, now: float) -> None:
        self.sender.count(len(self._corners))
        self._sent_s = now
        self._tries += 1
