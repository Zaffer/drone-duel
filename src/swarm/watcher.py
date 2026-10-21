"""The watcher: what the drones report, published for the 3D viewer (the COP).

It never commands a drone. Each step it reads the drones over MAVLink, anchors a local NED
frame on the first drone with a position, works out the leader and each drone's role from what
ArduPilot reports, and publishes one snapshot (swarm.telemetry) with the mission's point B,
plus each drone's uploaded mission, fence and rally points (swarm.mission) when they change.
`swarm sim`, `fly` and `rehearse` run one beside the stack; `swarm watch` runs one on its own
(hardware).
"""

import logging
import time
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from pymavlink.dialects.v20 import ardupilotmega as apm

from swarm.follow import follow_slot_ned
from swarm.geo import LocalFrame
from swarm.link import MavlinkLink
from swarm.mission import PLAN_TYPES, Plan, build_plan
from swarm.state import VehicleState, mode_name
from swarm.telemetry import Snapshot, TelemetryPublisher, VehicleSnapshot

log = logging.getLogger(__name__)

PLAN_BUILD_S = 1.0
PLAN_RESEND_S = 5.0


@dataclass(frozen=True)
class WatcherConfig:
    rate_hz: float = 10.0
    position_stale_s: float = 2.0
    """A drone whose last position is older than this cannot be the pilot's leader."""


class Watcher:
    def __init__(
        self,
        link: MavlinkLink,
        publisher: TelemetryPublisher | None = None,
        config: WatcherConfig | None = None,
        point_b: tuple[float, float] | None = None,
    ) -> None:
        self.link = link
        self.point_b = point_b
        """The mission's point B (lat, lon), published for the 3D viewer."""
        self.publisher = publisher
        self.config = config or WatcherConfig()
        self.frame: LocalFrame | None = None
        self.leader: int | None = None
        self.phase = "watching"
        self._plans_sent: dict[int, tuple[Plan, float]] = {}
        """Last plan published per vehicle, and when."""
        self._plans_built_s = 0.0
        self._t0 = time.monotonic()

    # -- geometry ----------------------------------------------------------------

    def local_position(self, v: VehicleState) -> np.ndarray:
        assert self.frame is not None
        n, e = self.frame.to_ne(v.lat_deg, v.lon_deg)
        return np.array([n, e, -v.rel_alt_m])

    def _fresh(self, v: VehicleState) -> bool:
        return (
            v.has_position and (time.monotonic() - v.last_position_s) < self.config.position_stale_s
        )

    # -- main loop ---------------------------------------------------------------

    def step(self) -> None:
        self.link.poll()
        if self.frame is None:
            self._anchor_frame()
            return
        self.leader = self._watched_leader()
        self._publish()
        self._publish_plans()

    def run_until(
        self,
        predicate: Callable[[], bool],
        timeout_s: float,
        extra: Callable[[], None] | None = None,
    ) -> bool:
        """Step the loop at the configured rate until `predicate` is true or time runs out."""
        period = 1.0 / self.config.rate_hz
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            tick = time.monotonic()
            self.step()
            if extra is not None:
                extra()
            if predicate():
                return True
            time.sleep(max(0.0, period - (time.monotonic() - tick)))
        return False

    def _anchor_frame(self) -> None:
        for v in sorted(self.link.vehicles.values(), key=lambda v: v.sysid):
            if v.position_ok:
                self.frame = LocalFrame(v.lat_deg, v.lon_deg)
                log.info("local frame anchored at vehicle %d", v.sysid)
                return

    # -- leader and roles ----------------------------------------------------------

    def _pilot_leader(self) -> int | None:
        """The vehicle a pilot took over most recently, if any."""
        candidates = [
            v
            for v in self.link.vehicles.values()
            if v.armed and v.in_air and v.in_pilot_mode and self._fresh(v)
        ]
        if not candidates:
            return None
        return max(candidates, key=lambda v: v.mode_changed_s).sysid

    def _watched_leader(self) -> int | None:
        """The leader as seen without commanding anyone: the drone ArduPilot FOLLOW vehicles
        follow (FOLL_SYSID), else the one the followers are set to follow, else the pilot's
        drone. swarm.launch keeps FOLL_SYSID on the leader while the radio flies another
        copter, so that copter is a pilot's drone, not the leader."""
        vehicles = self.link.vehicles

        def target(v: VehicleState) -> int | None:
            sysid = int(v.params.get("FOLL_SYSID", 0))
            return sysid if sysid != v.sysid and sysid in vehicles else None

        for following in (True, False):
            targets = [
                t
                for v in vehicles.values()
                if (v.mode == apm.COPTER_MODE_FOLLOW or not following)
                and (t := target(v)) is not None
            ]
            if targets:
                return max(set(targets), key=targets.count)
        return self._pilot_leader()

    def _role(self, sysid: int) -> str:
        v = self.link.vehicles.get(sysid)
        if sysid == self.leader:
            return "leader"
        if v is not None and v.mode == apm.COPTER_MODE_FOLLOW:
            return "follower"
        if v is not None and v.armed and v.in_pilot_mode:
            return "pilot"
        return "watched"

    def _follow_slot(self, v: VehicleState) -> np.ndarray | None:
        """The slot an ArduPilot FOLLOW vehicle is flying to, from its FOLL_* parameters."""
        if v.mode != apm.COPTER_MODE_FOLLOW:
            return None
        lead = self.link.vehicles.get(int(v.params.get("FOLL_SYSID", 0)))
        if lead is None or not lead.has_position:
            return None
        return follow_slot_ned(v, lead, self.local_position(lead))

    # -- telemetry ---------------------------------------------------------------

    def _publish(self) -> None:
        if self.publisher is None:
            return
        rows: list[VehicleSnapshot] = []
        for v in sorted(self.link.vehicles.values(), key=lambda v: v.sysid):
            if not v.has_position:
                continue
            pos = self.local_position(v)
            slot = self._follow_slot(v)
            rows.append(
                VehicleSnapshot(
                    sysid=v.sysid,
                    mode=mode_name(v.mode),
                    armed=v.armed,
                    pilot=v.armed and v.in_pilot_mode,
                    role=self._role(v.sysid),
                    n=float(pos[0]),
                    e=float(pos[1]),
                    d=float(pos[2]),
                    vn=float(v.velocity_ned[0]),
                    ve=float(v.velocity_ned[1]),
                    vd=float(v.velocity_ned[2]),
                    heading=v.heading_rad,
                    roll=v.roll_rad,
                    pitch=v.pitch_rad,
                    battery=v.battery_pct,
                    battery_v=v.battery_v,
                    sticks=list(v.rc_channels_us[:4]),
                    slot_n=None if slot is None else float(slot[0]),
                    slot_e=None if slot is None else float(slot[1]),
                    wp=v.mission_seq,
                    mission_state=v.mission_state,
                )
            )
        assert self.frame is not None
        b = None if self.point_b is None else self.frame.to_ne(*self.point_b)
        self.publisher.publish(
            Snapshot(
                t=time.monotonic() - self._t0,
                origin_lat=self.frame.origin_lat_deg,
                origin_lon=self.frame.origin_lon_deg,
                leader=self.leader,
                phase=self.phase,
                vehicles=rows,
                target_n=None if b is None else float(b[0]),
                target_e=None if b is None else float(b[1]),
            )
        )

    def _publish_plans(self) -> None:
        """Each vehicle's plan, logged when it changes and re-sent every PLAN_RESEND_S so a
        viewer started later gets it too."""
        now = time.monotonic()
        if self.publisher is None or now - self._plans_built_s < PLAN_BUILD_S:
            return
        self._plans_built_s = now
        for sysid, reader in self.link.plans.items():
            v = self.link.vehicles.get(sysid)
            if v is None or not reader.plans:
                continue
            mission, fence, rally = (reader.plans.get(t, ()) for t in PLAN_TYPES)
            plan = build_plan(
                sysid, now - self._t0, mission, fence, rally, v.params, v.fence_enabled
            )
            sent = self._plans_sent.get(sysid)
            changed = sent is None or {**sent[0], "t": 0} != {**plan, "t": 0}
            if changed or (sent is not None and now - sent[1] >= PLAN_RESEND_S):
                self.publisher.publish_plan(plan, log=changed)
                self._plans_sent[sysid] = (plan, now)
