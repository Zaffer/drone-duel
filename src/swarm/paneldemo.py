"""Pretend drones for `swarm panel-demo`: the switch panel flies them through the real launcher
and panel rules, with no sim and no radio.

A pretend drone arms when told, climbs on a takeoff, holds in GUIDED, keeps its slot's height
in FOLLOW, comes down in LAND and RTL, and disarms on the ground. It takes parameters and the
geofence as they are sent. The radio is a pretend pilot with the mode switch at LOITER: the
armed drone it is given climbs to PILOT_ALT_M. It arms nothing; ALT arms the leader.
"""

import math
import time
from collections.abc import Callable

from pymavlink.dialects.v20 import ardupilotmega as apm

from swarm.fence import VERTEX
from swarm.geo import LocalFrame
from swarm.mission import RawItem
from swarm.state import PILOT_MODES, VehicleState

PILOT_ALT_M = 12.0
"""How high the pretend pilot flies the drone on the radio."""
CLIMB_MS = 2.5
DESCEND_MS = 2.0
SPACING_M = 5.0
"""The drones stand this far apart in a line to the east, as in the sim."""
GROUND_M = 0.1


class DemoLink:
    """The MavlinkLink calls the launcher makes, answered by pretend drones."""

    def __init__(
        self,
        count: int,
        home: tuple[float, float],
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.clock = clock
        frame = LocalFrame(*home)
        self.vehicles: dict[int, VehicleState] = {}
        for sysid in range(1, count + 1):
            lat, lon = frame.to_latlon(0.0, SPACING_M * (sysid - 1))
            self.vehicles[sysid] = VehicleState(
                sysid,
                mode=apm.COPTER_MODE_STABILIZE,
                prearm_ok=True,
                gps_fix_type=apm.GPS_FIX_TYPE_3D_FIX,
                landed_state=apm.MAV_LANDED_STATE_ON_GROUND,
                lat_deg=lat,
                lon_deg=lon,
                last_heartbeat_s=clock(),
                last_position_s=clock(),
            )
        self.radio: int | None = None
        """The drone the pretend pilot flies."""
        self._target_m: dict[int, float] = {}
        self._fences: dict[int, tuple[RawItem, ...]] = {}
        self._last_s = clock()

    def poll(self) -> None:
        now = self.clock()
        dt, self._last_s = now - self._last_s, now
        for v in self.vehicles.values():
            v.last_heartbeat_s = v.last_position_s = now
            if v.armed:
                self._fly(v, dt)

    def set_mode(self, sysid: int, mode: int) -> None:
        v = self.vehicles[sysid]
        v.mode, v.mode_changed_s = mode, self.clock()
        if mode == apm.COPTER_MODE_GUIDED:
            self._target_m[sysid] = v.rel_alt_m  # hold where it is

    def arm(self, sysid: int) -> None:
        self.vehicles[sysid].armed = True

    def takeoff(self, sysid: int, altitude_m: float) -> None:
        self._target_m[sysid] = altitude_m

    def set_param(self, sysid: int, name: str, value: float) -> None:
        v = self.vehicles[sysid]
        v.params[name] = value
        if name == "FENCE_ENABLE":
            v.fence_enabled = value == 1.0

    def request_param(self, sysid: int, name: str) -> None:
        pass

    def upload_fence(self, sysid: int, corners: tuple[tuple[float, float], ...]) -> bool:
        self._fences[sysid] = tuple(
            RawItem(i, apm.MAV_FRAME_GLOBAL, VERTEX, (float(len(corners)), 0, 0, 0), lat, lon, 0.0)
            for i, (lat, lon) in enumerate(corners)
        )
        return True

    def fence_uploading(self, sysid: int) -> bool:
        return False

    def fence_on_vehicle(self, sysid: int) -> tuple[RawItem, ...] | None:
        return self._fences.get(sysid)

    def _fly(self, v: VehicleState, dt: float) -> None:
        if v.mode in (apm.COPTER_MODE_LAND, apm.COPTER_MODE_RTL):
            target = 0.0
        elif v.mode in PILOT_MODES:
            target = PILOT_ALT_M if v.sysid == self.radio else v.rel_alt_m
        elif v.mode == apm.COPTER_MODE_FOLLOW:
            leader = self.vehicles.get(int(v.params.get("FOLL_SYSID", 0)))
            target = (
                v.rel_alt_m
                if leader is None
                else leader.rel_alt_m - v.params.get("FOLL_OFS_Z", 0.0)
            )
        else:
            target = self._target_m.get(v.sysid, v.rel_alt_m)
        step = target - v.rel_alt_m
        rate = CLIMB_MS if step > 0 else DESCEND_MS
        v.rel_alt_m += math.copysign(min(abs(step), rate * dt), step)
        if v.rel_alt_m > GROUND_M:
            landing = v.mode == apm.COPTER_MODE_LAND
            v.landed_state = (
                apm.MAV_LANDED_STATE_LANDING if landing else apm.MAV_LANDED_STATE_IN_AIR
            )
        elif v.in_air and target <= 0.0:
            v.rel_alt_m, v.armed = 0.0, False  # touched down: ArduPilot disarms
            v.landed_state = apm.MAV_LANDED_STATE_ON_GROUND
            self._target_m.pop(v.sysid, None)


class DemoRadio:
    """The pretend pilot's radio: the mode switch at LOITER, flying what it is given."""

    def __init__(self, link: DemoLink) -> None:
        self.link = link

    def mode(self) -> int:
        return apm.COPTER_MODE_LOITER

    def retarget(self, sysid: int | None) -> None:
        self.link.radio = sysid
