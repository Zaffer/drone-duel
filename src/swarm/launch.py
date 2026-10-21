"""Launch followers one at a time behind a piloted leader, and run the mission phases.

Each follower waits disarmed on the ground until it is
launched: key a, b, c or d for copters 2, 3, 4, 5, or its switch on the switch panel
(`swarm.panel`). Then it:

1. arms in GUIDED, re-arming if ArduPilot disarms it for sitting on the ground (DISARM_DELAY);
2. waits ARM_TO_LIFTOFF_S after arming and until the phase lets it fly and the leader is in the
   air above `leader_min_alt_m` and within `control_radius_m` of it, horizontally;
3. takes off to the leader's height plus its own step (1.5 m per follower, followers above);
4. flies the phase: HOLD is GUIDED, holding where it is; FOLLOW is ArduPilot FOLLOW in its
   slot of the swarm formation, a square 5 m round the leader, north-up whatever way the
   leader faces (`formation_slots`).

Once in FOLLOW, ArduPilot flies it; this module only watches. Changing the phase moves every
flying follower to the new phase's job, so FOLLOW then HOLD pauses the swarm where it is while
the pilot flies off, and FOLLOW again brings it back. LAND lands every drone, the leader
included, but a drone armed on the ground waits there; RTL sends every drone in the air
home and arms nothing. A later phase calls off LAND's landings, so LAND then RTL flies home.
Key l, a drone's switch or `lock` lands where they are; the drone's switch back on (`launch`,
`arm_leader`) calls that off, and in the air it takes up the phase's job again. While locked
nothing arms or lifts off. Without a switch panel the phase stays FOLLOW.

The radio flies one copter at a time (`take_over`), the leader until told otherwise. The
copter it gets goes to LOITER at once, under the pilot's sticks, and the launcher leaves it
alone. The radio's flight-mode switch (`mode_switch`: LOITER, STABILIZE, ACRO) acts on it only
once it has been at LOITER since: a copter never starts in ACRO with the throttle wherever it
was. Given back, a copter takes up the phase's job again. The leader's job without the radio
is to hold where it is (GUIDED); its followers follow it or hold, by phase.
"""

import logging
import math
import time
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING

import numpy as np
from pymavlink.dialects.v20 import ardupilotmega as apm

from swarm.fence import Fence
from swarm.follow import PARAM_TOLERANCE, follow_params, param_mismatch
from swarm.formation import FORMATION_ALT_STEP_M, FORMATION_RADIUS_M, formation_slots
from swarm.link import AVOID_PARAMS
from swarm.state import mode_name

if TYPE_CHECKING:
    from swarm.link import MavlinkLink
    from swarm.state import VehicleState

log = logging.getLogger(__name__)

KEYS = "abcd"
"""Key n launches the n-th follower (copters 2, 3, 4, 5 with leader 1)."""
LAND_KEY = "l"
"""Lands every follower; they land where they are."""
ARM_TO_LIFTOFF_S = 5.0
LEADER_MIN_ALT_M = 5.0
"""Default: a follower lifts off only once the leader is this high
(mission.toml [swarm] liftoff_min_leader_height_m)."""
LEADER_RADIUS_M = 100.0
"""Default control area: a follower lifts off only this close to the leader, horizontally
(mission.toml [swarm] liftoff_max_leader_distance_m)."""
FOLLOW_DIST_MAX_M = 150.0
"""FOLL_DIST_MAX: ArduPilot FOLLOW stops following, and holds, when the leader is farther than
this. Beyond the control area, so a follower that lifts off at its edge still joins; a leader
that runs off farther leaves its followers holding where they are."""
TAKEOFF_REACHED_M = 1.0
RETRY_S = 3.0
FENCE_UPLOAD_RETRY_S = 15.0
"""Upload a drone's geofence again if it has not read back right this long after the last."""
FENCE_WAIT = "the geofence on board"
FLTMODE_CH_OFF = {"FLTMODE_CH": 0.0}
"""ArduPilot's own mode switch, off when `mode_switch` drives the modes. A copter first hearing
the radio would otherwise take the switch's mode, ACRO included, 200 ms later
(RC_Channel::debounce_completed)."""
RELEASE_HOLD_S = 5.0
"""A copter given back holds its job's mode this long. Losing the radio is an RC failsafe
after RC_FS_TIMEOUT (1 s), which brakes it in any mode but GUIDED, FOLLOW and LAND."""
HOLD_RESEND_S = 0.5
"""A held copter out of its mode gets it again at this interval."""
AVOID_ESCAPE = apm.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND
AVOID_REPORT = apm.MAV_COLLISION_ACTION_REPORT
"""AVD_F_ACTION: every drone escapes a collision threat by climbing or descending away
(sitl/swarm.parm) but the one the radio flies, which only reports it: the pilot's drone is
never taken over."""


class Phase(Enum):
    NONE = "no mission"
    HOLD = "hold where they are"
    FOLLOW = "follow the leader"
    LAND = "land"
    RTL = "return home"


FLYING_PHASES = frozenset({Phase.HOLD, Phase.FOLLOW})
"""Phases in which an armed follower lifts off. In LAND it arms and waits on the ground."""


class Stage(Enum):
    IDLE = "idle"
    ARMING = "armed, waiting"
    LIFTOFF = "lifting off"
    FOLLOWING = "following"
    HOLDING = "holding position"
    RETURNING = "returning home"
    LANDING = "landing"


AIRBORNE_STAGES = frozenset({Stage.FOLLOWING, Stage.HOLDING, Stage.RETURNING})
"""Stages of a follower in the air on a phase's job."""


@dataclass
class Follower:
    sysid: int
    key: str
    slot: np.ndarray
    """(north, east, down) from the leader; down is negative, followers fly above."""
    stage: Stage = Stage.IDLE
    armed_s: float | None = None
    """When the follower was first armed after its key."""
    takeoff_alt_m: float = 0.0
    last_command_s: float = -math.inf
    last_arm_s: float = -math.inf
    waiting_for: str = ""
    phase_landing: bool = False
    """Landing for the LAND phase; a later phase calls it off."""


def _escaping(v: "VehicleState", wanted: int) -> bool:
    """ArduPilot's avoidance has the drone escaping a collision. Modes are not re-sent then, so
    the escape is not cut short; LAND, the safety command, still is."""
    return v.mode == apm.COPTER_MODE_AVOID_ADSB and wanted != apm.COPTER_MODE_LAND


def _param_is(v: "VehicleState", name: str, value: float) -> bool:
    """The vehicle reported `name` at `value`, as near as its 32-bit float carries it."""
    return name in v.params and abs(v.params[name] - value) <= PARAM_TOLERANCE * max(
        1.0, abs(value)
    )


def horizontal_distance_m(a: "VehicleState", b: "VehicleState") -> float:
    """Flat-earth distance; good to well under a metre at the ranges that matter here."""
    north = math.radians(b.lat_deg - a.lat_deg) * 6_371_000.0
    east = math.radians(b.lon_deg - a.lon_deg) * 6_371_000.0 * math.cos(math.radians(a.lat_deg))
    return math.hypot(north, east)


class Launcher:
    def __init__(
        self,
        link: "MavlinkLink",
        leader: int,
        followers: list[int],
        radius_m: float = FORMATION_RADIUS_M,
        alt_step_m: float = FORMATION_ALT_STEP_M,
        dist_max_m: float = FOLLOW_DIST_MAX_M,
        control_radius_m: float = LEADER_RADIUS_M,
        leader_min_alt_m: float = LEADER_MIN_ALT_M,
        fence: Fence | None = None,
        clock: Callable[[], float] = time.monotonic,
        own_mode_switch: bool = False,
    ) -> None:
        if len(followers) > len(KEYS):
            raise ValueError(f"at most {len(KEYS)} followers, one per key {KEYS}")
        self.link = link
        self.leader = leader
        self.clock = clock
        self.control_radius_m = control_radius_m
        """A follower lifts off only this close to the leader, horizontally."""
        self.leader_min_alt_m = leader_min_alt_m
        """A follower lifts off only once the leader is this high."""
        self.fence = fence
        """The geofence every drone gets (swarm.fence); None leaves the drones' fences alone."""
        self._fence_sent_s = -math.inf
        self._fence_upload_s: dict[int, float] = {}
        slots = formation_slots(len(followers), radius_m, alt_step_m)
        self.followers = {
            key: Follower(sysid, key, slot)
            for key, sysid, slot in zip(KEYS, followers, slots, strict=False)
        }
        self.lead = Follower(leader, "", np.zeros(3))
        """The leader. ARMING is armed or arming in GUIDED on the ground, HOLDING in the air."""
        self.phase = Phase.FOLLOW
        self.live = True
        """False while locked: nothing arms or lifts off."""
        self.own_mode_switch = own_mode_switch
        """Turn ArduPilot's mode switch off on every copter: `mode_switch` sets the modes."""
        self.manual: int | None = leader
        """Copter the radio flies; the launcher leaves it alone."""
        self.mode_cleared = False
        """The mode switch has been at LOITER since the radio got this copter."""
        self._pilot_mode: int | None = None
        self._told_to_clear = False
        self._switch: int | None = None
        """The mode switch position last seen."""
        self._refused: int | None = None
        self._params = {
            f.sysid: follow_params(leader, f.slot, dist_max_m) for f in self.followers.values()
        }
        self._params_sent_s = -math.inf
        self._want: dict[int, int] = {}
        """Mode each armed drone is being moved to, re-sent until it reports it."""
        self._want_sent: dict[int, float] = {}
        self._hold_mode: dict[int, tuple[int, float]] = {}
        """Mode a copter given back is held in, and until when (RELEASE_HOLD_S)."""

    def describe(self) -> list[str]:
        return [
            f"  {f.key}: copter {f.sysid}, {f.slot[0]:+.1f} m north, {f.slot[1]:+.1f} m east, "
            f"{-f.slot[2]:+.1f} m up from the leader"
            for f in self.followers.values()
        ]

    def press(self, key: str) -> None:
        if key.lower() == LAND_KEY:
            self.land_all()
            return
        f = self.followers.get(key.lower())
        if f is None:
            log.info(
                "key %r: nothing; keys %s launch copters %s",
                key,
                KEYS,
                [f.sysid for f in self.followers.values()],
            )
            return
        if f.stage is not Stage.IDLE:
            log.info("copter %d already %s", f.sysid, f.stage.value)
            return
        self.launch(f.key)

    def launch(self, key: str) -> None:
        """Arm the follower on `key`. It lifts off when the phase and the leader let it.
        Landing, it stops and takes up the phase's job again."""
        f = self.followers[key]
        if f.stage is Stage.LANDING and self._call_off_landing(f):
            return
        if f.stage is not Stage.IDLE or not self._may_arm(f):
            return
        log.info("copter %d: arming", f.sysid)
        self._start_arming(f)

    def arm_leader(self) -> None:
        """Arm the leader in GUIDED: ready, holding on the ground until the radio takes it.
        Landing, it stops and holds again."""
        f = self.lead
        if f.stage is Stage.LANDING and self._call_off_landing(f):
            return
        if f.stage is not Stage.IDLE or not self._may_arm(f):
            return
        log.info("copter %d: arming", f.sysid)
        self._start_arming(f)

    def land(self, key: str) -> None:
        self._land(self.followers[key])

    def land_leader(self) -> None:
        self._land(self.lead)

    def land_all(self, leader: bool = False) -> None:
        """LAND every armed follower, and the leader if `leader`. A follower launches again
        once it has disarmed."""
        for f in self._fleet() if leader else self.followers.values():
            self._land(f)

    def lock(self) -> None:
        """Take the radio off every copter, land every armed drone, the leader included, and
        arm nothing until `unlock`."""
        self.live = False
        self.take_over(None)
        self.land_all(leader=True)

    def unlock(self) -> None:
        self.live = True

    @property
    def radio_sysid(self) -> int | None:
        """The copter the radio flies, if any."""
        return self.manual

    def take_over(self, sysid: int | None) -> None:
        """Hand copter `sysid` to the radio, in LOITER, or no copter. The copter it had goes
        back to the autopilot."""
        if sysid == self.manual:
            return
        if sysid is not None and sysid not in self.link.vehicles:
            if self._refused != sysid:
                log.warning("copter %d: not connected; the radio stays where it is", sysid)
            self._refused = sysid
            return
        self._refused = None
        old, self.manual = self.manual, sysid
        self.mode_cleared, self._told_to_clear = False, False
        self._params_sent_s = -math.inf  # avoidance off on the radio's copter at once
        if old is not None:
            self._give_back(old)
        if sysid is not None:
            log.info("copter %d: radio, LOITER", sysid)
            self._drop_mode(sysid)
            self._pilot_mode = apm.COPTER_MODE_LOITER
            self._command_mode(sysid, apm.COPTER_MODE_LOITER)

    def mode_switch(self, mode: int | None) -> None:
        """The radio's flight-mode switch: LOITER, STABILIZE or ACRO for the copter it flies,
        once the switch has been at LOITER since the radio got that copter."""
        if mode is None:
            return
        flicked, self._switch = mode != self._switch, mode
        name = mode_name(mode)
        if self.manual is None:
            if flicked:
                log.info("mode switch %s: the radio flies no copter", name)
            return
        if not self.mode_cleared:
            if mode != apm.COPTER_MODE_LOITER:
                if flicked or not self._told_to_clear:
                    log.warning(
                        "mode switch %s: copter %d waits for the switch at LOITER first",
                        name,
                        self.manual,
                    )
                    self._told_to_clear = True
                return
            self.mode_cleared = True
        if mode != self._pilot_mode:
            log.info("mode switch %s: copter %d to %s", name, self.manual, name)
            self._pilot_mode = mode
            self._command_mode(self.manual, mode)
        elif flicked:
            log.info("mode switch %s: copter %d stays in %s", name, self.manual, name)

    def _give_back(self, sysid: int) -> None:
        """Put a copter the pilot flew by hand back on the phase's job."""
        f = self._by_sysid(sysid)
        v = self.link.vehicles.get(sysid)
        if f is None or v is None or not v.armed:
            return
        log.info("copter %d: back to the autopilot", sysid)
        if f.stage is Stage.LANDING:
            self._command_mode(sysid, apm.COPTER_MODE_LAND, RELEASE_HOLD_S)
        elif not v.in_air:
            if f.stage is Stage.ARMING:
                self._command_mode(sysid, apm.COPTER_MODE_GUIDED, RELEASE_HOLD_S)
        elif self.phase is Phase.LAND:
            self._land(f, by_phase=True)
        elif self.phase is Phase.RTL:
            f.stage = Stage.RETURNING
            self._command_mode(sysid, apm.COPTER_MODE_RTL, RELEASE_HOLD_S)
        else:
            self._fly(f)

    def set_phase(self, phase: Phase) -> None:
        if phase is self.phase:
            return
        log.info("phase: %s", phase.value)
        self.phase = phase
        for f in self._fleet():
            self._retarget(f)

    def step(self) -> None:
        now = self.clock()
        self._send_params(now)
        self._keep_fence(now)
        self._hold_modes(now)
        for f in self._fleet():
            v = self.link.vehicles.get(f.sysid)
            if v is None or f.sysid == self.manual:
                continue
            if f.stage is Stage.LANDING:
                self._landing(f, v)
            elif f.stage in AIRBORNE_STAGES and not v.armed:
                log.info("copter %d disarmed", f.sysid)
                f.stage = Stage.IDLE
            elif not self.live:
                pass
            elif f is self.lead:
                if f.stage is Stage.ARMING:
                    self._leader_arming(f, v, now)
            elif f.stage is Stage.ARMING:
                self._arming(f, v, now)
            elif f.stage is Stage.LIFTOFF:
                self._liftoff(f, v, now)
        self._drive(now)

    def params_ok(self) -> bool:
        return all(
            s in self.link.vehicles and not param_mismatch(self.link.vehicles[s], wanted)
            for s, wanted in self._params.items()
        )

    def _fleet(self) -> list[Follower]:
        return [self.lead, *self.followers.values()]

    def _by_sysid(self, sysid: int) -> Follower | None:
        return next((f for f in self._fleet() if f.sysid == sysid), None)

    def _hold_modes(self, now: float) -> None:
        for sysid, (mode, until_s) in list(self._hold_mode.items()):
            v = self.link.vehicles.get(sysid)
            if now >= until_s or v is None or not v.armed:
                del self._hold_mode[sysid]
            elif _escaping(v, mode):
                continue
            elif v.mode != mode and now - self._want_sent.get(sysid, -math.inf) >= HOLD_RESEND_S:
                self._want.pop(sysid, None)
                self._want_mode(sysid, mode)

    def _may_arm(self, f: Follower) -> bool:
        why = ""
        if not self.live:
            why = "the panel is locked (gear DOWN, then UP)"
        elif self.phase is Phase.RTL:
            why = f"the phase is {self.phase.value}"
        elif f.sysid not in self.link.vehicles:
            why = "not connected"
        if why:
            log.warning("copter %d: not arming, %s", f.sysid, why)
        return not why

    def _start_arming(self, f: Follower) -> None:
        self._drop_mode(f.sysid)  # `_arming` and `_leader_arming` set the mode
        f.stage = Stage.ARMING
        f.armed_s = None
        f.waiting_for = ""
        f.last_command_s = f.last_arm_s = -math.inf

    def _land(self, f: Follower, by_phase: bool = False) -> None:
        """LAND where it is. On the radio's copter the mode switch then waits for LOITER again,
        so a flick of it does not undo the landing."""
        if f.stage is Stage.LANDING:
            f.phase_landing = f.phase_landing and by_phase
            return
        if f.sysid == self.manual:
            self.mode_cleared, self._pilot_mode = False, None
        v = self.link.vehicles.get(f.sysid)
        if v is not None and v.armed:
            log.info("copter %d: landing", f.sysid)
            f.stage = Stage.LANDING
            f.phase_landing = by_phase
            self._command_mode(f.sysid, apm.COPTER_MODE_LAND)
        else:
            f.stage = Stage.IDLE

    def _call_off_landing(self, f: Follower) -> bool:
        """Its switch is back on: stop landing and take up the phase's job, or wait armed on the
        ground. False when it has already disarmed: it arms again from the start."""
        v = self.link.vehicles.get(f.sysid)
        if v is None or not v.armed:
            f.stage = Stage.IDLE
            return False
        if self.phase is Phase.LAND:
            log.info("copter %d: keeps landing, the phase is land", f.sysid)
            f.phase_landing = True
            return True
        log.info("copter %d: landing called off", f.sysid)
        if f.sysid == self.manual:
            f.stage = Stage.HOLDING if v.in_air else Stage.ARMING  # its job when given back
            self.mode_cleared, self._pilot_mode = False, apm.COPTER_MODE_LOITER
            self._command_mode(f.sysid, apm.COPTER_MODE_LOITER)
        elif not v.in_air:
            self._start_arming(f)
        elif self.phase is Phase.RTL:
            f.stage = Stage.RETURNING
            self._command_mode(f.sysid, apm.COPTER_MODE_RTL, RELEASE_HOLD_S)
        else:
            self._fly(f)
        return True

    def _retarget(self, f: Follower) -> None:
        """Move `f` to the current phase's job. The copter flown by hand gets it when given back."""
        v = self.link.vehicles.get(f.sysid)
        if v is None or f.sysid == self.manual:
            return
        if f.stage is Stage.LANDING:
            called_off = f.phase_landing and self.phase not in (Phase.LAND, Phase.NONE)
            if not (called_off and v.armed and v.in_air):
                return
            log.info("copter %d: landing called off", f.sysid)
            # Back in the air: the branches below give it the phase's job, as after an RTL.
            f.stage = Stage.RETURNING
        if self.phase is Phase.LAND:
            if not (f.stage is Stage.ARMING and not v.in_air):  # armed on the ground, it waits
                self._land(f, by_phase=True)
        elif self.phase is Phase.RTL:
            if v.armed and v.in_air:
                log.info("copter %d: returning home", f.sysid)
                f.stage = Stage.RETURNING
                self._command_mode(f.sysid, apm.COPTER_MODE_RTL, RELEASE_HOLD_S)
            else:
                f.stage = Stage.IDLE  # on the ground; ArduPilot disarms it
        elif f.stage in AIRBORNE_STAGES and self.phase is not Phase.NONE:
            if not (f is self.lead and f.stage is Stage.HOLDING):  # the leader holds in any phase
                self._fly(f)

    def _fly(self, f: Follower) -> None:
        """Put a copter in the air on its job: a follower FOLLOW in the FOLLOW phase, anything
        else holds where it is. GUIDED without a target holds at the stopping point."""
        f.last_command_s = -math.inf
        if self.phase is Phase.FOLLOW and f is not self.lead:
            log.info("copter %d: following copter %d", f.sysid, self.leader)
            f.stage = Stage.FOLLOWING
            self._command_mode(f.sysid, apm.COPTER_MODE_FOLLOW, RELEASE_HOLD_S)
        else:
            log.info("copter %d: holding position", f.sysid)
            f.stage = Stage.HOLDING
            self._command_mode(f.sysid, apm.COPTER_MODE_GUIDED, RELEASE_HOLD_S)

    def _command_mode(self, sysid: int, mode: int, hold_s: float = 0.0) -> None:
        """Send `mode` now, and again every RETRY_S until the drone reports it. With `hold_s`,
        put it back in `mode` whenever it leaves it for that long."""
        if hold_s > 0.0:
            self._hold_mode[sysid] = (mode, self.clock() + hold_s)
        else:
            self._hold_mode.pop(sysid, None)
        self._want_mode(sysid, mode)

    def _drop_mode(self, sysid: int) -> None:
        self._want.pop(sysid, None)
        self._hold_mode.pop(sysid, None)

    def _want_mode(self, sysid: int, mode: int) -> None:
        if self._want.get(sysid) == mode:
            return
        self._want[sysid] = mode
        self._want_sent[sysid] = self.clock()
        self.link.set_mode(sysid, mode)

    def _drive(self, now: float) -> None:
        for sysid, mode in list(self._want.items()):
            v = self.link.vehicles.get(sysid)
            if v is None or v.mode == mode or not v.armed:
                del self._want[sysid]
            elif _escaping(v, mode):
                continue  # ArduPilot resumes its mode after the escape; retried then if not
            elif now - self._want_sent[sysid] >= RETRY_S:
                self._want_sent[sysid] = now
                self.link.set_mode(sysid, mode)

    def fence_ready(self, sysid: int) -> bool:
        """The drone has the geofence: its polygon read back, its FENCE_* set and on. True when
        there is no fence to keep, or it is off in mission.toml."""
        fence = self.fence
        if fence is None or not fence.enabled:
            return True
        v = self.link.vehicles.get(sysid)
        on_board = self.link.fence_on_vehicle(sysid)
        wanted = {**fence.params(), "FENCE_ENABLE": 1.0}
        return (
            v is not None
            and on_board is not None
            and fence.matches(on_board)
            and all(_param_is(v, name, value) for name, value in wanted.items())
        )

    def _fence_or_report(self, f: Follower) -> bool:
        """Arm only with the geofence on board; log once while waiting for it."""
        if self.fence_ready(f.sysid):
            return True
        if f.waiting_for != FENCE_WAIT:
            log.info("copter %d: waiting for %s before arming", f.sysid, FENCE_WAIT)
            f.waiting_for = FENCE_WAIT
        return False

    def _keep_fence(self, now: float) -> None:
        """Keep the geofence on every drone: FENCE_* set, the polygon uploaded until it reads
        back the same, then FENCE_ENABLE on. Off in mission.toml: FENCE_ENABLE off."""
        fence = self.fence
        if fence is None or now - self._fence_sent_s < RETRY_S:
            return
        self._fence_sent_s = now
        for f in self._fleet():
            v = self.link.vehicles.get(f.sysid)
            if v is None:
                continue
            wanted = {"FENCE_ENABLE": 0.0}
            if fence.enabled:
                on_board = self.link.fence_on_vehicle(f.sysid)
                loaded = on_board is not None and fence.matches(on_board)
                last = self._fence_upload_s.get(f.sysid, -math.inf)
                if (
                    not loaded
                    and not self.link.fence_uploading(f.sysid)
                    and now - last >= FENCE_UPLOAD_RETRY_S
                    and self.link.upload_fence(f.sysid, fence.corners)
                ):
                    log.info("copter %d: uploading the geofence", f.sysid)
                    self._fence_upload_s[f.sysid] = now
                wanted = fence.params() | ({"FENCE_ENABLE": 1.0} if loaded else {})
            for name, value in wanted.items():
                if not _param_is(v, name, value):
                    self.link.set_param(f.sysid, name, value)
                    self.link.request_param(f.sysid, name)

    def _send_params(self, now: float) -> None:
        """Keep FOLL_* on every follower at its slot, and ArduPilot's mode switch off when
        `own_mode_switch`; re-sent until the read-back matches."""
        if now - self._params_sent_s < RETRY_S:
            return
        self._params_sent_s = now
        for f in self._fleet() if self.own_mode_switch else []:
            v = self.link.vehicles.get(f.sysid)
            for name, value in FLTMODE_CH_OFF.items():
                if v is not None and v.params.get(name) != value:
                    self.link.set_param(f.sysid, name, value)
                    self.link.request_param(f.sysid, name)
        for f in self._fleet():
            v = self.link.vehicles.get(f.sysid)
            if v is None or v.params.get(AVOID_PARAMS[0]) != 1.0:
                continue  # avoidance off on this drone
            action = float(AVOID_REPORT if f.sysid == self.manual else AVOID_ESCAPE)
            if v.params.get(AVOID_PARAMS[1]) != action:
                self.link.set_param(f.sysid, AVOID_PARAMS[1], action)
                self.link.request_param(f.sysid, AVOID_PARAMS[1])
        for sysid, wanted in self._params.items():
            v = self.link.vehicles.get(sysid)
            if v is None:
                continue
            for name in param_mismatch(v, wanted):
                self.link.set_param(sysid, name, wanted[name])
                self.link.request_param(sysid, name)

    def leader_ready(self, f: Follower) -> str:
        """Empty when the leader is ready for `f` to lift off, else what is missing."""
        lead = self.link.vehicles.get(self.leader)
        v = self.link.vehicles.get(f.sysid)
        if lead is None or v is None:
            return f"copter {self.leader} not connected"
        if not (lead.armed and lead.in_air):
            return f"copter {self.leader} in the air"
        if lead.rel_alt_m < self.leader_min_alt_m:
            return f"copter {self.leader} above {self.leader_min_alt_m:g} m"
        distance = horizontal_distance_m(lead, v)
        if distance > self.control_radius_m:
            return f"copter {self.leader} within {self.control_radius_m:g} m (now {distance:.0f} m)"
        return ""

    def _landing(self, f: Follower, v: "VehicleState") -> None:
        if not v.armed:
            log.info("copter %d landed", f.sysid)
            f.stage = Stage.IDLE
        elif v.mode != apm.COPTER_MODE_LAND:
            self._command_mode(f.sysid, apm.COPTER_MODE_LAND)

    def _leader_arming(self, f: Follower, v: "VehicleState", now: float) -> None:
        """Keep the leader armed in GUIDED on the ground, re-arming it after DISARM_DELAY,
        until the radio takes it."""
        if v.in_air:
            self._fly(f)  # the radio flew it up and let go
            return
        if v.mode != apm.COPTER_MODE_GUIDED:
            if now - f.last_command_s >= RETRY_S:
                f.last_command_s = now
                self.link.set_mode(f.sysid, apm.COPTER_MODE_GUIDED)
        elif not v.armed and v.prearm_ok and now - f.last_arm_s >= RETRY_S:
            if self._fence_or_report(f):
                f.last_arm_s = now
                self.link.arm(f.sysid)
        elif v.armed and f.armed_s is None:
            f.armed_s = now
            log.info("copter %d armed, holding on the ground", f.sysid)
        if not v.armed:
            f.armed_s = None

    def _arming(self, f: Follower, v: "VehicleState", now: float) -> None:
        # Armed after a landing called off, it is still in LAND: takeoff needs GUIDED.
        if v.mode != apm.COPTER_MODE_GUIDED and not v.in_air:
            if now - f.last_command_s >= RETRY_S:
                f.last_command_s = now
                self.link.set_mode(f.sysid, apm.COPTER_MODE_GUIDED)
            return
        if not v.armed:
            if v.prearm_ok and now - f.last_arm_s >= RETRY_S and self._fence_or_report(f):
                # Also re-arms after ArduPilot's auto-disarm (DISARM_DELAY) while waiting.
                f.last_arm_s = now
                self.link.arm(f.sysid)
            return
        if f.armed_s is None:
            f.armed_s = now
            log.info("copter %d armed", f.sysid)
        if now - f.armed_s < ARM_TO_LIFTOFF_S:
            return
        if not self._ready_or_report(f):
            return
        lead = self.link.vehicles[self.leader]
        f.takeoff_alt_m = lead.rel_alt_m - float(f.slot[2])
        log.info("copter %d: lifting off to %.1f m", f.sysid, f.takeoff_alt_m)
        f.stage = Stage.LIFTOFF
        f.last_command_s = -math.inf
        self._liftoff(f, v, now)

    def _ready_or_report(self, f: Follower) -> bool:
        waiting_for = ""
        if self.phase not in FLYING_PHASES:
            waiting_for = "a mission phase that flies (rotary on START, BOTH or L)"
        if not waiting_for:
            waiting_for = self.leader_ready(f)
        if not waiting_for and not self.params_ok():
            waiting_for = "FOLL_* parameters on every follower"
        if not waiting_for and not self.fence_ready(f.sysid):
            waiting_for = FENCE_WAIT
        # Log when the reason changes, not each time the distance in it does.
        if waiting_for.split(" (")[0] != f.waiting_for.split(" (")[0] and waiting_for:
            log.info("copter %d armed, waiting for %s", f.sysid, waiting_for)
        f.waiting_for = waiting_for
        return not waiting_for

    def _liftoff(self, f: Follower, v: "VehicleState", now: float) -> None:
        if not v.armed:
            log.warning("copter %d disarmed during liftoff; launch it again", f.sysid)
            f.stage = Stage.IDLE
            return
        if v.rel_alt_m >= f.takeoff_alt_m - TAKEOFF_REACHED_M:
            self._fly(f)
            return
        # ArduPilot refuses a takeoff once airborne, so repeat it only while still on the ground.
        if not v.in_air and now - f.last_command_s >= RETRY_S:
            f.last_command_s = now
            self.link.takeoff(f.sysid, f.takeoff_alt_m)
