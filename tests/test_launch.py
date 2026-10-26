"""Key-triggered follower launch, with a fake link and a fake clock."""

import logging
import time

import pytest
from pymavlink.dialects.v20 import ardupilotmega as apm

from swarm.fence import VERTEX, Fence
from swarm.geo import LocalFrame
from swarm.launch import ARM_TO_LIFTOFF_S, Launcher, Phase, Stage
from swarm.launch import AVOID_ESCAPE as ESCAPE
from swarm.launch import AVOID_REPORT as REPORT
from swarm.mission import RawItem
from swarm.state import VehicleState

FRAME = LocalFrame(45.0, -77.0)


class FakeLink:
    def __init__(self, vehicles: list[VehicleState]) -> None:
        self.vehicles = {v.sysid: v for v in vehicles}
        self.calls: list[tuple[str, int, float]] = []
        self.fences = {}

    def set_mode(self, sysid: int, mode: int) -> None:
        self.calls.append(("mode", sysid, mode))
        v = self.vehicles[sysid]
        v.mode = mode
        # As ArduCopter reports it: LANDING while descending in LAND, IN_AIR again after.
        if mode == apm.COPTER_MODE_LAND and v.landed_state == apm.MAV_LANDED_STATE_IN_AIR:
            v.landed_state = apm.MAV_LANDED_STATE_LANDING
        elif mode != apm.COPTER_MODE_LAND and v.landed_state == apm.MAV_LANDED_STATE_LANDING:
            v.landed_state = apm.MAV_LANDED_STATE_IN_AIR

    def arm(self, sysid: int) -> None:
        self.calls.append(("arm", sysid, 1))
        self.vehicles[sysid].armed = True

    def takeoff(self, sysid: int, altitude_m: float) -> None:
        self.calls.append(("takeoff", sysid, altitude_m))

    def send_global_position(
        self, sysid: int, lat_deg: float, lon_deg: float, rel_alt_m: float
    ) -> None:
        self.calls.append(("goto", sysid, rel_alt_m))

    def set_param(self, sysid: int, name: str, value: float) -> None:
        self.vehicles[sysid].params[name] = value

    def request_param(self, sysid: int, name: str) -> None:
        pass

    # A fence upload lands at once; the drone reads it back on its next download.
    fences: dict[int, tuple[RawItem, ...]]
    hold_uploads = False

    def upload_fence(self, sysid: int, corners: tuple[tuple[float, float], ...]) -> bool:
        self.calls.append(("fence", sysid, len(corners)))
        if not self.hold_uploads:
            self.fences[sysid] = tuple(
                RawItem(i, 0, VERTEX, (float(len(corners)), 0, 0, 0), lat, lon, 0.0)
                for i, (lat, lon) in enumerate(corners)
            )
        return True

    def fence_uploading(self, sysid: int) -> bool:
        return False

    def fence_on_vehicle(self, sysid: int) -> tuple[RawItem, ...] | None:
        return self.fences.get(sysid)


class Clock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


def _copter(sysid: int, east: float, mode: int = apm.COPTER_MODE_STABILIZE) -> VehicleState:
    lat, lon = FRAME.to_latlon(0.0, east)
    return VehicleState(
        sysid=sysid,
        mode=mode,
        prearm_ok=True,
        lat_deg=lat,
        lon_deg=lon,
        landed_state=apm.MAV_LANDED_STATE_ON_GROUND,
        last_position_s=time.monotonic(),
    )


def _setup(own_mode_switch: bool = False) -> tuple[FakeLink, Launcher, Clock]:
    leader = _copter(1, 0.0, apm.COPTER_MODE_LOITER)
    link = FakeLink([leader, _copter(2, 10.0), _copter(3, 150.0)])
    clock = Clock()
    launcher = Launcher(link, 1, [2, 3], clock=clock, own_mode_switch=own_mode_switch)  # type: ignore[arg-type]
    launcher.take_over(None)  # the radio on no copter, as with the panel's selector off
    launcher.step()  # FOLL_* params land on the fakes immediately
    return link, launcher, clock


def _fly_leader(link: FakeLink, alt: float) -> None:
    lead = link.vehicles[1]
    lead.armed, lead.landed_state, lead.rel_alt_m = True, apm.MAV_LANDED_STATE_IN_AIR, alt


def test_nothing_happens_without_a_key() -> None:
    link, launcher, clock = _setup()
    _fly_leader(link, 10.0)
    for _ in range(5):
        clock.now += 3.0
        launcher.step()
    assert not [c for c in link.calls if c[0] in ("arm", "takeoff")]


def test_key_arms_in_guided_and_waits_for_the_leader() -> None:
    link, launcher, clock = _setup()
    launcher.press("a")
    launcher.step()  # GUIDED
    launcher.step()  # arm
    assert link.vehicles[2].armed and link.vehicles[2].mode == apm.COPTER_MODE_GUIDED
    clock.now += 10.0
    launcher.step()  # leader on the ground: keeps waiting
    assert launcher.followers["a"].stage is Stage.ARMING
    assert not [c for c in link.calls if c[0] == "takeoff"]


def test_lifts_off_5_s_after_arming_to_its_step_from_the_leader() -> None:
    link, launcher, clock = _setup()
    _fly_leader(link, 12.0)
    launcher.press("a")
    launcher.step()  # GUIDED, then arm on the next tick
    launcher.step()
    launcher.step()  # armed seen at t=0
    clock.now += 4.0
    launcher.step()  # 4 s after arming: not yet
    assert not [c for c in link.calls if c[0] == "takeoff"]
    clock.now += 1.5
    launcher.step()
    assert ("takeoff", 2, 13.5) in link.calls  # leader 12 m + 1.5 m step


def test_takeoff_is_not_repeated_while_climbing_out() -> None:
    link, launcher, clock = _setup()
    _fly_leader(link, 12.0)
    launcher.press("a")
    for _ in range(3):
        launcher.step()
    clock.now += 5.5
    launcher.step()
    v = link.vehicles[2]
    v.landed_state, v.rel_alt_m = apm.MAV_LANDED_STATE_TAKEOFF, 3.0  # ArduCopter while climbing
    clock.now += 10.0
    launcher.step()
    assert [c for c in link.calls if c[0] == "takeoff"] == [("takeoff", 2, 13.5)]


def test_leader_too_far_keeps_the_follower_waiting() -> None:
    link, launcher, clock = _setup()
    _fly_leader(link, 12.0)
    launcher.press("b")  # copter 3 is 150 m from the leader
    for _ in range(6):
        clock.now += 3.0
        launcher.step()
    assert launcher.followers["b"].stage is Stage.ARMING
    assert "within 100 m" in launcher.followers["b"].waiting_for


def test_control_radius_and_leader_height_are_settings() -> None:
    link = FakeLink([_copter(1, 0.0, apm.COPTER_MODE_LOITER), _copter(2, 30.0)])
    clock = Clock()
    launcher = Launcher(link, 1, [2], control_radius_m=20.0, leader_min_alt_m=15.0, clock=clock)  # type: ignore[arg-type]
    launcher.take_over(None)
    launcher.set_phase(Phase.HOLD)
    _fly_leader(link, 12.0)
    launcher.press("a")
    for _ in range(6):
        clock.now += 3.0
        launcher.step()
    assert "above 15 m" in launcher.followers["a"].waiting_for
    link.vehicles[1].rel_alt_m = 16.0
    clock.now += 3.0
    launcher.step()
    assert "within 20 m (now 30 m)" in launcher.followers["a"].waiting_for
    assert launcher.followers["a"].stage is Stage.ARMING


def test_switches_to_follow_once_at_height() -> None:
    link, launcher, clock = _setup()
    _fly_leader(link, 12.0)
    launcher.press("a")
    for _ in range(6):
        clock.now += 3.0
        launcher.step()
    assert launcher.followers["a"].stage is Stage.LIFTOFF
    link.vehicles[2].rel_alt_m = 13.5
    clock.now += 3.0
    launcher.step()
    assert link.vehicles[2].mode == apm.COPTER_MODE_FOLLOW
    launcher.step()
    assert launcher.followers["a"].stage is Stage.FOLLOWING


def test_l_lands_every_launched_follower() -> None:
    link, launcher, clock = _setup()
    launcher.press("a")
    launcher.step()
    launcher.step()
    launcher.press("l")
    assert ("mode", 2, apm.COPTER_MODE_LAND) in link.calls
    assert ("mode", 3, apm.COPTER_MODE_LAND) not in link.calls  # never launched
    assert launcher.followers["a"].stage is Stage.LANDING
    link.vehicles[2].armed = False
    launcher.step()
    assert launcher.followers["a"].stage is Stage.IDLE  # its key launches it again


def _following(link: FakeLink, launcher: Launcher, clock: Clock, key: str = "a") -> None:
    """Fly the leader at 12 m and take `key`'s follower up into FOLLOW."""
    _fly_leader(link, 12.0)
    launcher.press(key)
    for _ in range(6):
        clock.now += 3.0
        launcher.step()
    v = link.vehicles[launcher.followers[key].sysid]
    v.rel_alt_m, v.landed_state = 13.5, apm.MAV_LANDED_STATE_IN_AIR
    launcher.step()
    assert launcher.followers[key].stage is Stage.FOLLOWING


def test_no_liftoff_without_a_flying_phase() -> None:
    link, launcher, clock = _setup()
    launcher.phase = Phase.NONE
    _fly_leader(link, 12.0)
    launcher.press("a")
    for _ in range(6):
        clock.now += 3.0
        launcher.step()
    assert link.vehicles[2].armed  # armed and ready
    assert launcher.followers["a"].stage is Stage.ARMING
    assert "mission phase" in launcher.followers["a"].waiting_for
    launcher.set_phase(Phase.FOLLOW)
    launcher.step()
    assert ("takeoff", 2, 13.5) in link.calls


def test_hold_pauses_the_swarm_where_it_is_and_follow_brings_it_back() -> None:
    link, launcher, clock = _setup()
    _following(link, launcher, clock)
    launcher.set_phase(Phase.HOLD)
    assert launcher.followers["a"].stage is Stage.HOLDING
    assert link.vehicles[2].mode == apm.COPTER_MODE_GUIDED  # holds at its stopping point
    assert link.vehicles[1].mode == apm.COPTER_MODE_LOITER  # the pilot flies off
    launcher.set_phase(Phase.FOLLOW)
    assert launcher.followers["a"].stage is Stage.FOLLOWING
    assert link.vehicles[2].mode == apm.COPTER_MODE_FOLLOW


def test_hold_launches_followers_to_hold_not_follow() -> None:
    link, launcher, clock = _setup()
    launcher.phase = Phase.HOLD
    _fly_leader(link, 12.0)
    launcher.press("a")
    for _ in range(6):
        clock.now += 3.0
        launcher.step()
    assert ("takeoff", 2, 13.5) in link.calls
    v = link.vehicles[2]
    v.rel_alt_m, v.landed_state = 13.5, apm.MAV_LANDED_STATE_IN_AIR
    launcher.step()
    assert launcher.followers["a"].stage is Stage.HOLDING
    assert v.mode == apm.COPTER_MODE_GUIDED


def test_land_phase_lands_everything_and_arms_to_wait_on_the_ground() -> None:
    link, launcher, clock = _setup()
    _following(link, launcher, clock)
    launcher.set_phase(Phase.LAND)
    assert link.vehicles[1].mode == apm.COPTER_MODE_LAND  # the leader too
    assert link.vehicles[2].mode == apm.COPTER_MODE_LAND
    launcher.press("b")
    assert launcher.followers["b"].stage is Stage.ARMING
    for _ in range(3):
        launcher.step()
        clock.now += ARM_TO_LIFTOFF_S
    assert link.vehicles[3].armed and not link.vehicles[3].in_air
    assert launcher.followers["b"].stage is Stage.ARMING  # LAND does not fly


def test_rtl_phase_refuses_to_arm() -> None:
    link, launcher, clock = _setup()
    _following(link, launcher, clock)
    launcher.set_phase(Phase.RTL)
    launcher.press("b")
    assert launcher.followers["b"].stage is Stage.IDLE


def test_rtl_phase_sends_flyers_home_and_back_they_hold_and_follow() -> None:
    link, launcher, clock = _setup()
    _following(link, launcher, clock)
    launcher.set_phase(Phase.RTL)
    assert link.vehicles[1].mode == apm.COPTER_MODE_RTL
    assert link.vehicles[2].mode == apm.COPTER_MODE_RTL
    launcher.set_phase(Phase.FOLLOW)
    assert link.vehicles[1].mode == apm.COPTER_MODE_GUIDED  # no radio on it: it holds
    assert launcher.lead.stage is Stage.HOLDING
    assert link.vehicles[2].mode == apm.COPTER_MODE_FOLLOW


def test_land_then_rtl_calls_off_the_landing() -> None:
    link, launcher, clock = _setup()
    _following(link, launcher, clock)
    launcher.set_phase(Phase.LAND)
    assert link.vehicles[2].mode == apm.COPTER_MODE_LAND
    launcher.set_phase(Phase.RTL)
    assert link.vehicles[1].mode == apm.COPTER_MODE_RTL
    assert link.vehicles[2].mode == apm.COPTER_MODE_RTL
    assert launcher.followers["a"].stage is Stage.RETURNING


def test_a_phase_does_not_call_off_a_switch_or_key_landing() -> None:
    link, launcher, clock = _setup()
    _following(link, launcher, clock)
    launcher.press("l")
    launcher.set_phase(Phase.RTL)
    assert link.vehicles[2].mode == apm.COPTER_MODE_LAND
    assert link.vehicles[1].mode == apm.COPTER_MODE_RTL  # the leader was not landing


def test_switch_back_on_calls_off_its_landing() -> None:
    link, launcher, clock = _setup()
    _following(link, launcher, clock)
    launcher.land("a")
    launcher.land_leader()
    assert link.vehicles[2].mode == apm.COPTER_MODE_LAND
    launcher.launch("a")
    assert launcher.followers["a"].stage is Stage.FOLLOWING
    assert link.vehicles[2].mode == apm.COPTER_MODE_FOLLOW
    launcher.arm_leader()
    assert launcher.lead.stage is Stage.HOLDING
    assert link.vehicles[1].mode == apm.COPTER_MODE_GUIDED


def test_switch_back_on_after_touchdown_waits_armed_in_guided() -> None:
    link, launcher, clock = _setup()
    _following(link, launcher, clock)
    launcher.land("a")
    v = link.vehicles[2]
    v.landed_state = apm.MAV_LANDED_STATE_ON_GROUND  # down, not yet disarmed
    launcher.launch("a")
    assert launcher.followers["a"].stage is Stage.ARMING
    launcher.step()
    assert v.mode == apm.COPTER_MODE_GUIDED and v.armed


def test_radio_copter_goes_to_loiter_and_the_mode_switch_waits_for_loiter() -> None:
    link, launcher, clock = _setup()
    _following(link, launcher, clock)
    launcher.take_over(2)
    assert launcher.radio_sysid == 2
    assert link.vehicles[2].mode == apm.COPTER_MODE_LOITER
    launcher.mode_switch(apm.COPTER_MODE_ACRO)  # left in ACRO from the last copter
    assert link.vehicles[2].mode == apm.COPTER_MODE_LOITER and not launcher.mode_cleared
    launcher.mode_switch(apm.COPTER_MODE_LOITER)
    assert launcher.mode_cleared
    launcher.mode_switch(apm.COPTER_MODE_ACRO)
    assert link.vehicles[2].mode == apm.COPTER_MODE_ACRO
    launcher.mode_switch(apm.COPTER_MODE_STABILIZE)
    assert link.vehicles[2].mode == apm.COPTER_MODE_STABILIZE
    launcher.take_over(3)
    launcher.mode_switch(apm.COPTER_MODE_STABILIZE)  # cleared again for every new copter
    assert link.vehicles[3].mode == apm.COPTER_MODE_LOITER


def test_given_back_drone_rejoins_through_the_rc_failsafe() -> None:
    link, launcher, clock = _setup()
    _following(link, launcher, clock)
    launcher.take_over(2)
    launcher.take_over(None)
    assert launcher.radio_sysid is None
    assert link.vehicles[2].mode == apm.COPTER_MODE_FOLLOW
    link.vehicles[2].mode = apm.COPTER_MODE_BRAKE  # its RC failsafe, 1 s on
    clock.now += 1.0
    launcher.step()
    assert link.vehicles[2].mode == apm.COPTER_MODE_FOLLOW


def test_leader_without_the_radio_holds_where_it_is() -> None:
    link, launcher, clock = _setup()
    _following(link, launcher, clock)
    launcher.take_over(1)
    assert link.vehicles[1].mode == apm.COPTER_MODE_LOITER
    launcher.take_over(None)
    assert link.vehicles[1].mode == apm.COPTER_MODE_GUIDED
    assert launcher.lead.stage is Stage.HOLDING
    launcher.set_phase(Phase.HOLD)
    launcher.set_phase(Phase.FOLLOW)
    assert launcher.lead.stage is Stage.HOLDING  # the leader holds in every phase


def test_phases_skip_the_radio_copter_until_given_back() -> None:
    link, launcher, clock = _setup()
    _following(link, launcher, clock)
    launcher.take_over(2)
    launcher.set_phase(Phase.LAND)
    assert link.vehicles[2].mode == apm.COPTER_MODE_LOITER
    launcher.take_over(None)
    assert link.vehicles[2].mode == apm.COPTER_MODE_LAND


def test_landing_the_radio_copter_waits_for_the_mode_switch_at_loiter_again() -> None:
    link, launcher, clock = _setup()
    _following(link, launcher, clock)
    launcher.take_over(2)
    launcher.mode_switch(apm.COPTER_MODE_LOITER)
    launcher.mode_switch(apm.COPTER_MODE_ACRO)
    launcher.land("a")
    assert launcher.radio_sysid == 2
    assert link.vehicles[2].mode == apm.COPTER_MODE_LAND
    launcher.mode_switch(apm.COPTER_MODE_ACRO)
    assert link.vehicles[2].mode == apm.COPTER_MODE_LAND
    launcher.mode_switch(apm.COPTER_MODE_LOITER)  # the pilot takes it back on purpose
    assert link.vehicles[2].mode == apm.COPTER_MODE_LOITER


def test_landing_is_reasserted_until_disarmed() -> None:
    link, launcher, clock = _setup()
    _following(link, launcher, clock)
    launcher.lock()
    link.vehicles[2].mode = apm.COPTER_MODE_LOITER  # someone switches it away
    clock.now += 3.0
    launcher.step()
    assert link.vehicles[2].mode == apm.COPTER_MODE_LAND
    assert link.vehicles[1].mode == apm.COPTER_MODE_LAND


def test_leader_arms_in_guided_and_rearms_on_the_ground() -> None:
    link, launcher, clock = _setup()
    lead = link.vehicles[1]
    lead.mode = apm.COPTER_MODE_STABILIZE
    launcher.arm_leader()
    launcher.step()  # GUIDED
    launcher.step()  # arm
    assert lead.mode == apm.COPTER_MODE_GUIDED and lead.armed
    lead.armed = False  # DISARM_DELAY
    clock.now += 3.0
    launcher.step()
    assert lead.armed
    launcher.land_leader()
    assert lead.mode == apm.COPTER_MODE_LAND


def test_own_mode_switch_turns_ardupilots_off_on_every_copter() -> None:
    link, launcher, clock = _setup(own_mode_switch=True)
    assert all(v.params.get("FLTMODE_CH") == 0.0 for v in link.vehicles.values())


def test_every_mode_switch_flick_is_logged(caplog: pytest.LogCaptureFixture) -> None:
    link, launcher, clock = _setup(own_mode_switch=True)
    caplog.set_level(logging.INFO, logger="swarm.launch")
    stab, loiter = apm.COPTER_MODE_STABILIZE, apm.COPTER_MODE_LOITER
    launcher.mode_switch(stab)
    launcher.take_over(2)
    launcher.mode_switch(stab)  # not flicked: the copter waits for LOITER, said once
    launcher.mode_switch(loiter)
    launcher.mode_switch(stab)
    launcher.mode_switch(loiter)
    switch_lines = [r.getMessage() for r in caplog.records if r.getMessage().startswith("mode")]
    assert switch_lines == [
        "mode switch STABILIZE: the radio flies no copter",
        "mode switch STABILIZE: copter 2 waits for the switch at LOITER first",
        "mode switch LOITER: copter 2 stays in LOITER",
        "mode switch STABILIZE: copter 2 to STABILIZE",
        "mode switch LOITER: copter 2 to LOITER",
    ]
    assert link.vehicles[2].mode == loiter


def test_r_sends_home_every_drone_but_the_radios_and_it_follows_once_let_go() -> None:
    link, launcher, clock = _setup()
    _following(link, launcher, clock)
    launcher.take_over(2)  # the radio flies follower a; the leader holds
    launcher.step()
    launcher.set_phase(Phase.RTL)
    rtl = apm.COPTER_MODE_RTL
    assert [link.vehicles[s].mode for s in (1, 2)] == [rtl, apm.COPTER_MODE_LOITER]
    launcher.take_over(None)  # selector off: copter 2 goes home too
    assert link.vehicles[2].mode == rtl
    assert launcher.followers["a"].stage is Stage.RETURNING


def test_leader_let_go_under_r_returns_home() -> None:
    link, launcher, clock = _setup()
    _following(link, launcher, clock)
    launcher.take_over(1)  # the radio flies the leader
    launcher.set_phase(Phase.RTL)
    assert link.vehicles[1].mode == apm.COPTER_MODE_LOITER
    assert link.vehicles[2].mode == apm.COPTER_MODE_RTL
    launcher.take_over(None)
    assert link.vehicles[1].mode == apm.COPTER_MODE_RTL


def _avoid_actions(link: FakeLink) -> dict[int, float]:
    return {s: v.params["AVD_F_ACTION"] for s, v in link.vehicles.items()}


def _avoidance_on(link: FakeLink) -> None:
    for v in link.vehicles.values():
        v.params["AVD_ENABLE"] = 1.0


def test_avoidance_escapes_on_every_drone_but_the_radios() -> None:
    link, launcher, clock = _setup()
    _avoidance_on(link)
    clock.now += 3.0  # the next parameter round
    launcher.step()
    assert _avoid_actions(link) == {1: ESCAPE, 2: ESCAPE, 3: ESCAPE}
    launcher.take_over(2)
    launcher.step()  # at once, not at the next parameter round
    assert _avoid_actions(link) == {1: ESCAPE, 2: REPORT, 3: ESCAPE}
    launcher.take_over(None)
    launcher.step()
    assert link.vehicles[2].params["AVD_F_ACTION"] == ESCAPE


def test_avoidance_left_alone_where_it_is_off() -> None:
    link, launcher, clock = _setup()
    clock.now += 3.0
    launcher.step()
    assert all("AVD_F_ACTION" not in v.params for v in link.vehicles.values())


def test_modes_are_not_resent_during_an_escape_but_land_is() -> None:
    link, launcher, clock = _setup()
    _following(link, launcher, clock)
    launcher.set_phase(Phase.HOLD)  # copter 2 to GUIDED...
    v = link.vehicles[2]
    v.mode = apm.COPTER_MODE_AVOID_ADSB  # ...but avoidance has it escaping
    link.calls.clear()
    for _ in range(3):
        clock.now += 3.0
        launcher.step()
    assert not [c for c in link.calls if c[0] == "mode" and c[1] == 2]
    launcher.land("a")
    v.mode = apm.COPTER_MODE_AVOID_ADSB
    clock.now += 3.0
    launcher.step()
    assert ("mode", 2, apm.COPTER_MODE_LAND) in link.calls[-3:]


AROUND = ((45.01, -77.01), (45.01, -76.99), (44.99, -76.99), (44.99, -77.01))


def _fenced(
    enabled: bool = True, corners: tuple[tuple[float, float], ...] = AROUND
) -> tuple[FakeLink, Launcher, Clock]:
    link = FakeLink([_copter(1, 0.0, apm.COPTER_MODE_LOITER), _copter(2, 10.0)])
    clock = Clock()
    fence = Fence(enabled, corners, 80.0, "brake")
    launcher = Launcher(link, 1, [2], fence=fence, clock=clock)  # type: ignore[arg-type]
    launcher.take_over(None)
    return link, launcher, clock


def test_the_fence_goes_up_then_on_and_arming_waits_for_it() -> None:
    link, launcher, clock = _fenced()
    link.hold_uploads = True  # the drones have not taken it yet
    launcher.press("a")
    launcher.arm_leader()
    for _ in range(3):
        clock.now += 3.0
        launcher.step()
    assert ("fence", 1, 4) in link.calls and ("fence", 2, 4) in link.calls
    assert link.vehicles[2].params["FENCE_ACTION"] == 4.0  # brake
    assert link.vehicles[2].params["FENCE_ALT_MAX"] == 80.0
    assert "FENCE_ENABLE" not in link.vehicles[2].params  # not on without the polygon
    assert not link.vehicles[1].armed and not link.vehicles[2].armed
    assert launcher.followers["a"].waiting_for == "the geofence on board"
    link.hold_uploads = False
    clock.now += 15.0  # time to upload again
    for _ in range(3):
        launcher.step()
        clock.now += 3.0
    assert link.vehicles[2].params["FENCE_ENABLE"] == 1.0
    assert launcher.fence_ready(1) and launcher.fence_ready(2)
    assert link.vehicles[1].armed and link.vehicles[2].armed


def test_a_fence_without_corners_clears_the_polygon_and_keeps_the_ceiling() -> None:
    link, launcher, clock = _fenced(corners=())
    link.upload_fence(2, AROUND)  # a polygon from before
    for _ in range(3):
        clock.now += 3.0
        launcher.step()
    assert link.fences[2] == ()
    assert ("fence", 1, 0) in link.calls and ("fence", 2, 0) in link.calls
    assert link.vehicles[2].params["FENCE_TYPE"] == 1.0  # ceiling only
    assert link.vehicles[2].params["FENCE_ENABLE"] == 1.0
    assert launcher.fence_ready(1) and launcher.fence_ready(2)


def test_a_fence_turned_off_is_turned_off_on_every_drone() -> None:
    link, launcher, clock = _fenced(enabled=False)
    link.vehicles[2].params["FENCE_ENABLE"] = 1.0
    launcher.step()
    assert link.vehicles[1].params["FENCE_ENABLE"] == 0.0
    assert link.vehicles[2].params["FENCE_ENABLE"] == 0.0
    assert not [c for c in link.calls if c[0] == "fence"]
    assert launcher.fence_ready(2)
