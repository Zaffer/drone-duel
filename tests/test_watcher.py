"""The watcher's leader, roles and slots, with a fake link (no MAVLink). It never commands."""

import time
from collections.abc import Sequence

from pymavlink.dialects.v20 import ardupilotmega as apm

from swarm.geo import LocalFrame
from swarm.mission import PlanReader
from swarm.state import VehicleState
from swarm.telemetry import Snapshot
from swarm.watcher import Watcher

FRAME = LocalFrame(-35.0, 149.0)
"""Vehicle 1 sits at the origin in the tests, so the watcher anchors its frame here."""


class FakeLink:
    """Records any command; the watcher must send none."""

    def __init__(self, vehicles: Sequence[VehicleState]) -> None:
        self.vehicles = {v.sysid: v for v in vehicles}
        self.plans: dict[int, PlanReader] = {}
        self.commands: list[str] = []

    def poll(self) -> None:
        pass

    def __getattr__(self, name: str) -> object:
        if name.startswith(("set_", "send_", "arm", "takeoff")):
            return lambda *args: self.commands.append(name)
        raise AttributeError(name)


def _vehicle(
    sysid: int, mode: int, north: float = 0.0, changed: float = 0.0, east: float = 0.0
) -> VehicleState:
    now = time.monotonic()
    lat, lon = FRAME.to_latlon(north, east)
    return VehicleState(
        sysid=sysid,
        mode=mode,
        armed=True,
        gps_fix_type=3,
        ekf_flags=apm.EKF_POS_HORIZ_ABS,
        landed_state=apm.MAV_LANDED_STATE_IN_AIR,
        lat_deg=lat,
        lon_deg=lon,
        rel_alt_m=15.0,
        last_heartbeat_s=now,
        last_position_s=now,
        mode_changed_s=changed,
    )


class RecordingPublisher:
    def __init__(self) -> None:
        self.snapshots: list[Snapshot] = []

    def publish(self, snapshot: Snapshot) -> None:
        self.snapshots.append(snapshot)


def _follow_params(leader: int) -> dict[str, float]:
    return {
        "FOLL_SYSID": leader,
        "FOLL_OFS_TYPE": 1,
        "FOLL_OFS_X": -8.0,
        "FOLL_OFS_Y": 8.0,
        "FOLL_OFS_Z": 0.0,
    }


def _watch(link: FakeLink) -> Snapshot:
    publisher = RecordingPublisher()
    watcher = Watcher(link, publisher)  # type: ignore[arg-type]
    for _ in range(2):
        watcher.step()
    return publisher.snapshots[-1]


def test_publishes_every_drone_and_never_commands() -> None:
    dropped = _vehicle(3, apm.COPTER_MODE_BRAKE, north=-1)
    dropped.rc_ok = False
    link = FakeLink(
        [_vehicle(1, apm.COPTER_MODE_ACRO), _vehicle(2, apm.COPTER_MODE_GUIDED, north=-5), dropped]
    )
    snap = _watch(link)
    assert [v["sysid"] for v in snap["vehicles"]] == [1, 2, 3]
    assert (snap["target_n"], snap["target_e"]) == (None, None)  # no point B given
    assert link.commands == []


def test_publishes_point_b_in_the_local_frame() -> None:
    publisher = RecordingPublisher()
    point_b = FRAME.to_latlon(300.0, -600.0)
    watcher = Watcher(FakeLink([_vehicle(1, apm.COPTER_MODE_LOITER)]), publisher, point_b=point_b)  # type: ignore[arg-type]
    for _ in range(2):
        watcher.step()
    snap = publisher.snapshots[-1]
    assert abs(snap["target_n"] - 300.0) < 0.01 and abs(snap["target_e"] + 600.0) < 0.01  # type: ignore[operator]


def test_shows_the_ardupilot_follow_formation() -> None:
    follower = _vehicle(2, apm.COPTER_MODE_FOLLOW, north=-8, east=8)
    follower.params = _follow_params(1)
    link = FakeLink([_vehicle(1, apm.COPTER_MODE_LOITER), follower])
    snap = _watch(link)
    rows = {v["sysid"]: v for v in snap["vehicles"]}
    assert snap["leader"] == 1
    assert (rows[1]["role"], rows[2]["role"]) == ("leader", "follower")
    assert rows[2]["slot_n"] is not None and rows[2]["slot_e"] is not None
    assert abs(rows[2]["slot_n"] + 8.0) < 0.1 and abs(rows[2]["slot_e"] - 8.0) < 0.1
    assert link.commands == []


def test_a_drone_the_radio_takes_over_is_a_pilot_not_the_leader() -> None:
    """swarm.launch keeps the followers on the leader while the radio flies another copter."""
    leader = _vehicle(1, apm.COPTER_MODE_GUIDED)
    leader.params = _follow_params(0)
    taken = _vehicle(2, apm.COPTER_MODE_LOITER, north=-8, east=8)
    taken.params = _follow_params(1)
    taken.mode_changed_s = time.monotonic()
    follower = _vehicle(3, apm.COPTER_MODE_FOLLOW, north=-8, east=-8)
    follower.params = _follow_params(1)
    snap = _watch(FakeLink([leader, taken, follower]))
    roles = {v["sysid"]: v["role"] for v in snap["vehicles"]}
    assert snap["leader"] == 1
    assert roles == {1: "leader", 2: "pilot", 3: "follower"}


def test_without_follow_targets_the_pilots_drone_leads() -> None:
    snap = _watch(
        FakeLink([_vehicle(1, apm.COPTER_MODE_LOITER), _vehicle(2, apm.COPTER_MODE_GUIDED)])
    )
    roles = {v["sysid"]: v["role"] for v in snap["vehicles"]}
    assert snap["leader"] == 1
    assert roles == {1: "leader", 2: "watched"}


def test_a_drone_gone_quiet_is_published_with_its_link_down() -> None:
    quiet = _vehicle(2, apm.COPTER_MODE_LOITER, north=-5)
    quiet.last_position_s = time.monotonic() - 10
    snap = _watch(FakeLink([_vehicle(1, apm.COPTER_MODE_LOITER), quiet]))
    assert [(v["sysid"], v["link"]) for v in snap["vehicles"]] == [(1, True), (2, False)]
    assert all(v["nav"] for v in snap["vehicles"])
