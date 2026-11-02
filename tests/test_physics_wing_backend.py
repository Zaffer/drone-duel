"""The wing's physics process backend (physics.wing.backend), without INAV: the pose it streams,
the throw, the reset, the map's ground, the wind, the dynamics and the flight's record."""

import csv
import json
import math
import subprocess
import sys
import time
from collections.abc import Callable, Iterator
from pathlib import Path

import pytest

from drone_duels import ports
from drone_duels.physics.core.rigidbody import rotate
from drone_duels.physics.wing.airframe import CHUPITO
from drone_duels.physics.wing.backend import NEUTRAL, WingBackend
from drone_duels.physics.wing.recorder import FlightRecorder
from drone_duels.server.protocol import POSE, POSE_FIELDS

ORIGIN = (43.630930, -79.389450, 77.0)
WIND = {
    "speed_ms": 3.0,
    "from_deg": 250.0,
    "turbulence": 1.0,
    "eddy_scale": 1.0,
    "gust_factor": 1.0,
    "gust_every_s": 10.0,
}
SLOT = ports.slot(7)
"""Clear of every slot the SITL tests use."""


def _wait(done: Callable[[], bool], seconds: float = 3.0) -> None:
    deadline = time.monotonic() + seconds
    while not done():
        assert time.monotonic() < deadline, "timed out"
        time.sleep(0.01)


@pytest.fixture
def backend(tmp_path: Path) -> Iterator[WingBackend]:
    b = WingBackend("chupito", SLOT, ORIGIN, 270.0, WIND, record=tmp_path)
    b.start()
    yield b
    b.stop()


def test_the_pose_is_the_wing_in_the_hand_at_the_pad(backend: WingBackend) -> None:
    _wait(lambda: backend.pose() is not None)
    pose = backend.pose()
    assert pose is not None and len(pose) == len(POSE_FIELDS)
    POSE.pack(*pose)  # as the runner sends it
    p = dict(zip(POSE_FIELDS, pose, strict=True))
    assert p["t"] > 0
    assert (p["lat"], p["lon"]) == (pytest.approx(ORIGIN[0]), pytest.approx(ORIGIN[1]))
    assert p["alt"] == pytest.approx(ORIGIN[2] + CHUPITO.launch.height)
    q = (p["qw"], p["qx"], p["qy"], p["qz"])
    assert math.fsum(c * c for c in q) == pytest.approx(1.0)
    nose = rotate(q, (1.0, 0.0, 0.0))  # body forward, in NED
    assert math.degrees(math.atan2(nose[1], nose[0])) % 360 == pytest.approx(270.0, abs=0.1)
    assert nose[2] < 0, "nose up in the hand"
    assert (p["vn"], p["ve"], p["vd"]) == (0.0, 0.0, 0.0)
    assert (p["a0"], p["a1"], p["a2"], p["a3"]) == (0.0, 0.0, 0.0, 0.0)
    assert backend.phase() == "hand"
    assert not backend.connected(), "no INAV"


def test_thrown_it_flies_west_and_reset_puts_it_back(backend: WingBackend) -> None:
    _wait(lambda: backend.pose() is not None)
    backend.throw()
    _wait(lambda: backend.phase() == "flying")
    time.sleep(0.3)
    pose = backend.pose()
    assert pose is not None
    p = dict(zip(POSE_FIELDS, pose, strict=True))
    assert p["ve"] < -5, "thrown west, the way the pad faces"
    assert p["lon"] < ORIGIN[1]
    backend.reset()
    _wait(lambda: backend.phase() == "hand")
    pose = backend.pose()
    assert pose is not None
    assert pose[1:4] == pytest.approx((*ORIGIN[:2], ORIGIN[2] + CHUPITO.launch.height))


def test_the_maps_ground_moves_the_hand(backend: WingBackend) -> None:
    """A roof under the pad, from the page: back in the hand, it is held above the roof."""
    backend.ground(ORIGIN[0], ORIGIN[1], ORIGIN[2] + 10)
    backend.reset()
    _wait(lambda: (pose := backend.pose()) is not None and pose[3] > ORIGIN[2] + 5)
    pose = backend.pose()
    assert pose is not None
    assert pose[3] == pytest.approx(ORIGIN[2] + 10 + CHUPITO.launch.height)


def test_the_sticks_go_to_inav_and_the_link_can_be_lost(backend: WingBackend) -> None:
    server = backend.server
    assert server.readings()["sim/joystick/has_joystick"] == 1.0, "a radio from the start"
    assert server.sticks == NEUTRAL, "at rest until the pilot's radio speaks"
    backend.rc([1600, 1400, 1300, 1500, 2000, 1500, 1000, 1000, 1500, 1500])
    server.readings()  # as sent to INAV
    assert server.sticks == (1600, 1400, 1300, 1500, 2000, 1500, 1000, 1000)
    backend.rc([1500, 1500, 1000, 1500])
    server.readings()
    assert server.sticks == (1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000)
    backend.rc(None)  # the link is lost: no joystick, so INAV's receiver hears nothing
    assert server.readings()["sim/joystick/has_joystick"] == 0.0
    assert server.sticks == ()
    backend.rc([1500, 1500, 1000, 1500, 2000])  # and back
    assert server.readings()["sim/joystick/has_joystick"] == 1.0
    assert server.sticks[4] == 2000


def test_the_wind_changes_live(backend: WingBackend) -> None:
    w = backend.weather
    assert (w.speed, w.from_deg, w.turbulence, w.gusts) == (3.0, 250.0, 1.0, 0.0)
    assert w.span == CHUPITO.wing.reference.span
    gusty = {
        "speed_ms": 8.0,
        "from_deg": 90.0,
        "turbulence": 2.0,
        "eddy_scale": 2.0,
        "gust_factor": 1.5,
        "gust_every_s": 20.0,
    }
    backend.wind(gusty, False)
    assert (w.speed, w.from_deg, w.turbulence, w.scale, w.gusts, w.gust_every) == (
        8.0,
        90.0,
        2.0,
        2.0,
        4.0,
        20.0,
    )
    backend.wind({**gusty, "turbulence": 0.0, "gust_factor": 1.0}, True)  # gust now, gusts off
    assert w.gusts == 0
    _wait(lambda: w.gust != (0.0, 0.0, 0.0))
    assert w is backend.aircraft.weather


def test_still_air_by_default() -> None:
    b = WingBackend("chupito", SLOT, ORIGIN, 0.0, {}, record=None)
    assert (b.weather.speed, b.weather.gusts) == (0.0, 0.0)  # no wind: no turbulence either
    assert b.recorder is None
    with pytest.raises(SystemExit):
        WingBackend("nanawing", SLOT, ORIGIN, 0.0, {}, record=None)


def test_the_dynamics_are_the_overlays(backend: WingBackend) -> None:
    backend.throw()
    _wait(lambda: backend.phase() == "flying")
    time.sleep(0.2)
    d = backend.dynamics()
    assert d is not None
    sent = json.dumps({"t": "dynamics", **d}, separators=(",", ":"))
    assert len(sent) < 60000, "fits one datagram from the physics process"
    d = json.loads(sent)  # as it goes to the page
    for key in ("airspeed", "alpha", "coefficients", "strips", "load", "wind", "motor"):
        assert key in d
    assert d["strips"] and "attached" in d["strips"][0]
    assert d["airspeed"] > 5
    assert d["motor"] == 0.0, "no INAV, no throttle"


def test_each_flight_is_recorded(tmp_path: Path) -> None:
    b = WingBackend("chupito", SLOT, ORIGIN, 270.0, WIND, record=tmp_path)
    b.start()
    try:
        time.sleep(0.1)
        b.throw()
        time.sleep(0.5)
    finally:
        b.stop()
    assert b.recorder is not None
    (path,) = tmp_path.glob("*-chupito-slot7.csv")
    rows = list(csv.reader(path.open()))
    assert rows[0] == FlightRecorder.FIELDS
    assert len(rows) > 10
    assert {r[1] for r in rows[1:]} >= {"hand"}
    assert any(r[1] in ("throw", "flying") for r in rows[1:])


def test_the_wing_physics_imports_only_the_standard_library() -> None:
    """physics.core and physics.wing, and the wing's backend with its X-Plane bridge, import
    only themselves and the standard library (DESIGN.md §2.4)."""
    code = (
        "import json, sys\n"
        "before = set(sys.modules)\n"
        "import drone_duels.physics.wing.backend, drone_duels.physics.wing.dynamics\n"
        "names = {m.split('.')[0] for m in set(sys.modules) - before}\n"
        "print(json.dumps(sorted(names - set(sys.stdlib_module_names))))\n"
    )
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=True)
    assert json.loads(out.stdout) == ["drone_duels"]
