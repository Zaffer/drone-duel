"""Shoot Down for real: ArduCopter SITL flies itself up to hover over the pad as a ground station
would ask it (Session.target), a hit kills a motor in the physics and it comes down, and a reset
and a new hover put it back up. Slot 1, as test_sitl_quad.py (which has finished with it by
then).
"""

import time
from collections.abc import Iterator

import pytest

from drone_duels.content.store import load
from drone_duels.firmware.ardupilot import ARDUCOPTER, ROUTER, ArduCopter
from drone_duels.server.protocol import POSE_FIELDS, PhysicsMsg
from drone_duels.session.base import Events, Firmware, PhysicsProcess, SpawnContext
from drone_duels.session.session import Session, make_firmware
from tests.test_sitl_quad import Recorder, wait_for

pytestmark = [
    pytest.mark.sitl,
    pytest.mark.skipif(
        not (ARDUCOPTER.exists() and ROUTER.exists()), reason="no ArduPilot SITL: scripts/setup.sh"
    ),
]

SLOT = 1
ALT = POSE_FIELDS.index("alt")
VD = POSE_FIELDS.index("vd")
HEIGHT = 30.0


@pytest.fixture
def shoot() -> Iterator[tuple[Session, Recorder]]:
    events = Recorder()
    made: list[Firmware] = []

    def factory(ctx: SpawnContext, ev: Events, physics: PhysicsProcess) -> Firmware:
        firmware = make_firmware(ctx, ev, physics)
        made.append(firmware)
        return firmware

    session = Session(load(), events, slot=SLOT, firmware_factory=factory)
    session.start()
    try:
        session.command("start", location="billy-bishop", vehicle="quad5")
        wait_for("ready", lambda: session.state in ("ready", "failed"), 180)
        assert session.state == "ready", session.step
        yield session, events
    finally:
        session.close()
        for firmware in made:
            assert isinstance(firmware, ArduCopter)
            assert firmware.sitl is not None and firmware.sitl.poll() is not None
            assert firmware.router is not None and firmware.router.poll() is not None
            assert firmware.physics.proc is not None and firmware.physics.proc.poll() is not None


def height(session: Session, events: Recorder) -> float:
    pose = events.pose()
    assert pose is not None and session.origin is not None
    return pose[ALT] - session.origin[2]


def hover_at(session: Session, events: Recorder, h: float) -> float:
    """Ask for a hover at `h` m and wait until it holds there; how long that took, s."""
    asked = time.monotonic()
    session.target("hover", h)
    wait_for("ready", lambda: session.state == "ready", 180)
    ready = time.monotonic()
    wait_for(
        "armed and climbing",
        lambda: (t := events.telemetry()) is not None and t.armed and (t.alt_rel or 0) > 1,
        30,
    )
    wait_for(f"{h:.0f} m up", lambda: height(session, events) > h - 1.0, 60)
    took = time.monotonic() - asked
    print(f"\n{h:.0f} m reached {time.monotonic() - ready:.1f} s after ready")
    time.sleep(3)
    t = events.telemetry()
    assert t is not None and t.mode == "GUIDED" and t.armed
    assert abs((t.alt_rel or 0) - h) < 1.5 and abs(t.climb) < 0.5
    assert abs(height(session, events) - h) < 1.5  # the truth agrees
    return took


def test_hover_shoot_a_motor_reset_and_hover_again(shoot: tuple[Session, Recorder]) -> None:
    session, events = shoot
    took = hover_at(session, events, HEIGHT)
    print(f"\nhover: {HEIGHT:.0f} m held {took:.1f} s after asking")

    # A bullet through motor 1 (front right), pushing it east, off centre.
    shot = time.monotonic()
    session.hit("motor", 1, (0.0, 1.0, 0.0), (0.08, 0.08, 0.0))
    fastest = 0.0
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        pose = events.pose()
        assert pose is not None
        fastest = max(fastest, pose[VD])
        phase = (session._physics.status if session._physics else {}).get("phase")  # pyright: ignore[reportPrivateUsage]
        if phase in ("crashed", "ground"):
            break
        time.sleep(0.02)
    down = time.monotonic() - shot
    phase = (session._physics.status if session._physics else {}).get("phase")  # pyright: ignore[reportPrivateUsage]
    print(f"shot down: {phase} after {down:.1f} s, falling at up to {fastest:.1f} m/s")
    print("ArduPilot said:", [m["text"] for m in events.of("log")][-8:])
    assert phase in ("crashed", "ground") and height(session, events) < 0.5
    assert fastest > 8.0
    physics = [PhysicsMsg.model_validate(m) for m in events.of("physics")]
    damaged = [m for m in physics if m.damage is not None]
    assert damaged and damaged[-1].damage is not None
    assert damaged[-1].damage.thrust == [0.0, 1.0, 1.0, 1.0]

    # Respawn and up again: a reset (ArduCopter boots again) and a hover queued behind it.
    asked = time.monotonic()
    session.command("reset")
    wait_for("the respawn", lambda: session.state != "ready", 10)
    took = hover_at(session, events, 10.0)
    print(f"reset and hover again: 10 m held {time.monotonic() - asked:.1f} s after the reset "
          f"({took:.1f} s after asking, the respawn included)")
    wait_for(
        "the damage cleared",
        lambda: PhysicsMsg.model_validate(events.of("physics")[-1]).damage is None,
        5,
    )

    session.target("land")
    wait_for("landed", lambda: height(session, events) < 0.2, 30)
    wait_for("disarmed", lambda: (t := events.telemetry()) is not None and not t.armed, 20)
