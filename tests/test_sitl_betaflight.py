"""The Betaflight 5" quad for real: Betaflight SITL on our racer5in physics, through the session,
flown on the sticks as the page flies it, and flying itself as Shoot Down's target. Slot 4, clear
of the other vehicles' tests.
"""

import math
import time
from collections.abc import Iterator

import pytest
from test_sitl_quad import Radio, Recorder, wait_for

from drone_duels.content.store import load
from drone_duels.firmware.betaflight import BINARY, Betaflight
from drone_duels.physics.core.rigidbody import to_euler
from drone_duels.server.protocol import POSE_FIELDS, OsdMsg, TelemetryMsg
from drone_duels.session.base import Events, Firmware, PhysicsProcess, SpawnContext
from drone_duels.session.session import Session, make_firmware
from drone_duels.world.geo import LocalFrame

pytestmark = [
    pytest.mark.sitl,
    pytest.mark.skipif(not BINARY.exists(), reason="no Betaflight SITL: scripts/setup.sh"),
]

SLOT = 4
ALT, QW = POSE_FIELDS.index("alt"), POSE_FIELDS.index("qw")
LAT, LON = POSE_FIELDS.index("lat"), POSE_FIELDS.index("lon")

Flight = tuple[Session, Recorder, list[Firmware]]


@pytest.fixture(scope="module")
def flight() -> Iterator[Flight]:
    events = Recorder()
    made: list[Firmware] = []

    def factory(ctx: SpawnContext, ev: Events, physics: PhysicsProcess) -> Firmware:
        firmware = make_firmware(ctx, ev, physics)
        made.append(firmware)
        return firmware

    session = Session(load(), events, slot=SLOT, firmware_factory=factory)
    session.start()
    try:
        session.command("start", location="billy-bishop", vehicle="quad5-betaflight")
        wait_for("ready", lambda: session.state in ("ready", "failed"), 120)
        assert session.state == "ready", session.step
        yield session, events, made
    finally:
        session.close()
        for firmware in made:
            assert isinstance(firmware, Betaflight)
            assert firmware.proc is not None and firmware.proc.poll() is not None
            assert firmware.physics.proc is not None and firmware.physics.proc.poll() is not None


def telemetry(events: Recorder) -> TelemetryMsg:
    t = events.telemetry()
    assert t is not None
    return t


def truth(events: Recorder) -> tuple[float, float, float, float]:
    """Roll, pitch, heading (degrees) and height above sea level, from the physics."""
    pose = events.pose()
    assert pose is not None
    roll, pitch, yaw = (math.degrees(a) for a in to_euler(pose[QW : QW + 4]))  # pyright: ignore[reportArgumentType]
    return roll, pitch, yaw % 360, pose[ALT]


def position(session: Session, events: Recorder) -> tuple[float, float, float]:
    """North and east of the pad and the height above it, m, from the physics."""
    pose = events.pose()
    assert pose is not None and session.origin is not None
    lat, lon, ground = session.origin
    north, east = LocalFrame(lat, lon).to_ne(pose[LAT], pose[LON])
    return north, east, pose[ALT] - ground


def heading_gap(a: float, b: float) -> float:
    return abs((a - b + 180) % 360 - 180)


def test_osd_screens_come_through(flight: Flight) -> None:
    _, events, _ = flight
    wait_for("an OSD screen", lambda: bool(events.of("osd")), 10)
    screen = OsdMsg.model_validate(events.of("osd")[-1])
    assert (screen.font, screen.cols, screen.rows) == ("betaflight", 30, 13)  # analog: NTSC
    assert any(c != 0x20 for c in screen.codes)


def test_fly_lose_the_radio_land_reset(flight: Flight) -> None:
    session, events, made = flight
    assert session.origin is not None
    ground = session.origin[2]
    with Radio(session) as radio:
        radio.set(mode=1000)  # POSHOLD: Betaflight will not arm in it
        time.sleep(1)
        blocked = telemetry(events).blockers
        assert "altitude hold switch" in blocked and "position hold switch" in blocked
        radio.set(mode=1500)  # ANGLE
        time.sleep(1)
        assert not telemetry(events).armed and telemetry(events).blockers == []
        radio.set(arm=2000)
        wait_for("armed", lambda: telemetry(events).armed, 10)
        assert telemetry(events).mode == "ANGLE"

        radio.set(throttle=1600)
        wait_for("10 m up", lambda: truth(events)[3] > ground + 10, 20)
        assert telemetry(events).climb > 1 and (telemetry(events).alt_rel or 0) > 7

        # Betaflight's attitude is the truth's: its frames are right.
        radio.set(throttle=1250, roll=1700)
        time.sleep(0.8)
        roll, _, _, _ = truth(events)
        assert roll > 8 and telemetry(events).roll_deg == pytest.approx(roll, abs=3)
        radio.set(roll=1500, pitch=1700)  # forward: nose down
        time.sleep(0.8)
        _, pitch, heading, _ = truth(events)
        assert pitch < -8 and telemetry(events).pitch_deg == pytest.approx(pitch, abs=4)
        assert heading_gap(telemetry(events).yaw_deg, heading) < 5
        radio.set(pitch=1500)
        lat, lon = telemetry(events).lat, telemetry(events).lon
        assert lat is not None and lon is not None
        assert abs(lat - session.origin[0]) < 1e-3 and abs(lon - session.origin[1]) < 1e-3

        # POSHOLD, the sticks let go: it stops where it is, in the place's wind (in ANGLE it
        # drifts metres a second), and stays at its height.
        radio.set(throttle=1275, mode=1000)
        wait_for("POSHOLD", lambda: telemetry(events).mode == "POSHOLD", 3)
        time.sleep(4)  # it brakes
        held = position(session, events)
        time.sleep(4)
        moved = position(session, events)
        assert math.dist(held[:2], moved[:2]) < 1.5 and abs(held[2] - moved[2]) < 1.5
        radio.set(mode=1500)

        # The page goes quiet: Betaflight's failsafe lands it and disarms on touchdown.
        radio.on = False
        wait_for("failsafe", lambda: telemetry(events).mode == "FAILSAFE", 5)
        wait_for("landed and disarmed", lambda: not telemetry(events).armed, 40)
        assert truth(events)[3] < ground + 0.3
        assert session._physics is not None  # pyright: ignore[reportPrivateUsage]
        assert session._physics.status.get("phase") == "ground"  # pyright: ignore[reportPrivateUsage]

        # It comes back; the pilot sets the switches down and resets to the pad.
        radio.set(throttle=1000, arm=1000)
        radio.on = True
        wait_for("failsafe over", lambda: telemetry(events).mode != "FAILSAFE", 10)
        firmware = session._firmware  # pyright: ignore[reportPrivateUsage]
        assert isinstance(firmware, Betaflight) and firmware.proc is not None
        pid = firmware.proc.pid
        session.command("reset")
        pad_lat, pad_lon, _ = session.origin
        wait_for(
            "back on the pad",
            lambda: (
                (p := events.pose()) is not None
                and abs(p[1] - pad_lat) < 1e-6
                and abs(p[2] - pad_lon) < 1e-6
            ),
            5,
        )
        assert session.state == "ready" and session._firmware is firmware  # pyright: ignore[reportPrivateUsage]
        assert firmware.proc.pid == pid and firmware.proc.poll() is None, "reset in place"
        wait_for("ready to arm", lambda: telemetry(events).blockers == [], 10)
        radio.set(arm=2000)
        wait_for("armed again", lambda: telemetry(events).armed, 10)
        radio.set(throttle=1500)
        wait_for("up again", lambda: truth(events)[3] > ground + 3, 10)
        radio.set(throttle=1000, arm=1000)
        wait_for("disarmed", lambda: not telemetry(events).armed, 5)
    assert len(made) == 1


def test_another_video_system_respawns_with_its_grid(flight: Flight) -> None:
    """Betaflight takes the HD grid only at boot: the session starts it again."""
    session, events, made = flight

    def grid() -> tuple[int, int] | None:
        screens = events.of("osd")
        if not screens:
            return None
        screen = OsdMsg.model_validate(screens[-1])
        return (screen.cols, screen.rows)

    try:
        session.set_video("digital")
        wait_for("ready again", lambda: session.state == "ready" and len(made) == 2, 60)
        wait_for("the HD grid", lambda: grid() == (53, 20), 10)
    finally:
        session.set_video("analog")
    wait_for("ready on analog", lambda: session.state == "ready" and len(made) == 3, 60)
    wait_for("the analog grid", lambda: grid() == (30, 13), 10)


def hover_at(session: Session, events: Recorder, h: float) -> None:
    """Ask for a hover at `h` m (Shoot Down's target) and wait until it holds there."""
    asked = time.monotonic()
    session.target("hover", h)
    wait_for("armed", lambda: telemetry(events).armed, 15)
    wait_for(f"{h:.0f} m up", lambda: position(session, events)[2] > h - 1.0, 40)
    print(f"\n{h:.0f} m reached {time.monotonic() - asked:.1f} s after asking")
    time.sleep(4)
    t = telemetry(events)
    assert t.mode == "POSHOLD" and t.armed
    assert abs((t.alt_rel or 0) - h) < 1.5 and abs(t.climb) < 0.7
    north, east, height = position(session, events)
    assert abs(height - h) < 1.5  # the truth agrees
    time.sleep(3)
    assert math.dist((north, east), position(session, events)[:2]) < 1.5  # held there


def test_flies_itself_as_a_target(flight: Flight) -> None:
    """Shoot Down's target: Betaflight takes no ground station's commands, so the game flies it
    on its own switches: armed, POSHOLD, up to the height and held there. Shot through a motor
    it comes down; a reset (in place) and it goes up again; and it lands on its own."""
    session, events, _ = flight
    hover_at(session, events, 20.0)

    session.hit("motor", 1, (0.0, 1.0, 0.0), (0.08, 0.08, 0.0))

    def phase() -> str | None:
        physics = session._physics  # pyright: ignore[reportPrivateUsage]
        return physics.status.get("phase") if physics else None

    wait_for("shot down", lambda: phase() in ("crashed", "ground"), 20)
    print("shot down:", phase(), [m["text"] for m in events.of("log")][-6:])
    assert position(session, events)[2] < 0.5

    session.command("reset")
    wait_for("back on the pad, disarmed", lambda: not telemetry(events).armed, 10)
    wait_for("on the pad", lambda: math.dist(position(session, events)[:2], (0, 0)) < 0.1, 5)
    hover_at(session, events, 10.0)

    session.target("land")
    wait_for("landed and disarmed", lambda: not telemetry(events).armed, 40)
    assert position(session, events)[2] < 0.3
    assert phase() == "ground"
    assert not any("crash" in m.get("text", "") for m in events.of("log")[-5:])
