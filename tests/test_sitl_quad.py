"""The 5" quad for real: ArduCopter SITL on our racer5in physics, through the session, flown on
the sticks as the page flies it. Slot 1, clear of slot 0 (another vehicle's tests may run there).
"""

import struct
import threading
import time
from collections.abc import Callable, Iterator
from typing import Any

import pytest

from drone_duels.content.store import load
from drone_duels.firmware.ardupilot import ARDUCOPTER, ROUTER, ArduCopter
from drone_duels.server.protocol import POSE, POSE_FIELDS, OsdMsg, TelemetryMsg
from drone_duels.session.base import Events, Firmware, PhysicsProcess, SpawnContext
from drone_duels.session.session import Session, make_firmware

pytestmark = [
    pytest.mark.sitl,
    pytest.mark.skipif(
        not (ARDUCOPTER.exists() and ROUTER.exists()), reason="no ArduPilot SITL: scripts/setup.sh"
    ),
]

SLOT = 1
ALT = POSE_FIELDS.index("alt")


class Recorder:
    """The server's hub, as far as the tests need: everything published, kept."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.msgs: list[dict[str, Any]] = []
        self.poses: list[tuple[float, ...]] = []

    def publish(self, msg: dict[str, Any]) -> None:
        with self.lock:
            self.msgs.append(msg)

    def publish_pose(self, frame: bytes) -> None:
        with self.lock:
            self.poses.append(POSE.unpack(frame))

    def of(self, ch: str) -> list[dict[str, Any]]:
        with self.lock:
            return [m for m in self.msgs if m["ch"] == ch]

    def telemetry(self) -> TelemetryMsg | None:
        found = self.of("telemetry")
        return TelemetryMsg.model_validate(found[-1]) if found else None

    def pose(self) -> tuple[float, ...] | None:
        with self.lock:
            return self.poses[-1] if self.poses else None


class Radio:
    """The page's sticks at 50 Hz: roll, pitch, throttle, yaw, arm (5), mode (6)."""

    def __init__(self, session: Session) -> None:
        self.session = session
        self.us = [1500, 1500, 1000, 1500, 1000, 1000] + [1500] * 10
        self.on = True
        """False: the radio is off, nothing is sent (the page went away)."""
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def set(self, **channels: int) -> None:
        index = {"roll": 0, "pitch": 1, "throttle": 2, "yaw": 3, "arm": 4, "mode": 5}
        for name, us in channels.items():
            self.us[index[name]] = us

    def _run(self) -> None:
        while not self._stop.wait(0.02):
            if self.on:
                self.session.send_rc(list(self.us))

    def __enter__(self) -> "Radio":
        self._thread.start()
        return self

    def __exit__(self, *exc: object) -> None:
        self._stop.set()
        self._thread.join(1)


def wait_for(what: str, done: Callable[[], bool], timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while not done():
        if time.monotonic() > deadline:
            pytest.fail(f"timed out waiting for {what}")
        time.sleep(0.1)


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
        session.command("start", location="billy-bishop", vehicle="quad5")
        wait_for("ready", lambda: session.state in ("ready", "failed"), 180)
        assert session.state == "ready", session.step
        yield session, events, made
    finally:
        session.close()
        for firmware in made:
            assert isinstance(firmware, ArduCopter)
            assert firmware.sitl is not None and firmware.sitl.poll() is not None
            assert firmware.router is not None and firmware.router.poll() is not None
            assert firmware.physics.proc is not None and firmware.physics.proc.poll() is not None


def test_osd_screens_come_through(flight: Flight) -> None:
    _, events, _ = flight
    wait_for("an OSD screen", lambda: bool(events.of("osd")), 10)
    screen = OsdMsg.model_validate(events.of("osd")[-1])
    assert (screen.cols, screen.rows) == (30, 13)  # analog: the chip's grid as NTSC shows it
    assert len(screen.codes) == 30 * 13
    assert any(c != 0x20 for c in screen.codes)  # something drawn, not a blank screen


def test_the_video_system_switches_in_flight(flight: Flight) -> None:
    """ArduPilot takes the HD grid without a reboot, so the vehicle stays up and ready."""
    session, events, _ = flight
    firmware = session._firmware
    assert isinstance(firmware, ArduCopter) and firmware.sitl is not None
    pid = firmware.sitl.pid

    def grid() -> tuple[int, int]:
        screen = OsdMsg.model_validate(events.of("osd")[-1])
        return (screen.cols, screen.rows)

    try:
        session.set_video("digital")
        wait_for("the HD grid", lambda: grid() == (50, 18), 10)
    finally:
        session.set_video("analog")
    wait_for("the analog grid again", lambda: grid() == (30, 13), 10)
    assert session.state == "ready" and session._firmware is firmware
    assert firmware.sitl.pid == pid and firmware.sitl.poll() is None


def test_arm_climb_land_disarm(flight: Flight) -> None:
    session, events, _ = flight
    assert session.origin is not None
    origin_alt = session.origin[2]
    with Radio(session) as radio:
        time.sleep(1)
        telemetry = events.telemetry()
        assert telemetry is not None and not telemetry.armed and telemetry.blockers == []
        radio.set(arm=2000)  # arm switch high, throttle low, LOITER
        wait_for("armed", lambda: (t := events.telemetry()) is not None and t.armed, 10)
        assert events.telemetry().mode == "LOITER"  # pyright: ignore[reportOptionalMemberAccess]

        radio.set(throttle=1850)
        wait_for(
            "5 m up", lambda: (t := events.telemetry()) is not None and (t.alt_rel or 0) > 5.0, 30
        )
        pose = events.pose()
        assert pose is not None and pose[ALT] > origin_alt + 4.5  # the truth agrees
        climb = events.telemetry()
        assert climb is not None and climb.climb > 0.5

        radio.set(throttle=1500)  # LOITER holds
        time.sleep(3)
        held = events.telemetry()
        assert held is not None and abs(held.climb) < 0.5 and held.alt_rel and held.alt_rel > 5

        # The page goes quiet: the receiver stops its pulses, and ArduPilot's radio failsafe
        # brakes and holds (FS_THR_ENABLE 7).
        radio.on = False
        wait_for(
            "radio failsafe",
            lambda: (t := events.telemetry()) is not None and t.mode == "BRAKE",
            10,
        )
        assert any("Radio Failsafe" in m["text"] for m in events.of("log"))
        braked = events.telemetry()
        assert braked is not None and braked.armed and (braked.alt_rel or 0) > 4
        # It comes back; the pilot flicks the mode switch to fly LOITER again.
        radio.on = True
        time.sleep(1.5)
        radio.set(mode=1500)
        time.sleep(0.7)
        radio.set(mode=1000)
        wait_for(
            "LOITER again", lambda: (t := events.telemetry()) is not None and t.mode == "LOITER", 10
        )

        radio.set(throttle=1000)  # down, and LOITER lands
        wait_for(
            "landed", lambda: (p := events.pose()) is not None and p[ALT] < origin_alt + 0.1, 30
        )
        physics = session._physics  # pyright: ignore[reportPrivateUsage]
        assert physics is not None
        # Within 0.1 m it may be a few ms yet from its legs touching (0.03 m at 2.5 m/s).
        wait_for("on the ground", lambda: physics.status.get("phase") == "ground", 1)
        time.sleep(2)
        radio.set(arm=1000)
        wait_for("disarmed", lambda: (t := events.telemetry()) is not None and not t.armed, 10)
    assert not any("crash" in m.get("text", "") for m in events.of("log"))


def test_pose_stream_is_the_physics(flight: Flight) -> None:
    session, events, _ = flight
    pose = events.pose()
    assert pose is not None and len(pose) == len(POSE_FIELDS)
    fields: dict[str, float] = dict(zip(POSE_FIELDS, pose, strict=True))
    assert session.origin is not None
    lat, lon, _ = session.origin
    assert abs(fields["lat"] - lat) < 1e-4 and abs(fields["lon"] - lon) < 1e-4
    assert struct.calcsize("<18d") == POSE.size
