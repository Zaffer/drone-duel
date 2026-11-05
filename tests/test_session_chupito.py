"""The Chupito end to end, as the page flies it: the session starts INAV SITL on the wing's
physics, the sticks arm it, a throw launches it, and it flies; a stick step moves the elevons
within 45 ms; the pose, telemetry and OSD streams come out as the page reads them. Then the
radio link is lost, and INAV's own failsafe brings it home until the pilot takes it back."""

import statistics
import subprocess
import threading
import time
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from typing import Any

import pytest

from drone_duels.content.store import load
from drone_duels.firmware import inav
from drone_duels.physics.wing.airframe import CHUPITO
from drone_duels.server.protocol import POSE, POSE_FIELDS, OsdMsg, TelemetryMsg, VideoSystem
from drone_duels.session.session import Session

SLOT = 2
"""Clear of slot 0, where ArduPilot's tests fly."""

pytestmark = [
    pytest.mark.sitl,
    pytest.mark.skipif(not inav.BINARY.exists(), reason="no INAV SITL: scripts/setup.sh"),
]


class Recorded:
    """An Events that keeps everything."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.messages: list[dict[str, Any]] = []
        self.poses: list[dict[str, float]] = []
        self.arrived: list[float] = []

    def publish(self, msg: dict[str, Any]) -> None:
        with self.lock:
            self.messages.append(msg)

    def publish_pose(self, frame: bytes) -> None:
        pose: dict[str, float] = dict(zip(POSE_FIELDS, POSE.unpack(frame), strict=True))
        with self.lock:
            self.poses.append(pose)
            self.arrived.append(time.monotonic())

    def channel(self, ch: str) -> list[dict[str, Any]]:
        with self.lock:
            return [m for m in self.messages if m.get("ch") == ch]

    def last(self, ch: str) -> dict[str, Any] | None:
        found = self.channel(ch)
        return found[-1] if found else None

    def pose(self) -> dict[str, float]:
        with self.lock:
            return self.poses[-1]

    def mode(self) -> str | None:
        t = self.last("telemetry")
        return t["mode"] if t else None


class Radio:
    """The page's sticks at 50 Hz: AETR then AUX, µs. `quiet`: the page sends nothing."""

    def __init__(self, session: Session) -> None:
        self.session = session
        self.sticks = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]
        self.quiet = False
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.wait(0.02):
            if not self.quiet:
                self.session.send_rc(list(self.sticks))

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(1)


def _wait(done: Callable[[], bool], seconds: float, why: Callable[[], str]) -> None:
    deadline = time.monotonic() + seconds
    while not done():
        if time.monotonic() > deadline:
            pytest.fail(why())
        time.sleep(0.05)


def _ours() -> list[str]:
    """INAV SITL and physics processes of this test's slot still running."""
    found = []
    for pattern in (f"tcpbaseport={6760 + 10 * SLOT}", f"physics.runner.*--slot {SLOT}"):
        out = subprocess.run(["pgrep", "-af", pattern], capture_output=True, text=True).stdout
        found += [line for line in out.splitlines() if "pgrep" not in line]
    return found


def _stick_to_elevons(session: Session, events: Recorded, radio: Radio, roll: int) -> float:
    """Seconds from a roll stick step to the elevons moving (in the poses as they come out)."""
    with events.lock:
        before = events.poses[-1]["a0"] - events.poses[-1]["a1"]
        seen = len(events.poses)
    radio.sticks[0] = roll
    sent = time.monotonic()
    session.send_rc(list(radio.sticks))
    deadline = sent + 1.0
    while time.monotonic() < deadline:
        with events.lock:
            for pose, at in zip(events.poses[seen:], events.arrived[seen:], strict=True):
                if abs(pose["a0"] - pose["a1"] - before) > 0.02:
                    return at - sent
            seen = len(events.poses)
        time.sleep(0.001)
    pytest.fail(f"the elevons did not move for a roll of {roll}")


def _speed(p: dict[str, float]) -> float:
    return (p["vn"] ** 2 + p["ve"] ** 2 + p["vd"] ** 2) ** 0.5


def _text(osd: OsdMsg, row: int) -> str:
    return bytes(osd.codes[row * osd.cols : (row + 1) * osd.cols]).decode("latin-1")


@contextmanager
def _flying(video: VideoSystem) -> Iterator[tuple[Session, Recorded, Radio, float]]:
    """The Chupito spawned on `video`, armed in ANGLE, thrown at full throttle and climbing
    out: the session, its streams, the radio and the ground's height. Everything stops after."""
    assert not _ours(), "left over from before"
    events = Recorded()
    session = Session(load(), events, slot=SLOT)
    session.set_video(video)
    session.start()
    radio: Radio | None = None
    try:
        session.command("start", location="billy-bishop", vehicle="chupito", pad=0)
        _wait(
            lambda: session.state in ("ready", "failed"),
            90,
            lambda: f"not ready: {session.state} {session.step}",
        )
        assert session.state == "ready", session.step
        assert session.origin is not None
        ground = session.origin[2]
        assert events.pose()["alt"] == pytest.approx(ground + CHUPITO.launch.height), "in hand"

        radio = Radio(session)
        time.sleep(0.5)
        radio.sticks[4] = 2000  # arm, CH6 low: ANGLE
        _wait(
            lambda: bool((t := events.last("telemetry")) and t["armed"]),
            5,
            lambda: f"did not arm: {events.last('telemetry')}",
        )
        radio.sticks[2] = 1800  # INAV's launch waits for the throw with the throttle up
        time.sleep(1.0)
        session.set_dynamics(True)  # the overlay, D
        session.command("throw")
        time.sleep(1.0)
        yield session, events, radio, ground
        assert session.state == "ready", "INAV lives on (it has a battery)"
    finally:
        if radio is not None:
            radio.stop()
        session.command("stop")
        _wait(lambda: session.state == "idle", 15, lambda: f"did not stop: {session.state}")
        session.close()
    assert not _ours(), "nothing left behind"
    assert not [m for m in events.channel("log") if m["level"] == "error"]


def test_the_chupito_flies_on_inav_through_the_session() -> None:
    """On digital video: it climbs out, INAV sees it, and its OSD is Betaflight HD's 53 x 20."""
    with _flying("digital") as (session, events, radio, ground):
        launch = events.pose()
        samples = []
        for _ in range(12):
            time.sleep(0.5)
            samples.append(events.pose())
        top = max(p["alt"] for p in samples)
        assert all(p["alt"] > ground + 5 for p in samples), "airborne all along"
        assert top > launch["alt"] + 15, f"climbs: from {launch['alt']:.1f} to {top:.1f}"
        assert max(_speed(p) for p in samples) > 12, "flies"
        assert samples[-1]["a2"] > 100, "the prop turns"

        # The firmware loop, end to end: about 30 ms (2026-11-04). Ours is at most 16 ms of it
        # (the physics takes the sticks every 4 ms, the bridge every 4 ms, the pose goes at
        # 120 Hz); the rest is INAV's own, as on the real aircraft.
        delays = []
        for roll in (1700, 1500, 1300, 1500) * 3:
            delays.append(_stick_to_elevons(session, events, radio, roll))
            time.sleep(0.3)
        assert statistics.median(delays) < 0.045, delays
        assert max(delays) < 0.08, delays

        t = TelemetryMsg.model_validate(events.last("telemetry"))
        assert t.armed and t.mode in ("LAUNCH", "ANGLE"), t.mode
        assert t.lat is not None and t.alt_amsl is not None and t.gps_sats > 0
        assert t.alt_rel is not None and t.alt_rel > 10, "INAV sees it climb"
        assert t.groundspeed > 5 and t.throttle_pct > 30

        dynamics = events.channel("dynamics")
        assert dynamics and dynamics[-1]["data"]["airspeed"] > 10, "the overlay's numbers"
        assert dynamics[-1]["data"]["motor"] > 0.3

        screens = [OsdMsg.model_validate(m) for m in events.channel("osd")]
        assert screens, "INAV's OSD arrives"
        assert {(s.font, s.cols, s.rows) for s in screens} == {("betaflight", 53, 20)}
        osd = screens[-1]
        assert _text(osd, 17)[6:10] == "ANGL", "TBS's flight mode, bottom left"

        radio.sticks[2] = 1000
        radio.sticks[4] = 1000  # disarm
        _wait(
            lambda: bool((t := events.last("telemetry")) and not t["armed"]),
            5,
            lambda: f"did not disarm: {events.last('telemetry')}",
        )


def test_a_lost_link_is_inavs_failsafe_until_the_pilot_takes_it_back() -> None:
    """On analog video (SD OSD, NTSC's 13 rows). The page goes quiet: the physics stops
    hearing sticks after a second, INAV's receiver after 0.2 s more, and after INAV's
    failsafe_delay (0.5 s) its failsafe flies it home (failsafe_procedure RTH), armed and
    airborne. The sticks come back centred: INAV stays in failsafe, as it does until the pilot
    moves the sticks (failsafe_stick_threshold); a touch of roll and it is the pilot's again,
    in the mode the switches say (ANGLE)."""
    with _flying("analog") as (_, events, radio, ground):
        time.sleep(6)  # INAV's launch ends; ANGLE
        radio.sticks[2] = 1500
        _wait(lambda: events.mode() == "ANGLE", 5, lambda: f"mode {events.mode()}")

        radio.quiet = True
        t0 = time.monotonic()
        lowest = events.pose()["alt"]
        while time.monotonic() - t0 < 3.0:
            time.sleep(0.1)
            lowest = min(lowest, events.pose()["alt"])
        lost = events.last("telemetry")
        assert lost is not None and lost["armed"], "a lost link does not disarm it"
        assert lost["mode"] in ("FAILSAFE", "RTH"), f"INAV's failsafe: {lost['mode']}"
        assert lowest > ground + 10, "airborne all along"

        radio.quiet = False  # the link is back, the sticks centred
        time.sleep(2.0)
        assert events.mode() in ("FAILSAFE", "RTH"), "INAV waits for the sticks to move"
        radio.sticks[0] = 1600  # a touch of roll
        _wait(lambda: events.mode() == "ANGLE", 5, lambda: f"still {events.mode()}")
        radio.sticks[0] = 1500
        assert events.pose()["alt"] > ground + 10

        def mode_shown() -> str:
            last = events.last("osd")
            return _text(OsdMsg.model_validate(last), 11)[1:5] if last else ""

        _wait(lambda: mode_shown() == "ANGL", 3, lambda: f"the OSD shows {mode_shown()!r}")
        screens = [OsdMsg.model_validate(m) for m in events.channel("osd")]
        assert {(s.font, s.cols, s.rows) for s in screens} == {("betaflight", 30, 13)}
        shown = [_text(s, 11)[1:5] for s in screens]
        assert "!FS!" in shown, "INAV's failsafe, on NTSC's picture"

        radio.sticks[2] = 1000
        radio.sticks[4] = 1000  # disarm
        _wait(
            lambda: bool((t := events.last("telemetry")) and not t["armed"]),
            5,
            lambda: f"did not disarm: {events.last('telemetry')}",
        )
