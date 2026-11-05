"""The server's radio, flown from the server (drone_duels.rc.pilot; DESIGN.md §3.1, §12): with a
stand-in radio and firmware, no page and no SITL. Each stick movement reaches the firmware at
once, the sticks repeat at 250 Hz between movements, the pages see the channels at 60 Hz, and
the silence of an unplugged radio is a lost link."""

import dataclasses
import statistics
import threading
import time
from collections.abc import Callable, Sequence
from typing import Any

from drone_duels.content.schema import AuxSpec, FirmwareSpec
from drone_duels.content.store import Content, load
from drone_duels.rc.pilot import Pilot
from drone_duels.session.base import Firmware, PhysicsProcess, SpawnContext
from drone_duels.session.session import Session

REST = [0.0, 0.0, -1.0, 0.0, -1.0, -1.0, -1.0, -1.0]
"""Sticks centred, throttle and switches low."""


class Radio:
    """A radio as rc.evdev reads it, moved by the test."""

    start_axes = REST

    def __init__(self, on_change: Callable[[], None]) -> None:
        self.on_change = on_change
        self._axes: list[float] | None = list(Radio.start_axes)
        self.name: str | None = "Test radio (Vendor: 1234 Product: 5678)"

    def axes(self) -> list[float] | None:
        return None if self._axes is None else list(self._axes)

    def move(self, axes: list[float] | None) -> None:
        self._axes = axes
        self.on_change()

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


class FakeInav(Firmware):
    """INAV, as far as the sticks go: when each came, and whether the link was lost."""

    name = "fake inav"
    sticks: list[tuple[float, list[int]]] = []
    lost = 0

    def start(self) -> None:
        pass

    def wait_ready(self, step: Callable[[str], None]) -> None:
        pass

    def send_rc(self, us: Sequence[int]) -> None:
        FakeInav.sticks.append((time.monotonic(), list(us)))

    def rc_lost(self) -> None:
        FakeInav.lost += 1

    def alive(self) -> bool:
        return True

    def stop(self) -> None:
        pass


class Events:
    def __init__(self) -> None:
        self.msgs: list[dict[str, Any]] = []
        self.lock = threading.Lock()

    def publish(self, msg: dict[str, Any]) -> None:
        with self.lock:
            self.msgs.append(msg)

    def publish_pose(self, frame: bytes) -> None:
        pass

    def channel(self, ch: str) -> list[dict[str, Any]]:
        with self.lock:
            return [m for m in self.msgs if m["ch"] == ch]


def _content() -> Content:
    """The truck, made an INAV aircraft with a launch switch on CH8: a vehicle with firmware
    and no physics process."""
    c = load()
    truck = c.vehicles["truck"].model_copy(
        update={"firmware": FirmwareSpec(kind="inav"), "aux": AuxSpec(arm=5, launch=8)}
    )
    return dataclasses.replace(c, vehicles={**c.vehicles, "truck": truck})


def _flying(
    prefs: dict[str, Any] | None = None, axes: list[float] = REST
) -> tuple[Session, Pilot, Radio, Events]:
    """Flying the INAV truck on the server's radio, its sticks at `axes`."""
    Radio.start_axes = axes
    FakeInav.sticks = []
    FakeInav.lost = 0
    events = Events()
    content = _content()

    def firmware(ctx: SpawnContext, ev: Any, physics: PhysicsProcess) -> Firmware:  # noqa: ANN401
        return FakeInav(ctx, ev, physics)

    session = Session(content, events, slot=7, firmware_factory=firmware)
    session.start()
    radios: list[Radio] = []

    def reader(on_change: Callable[[], None]) -> Radio:
        radios.append(Radio(on_change))
        return radios[-1]

    pilot = Pilot(session, events.publish, content, prefs or {}, reader=reader)
    pilot.want("page", on=True, fly=True)
    session.command("start", "billy-bishop", "truck")
    deadline = time.monotonic() + 5
    while session.state != "ready":
        assert time.monotonic() < deadline, session.step
        time.sleep(0.01)
    return session, pilot, radios[0], events


def _close(session: Session, pilot: Pilot) -> None:
    pilot.close()
    session.close()


def test_a_stick_movement_reaches_the_firmware_at_once_with_no_page() -> None:
    session, pilot, radio, _ = _flying()
    try:
        delays = []
        for i in range(20):
            roll = 0.5 if i % 2 == 0 else -0.5
            want = 1750 if roll > 0 else 1250
            time.sleep(0.02)
            moved = time.monotonic()
            radio.move([roll, *REST[1:]])
            deadline = moved + 0.5
            while not any(t >= moved and us[0] == want for t, us in FakeInav.sticks):
                assert time.monotonic() < deadline, "the sticks never came"
                time.sleep(0.0005)
            delays.append(
                next(t for t, us in FakeInav.sticks if t >= moved and us[0] == want) - moved
            )
        assert statistics.median(delays) < 0.005, delays
        assert max(delays) < 0.02, delays
    finally:
        _close(session, pilot)


def test_the_sticks_repeat_at_250_hz_between_movements() -> None:
    session, pilot, _, _ = _flying()
    try:
        time.sleep(0.2)
        start = len(FakeInav.sticks)
        time.sleep(1.0)
        assert 200 <= len(FakeInav.sticks) - start <= 260
    finally:
        _close(session, pilot)


def test_the_pages_see_the_channels_at_60_hz_as_the_calibration_maps_them() -> None:
    name = "Test radio (Vendor: 1234 Product: 5678)"
    reversed_roll = {
        "channels": [{"axis": 0, "reverse": True, "cal": {"min": -1, "mid": 0, "max": 1}}]
    }
    prefs = {"controls": {"calibrations": {name: reversed_roll}}}
    session, pilot, radio, events = _flying(prefs)
    try:
        radio.move([0.5, *REST[1:]])
        time.sleep(0.05)
        start = len(events.channel("radio"))
        time.sleep(1.0)
        shown = events.channel("radio")
        assert 50 <= len(shown) - start <= 70
        assert shown[-1]["name"] == name
        assert shown[-1]["axes"][0] == 0.5
        assert shown[-1]["channels"] == [-0.5]
        assert FakeInav.sticks[-1][1][0] == 1250, "flown as shown"
    finally:
        _close(session, pilot)


def test_an_unplugged_radio_goes_quiet_and_the_firmware_hears_a_lost_link() -> None:
    session, pilot, radio, _ = _flying()
    try:
        time.sleep(0.1)
        assert FakeInav.lost == 0
        radio.move(None)
        time.sleep(0.05)
        count = len(FakeInav.sticks)
        time.sleep(0.4)
        assert len(FakeInav.sticks) == count, "nothing sent"
        assert FakeInav.lost == 1, "the session's failsafe"
    finally:
        _close(session, pilot)


def test_a_page_that_flies_the_keyboard_has_the_radio_send_nothing() -> None:
    session, pilot, _, _ = _flying()
    try:
        pilot.want("page", on=True, fly=False)
        time.sleep(0.05)
        count = len(FakeInav.sticks)
        time.sleep(0.2)
        assert len(FakeInav.sticks) == count
    finally:
        _close(session, pilot)


LAUNCH_HIGH = [*REST[:7], 1.0]


def _throws(session: Session) -> list[float]:
    """When the session is told to throw, from now on."""
    throws: list[float] = []
    command = session.command

    def spy(action: str, *args: Any, **kwargs: Any) -> None:  # noqa: ANN401
        if action == "throw":
            throws.append(time.monotonic())
        command(action, *args, **kwargs)

    session.command = spy  # type: ignore[method-assign]
    return throws


def test_the_launch_switch_throws_on_its_flick_up_once() -> None:
    session, pilot, radio, _ = _flying()
    throws = _throws(session)
    try:
        time.sleep(0.05)
        radio.move(LAUNCH_HIGH)
        time.sleep(0.1)
        radio.move(LAUNCH_HIGH)
        time.sleep(0.05)
        assert len(throws) == 1
    finally:
        _close(session, pilot)


def test_a_launch_switch_high_at_the_start_waits_to_go_low_first() -> None:
    session, pilot, radio, _ = _flying(axes=LAUNCH_HIGH)
    throws = _throws(session)
    try:
        time.sleep(0.1)
        assert throws == []
        radio.move(list(REST))
        time.sleep(0.05)
        radio.move(LAUNCH_HIGH)
        time.sleep(0.05)
        assert len(throws) == 1
    finally:
        _close(session, pilot)
