"""The session's lifecycle with a stand-in firmware and no physics (the truck), and with a fake
firmware that fails or blocks: start, switch, reset, destroy, stop, cancellation."""

import threading
import time
from collections.abc import Callable, Sequence
from typing import Any, cast

from drone_duels.content.store import load
from drone_duels.session.base import Firmware, FirmwareError, PhysicsProcess, SpawnContext
from drone_duels.session.session import Session


class Recorder:
    def __init__(self) -> None:
        self.msgs: list[dict[str, Any]] = []
        self.lock = threading.Lock()

    def publish(self, msg: dict[str, Any]) -> None:
        with self.lock:
            self.msgs.append(msg)

    def publish_pose(self, frame: bytes) -> None:
        pass

    def states(self) -> list[str]:
        with self.lock:
            return [m["state"] for m in self.msgs if m["ch"] == "status"]

    def wait_state(self, state: str, timeout: float = 5) -> dict[str, Any]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                for m in reversed(self.msgs):
                    if m["ch"] == "status":
                        if m["state"] == state:
                            return m
                        break
            time.sleep(0.01)
        raise AssertionError(f"no {state}: {self.states()}")


def test_the_truck_starts_without_firmware_and_stops() -> None:
    events = Recorder()
    s = Session(load(), events, slot=7)
    s.start()
    try:
        s.command("start", "billy-bishop", "truck")
        ready = events.wait_state("ready")
        assert ready["vehicle"] == "truck"
        assert ready["origin"] == [43.63093, -79.38945, 77.0] or ready["origin"] == (
            43.63093,
            -79.38945,
            77.0,
        )
        s.command("stop")
        events.wait_state("idle")
    finally:
        s.close()


class FakeFirmware(Firmware):
    name = "fake"
    behaviour: str = "ok"
    sticks: list[Sequence[int]] = []

    def start(self) -> None:
        pass

    def wait_ready(self, step: Callable[[str], None]) -> None:
        step("waiting")
        if self.behaviour == "fail":
            raise FirmwareError("it broke: see fake.log")
        if self.behaviour == "block":
            while True:
                self.ctx.check()
                time.sleep(0.01)

    def send_rc(self, us: Sequence[int]) -> None:
        FakeFirmware.sticks.append(us)

    def rc_lost(self) -> None:
        pass

    def alive(self) -> bool:
        return True

    def stop(self) -> None:
        pass


def _session(events: Recorder) -> Session:
    def factory(ctx: SpawnContext, ev: Any, physics: PhysicsProcess) -> Firmware:  # noqa: ANN401
        return FakeFirmware(ctx, ev, physics)

    # The truck has no physics process, so the fake firmware alone decides readiness.
    s = Session(load(), events, slot=7, firmware_factory=factory)
    s.start()
    return s


def test_a_failing_firmware_says_why() -> None:
    events = Recorder()
    FakeFirmware.behaviour = "fail"
    s = _session(events)
    try:
        s.command("start", "billy-bishop", "truck")
        failed = events.wait_state("failed")
        assert "fake.log" in failed["step"]
        assert any(m["ch"] == "log" and m["level"] == "error" for m in events.msgs)
    finally:
        s.close()


def test_a_new_start_cancels_a_spawn_in_progress() -> None:
    events = Recorder()
    FakeFirmware.behaviour = "block"
    s = _session(events)
    try:
        s.command("start", "billy-bishop", "truck")
        events.wait_state("starting")
        time.sleep(0.1)
        FakeFirmware.behaviour = "ok"
        s.command("start", "area-xo", "truck")
        ready = events.wait_state("ready", timeout=30)  # area-xo may look its ground up online
        assert ready["location"] == "area-xo"
    finally:
        s.close()


def test_sticks_reach_the_firmware_only_when_ready_and_destroy_stops_it() -> None:
    events = Recorder()
    FakeFirmware.behaviour = "ok"
    FakeFirmware.sticks = []
    s = _session(events)
    try:
        s.send_rc([1500] * 8)
        assert FakeFirmware.sticks == []
        s.command("start", "billy-bishop", "truck")
        events.wait_state("ready")
        s.send_rc([1500, 1500, 1000, 1500])
        assert FakeFirmware.sticks == [[1500, 1500, 1000, 1500]]
        s.command("destroy")
        events.wait_state("destroyed")
        s.command("reset")
        events.wait_state("ready")
    finally:
        s.close()


class InPlaceFirmware(FakeFirmware):
    resets_in_place = True
    starts = 0

    def start(self) -> None:
        InPlaceFirmware.starts += 1


class FakePhysics:
    def __init__(self) -> None:
        self.sent: list[dict[str, Any]] = []

    def send(self, msg: dict[str, Any]) -> None:
        self.sent.append(msg)

    def alive(self) -> bool:
        return True

    def stop(self) -> None:
        pass


def test_a_reset_leaves_a_firmware_that_resets_in_place_running() -> None:
    events = Recorder()
    FakeFirmware.behaviour = "ok"
    InPlaceFirmware.starts = 0

    def factory(ctx: SpawnContext, ev: Any, physics: PhysicsProcess) -> Firmware:  # noqa: ANN401
        return InPlaceFirmware(ctx, ev, physics)

    s = Session(load(), events, slot=7, firmware_factory=factory)
    s.start()
    try:
        s.command("start", "billy-bishop", "truck")
        events.wait_state("ready")
        physics = FakePhysics()
        s._physics = cast(PhysicsProcess, physics)  # the truck has none; stand one in
        before = len(events.states())
        s.command("reset")
        deadline = time.monotonic() + 5
        while len(events.states()) == before and time.monotonic() < deadline:
            time.sleep(0.01)
        assert physics.sent == [{"t": "reset"}]
        assert InPlaceFirmware.starts == 1
        # Said again, so a page waiting on the reset hears it is done.
        assert events.states()[before:] == ["ready"]
    finally:
        s.close()


class LiveVideoFirmware(InPlaceFirmware):
    videos: list[str] = []

    def set_video(self, video: Any) -> bool:  # noqa: ANN401
        LiveVideoFirmware.videos.append(video)
        return True


def test_a_video_switch_leaves_a_firmware_that_takes_it_live_flying() -> None:
    events = Recorder()
    FakeFirmware.behaviour = "ok"
    InPlaceFirmware.starts = 0
    LiveVideoFirmware.videos = []

    def factory(ctx: SpawnContext, ev: Any, physics: PhysicsProcess) -> Firmware:  # noqa: ANN401
        return LiveVideoFirmware(ctx, ev, physics)

    s = Session(load(), events, slot=7, firmware_factory=factory)
    s.start()
    try:
        s.command("start", "billy-bishop", "truck")
        events.wait_state("ready")
        s.set_video("digital")
        assert LiveVideoFirmware.videos == ["digital"]
        assert events.wait_state("ready")["video"] == "digital"
        time.sleep(0.1)
        assert InPlaceFirmware.starts == 1
        assert "starting" not in events.states()[-2:]
    finally:
        s.close()


def test_a_video_switch_respawns_a_firmware_that_takes_it_only_at_boot() -> None:
    events = Recorder()
    FakeFirmware.behaviour = "ok"
    InPlaceFirmware.starts = 0

    def factory(ctx: SpawnContext, ev: Any, physics: PhysicsProcess) -> Firmware:  # noqa: ANN401
        return InPlaceFirmware(ctx, ev, physics)

    s = Session(load(), events, slot=7, firmware_factory=factory)
    s.start()
    try:
        s.command("start", "billy-bishop", "truck")
        events.wait_state("ready")
        s.set_video("digital")
        deadline = time.monotonic() + 5
        while InPlaceFirmware.starts < 2 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert InPlaceFirmware.starts == 2
        assert events.wait_state("ready")["video"] == "digital"
    finally:
        s.close()
