"""What a session is made of: the context a vehicle spawns in, where its streams go, its firmware,
and its physics process (DESIGN.md §3, §6.4).

Every vehicle is a firmware process plus a physics process, joined by that firmware's own
simulator protocol. The firmware adapters (drone_duels.firmware) start and watch the firmware and
read what a ground station would (telemetry, OSD, plans); the physics process
(drone_duels.physics.runner) plays the world to it in lockstep.
"""

import json
import logging
import subprocess
import sys
import threading
from abc import ABC, abstractmethod
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

from drone_duels.content.schema import Location, StartPad, Vehicle, Wind
from drone_duels.ports import SlotPorts
from drone_duels.server.protocol import POSE, VideoSystem

log = logging.getLogger(__name__)

ROOT = Path(__file__).resolve().parents[3]
LOGS = ROOT / "logs"
FIRMWARE_FILES = ROOT / "firmware"
EXTERNAL = ROOT / "external"


class Events(Protocol):
    """Where a running vehicle's streams go: the server's hub, which fans them out to pages."""

    def publish(self, msg: dict[str, Any]) -> None:
        """A server → page message as a dict with its `ch` (drone_duels.server.protocol)."""
        ...

    def publish_pose(self, frame: bytes) -> None:
        """A binary pose frame (protocol.POSE)."""
        ...


class Cancelled(Exception):  # noqa: N818 - a signal to stop, not an error
    """The spawn was overtaken by another session command."""


class FirmwareError(RuntimeError):
    """The firmware did not start, or stopped. The message says what to look at."""


@dataclass(frozen=True)
class SpawnContext:
    vehicle: Vehicle
    location: Location
    pad: StartPad
    ground_alt_m: float
    """The ground at the pad, m above sea level."""
    ports: SlotPorts
    video: VideoSystem
    wind: Wind
    workdir: Path
    """The firmware's own directory for this vehicle and slot, kept between spawns."""
    cancel: threading.Event

    @property
    def origin(self) -> tuple[float, float, float]:
        return (self.pad.at[0], self.pad.at[1], self.ground_alt_m)

    def check(self) -> None:
        if self.cancel.is_set():
            raise Cancelled()

    def firmware_file(self, name: str) -> Path:
        return FIRMWARE_FILES / name


class PhysicsProcess:
    """The physics, a child process (drone_duels.physics.runner), and the UDP pair to it: control
    in, pose, dynamics and status out."""

    def __init__(self, ctx: SpawnContext, events: Events) -> None:
        self.ctx = ctx
        self.events = events
        self.status: dict[str, Any] = {}
        self.proc: subprocess.Popen[bytes] | None = None
        self._sock: Any = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._status_changed = threading.Condition()

    def start(self) -> None:
        import socket

        ctx = self.ctx
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("127.0.0.1", ctx.ports.physics_state))
        sock.settimeout(0.2)
        self._sock = sock
        lat, lon, alt = ctx.origin
        args = [
            sys.executable,
            "-m",
            "drone_duels.physics.runner",
            "--kind",
            ctx.vehicle.physics.kind,
            "--airframe",
            ctx.vehicle.physics.airframe,
            "--slot",
            str(ctx.ports.slot),
            "--origin",
            f"{lat:.8f},{lon:.8f},{alt:.2f}",
            "--heading",
            f"{ctx.pad.heading_deg:.1f}",
            "--wind",
            ctx.wind.model_dump_json(),
        ]
        ctx.workdir.mkdir(parents=True, exist_ok=True)
        out = (ctx.workdir / "physics.log").open("ab")
        log.info("physics: %s", " ".join(args))
        self.proc = subprocess.Popen(args, stdout=out, stderr=subprocess.STDOUT, cwd=ROOT)
        self._thread = threading.Thread(target=self._receive, name="physics-rx", daemon=True)
        self._thread.start()

    def send(self, msg: dict[str, Any]) -> None:
        if self._sock is None:
            return
        data = json.dumps(msg, separators=(",", ":")).encode()
        try:
            self._sock.sendto(data, ("127.0.0.1", self.ctx.ports.physics_control))
        except OSError:
            pass

    def wait_status(self, done: Callable[[dict[str, Any]], bool], timeout: float) -> bool:
        """Wait until the physics' last status satisfies `done`."""
        import time

        deadline = time.monotonic() + timeout
        with self._status_changed:
            while not done(self.status):
                left = deadline - time.monotonic()
                if left <= 0 or self.ctx.cancel.is_set() or not self.alive():
                    return False
                self._status_changed.wait(min(left, 0.2))
        return True

    def alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def stop(self) -> None:
        self.send({"t": "stop"})
        self._stop.set()
        if self.proc is not None:
            try:
                self.proc.wait(2)
            except subprocess.TimeoutExpired:
                self.proc.terminate()
                try:
                    self.proc.wait(3)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
        if self._thread:
            self._thread.join(1)
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def _receive(self) -> None:
        sock = self._sock
        while not self._stop.is_set():
            try:
                data = sock.recv(65536)
            except TimeoutError:
                continue
            except OSError:
                return
            if data[:1] == b"P" and len(data) == 1 + POSE.size:
                self.events.publish_pose(data[1:])
                continue
            try:
                msg = json.loads(data)
            except ValueError:
                continue
            kind = msg.pop("t", None)
            if kind == "status":
                changed = (msg.get("phase"), msg.get("connected")) != (
                    self.status.get("phase"),
                    self.status.get("connected"),
                )
                with self._status_changed:
                    self.status = msg
                    self._status_changed.notify_all()
                if changed:
                    self.events.publish(
                        {"ch": "physics", "phase": str(msg.get("phase", "")),
                         "connected": bool(msg.get("connected"))}
                    )
            elif kind == "dynamics":
                self.events.publish({"ch": "dynamics", "data": msg})
            elif kind == "log":
                self.events.publish(
                    {"ch": "log", "level": msg.get("level", "info"), "source": "physics",
                     "text": msg.get("text", "")}
                )


class Firmware(ABC):
    """A flight controller's SITL: start it against the physics, wait until it is ready to arm,
    feed it the pilot's sticks, and read what a ground station would."""

    name: str = "firmware"

    def __init__(self, ctx: SpawnContext, events: Events, physics: PhysicsProcess) -> None:
        self.ctx = ctx
        self.events = events
        self.physics = physics

    @abstractmethod
    def start(self) -> None:
        """Start the firmware (and anything it needs, such as mavlink-router)."""

    @abstractmethod
    def wait_ready(self, step: Callable[[str], None]) -> None:
        """Block until it could arm (or raise FirmwareError, or Cancelled via ctx.check()).
        `step` reports progress for the page ("waiting for GPS")."""

    @abstractmethod
    def send_rc(self, us: Sequence[int]) -> None:
        """The pilot's channels, from channel 1, µs. Called at the radio's rate (250 Hz)."""

    @abstractmethod
    def rc_lost(self) -> None:
        """The radio went quiet (the page is gone): behave as a receiver that lost its link."""

    @abstractmethod
    def alive(self) -> bool: ...

    @abstractmethod
    def stop(self) -> None:
        """Stop it and every thread reading it. Safe to call twice."""


class NoFirmware(Firmware):
    """A vehicle without a flight controller (the truck): nothing to start."""

    name = "none"

    def start(self) -> None:
        pass

    def wait_ready(self, step: Callable[[str], None]) -> None:
        pass

    def send_rc(self, us: Sequence[int]) -> None:
        pass

    def rc_lost(self) -> None:
        pass

    def alive(self) -> bool:
        return True

    def stop(self) -> None:
        pass
