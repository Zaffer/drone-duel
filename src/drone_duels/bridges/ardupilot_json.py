"""ArduPilot SITL's JSON physics backend (`--model JSON`), the physics' side of it
(libraries/SITL/SIM_JSON.cpp; its examples in libraries/SITL/examples/JSON).

Each frame SITL sends its servo outputs over UDP to the physics: a little-endian packet of magic
(18458 for 16 channels, 29569 for 32), the frame rate (Hz), a frame count, and the PWM, µs. It
then waits for the physics' answer, one line of JSON, newline before and after, back to the port it
sent from: the time, the IMU (gyro rad/s and specific force m/s², body FRD), the position (m NED
from home), the attitude (quaternion, body to NED) and the velocity (m/s NED), plus the battery.
SITL blocks until it comes, so the two run in lockstep: the physics steps only when a servo
packet arrives, by one frame (1 / frame rate), and never otherwise.

SITL resends its servos when an answer is a second late: a repeated frame count gets the last
answer again, unstepped. A frame count that goes backwards is a restarted SITL: the physics goes
back to its start.

SIM_JSON's parser finds each key with strstr in the whole line, so no key may appear inside
another's name or before it ("velocity" before a "velocity_wind", say): `encode_state` sends only
these.
"""

import json
import logging
import socket
import struct
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol

from drone_duels.physics.core.rigidbody import Quat

log = logging.getLogger(__name__)

SERVO16 = struct.Struct("<HHI16H")
SERVO32 = struct.Struct("<HHI32H")
MAGIC16 = 18458
MAGIC32 = 29569
CONNECTED_S = 1.0
"""No servo packet for this long: SITL is not stepping the physics."""


@dataclass(frozen=True)
class Fdm:
    """The physics' state for one frame, in ArduPilot's frames."""

    t: float
    gyro: tuple[float, float, float]
    """Body FRD, rad/s."""
    accel: tuple[float, float, float]
    """Specific force, body FRD, m/s²."""
    position: tuple[float, float, float]
    """NED from home, m."""
    quat: Quat
    """Body FRD to NED."""
    velocity: tuple[float, float, float]
    """NED, m/s."""
    battery_v: float
    battery_a: float


@dataclass(frozen=True)
class ServoPacket:
    frame_rate: int
    frame_count: int
    pwm: tuple[int, ...]


def decode_servo(data: bytes) -> ServoPacket | None:
    """A servo packet, or None if `data` is not one."""
    if len(data) == SERVO16.size:
        fields = SERVO16.unpack(data)
        magic = MAGIC16
    elif len(data) == SERVO32.size:
        fields = SERVO32.unpack(data)
        magic = MAGIC32
    else:
        return None
    if fields[0] != magic:
        return None
    return ServoPacket(frame_rate=fields[1], frame_count=fields[2], pwm=tuple(fields[3:]))


def encode_servo(frame_rate: int, frame_count: int, pwm: list[int] | tuple[int, ...]) -> bytes:
    """As SITL sends them (for the tests)."""
    if len(pwm) == 16:
        return SERVO16.pack(MAGIC16, frame_rate, frame_count, *pwm)
    if len(pwm) == 32:
        return SERVO32.pack(MAGIC32, frame_rate, frame_count, *pwm)
    raise ValueError("16 or 32 channels")


def _r(v: tuple[float, ...], digits: int = 9) -> list[float]:
    return [round(x, digits) for x in v]


def encode_state(fdm: Fdm) -> bytes:
    """The answer to one servo packet."""
    body = {
        "timestamp": round(fdm.t, 9),
        "imu": {"gyro": _r(fdm.gyro), "accel_body": _r(fdm.accel)},
        "position": _r(fdm.position),
        "quaternion": _r(fdm.quat),
        "velocity": _r(fdm.velocity),
        "battery": {"voltage": round(fdm.battery_v, 4), "current": round(fdm.battery_a, 4)},
    }
    return b"\n" + json.dumps(body, separators=(",", ":")).encode() + b"\n"


class Model(Protocol):
    def step(self, dt: float, pwm: tuple[int, ...]) -> Fdm: ...

    def reset(self) -> None: ...


class JsonBridge:
    """Listens for SITL on UDP `port` and steps `model` a frame per servo packet, on a thread."""

    def __init__(
        self,
        model: Model,
        port: int,
        host: str = "127.0.0.1",
        on_frame: Callable[[Fdm], None] | None = None,
    ) -> None:
        self.model = model
        self.port = port
        self.host = host
        self.on_frame = on_frame
        self.frames = 0
        self.restarts = 0
        self._sock: socket.socket | None = None
        self._last_count: int | None = None
        self._last_reply = b""
        self._last_rx = 0.0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, self.port))
        sock.settimeout(0.2)
        self._sock = sock
        self._thread = threading.Thread(target=self._run, name="ardupilot-json", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(2)
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def connected(self) -> bool:
        return self._last_rx > 0 and time.monotonic() - self._last_rx < CONNECTED_S

    def handle(self, data: bytes) -> bytes | None:
        """One datagram from SITL: the answer to send back, or None to ignore it."""
        pkt = decode_servo(data)
        if pkt is None or pkt.frame_rate == 0:
            return None
        last = self._last_count
        if last is not None and pkt.frame_count == last and self._last_reply:
            return self._last_reply  # SITL resent: answer again, unstepped
        if last is not None and pkt.frame_count < last:
            log.info("SITL restarted (frame %d after %d): back to the start", pkt.frame_count, last)
            self.restarts += 1
            self.model.reset()
        self._last_count = pkt.frame_count
        fdm = self.model.step(1.0 / pkt.frame_rate, pkt.pwm)
        self.frames += 1
        if self.on_frame is not None:
            self.on_frame(fdm)
        self._last_reply = encode_state(fdm)
        return self._last_reply

    def _run(self) -> None:
        sock = self._sock
        assert sock is not None
        while not self._stop.is_set():
            try:
                data, sender = sock.recvfrom(512)
            except TimeoutError:
                continue
            except OSError:
                return
            self._last_rx = time.monotonic()
            try:
                reply = self.handle(data)
            except Exception:
                log.exception("physics step failed")
                continue
            if reply is not None:
                try:
                    sock.sendto(reply, sender)
                except OSError as exc:
                    log.warning("answer to SITL: %s", exc)
