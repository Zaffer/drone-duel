"""Betaflight SITL's simulator protocol, the physics' side of it (src/platform/SIMULATOR/sitl.c and
target/SITL/target.h in Betaflight; its harness, src/test/sitl/sitl_harness.py, is the
reference for the frames).

Betaflight does not run in lockstep: it keeps its own clock, which runs at the rate the physics'
timestamps advance against the wall clock, measured packet to packet. So the physics paces
itself, FRAME_HZ frames a second in real time. Each frame it steps with the motors Betaflight
last sent and sends the state (fdm_packet: 18 doubles) to Betaflight's state port. For each state
packet Betaflight sends its motors once, on its next loop, two ways: raw (the motor count, two
bytes' padding, 16 floats: the outputs in µs, 1000 stopped, 1055 armed at idle, 2000 full) and
normalised (4 floats). The raw ones are used: the model takes µs as ArduPilot's outputs, on
the same 1000-2000 range.

The default SITL build (ENABLE_GAZEBO_BRIDGE) reads the frames of Gazebo's BetaflightPlugin,
which `encode_state` writes from ours (ArduPilot's: body FRD, NED):

- time, s: the bridge's own, which never goes back (Betaflight drops a packet older than the
  last; the model's own time starts again on a reset);
- gyro: roll right, pitch nose up, yaw anticlockwise from above: (p, q, -r);
- accelerometer: the specific force on the body's FLU axes, negated: (-fx, fy, fz);
- attitude: body FLU to world NWU, turned -90° about the vertical, then conjugated by a half
  turn about x (Betaflight undoes both);
- velocity: ENU, m/s;
- position: longitude and latitude, mirrored about the pad, and the height above sea level, m.
  Gazebo's are mirrored about its origin, and Betaflight mirrors them back about the first it
  gets. The physics is at the pad when Betaflight starts, so that is the pad: exact.
- pressure: unused (this build takes the barometer from the height).

Motors: Betaflight's quad X numbers them rear right, front right, rear left, front left; the
model takes ArduPilot's order, front right, rear left, front left, rear right. They spin the
same ways (Betaflight's default, props in: 1 and 4 clockwise, as ArduPilot's 4 and 3).
"""

import logging
import math
import select
import socket
import struct
import threading
import time
from typing import Protocol

from drone_duels.bridges.ardupilot_json import Fdm
from drone_duels.world.geo import LocalFrame

log = logging.getLogger(__name__)

FRAME_HZ = 1000.0
"""Physics frames a second, in real time: Betaflight's gyro updates at this rate."""
FDM = struct.Struct("<18d")
MOTORS_RAW = struct.Struct("<H2x16f")
MOTORS = struct.Struct("<4f")
AP_FROM_BF = (1, 2, 3, 0)
"""ArduPilot's motor i (front right, rear left, front left, rear right) is Betaflight's
AP_FROM_BF[i] (0-based: rear right, front right, rear left, front left)."""
STOPPED_US = 1000
CONNECTED_S = 1.0
"""No motors from Betaflight for this long: it is not flying the physics, and its motors stop."""
LATE_S = 0.1
"""Behind real time by more than this (a pause), the frames carry on from now rather than
catching up."""
GRAVITY = 9.80665
HALF = math.sqrt(0.5)

Quat = tuple[float, float, float, float]


def _mul(a: Quat, b: Quat) -> Quat:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _conj_x180(q: Quat) -> Quat:
    """A half turn about x either side: NED and NWU (or FRD and FLU) are that turn apart."""
    return (q[0], q[1], -q[2], -q[3])


RZ_NEG90: Quat = (HALF, 0.0, 0.0, -HALF)


def gazebo_quat(q_ned: Quat) -> Quat:
    """Body FRD to NED, as the Gazebo plugin sends it: Betaflight takes it to body FLU to NWU."""
    return _conj_x180(_mul(RZ_NEG90, _conj_x180(q_ned)))


def encode_state(
    fdm: Fdm, t: float, lat: float, lon: float, alt_m: float, pad: tuple[float, float]
) -> bytes:
    """One fdm_packet: the model's frame `fdm` at the bridge's time `t`, at `lat`, `lon` and
    `alt_m` above sea level, with the pad (lat, lon) the GPS is mirrored about."""
    p, q, r = fdm.gyro
    fx, fy, fz = fdm.accel
    vn, ve, vd = fdm.velocity
    qw, qx, qy, qz = gazebo_quat(fdm.quat)
    pressure = 101325.0 * (1.0 - 2.25577e-5 * alt_m) ** 5.25588
    return FDM.pack(
        t,
        p, q, -r,
        -fx, fy, fz,
        qw, qx, qy, qz,
        ve, vn, -vd,
        2 * pad[1] - lon, 2 * pad[0] - lat, alt_m,
        pressure,
    )  # fmt: skip


def decode_motors(data: bytes) -> tuple[int, int, int, int] | None:
    """A raw motor packet as ArduPilot's motors 1-4, µs; None if `data` is not one."""
    if len(data) != MOTORS_RAW.size:
        return None
    count, *raw = MOTORS_RAW.unpack(data)
    if count < 4:
        return None
    us = [round(raw[bf]) for bf in AP_FROM_BF]
    return (us[0], us[1], us[2], us[3])


def encode_motors(bf_us: tuple[float, ...]) -> bytes:
    """As Betaflight sends them, in its own order (for the tests)."""
    values = list(bf_us) + [0.0] * (16 - len(bf_us))
    return MOTORS_RAW.pack(len(bf_us), *values)


class Model(Protocol):
    def step(self, dt: float, pwm: tuple[int, ...]) -> Fdm: ...


class BetaflightBridge:
    """Steps `model` FRAME_HZ times a second in real time on a thread, flown by Betaflight SITL:
    motors from it on UDP `motors_port` (raw; the normalised ones on `motors_port` + 1 are
    taken and dropped), the state to it on `state_port`. The model starts at `origin` (lat, lon,
    m above sea level), the pad."""

    def __init__(
        self,
        model: Model,
        motors_port: int,
        state_port: int,
        origin: tuple[float, float, float],
        frame_hz: float = FRAME_HZ,
        host: str = "127.0.0.1",
    ) -> None:
        self.model = model
        self.motors_port = motors_port
        self.state_port = state_port
        self.origin = origin
        self.local = LocalFrame(origin[0], origin[1])
        self.period = 1.0 / frame_hz
        self.host = host
        self.t = 0.0
        self.frames = 0
        self.motors: tuple[int, ...] = (STOPPED_US,) * 4
        self._last_rx = 0.0
        self._socks: list[socket.socket] = []
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        for port in (self.motors_port, self.motors_port + 1):
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((self.host, port))
            sock.setblocking(False)
            self._socks.append(sock)
        self._thread = threading.Thread(target=self._run, name="betaflight-fdm", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(2)
        for sock in self._socks:
            sock.close()
        self._socks = []

    def connected(self) -> bool:
        return self._last_rx > 0 and time.monotonic() - self._last_rx < CONNECTED_S

    def frame(self) -> bytes:
        """One frame: step with the motors Betaflight last sent; the state to send it."""
        if not self.connected():
            self.motors = (STOPPED_US,) * 4
        fdm = self.model.step(self.period, self.motors)
        self.t += self.period
        self.frames += 1
        n, e, d = fdm.position
        lat, lon = self.local.to_latlon(n, e)
        return encode_state(fdm, self.t, lat, lon, self.origin[2] - d, self.origin[:2])

    def take(self, data: bytes) -> None:
        """A datagram from Betaflight on the raw motors' port."""
        motors = decode_motors(data)
        if motors is not None:
            self.motors = motors
            self._last_rx = time.monotonic()

    def _drain(self, timeout: float) -> None:
        """Take what Betaflight sends until `timeout` s from now."""
        deadline = time.monotonic() + timeout
        while True:
            left = deadline - time.monotonic()
            ready = select.select(self._socks, [], [], max(0.0, left))[0]
            for sock in ready:
                try:
                    while True:
                        data = sock.recv(512)
                        if sock is self._socks[0]:
                            self.take(data)
                except BlockingIOError:
                    pass
            if left <= 0:
                return

    def _run(self) -> None:
        out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        target = (self.host, self.state_port)
        due = time.monotonic()
        try:
            while not self._stop.is_set():
                try:
                    state = self.frame()
                except Exception:
                    log.exception("physics step failed")
                    state = b""
                if state:
                    try:
                        out.sendto(state, target)
                    except OSError:
                        pass  # Betaflight not up yet
                due += self.period
                now = time.monotonic()
                if now - due > LATE_S:
                    due = now
                self._drain(due - now)
        finally:
            out.close()
