"""Each SITL copter's true physical state, straight from its physics: the sim's FPV camera.

On a real drone the pilot flies by the camera's video, a link of its own (analog or digital,
some 20-40 ms glass to glass) that shows where the frame really is. MAVLink telemetry is the
autopilot's estimate, at whatever rate the ground asks for; it feeds the map, not the
goggles. In the sim the physics stands in for the camera: `swarm sim` starts every SITL with
--enable-fgview, which sends a FlightGear FDM packet on each physics step (1200 Hz) to UDP
5503 + 10 * instance. PhysicsFeed keeps the latest state of each copter for the 3D viewer's
camera feed, which draws the drone it looks from and every other drone where they really are.

Sim only. On hardware the FPV video comes from the drone's camera.
"""

import logging
import math
import select
import socket
import struct
import threading
from dataclasses import dataclass

log = logging.getLogger(__name__)

FG_VIEW_BASE_PORT = 5503
FG_VIEW_PORT_STRIDE = 10
FDM_VERSION = 0x18
FDM_HEAD = struct.Struct(">II3d4f")
"""The start of FlightGear's FGNetFDM, network order: version, padding, longitude, latitude
(radians), altitude (m AMSL), agl, roll, pitch, yaw (radians)."""


def fg_view_port(sysid: int) -> int:
    """`swarm sim` starts copter `sysid` as SITL instance sysid - 1."""
    return FG_VIEW_BASE_PORT + FG_VIEW_PORT_STRIDE * (sysid - 1)


@dataclass(frozen=True)
class PhysicsState:
    lat: float
    lon: float
    """Degrees."""
    alt: float
    """Metres above home, as the viewer places drones."""
    roll: float
    pitch: float
    yaw: float
    """Radians: right side down, nose up, clockwise from north."""


def parse_fdm(data: bytes, home_alt_m: float) -> PhysicsState | None:
    if len(data) < FDM_HEAD.size:
        return None
    version, _, lon, lat, alt, _, roll, pitch, yaw = FDM_HEAD.unpack_from(data)
    if version != FDM_VERSION:
        return None
    return PhysicsState(
        lat=math.degrees(lat),
        lon=math.degrees(lon),
        alt=alt - home_alt_m,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
    )


class PhysicsFeed:
    """Receives every copter's physics in a thread; `latest` from any."""

    def __init__(self, sysids: list[int], home_alt_m: float, host: str = "127.0.0.1") -> None:
        self.home_alt_m = home_alt_m
        self._socks: dict[socket.socket, int] = {}
        for sysid in sysids:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setblocking(False)
            try:
                sock.bind((host, fg_view_port(sysid)))
            except OSError as exc:
                sock.close()
                port = fg_view_port(sysid)
                log.warning("no FPV physics for copter %d: UDP %d (%s)", sysid, port, exc)
                continue
            self._socks[sock] = sysid
        self._lock = threading.Lock()
        self._latest: dict[int, tuple[int, PhysicsState]] = {}
        """Per copter: how many states so far, and the last."""
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="physics", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)
        for sock in self._socks:
            sock.close()

    def latest(self) -> dict[int, tuple[int, PhysicsState]]:
        """Per copter heard from: how many states it has sent, and its last."""
        with self._lock:
            return dict(self._latest)

    def _run(self) -> None:
        socks = list(self._socks)
        while not self._stop.is_set() and socks:
            ready, _, _ = select.select(socks, [], [], 0.5)
            for sock in ready:
                # Only the newest packet counts: skip any that queued up behind it.
                data = b""
                try:
                    while True:
                        data = sock.recv(1024)
                except BlockingIOError:
                    pass
                except OSError:
                    continue
                state = parse_fdm(data, self.home_alt_m)
                if state is None:
                    continue
                sysid = self._socks[sock]
                with self._lock:
                    count = self._latest.get(sysid, (0, None))[0]
                    self._latest[sysid] = (count + 1, state)
