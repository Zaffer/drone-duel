"""Missions over MAVLink, as a ground station uploads them (MAVLink's mission protocol), on a
link of their own: Betaflight's MAVLink UART (firmware/betaflight.py), where the game is the
only ground station. The game is system 254 (telemetry.mavlink.GAME_SYSID).

`MissionLink` reads on a thread of its own and keeps what the vehicle says it is flying
(MISSION_CURRENT: the waypoint, and whether the mission is active or complete). `upload` sends a
mission of waypoints and waits for the vehicle's answer: MISSION_COUNT, then each item as the
vehicle asks for it (MISSION_REQUEST_INT), then its MISSION_ACK.
"""

import logging
import socket
import threading
import time
from collections.abc import Sequence

from pymavlink.dialects.v20 import ardupilotmega as apm

from drone_duels.telemetry.mavlink import GAME_SYSID

log = logging.getLogger(__name__)

COMPONENT = apm.MAV_COMP_ID_MISSIONPLANNER
STATE_ACTIVE = apm.MISSION_STATE_ACTIVE
STATE_COMPLETE = apm.MISSION_STATE_COMPLETE
ACCEPTED = apm.MAV_MISSION_ACCEPTED
DENIED = apm.MAV_MISSION_DENIED


def result_name(result: int | None) -> str:
    """MAV_MISSION_RESULT, as words: "accepted", "denied", ...; "no answer" for None."""
    if result is None:
        return "no answer"
    entry = apm.enums["MAV_MISSION_RESULT"].get(result)
    name = entry.name if entry else str(result)
    return name.removeprefix("MAV_MISSION_").lower().replace("_", " ")


class _Sender:
    """What pymavlink writes to: the socket."""

    def __init__(self, sock: socket.socket) -> None:
        self.sock = sock

    def write(self, data: bytes) -> None:
        self.sock.sendall(data)


class MissionLink:
    """A ground station's mission link to one vehicle (system `target`, component 1) over TCP on
    this machine. Raises OSError if it cannot connect."""

    def __init__(self, port: int, target: int = 1, connect_s: float = 3.0) -> None:
        self.target = target
        self._sock = socket.create_connection(("127.0.0.1", port), timeout=connect_s)
        self._sock.settimeout(0.2)
        # One to send (on the caller's thread), one to parse (on the reader's).
        self._out = apm.MAVLink(_Sender(self._sock), srcSystem=GAME_SYSID, srcComponent=COMPONENT)
        self._in = apm.MAVLink(None, srcSystem=GAME_SYSID, srcComponent=COMPONENT)
        self._in.robust_parsing = True
        self._cond = threading.Condition()
        self._requests: list[int] = []
        self._ack: int | None = None
        self.current: tuple[int, int] | None = None
        """MISSION_CURRENT's waypoint and MISSION_STATE, as last heard."""
        self.closed = False
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read, name="mission-link", daemon=True)
        self._thread.start()

    def upload(
        self, points: Sequence[tuple[float, float, float]], timeout_s: float = 2.0
    ) -> int | None:
        """`points` (lat, lon, m above home) as a mission of waypoints, replacing the one there:
        the vehicle's answer (MAV_MISSION_RESULT: ACCEPTED, DENIED, ...), None if none came in
        `timeout_s`."""
        with self._cond:
            self._requests.clear()
            self._ack = None
        self._out.mission_count_send(
            self.target, 1, len(points), apm.MAV_MISSION_TYPE_MISSION
        )
        deadline = time.monotonic() + timeout_s
        while True:
            with self._cond:
                self._cond.wait_for(
                    lambda: self._requests or self._ack is not None or self.closed,
                    max(0.0, deadline - time.monotonic()),
                )
                requests, self._requests = self._requests, []
                ack = self._ack
            for seq in requests:
                if 0 <= seq < len(points):
                    lat, lon, alt = points[seq]
                    self._out.mission_item_int_send(
                        self.target, 1, seq, apm.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                        apm.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0,
                        round(lat * 1e7), round(lon * 1e7), alt, apm.MAV_MISSION_TYPE_MISSION,
                    )
            if ack is not None:
                return ack
            if self.closed or time.monotonic() >= deadline:
                return None

    def close(self) -> None:
        self._stop.set()
        self._thread.join(1)
        try:
            self._sock.close()
        except OSError:
            pass

    def _read(self) -> None:
        while not self._stop.is_set():
            try:
                data = self._sock.recv(4096)
            except TimeoutError:
                continue
            except OSError:
                break
            if not data:
                break
            for msg in self._in.parse_buffer(data) or []:
                self._heard(msg)
        with self._cond:
            self.closed = True
            self._cond.notify_all()

    def _heard(self, msg: apm.MAVLink_message) -> None:
        kind = msg.get_type()
        if msg.get_srcSystem() != self.target:
            return
        with self._cond:
            if kind == "MISSION_CURRENT":
                self.current = (msg.seq, getattr(msg, "mission_state", 0))  # pyright: ignore[reportAttributeAccessIssue]
            elif kind in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
                self._requests.append(msg.seq)  # pyright: ignore[reportAttributeAccessIssue]
            elif kind == "MISSION_ACK":
                self._ack = msg.type  # pyright: ignore[reportAttributeAccessIssue]
            else:
                return
            self._cond.notify_all()
