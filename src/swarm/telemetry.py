"""Publish the watcher's snapshots as JSON lines over UDP and to a log file."""

import json
import socket
from pathlib import Path
from typing import TYPE_CHECKING, TypedDict

if TYPE_CHECKING:
    from swarm.mission import Plan

DEFAULT_PORT = 14600


class VehicleSnapshot(TypedDict):
    sysid: int
    link: bool
    """Its position is fresh (swarm.watcher WatcherConfig.position_stale_s): the link is up."""
    nav: bool
    """Its EKF can navigate (VehicleState.position_ok): it can take off in GUIDED."""
    mode: str
    armed: bool
    pilot: bool
    """A human is flying it: armed in a pilot mode (swarm.state.PILOT_MODES)."""
    role: str
    n: float
    e: float
    d: float
    vn: float
    ve: float
    vd: float
    heading: float
    roll: float
    pitch: float
    battery: int | None
    battery_v: float | None
    sticks: list[int]
    """RC input of channels 1-4 (roll, pitch, throttle, yaw) in µs; empty until reported."""
    motors: list[int]
    """Output to motors 1-4 in µs (SERVO_OUTPUT_RAW); empty until reported."""
    slot_n: float | None
    slot_e: float | None
    wp: int | None
    """Current mission item (MISSION_CURRENT seq)."""
    mission_state: int


class Snapshot(TypedDict):
    t: float
    origin_lat: float
    origin_lon: float
    """WGS84 origin of the local north-east frame the n/e fields are in."""
    leader: int | None
    phase: str
    vehicles: list[VehicleSnapshot]
    target_n: float | None
    target_e: float | None
    """The mission's point B in the local frame, when the watcher knows it."""


class TelemetryPublisher:
    def __init__(self, port: int = DEFAULT_PORT, log_path: Path | None = None) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._addr = ("127.0.0.1", port)
        self._log = log_path.open("a") if log_path else None

    def publish(self, snapshot: Snapshot) -> None:
        line = json.dumps(snapshot)
        self._sock.sendto(line.encode(), self._addr)
        if self._log:
            self._log.write(line + "\n")

    def publish_plan(self, plan: "Plan", log: bool = True) -> None:
        """A vehicle's mission, fence and rally points. Logged only when `log` (on change)."""
        line = json.dumps(plan)
        self._sock.sendto(line.encode(), self._addr)
        if self._log and log:
            self._log.write(line + "\n")

    def close(self) -> None:
        if self._log:
            self._log.close()
