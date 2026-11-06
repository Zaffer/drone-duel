"""The wing behind INAV: the flight model playing X-Plane to INAV SITL, for the physics process
(drone_duels.physics.runner's Backend).

The aircraft starts in the hand at the pad, facing its heading, in the location's wind. INAV
reads its sticks through the simulator (receiver type SIM), so the pilot's channels come here
and go to INAV as X-Plane's joystick axes. The map's ground under the wing comes from the page
(MapGround). With no sticks from the server (the page's radio link lost), INAV's receiver
loses its link too, and INAV's own failsafe flies it. Each run is recorded to logs/flights/
(FlightRecorder).

Like the rest of physics.wing, this imports only the standard library and the wing's own
modules (the X-Plane bridge and the ports are stdlib-only too).
"""

import logging
import math
import time
from collections.abc import Sequence
from dataclasses import asdict
from pathlib import Path
from typing import Any

from drone_duels.bridges.xplane import THROTTLE, XPlaneServer
from drone_duels.physics.core import weather
from drone_duels.physics.wing.aircraft import Aircraft, Airframe
from drone_duels.physics.wing.airframe import CHUPITO
from drone_duels.physics.wing.dynamics import Dynamics
from drone_duels.physics.wing.ground import MapGround
from drone_duels.physics.wing.recorder import FlightRecorder
from drone_duels.ports import SlotPorts

log = logging.getLogger(__name__)

AIRFRAMES: dict[str, Airframe] = {"chupito": CHUPITO}
FLIGHT_LOGS = Path(__file__).resolve().parents[4] / "logs" / "flights"
NEUTRAL = (1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000)
"""Sticks centred, throttle and switches low (disarmed, ANGLE): what INAV gets until the pilot's
radio speaks, so it can get ready to arm."""
CHANNELS = 8
"""INAV reads eight channels from X-Plane's joystick."""
HEARD_S = 1.0
"""INAV is stepping the physics while it was heard from this recently."""


class WingBackend:
    """The wing `airframe` ("chupito") for INAV SITL in slot `ports`: in the hand at `origin`
    (lat, lon, the ground's m above sea level) facing `heading_deg`, in `wind` (content's Wind,
    as a dict). `record`: the directory for the flight's CSV; None: no record."""

    def __init__(
        self,
        airframe: str,
        ports: SlotPorts,
        origin: tuple[float, float, float],
        heading_deg: float,
        wind: dict[str, Any],
        record: Path | None = FLIGHT_LOGS,
    ) -> None:
        try:
            frame = AIRFRAMES[airframe]
        except KeyError:
            raise SystemExit(f"no wing airframe {airframe!r}: {', '.join(AIRFRAMES)}") from None
        self.weather = weather.from_settings(wind, span=frame.wing.reference.span)
        self.aircraft = Aircraft(frame, heading=math.radians(heading_deg), weather=self.weather)
        self.map_ground = MapGround(origin)
        self._sticks: tuple[int, ...] | None = NEUTRAL
        self.server = XPlaneServer(
            self.aircraft,
            origin,
            rc=lambda: self._sticks,
            port=ports.inav_xplane,
            ground=self.map_ground,
        )
        self.recorder: FlightRecorder | None = None
        if record is not None:
            name = time.strftime(f"%Y%m%d-%H%M%S-{airframe}-slot{ports.slot}.csv")
            self.recorder = FlightRecorder(self.server, record / name)

    def start(self) -> None:
        self.server.start()
        if self.recorder is not None:
            self.recorder.start()
            log.info("recording to %s", self.recorder.path)

    def stop(self) -> None:
        if self.recorder is not None:
            self.recorder.stop()
        self.server.stop()

    def connected(self) -> bool:
        heard = self.server.heard
        return heard > 0 and time.monotonic() - heard < HEARD_S

    def phase(self) -> str:
        return self.aircraft.phase.value

    def pose(self) -> tuple[float, ...] | None:
        server = self.server
        if server.time == 0:
            return None
        ac = server.aircraft
        body = ac.body
        lat, lon, alt = server.position()
        left, right = ac.elevons
        return (
            server.time,
            lat,
            lon,
            alt,
            *body.attitude,
            *body.velocity,
            *body.rates,
            left,
            right,
            ac.power.omega,
            0.0,
        )

    def rc(self, us: Sequence[int] | None) -> None:
        if us is None:
            self._sticks = None  # INAV's receiver loses its link: its failsafe runs
            return
        sticks = tuple(int(v) for v in us[:CHANNELS])
        self._sticks = sticks + NEUTRAL[len(sticks) :]

    def ground(self, lat: float, lon: float, alt: float) -> None:
        self.map_ground.report(lat, lon, alt)

    def wind(self, wind: dict[str, Any], gust_now: bool) -> None:
        weather.update(self.weather, wind, gust_now)

    def reset(self) -> None:
        self.server.reset()

    def throw(self) -> None:
        self.server.throw()

    def hit(
        self,
        part: str,
        motor: int | None,
        impulse_ned: tuple[float, float, float],
        at_frd: tuple[float, float, float],
    ) -> None:
        pass  # the wing takes no damage (yet)

    def damage(self) -> dict[str, Any] | None:
        return None

    def dynamics(self) -> dict[str, Any] | None:
        """Dynamics.of the aircraft, as wing.html's overlay reads it, and `motor`: INAV's
        throttle, 0..1 (its throttle row)."""
        data = asdict(Dynamics.of(self.aircraft))
        data["motor"] = max(0.0, self.server.outputs.get(THROTTLE, 0.0))
        return data
