"""`swarm wingsim`: the TBS Chupito on INAV SITL, its physics from swarm.wingsim.

Starts the flight model (playing X-Plane), then INAV SITL against it, loads the Chupito's INAV
settings, and flies: the drone radio on INAV's sticks (swarm.radio, mode 2 as it reports them),
or with --demo a scripted pilot that arms, throws it and circles. Keys, each then Enter: t
throws it, r puts it back in the hand, q quits.

Each run is recorded to logs/wingsim/ (FlightRecorder): the wing's state, the sticks, INAV's
outputs and its estimate of the attitude, the wind, 50 times a second, to look into how it flew.

The 3D viewer's wing.html draws it on the map (WingView, swarm.view's /wing), and tells it where
the map's ground is under it (MapGround): the 3D tiles' roofs and trees too. Its D key shows the
flight model at work (swarm.wingsim.dynamics, in every frame): the forces on the wing, and every
number beside it.

INAV's modes (swarm.wingsim.inav): CH5 high arms; CH6 low ANGLE, middle ACRO, high MANUAL; CH7
high RTH. INAV's launch mode is on: arm, raise the throttle, throw, and INAV runs the motor.
"""

import csv
import logging
import math
import select
import sys
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

from swarm.wingsim import inav
from swarm.wingsim.aircraft import Aircraft, Phase
from swarm.wingsim.airframe import CHUPITO
from swarm.wingsim.dynamics import Dynamics
from swarm.wingsim.server import DEFAULT_PORT, YOKES, Snapshot, XPlaneServer
from swarm.wingsim.weather import Weather

if TYPE_CHECKING:
    from swarm.view import ViewServer

log = logging.getLogger(__name__)

ROOT = Path(__file__).resolve().parents[2]
INAV_BINARY = ROOT / "external" / "inav" / "inav_SITL"
INAV_WORKDIR = ROOT / "external" / "inav" / "chupito"
"""INAV's eeprom (its settings) and log."""
FLIGHT_LOGS = ROOT / "logs" / "wingsim"
RECORD_HZ = 50
ATTITUDE_EVERY = 5
"""INAV's attitude estimate every this many records: MSP is slower than the flight model."""
NEUTRAL = [1500.0, 1500.0, 1000.0, 1500.0, 1000.0, 1000.0, 1000.0, 1000.0]
"""Sticks centred, throttle and switches low: disarmed, ANGLE."""
THROW_SWITCH = 7
"""CH8 (index 7): flicked high, it throws the wing (it is not INAV's)."""
FRAME_HZ = 60
"""How often the wing goes to the 3D viewer."""


class MapGround:
    """The ground's height above sea level under the wing, as the 3D viewer's map has it: the
    page reports it where the wing is about to be, a little ahead, so a report is late by the
    time the wing gets there. The ground at a place is the latest report within NEAR_M of it, or
    else the latest of all; at the start, the start's. Flat at the origin's height until a page
    reports."""

    NEAR_M = 4.0
    KEPT = 64

    def __init__(self, origin: tuple[float, float, float]) -> None:
        self.origin = origin
        self.reports: deque[tuple[float, float, float]] = deque(maxlen=self.KEPT)
        self.home: float | None = None

    def report(self, lat: float, lon: float, alt: float) -> None:
        self.reports.append((lat, lon, alt))
        if self._metres(lat, lon, *self.origin[:2]) < self.NEAR_M:
            self.home = alt

    def _metres(self, lat0: float, lon0: float, lat1: float, lon1: float) -> float:
        north = math.radians(lat1 - lat0) * 6378137.0
        east = math.radians(lon1 - lon0) * 6378137.0 * math.cos(math.radians(lat0))
        return math.hypot(north, east)

    def __call__(self, lat: float, lon: float) -> float:
        for r_lat, r_lon, alt in reversed(self.reports):
            if self._metres(lat, lon, r_lat, r_lon) < self.NEAR_M:
                return alt
        if self.home is not None and self._metres(lat, lon, *self.origin[:2]) < self.NEAR_M:
            return self.home
        return self.reports[-1][2] if self.reports else self.origin[2]


class FlightRecorder:
    """The flight, RECORD_HZ times a second, to a CSV file in `directory`: the wing's true state
    (degrees, m, m/s, g), the elevons (trailing edge down +), INAV's outputs and its estimate of
    the attitude, the eight RC channels (µs), and whether INAV is armed."""

    FIELDS = (
        "t phase lat lon alt agl roll pitch heading p q r vn ve vd airspeed alpha beta "
        "nx ny nz motor servo1 servo2 elevon_l elevon_r rpm pack_v pack_a "
        "rc1 rc2 rc3 rc4 rc5 rc6 rc7 rc8 armed inav_roll inav_pitch wind_n wind_e wind_d"
    ).split()

    def __init__(self, server: XPlaneServer, view: "WingView", directory: Path) -> None:
        self.server = server
        self.view = view
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / time.strftime("%Y%m%d-%H%M%S.csv")
        self.msp: inav.Msp | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="flight-recorder", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)

    def _run(self) -> None:
        estimate = (math.nan, math.nan)
        t0 = time.monotonic()
        with self.path.open("w", newline="") as f:
            out = csv.writer(f)
            out.writerow(self.FIELDS)
            n = 0
            while not self._stop.wait(1 / RECORD_HZ):
                if self.msp is not None and n % ATTITUDE_EVERY == 0:
                    try:
                        estimate = self.msp.attitude()[:2]
                    except OSError:
                        estimate = (math.nan, math.nan)
                out.writerow(self._row(time.monotonic() - t0, estimate))
                n += 1
                if n % RECORD_HZ == 0:
                    f.flush()

    def _row(self, t: float, estimate: tuple[float, float]) -> list[object]:
        server = self.server
        ac = server.aircraft
        s = server.snapshot()
        status = self.view.status
        g = ac.gravity
        rc = [*server.sticks, *[math.nan] * 8][:8]
        values = [
            t,
            s.phase,
            s.lat,
            s.lon,
            s.alt,
            s.agl,
            s.roll,
            s.pitch,
            s.heading,
            *(math.degrees(w) for w in ac.body.rates),
            *s.velocity,
            s.airspeed,
            math.degrees(ac.air.alpha),
            math.degrees(ac.air.beta),
            *(-f / g for f in ac.specific_force),
            s.motor,
            server.outputs.get(YOKES[0], 0.0),
            server.outputs.get(YOKES[1], 0.0),
            *s.elevons,
            ac.power.omega * 60 / (2 * math.pi),
            s.pack_volts,
            s.pack_amps,
            *rc,
            int(bool(status and status.armed)),
            *estimate,
            *ac.wind,
        ]
        return [round(v, 4) if isinstance(v, float) else v for v in values]


@dataclass(frozen=True)
class WingFrame:
    """What the 3D viewer draws: the wing, whether INAV is armed or what stops it, the RC
    channels INAV gets, µs, and the flight model at work."""

    wing: Snapshot
    armed: bool
    blockers: tuple[str, ...]
    rc: tuple[float, ...]
    dynamics: Dynamics


class WingView:
    """The wing for the 3D viewer (swarm.view's WingSource)."""

    def __init__(self, server: XPlaneServer, ground: MapGround) -> None:
        self.server = server
        self.map_ground = ground
        self.status: inav.Status | None = None
        """INAV's, as last read; None until it answers."""
        self._tick = -1
        self._frame: WingFrame | None = None

    @property
    def origin(self) -> tuple[float, float, float]:
        return self.server.origin

    def latest(self) -> dict[int, tuple[int, Any]]:
        tick = int(time.monotonic() * FRAME_HZ)
        frame = self._frame
        if tick != self._tick or frame is None:
            status = self.status
            armed, blockers = (status.armed, status.blockers) if status else (False, ("starting",))
            server = self.server
            dynamics = Dynamics.of(server.aircraft)
            frame = WingFrame(server.snapshot(), armed, blockers, server.sticks, dynamics)
            self._frame, self._tick = frame, tick
        return {0: (tick, frame)}

    def throw(self) -> None:
        self.server.throw()

    def reset(self) -> None:
        self.server.reset()

    def ground(self, lat: float, lon: float, alt: float) -> None:
        self.map_ground.report(lat, lon, alt)


class RadioSticks:
    """The drone radio (swarm.radio's channels, -1..1) as INAV's eight channels, µs."""

    def __init__(self, throw: Callable[[], None]) -> None:
        from swarm.radio import RadioReader

        self.reader = RadioReader()
        self.reader.start()
        self.throw = throw
        self._switch_was_high = True  # no throw until it has been low

    def __call__(self) -> list[float]:
        channels = self.reader.channels()
        if channels is None:
            return list(NEUTRAL)
        us = [1500.0 + 500.0 * v for v in channels[:8]]
        us += NEUTRAL[len(us) :]
        high = us[THROW_SWITCH] > 1700
        if high and not self._switch_was_high:
            self.throw()
        self._switch_was_high = high
        return us

    def stop(self) -> None:
        self.reader.stop()


class DemoPilot:
    """Arms once INAV allows it, raises the throttle, throws, and after the launch circles
    gently right in ANGLE mode. Back in the hand after a crash or landing, it goes again."""

    def __init__(self, server: XPlaneServer, ready: Callable[[], bool]) -> None:
        self.server = server
        self.ready = ready
        self.sticks = list(NEUTRAL)
        self._since = time.monotonic()
        self._stage = "wait"
        self._ready_polls = 0
        """Polls in a row INAV could arm: just started, it shows nothing in the way for a
        moment, before its calibration."""

    def __call__(self) -> list[float]:
        return self.sticks

    def update(self) -> None:
        """Called about once a second, from the main loop."""
        now = time.monotonic()
        elapsed = now - self._since
        phase = self.server.aircraft.phase
        stage = self._stage
        if stage == "wait":
            self._ready_polls = self._ready_polls + 1 if self.ready() else 0
        if stage == "wait" and self._ready_polls >= 2:
            self.sticks[4] = 2000.0  # arm
            self._next("armed")
        elif stage == "armed" and elapsed > 1:
            self.sticks[2] = 1800.0  # raise the throttle: INAV's launch waits for the throw
            self._next("throttle")
        elif stage == "throttle" and elapsed > 1:
            self.server.throw()
            self._next("launch")
        elif stage == "launch" and elapsed > 7:
            self.sticks[2] = 1600.0
            self.sticks[0] = 1620.0  # a gentle right turn
            self._next("circle")
        if phase in (Phase.CRASHED, Phase.LANDED) and stage in ("launch", "circle"):
            self.sticks = list(NEUTRAL)  # disarm, back to the hand, again
            self.server.reset()
            self._next("wait")

    def _next(self, stage: str) -> None:
        self._stage = stage
        self._since = time.monotonic()


def run(
    origin: tuple[float, float, float],
    heading_deg: float,
    demo: bool,
    view: "ViewServer | None" = None,
    port: int = DEFAULT_PORT,
    weather: Weather | None = None,
) -> None:
    """Fly it until q or Ctrl-C; on `view`'s wing.html too, which this starts and stops. In
    `weather` (default: still air)."""
    if not INAV_BINARY.exists():
        raise SystemExit(f"no INAV SITL at {INAV_BINARY}: run scripts/setup.sh")
    aircraft = Aircraft(CHUPITO, heading=math.radians(heading_deg), weather=weather)
    if weather is not None and (weather.speed or weather.gusts):
        print(
            f"wind {weather.speed:g} m/s from {weather.from_deg:03.0f}, gusts {weather.gusts:g}",
            flush=True,
        )
    sticks: list[Callable[[], list[float]]] = [lambda: list(NEUTRAL)]
    ground = MapGround(origin)
    server = XPlaneServer(aircraft, origin, rc=lambda: sticks[0](), port=port, ground=ground)
    wing_view = WingView(server, ground)
    if view:
        view.wing = wing_view
        try:
            view.start()
        except OSError as exc:
            raise SystemExit(
                f"3D viewer: {exc}; is `swarm sim` or `swarm view` running? Stop it first"
            ) from None
    server.start()
    recorder = FlightRecorder(server, wing_view, FLIGHT_LOGS)
    recorder.start()
    print(f"recording to {recorder.path}", flush=True)
    proc = inav.start(INAV_BINARY, INAV_WORKDIR, port)
    msp: inav.Msp | None = None
    radio: RadioSticks | None = None
    try:
        _wait(lambda: server.packets > 0, 20, "INAV did not connect")
        if inav.configure(INAV_WORKDIR, inav.CHUPITO_SETTINGS):
            time.sleep(2)  # it restarts with the settings
        msp = _msp()
        recorder.msp = msp

        def ready() -> bool:
            assert msp is not None
            status = msp.status()
            return not status.armed and set(status.blockers) <= {"arm switch"}

        demo_pilot = DemoPilot(server, ready) if demo else None
        if demo_pilot:
            sticks[0] = demo_pilot
        else:
            radio = RadioSticks(server.throw)
            sticks[0] = radio
        if view:
            print(f"watch it at {view.url}wing.html", flush=True)
        print("t: throw  r: back in the hand  q: quit (then Enter)", flush=True)
        while True:
            command = _key()
            if command == "q":
                break
            if command == "t":
                server.throw()
            elif command == "r":
                server.reset()
            if demo_pilot:
                demo_pilot.update()
            wing_view.status = msp.status()
            print(_status(server, wing_view.status), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.stop()
        if radio:
            radio.stop()
        if msp:
            msp.close()
        proc.terminate()
        proc.wait(5)
        server.stop()
        if view:
            view.stop()


def _wait(done: Callable[[], bool], seconds: float, failure: str) -> None:
    deadline = time.monotonic() + seconds
    while not done():
        if time.monotonic() > deadline:
            raise SystemExit(f"{failure}: see {INAV_WORKDIR / 'inav.log'}")
        time.sleep(0.1)


def _msp() -> inav.Msp:
    deadline = time.monotonic() + 15
    while True:
        try:
            return inav.Msp()
        except OSError:
            if time.monotonic() > deadline:
                raise
            time.sleep(0.5)


def _key() -> str | None:
    """A line typed at the terminal, waiting up to a second for it."""
    if select.select([sys.stdin], [], [], 1.0)[0]:
        line = sys.stdin.readline()
        return line.strip().lower()[:1] if line else "q"
    return None


def _status(server: XPlaneServer, status: inav.Status) -> str:
    s = server.snapshot()
    blockers = f" ({', '.join(status.blockers)})" if status.blockers else ""
    armed = "ARMED" if status.armed else "disarmed" + blockers
    return (
        f"{s.phase:7} {armed:10} agl {s.agl:6.1f} m  {s.airspeed:5.1f} m/s  "
        f"roll {s.roll:6.1f} pitch {s.pitch:5.1f} hdg {s.heading:5.1f}  "
        f"motor {s.motor:4.0%} {s.pack_volts:4.1f} V {s.pack_amps:4.1f} A"
    )
