"""The real-time loop between INAV SITL and the aircraft, over X-Plane's protocol.

Each tick it reads INAV's outputs (the motor, and the elevons' servos as X-Plane yoke ratios),
steps the aircraft at the physics rate, and answers with every dataref INAV subscribed to: GPS,
attitude, rates, g-loads, airspeed, the barometer and the RC channels, which INAV reads as a
joystick (receiver type SIM). Answers paced at `rate` set INAV's loop rate: one loop each.

The RC comes from a callable returning eight channels in microseconds (1000-2000), as a receiver
gives them: AETR, then four switches; or None when the radio link is lost. Then there is no
joystick (has_joystick 0): INAV's SIM receiver gets no new frames, and INAV's own RX failsafe
runs, as on a real receiver that lost its link.
"""

import logging
import math
import select
import socket
import threading
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass

from drone_duels.bridges import xplane_packets as xplane
from drone_duels.physics.wing.aircraft import Aircraft, Controls

log = logging.getLogger(__name__)

DEFAULT_PORT = 49000
EARTH_RADIUS = 6378137.0
INHG = 3386.39
"""Pascals per inch of mercury."""

THROTTLE = "sim/cockpit2/engine/actuators/throttle_ratio_all"
YOKES = (
    "sim/joystick/yoke_roll_ratio",
    "sim/joystick/yoke_pitch_ratio",
    "sim/joystick/yoke_heading_ratio",
)
"""INAV's servo outputs, in its --chanmap order: the first servo mapped, the second, the third."""


@dataclass(frozen=True)
class ElevonMap:
    """Which of INAV's servo outputs (0, 1, 2: --chanmap order) drives each elevon, and which way:
    `sign` +1 when more microseconds lower the trailing edge.

    TBS's mixer gives both servos the roll with the same sign and the pitch with opposite ones:
    the servos are mirrored. Rolling right lowers servo 1's microseconds and the left elevon's
    trailing edge, so servo 1 is the left elevon, its trailing edge up with more microseconds;
    servo 2 the right, down with more. (Seen in SITL: nose up in the hand, ANGLE mode lowers
    both trailing edges, servo 1 down and servo 2 up.)"""

    left: int = 0
    left_sign: float = -1.0
    right: int = 1
    right_sign: float = 1.0


def isa_pressure(altitude: float) -> float:
    """Standard atmosphere, Pa, at `altitude` m above sea level."""
    return 101325.0 * (1 - 2.25577e-5 * altitude) ** 5.25588


def centred(us: float) -> float:
    return max(-1.0, min(1.0, (us - 1500) / 500))


def upward(us: float) -> float:
    return max(0.0, min(1.0, (us - 1000) / 1000))


class XPlaneServer:
    """Plays X-Plane to INAV SITL for `aircraft`, which starts at `origin` (lat, lon, m above sea
    level). `ground(lat, lon)`: the ground's height above sea level there (default: flat, at the
    origin's)."""

    def __init__(
        self,
        aircraft: Aircraft,
        origin: tuple[float, float, float],
        rc: Callable[[], Sequence[float] | None],
        port: int = DEFAULT_PORT,
        rate: float = 250.0,
        physics_rate: float = 1000.0,
        elevons: ElevonMap | None = None,
        ground: Callable[[float, float], float] | None = None,
    ) -> None:
        self.aircraft = aircraft
        self.origin = origin
        self.rc = rc
        self.port = port
        self.rate = rate
        self.substeps = max(1, round(physics_rate / rate))
        self.elevons = elevons or ElevonMap()
        self.ground = ground or (lambda _lat, _lon: origin[2])
        self.subscriptions: dict[int, str] = {}
        self.outputs: dict[str, float] = {}
        self.sticks: tuple[float, ...] = ()
        """The eight RC channels as last sent to INAV, µs; () while the link is lost."""
        self.peer: tuple[str, int] | None = None
        self.packets = 0
        self.time = 0.0
        """The flight model's time, s: one period on each tick."""
        self.heard = 0.0
        """When INAV's last packet came, time.monotonic(); 0 never."""
        self._commands: list[Callable[[], None]] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None

    # Called from other threads.

    def throw(self) -> None:
        self._queue(self.aircraft.throw)

    def reset(self) -> None:
        """Back in the hand at the start."""

        def home() -> None:
            self.aircraft.body.position = (0.0, 0.0, self.aircraft.body.position[2])
            self.aircraft.reset(ground_down=self._ground_down(), at=(0.0, 0.0))

        self._queue(home)

    def _queue(self, command: Callable[[], None]) -> None:
        with self._lock:
            self._commands.append(command)

    def start(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", self.port))
        self._sock = sock
        self._thread = threading.Thread(target=self._run, name="wingsim", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        if self._sock:
            self._sock.close()

    # The loop.

    def _run(self) -> None:
        assert self._sock is not None
        period = 1 / self.rate
        dt = period / self.substeps
        next_tick = time.monotonic()
        while not self._stop.is_set():
            self._receive()
            self._run_commands()
            controls = self._controls()
            ground_down = self._ground_down()
            for i in range(self.substeps):
                last = i == self.substeps - 1
                self.aircraft.step(controls, dt, ground_down, detail=last)
            self.time += period
            if self.peer and self.subscriptions:
                readings = self.readings()
                values = {i: readings.get(name, 0.0) for i, name in self.subscriptions.items()}
                self._sock.sendto(xplane.values_packet(values), self.peer)
                self.packets += 1
            next_tick += period
            late = time.monotonic() - next_tick
            if late > 0.1:
                next_tick = time.monotonic()  # fell behind: don't run to catch up
            elif late < 0:
                time.sleep(-late)

    def _run_commands(self) -> None:
        with self._lock:
            commands, self._commands = self._commands, []
        for command in commands:
            command()

    def _receive(self) -> None:
        assert self._sock is not None
        while select.select([self._sock], [], [], 0)[0]:
            try:
                packet, peer = self._sock.recvfrom(2048)
            except OSError:
                return
            message = xplane.parse(packet)
            if message is None:
                continue
            self.heard = time.monotonic()
            if peer != self.peer:
                log.info("INAV at %s:%d", *peer)
                self.peer = peer
            if isinstance(message, xplane.Subscribe):
                if message.frequency > 0:
                    self.subscriptions[message.index] = message.name
                else:
                    self.subscriptions.pop(message.index, None)
            else:
                self.outputs[message.name] = message.value

    def _controls(self) -> Controls:
        servos = [self.outputs.get(name, 0.0) for name in YOKES]
        m = self.elevons
        return Controls(
            throttle=self.outputs.get(THROTTLE, 0.0),
            left=m.left_sign * servos[m.left],
            right=m.right_sign * servos[m.right],
        )

    # Where the aircraft is, as INAV reads it.

    def position(self) -> tuple[float, float, float]:
        """Latitude, longitude (degrees) and height above sea level (m)."""
        north, east, down = self.aircraft.body.position
        lat0, lon0, alt0 = self.origin
        lat = lat0 + math.degrees(north / EARTH_RADIUS)
        lon = lon0 + math.degrees(east / (EARTH_RADIUS * math.cos(math.radians(lat0))))
        return lat, lon, alt0 - down

    def _ground_down(self) -> float:
        lat, lon, _ = self.position()
        return self.origin[2] - self.ground(lat, lon)

    def readings(self) -> dict[str, float]:
        """Every dataref INAV may ask for, by name, as X-Plane would give it."""
        ac = self.aircraft
        b = ac.body
        g = ac.gravity
        lat, lon, alt = self.position()
        roll, pitch, yaw = ac.euler
        vn, ve, vd = b.velocity
        speed = math.hypot(vn, ve)
        heading = math.degrees(yaw) % 360
        sticks = self.rc()
        ch = [*(sticks or ())][:8]
        ch += [1500.0] * (8 - len(ch))
        self.sticks = tuple(ch) if sticks is not None else ()
        return {
            "inav_xitl/plugin/xitlDrefVersion": 0.0,  # plain X-Plane, not INAV's own plugin
            "sim/flightmodel/position/latitude": lat,
            "sim/flightmodel/position/longitude": lon,
            "sim/flightmodel/position/elevation": alt,
            "sim/flightmodel/position/y_agl": alt - self.ground(lat, lon),
            # X-Plane's local frame: x east, y up, z south.
            "sim/flightmodel/position/local_vx": ve,
            "sim/flightmodel/position/local_vy": -vd,
            "sim/flightmodel/position/local_vz": -vn,
            "sim/flightmodel/position/groundspeed": speed,
            "sim/flightmodel/position/true_airspeed": ac.air.airspeed,
            "sim/flightmodel/position/phi": math.degrees(roll),
            "sim/flightmodel/position/theta": math.degrees(pitch),
            "sim/flightmodel/position/psi": heading,
            "sim/flightmodel/position/hpath": (
                math.degrees(math.atan2(ve, vn)) % 360 if speed > 0.5 else heading
            ),
            # INAV reads these as its accelerometer: forward -g_axil, left g_side, up g_nrml.
            "sim/flightmodel/forces/g_axil": -ac.specific_force[0] / g,
            "sim/flightmodel/forces/g_side": -ac.specific_force[1] / g,
            "sim/flightmodel/forces/g_nrml": -ac.specific_force[2] / g,
            "sim/flightmodel/position/P": math.degrees(b.rates[0]),
            "sim/flightmodel/position/Q": math.degrees(b.rates[1]),
            "sim/flightmodel/position/R": math.degrees(b.rates[2]),
            "sim/weather/barometer_current_inhg": isa_pressure(alt) / INHG,
            # The RC, as joystick axes: INAV (target/SITL/sim/xplane.c) maps them to channels.
            "sim/joystick/has_joystick": 1.0 if sticks is not None else 0.0,
            "sim/joystick/joy_mapped_axis_value[2]": centred(ch[0]),  # roll, CH1
            "sim/joystick/joy_mapped_axis_value[1]": centred(ch[1]),  # pitch, CH2
            "sim/joystick/joy_mapped_axis_value[57]": upward(ch[2]),  # throttle, CH3
            "sim/joystick/joy_mapped_axis_value[3]": centred(ch[3]),  # yaw, CH4
            "sim/joystick/joy_mapped_axis_value[58]": upward(ch[4]),
            "sim/joystick/joy_mapped_axis_value[59]": upward(ch[5]),
            "sim/joystick/joy_mapped_axis_value[60]": upward(ch[6]),
            "sim/joystick/joy_mapped_axis_value[61]": upward(ch[7]),
        }

    def snapshot(self) -> "Snapshot":
        ac = self.aircraft
        lat, lon, alt = self.position()
        roll, pitch, yaw = (math.degrees(a) for a in ac.euler)
        return Snapshot(
            phase=ac.phase.value,
            lat=lat,
            lon=lon,
            alt=alt,
            agl=alt - self.ground(lat, lon),
            roll=roll,
            pitch=pitch,
            heading=yaw % 360,
            velocity=ac.body.velocity,
            airspeed=ac.air.airspeed,
            motor=max(0.0, self.outputs.get(THROTTLE, 0.0)),  # -1 from INAV as it starts
            elevons=(math.degrees(ac.elevons[0]), math.degrees(ac.elevons[1])),
            pack_volts=ac.power.pack_voltage,
            pack_amps=ac.power.pack_current,
        )


@dataclass(frozen=True)
class Snapshot:
    """Where the wing is and what it is doing: degrees, metres (above sea level, or the ground:
    agl), m/s (velocity north, east, down), the motor 0..1, the elevons' trailing edges down
    positive."""

    phase: str
    lat: float
    lon: float
    alt: float
    agl: float
    roll: float
    pitch: float
    heading: float
    velocity: tuple[float, float, float]
    airspeed: float
    motor: float
    elevons: tuple[float, float]
    pack_volts: float
    pack_amps: float
