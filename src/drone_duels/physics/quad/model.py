"""The quad as its firmware flies it: the plant (racer5in on SkyFlow-Dynamics' JAX backend) with
everything SkyFlow-Dynamics leaves to the harness around it: the ESCs and the pack (esc.py), the
wind (physics.core.weather, MIL-F-8785C) and the ground.

`step` takes one firmware frame: ArduPilot's motor outputs in, the IMU and the true state out, in
ArduPilot's frames (NED from the origin, body FRD). Positions are metres from the origin, the
start pad, whose ground is `origin`'s altitude above sea level until the map says otherwise
(`set_ground`).

The ground: touching it, a quad is held level, keeping its heading, its rates zero, as
ArduPilot's own SITL holds one (SIM_Aircraft::update_dynamics, GROUND_BEHAVIOR_NO_MOVEMENT), and
slides to a stop under friction. Up and down it sits on its legs, a critically damped spring
(CONTACT_HZ), so a landing stops it over a few tens of milliseconds and its accelerometer feels
that, as a real one does: the EKF, which integrates it, sees the quad stop where the baro and GPS
say it has (a landing reported without its jolt leaves the EKF believing it still sinks). It
leaves the ground when its thrust lifts it off its legs. At rest it settles to a fixed state, so
ArduPilot's EKF and land detector see a still vehicle. Coming down faster than CRASH_SPEED, or on
its side or back, it has crashed: its motors stop, whatever the firmware asks, and it rests there
until a reset.

Hits (`hit`, a bullet as the page traced it; protocol.HitMsg): each part takes away what it does
(`Damage`), the firmware flying on with what is left, and every hit leaves its momentum: the
impulse goes in as SkyFlow-Dynamics' external force over the next frame (F = J / dt, through the
centre of mass) and its moment about the centre of mass (at × F, body) as the external torque.
A reset mends it all.
"""

import logging
import math
import threading
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from drone_duels.bridges.ardupilot_json import Fdm
from drone_duels.physics.core import weather as wx
from drone_duels.physics.core.rigidbody import Quat, rotate, to_euler, unrotate
from drone_duels.physics.quad.esc import Battery, Esc, motor_throttles
from drone_duels.physics.quad.plant import (
    FLIP,
    MOTOR_OF_ROTOR,
    ROTOR_OF_MOTOR,
    ROTORS,
    Airframe,
    N,
    Plant,
    Q,
    V,
    W,
    X,
    ned_quat,
    skyflow_quat,
    yaw_quat,
)
from drone_duels.world.geo import LocalFrame

log = logging.getLogger(__name__)

SUBSTEP_HZ = 1200.0
"""The plant's step rate at least: one step a frame at SITL's 1.2 kHz (sitl.parm)."""
CONTACT_HEIGHT = 0.03
"""EST: how high the centre of mass is when the legs just touch the ground, m."""
CONTACT_HZ = 4.0
"""EST: the legs' natural frequency on the ground, critically damped. They give 15 mm under the
quad's weight, and stop a 2.5 m/s landing within 4 cm at 12 g at most."""
CRASH_SPEED = 8.0
"""Into the ground faster than this, m/s, and it has crashed."""
CRASH_TILT_DEG = 75.0
"""Touching the ground tilted further than this, it is on its side or back: crashed."""
FRICTION = 0.5
"""EST: sliding friction against the ground, as a share of its weight."""
WIND_SPAN = 0.26
"""The turbulence's scale across the airframe, m: racer5in's motor-to-motor diagonal."""

BLADES = 3
"""Blades on each prop: a 5 inch freestyle prop is a polycarbonate tri-blade (5x4.3x3 and the
like). EST: a 5.56 mm bullet through one takes one blade off (the hub and the other blades stay:
a blade is 2 to 3 mm thick and 10 to 15 mm wide, and the bullet carries on through), so a prop
hit takes a third of the rotor's thrust and drag torque at a given speed (ct2, cq2: blade
element theory makes both proportional to the blade count at a given inflow; with inflow they
fall a little less than that, ignored). Its rotor turns as fast as before (the plant's motor lag
does not know its load; a real one, unloaded, speeds up a little), and the imbalance it shakes
the frame with is not modelled (the plant has no vibration: a real one's gyros would see it)."""
BATTERY_HOLD_S = 0.3
"""EST: how long a pierced pack keeps the motors going, s. A bullet through a 6S pack's stack of
pouch cells cuts through the layers of several cells at once (the pack is 35 to 40 mm thick, the
cells stacked through it) and shorts them through the bullet's path and the torn foil, so each
shorted cell's voltage collapses as it discharges into itself: nail-penetration tests of
lithium pouch cells see the voltage fall to near nothing within a fraction of a second to a
second, before the heat builds. With two or more of six cells gone, and the rest sagging under
that short and the motors' current, the ESCs and the flight controller brown out: we take
0.3 s, and then the pack as dead (the motors stop, as for a stack hit). The fire after it is
the page's to draw."""
STACK_PARTS = ("stack",)
"""The flight controller and ESC stack: a hit stops every motor at once. (The flight controller
dies with it, but SITL flies on: it only loses its motors.)"""


@dataclass
class Damage:
    """What hits have taken: by racer5in's rotor (FL, FR, RR, RL), blades left and whether its
    motor is dead; and when the motors lose their power (sim time, s; None: never)."""

    blades: list[int] = field(default_factory=lambda: [BLADES] * N)
    dead: list[bool] = field(default_factory=lambda: [False] * N)
    stack_off_at: float | None = None
    """When the stack was hit."""
    pack_dead_at: float | None = None
    """When a pierced pack gives out."""
    hits: int = 0

    @property
    def power_off_at(self) -> float | None:
        times = [t for t in (self.stack_off_at, self.pack_dead_at) if t is not None]
        return min(times) if times else None

    def pack_dead(self, t: float) -> bool:
        return self.pack_dead_at is not None and t >= self.pack_dead_at

    def share(self) -> np.ndarray:
        """Each rotor's thrust and drag torque left at a given speed, of whole."""
        return np.array([b / BLADES for b in self.blades])

    def powered(self, t: float) -> np.ndarray:
        """Which rotors' ESCs drive them at time `t` (1 driven, 0 not)."""
        if self.power_off_at is not None and t >= self.power_off_at:
            return np.zeros(N)
        return np.array([0.0 if d else 1.0 for d in self.dead])

    def power(self, t: float) -> str:
        if self.power_off_at is None:
            return "on"
        return "failing" if t < self.power_off_at else "off"

    def summary(self, t: float) -> dict[str, Any]:
        """protocol.Damage: by ArduPilot's motor 1-4."""
        share = self.share()
        thrust = [
            0.0 if self.dead[r] else round(float(share[r]), 4)
            for r in (ROTOR_OF_MOTOR[m] for m in range(N))
        ]
        return {"thrust": thrust, "power": self.power(t), "hits": self.hits}


def _vec(a: np.ndarray) -> tuple[float, float, float]:
    return (float(a[0]), float(a[1]), float(a[2]))


def _tilt(q: Quat) -> float:
    """The angle between the body's down and the world's, rad."""
    down = rotate(q, (0.0, 0.0, 1.0))
    return math.acos(max(-1.0, min(1.0, down[2])))


class QuadModel:
    """The quad and its world. Thread-safe: the bridge steps it, the runner reads and sets it."""

    def __init__(
        self,
        frame: Airframe,
        origin: tuple[float, float, float],
        heading_deg: float,
        wind: dict[str, Any] | None = None,
        plant: Plant | None = None,
        seed: int | None = None,
    ) -> None:
        self.frame = frame
        self.origin = origin
        self.heading_deg = heading_deg
        self.local = LocalFrame(origin[0], origin[1])
        self.plant = plant or Plant(frame)
        self.esc = Esc(frame)
        self.weather = wx.from_settings(wind or {}, span=WIND_SPAN, seed=seed)
        self._lock = threading.Lock()
        self.ground_alt = origin[2]
        """The ground under the quad, m above sea level."""
        self.t = 0.0
        self.phase = "ground"
        self.battery = Battery()
        self.s = self._start_state()
        self.last: Fdm | None = None
        self.damage = Damage()
        self._params = self.plant.params
        self._impulse = np.zeros(3)
        """Momentum hits have left since the last frame, NED, N·s; and its moment about the
        centre of mass, body FRD, N·m·s. Both go in over the next frame."""
        self._angular = np.zeros(3)

    # Setters, from any thread.

    def reset(self) -> None:
        """Back to the origin, still, on the ground, a fresh pack, mended."""
        with self._lock:
            self.t = 0.0
            self.ground_alt = self.origin[2]
            self.phase = "ground"
            self.battery = Battery()
            self.s = self._start_state()
            self.last = None
            self.damage = Damage()
            self._params = self.plant.params
            self._impulse = np.zeros(3)
            self._angular = np.zeros(3)

    def hit(
        self,
        part: str,
        motor: int | None,
        impulse_ned: tuple[float, float, float] | list[float],
        at_frd: tuple[float, float, float] | list[float],
    ) -> None:
        """A bullet struck `part` (protocol.HitMsg; `motor` ArduPilot's 1-4 for a prop, motor
        or arm), leaving momentum `impulse_ned` (N·s) at `at_frd` (m from the centre of mass)."""
        with self._lock:
            d = self.damage
            d.hits += 1
            rotor = ROTOR_OF_MOTOR[motor - 1] if motor is not None and 1 <= motor <= N else None
            if part == "prop" and rotor is not None:
                if d.blades[rotor] > 0:
                    d.blades[rotor] -= 1
                    self._params = self.plant.damaged_params(d.share())
            elif part in ("motor", "arm") and rotor is not None:
                d.dead[rotor] = True
            elif part in STACK_PARTS:
                if d.stack_off_at is None:
                    d.stack_off_at = self.t
            elif part == "battery":
                if d.pack_dead_at is None:
                    d.pack_dead_at = self.t + BATTERY_HOLD_S
            # camera, frame: nothing that flies it is lost.
            j = np.asarray(impulse_ned, dtype=float)
            body = np.asarray(unrotate(ned_quat(self.s[Q]), _vec(j)))
            self._impulse += j
            self._angular += np.cross(np.asarray(at_frd, dtype=float), body)
            log.info("hit: %s%s, %.2f N·s", part, f" {motor}" if motor else "",
                     float(np.linalg.norm(j)))

    def damage_summary(self) -> dict[str, Any] | None:
        """protocol.Damage, or None if not hit since the reset."""
        with self._lock:
            return self.damage.summary(self.t) if self.damage.hits else None

    def set_ground(self, alt_amsl: float) -> None:
        """The ground under the quad is at `alt_amsl`. On the ground, the quad moves with it (the
        map's ground is finer than the location's); flying, it meets it where it is."""
        with self._lock:
            if self.phase != "flying":
                self.s[X.start + 2] += alt_amsl - self.ground_alt
            self.ground_alt = alt_amsl

    def set_wind(self, wind: dict[str, Any], gust_now: bool = False) -> None:
        with self._lock:
            wx.update(self.weather, wind, gust_now)

    # State.

    def _ground_down(self) -> float:
        """The ground's NED down from the origin, m."""
        return self.origin[2] - self.ground_alt

    def _start_state(self) -> np.ndarray:
        """At the origin, facing the start's heading, settled on its legs."""
        sag = self.frame.grav / (2 * math.pi * CONTACT_HZ) ** 2
        return Plant.rest_state(
            (0.0, 0.0, self._ground_down() - CONTACT_HEIGHT + sag), self.heading_deg
        )

    def pose(self) -> tuple[float, ...] | None:
        """protocol.POSE_FIELDS: t, lat, lon, alt, q (NED ← FRD), v NED, rates FRD, and the
        rotor speeds of motors 1-4 (ArduPilot's quad X: front right, rear left, front left,
        rear right), rad/s."""
        with self._lock:
            fdm, s = self.last, self.s
            if fdm is None:
                return None
            n, e, d = fdm.position
            lat, lon = self.local.to_latlon(n, e)
            rotors = s[ROTORS]
            return (
                fdm.t,
                lat,
                lon,
                self.origin[2] - d,
                *fdm.quat,
                *fdm.velocity,
                *fdm.gyro,
                *(float(rotors[MOTOR_OF_ROTOR.index(m)]) for m in range(N)),
            )

    # The frame.

    def step(self, dt: float, pwm: tuple[int, ...] | list[int]) -> Fdm:
        """On by `dt` s with ArduPilot's servo outputs `pwm` (µs, SERVO1 first)."""
        with self._lock:
            return self._step(dt, pwm)

    def _step(self, dt: float, pwm: tuple[int, ...] | list[int]) -> Fdm:
        frame, s = self.frame, self.s
        substeps = max(1, round(dt * SUBSTEP_HZ))
        throttles = motor_throttles(pwm) if self.phase != "crashed" else np.zeros(N)
        throttles = throttles * self.damage.powered(self.t)
        cmd = self.esc.command(throttles, self.battery.voltage)

        quat = ned_quat(s[Q])
        position = FLIP * s[X]
        velocity = FLIP * s[V]
        height = self._ground_down() - position[2]
        air = velocity - np.asarray(self.weather.wind)
        self.weather.step(dt, max(0.0, height), float(np.linalg.norm(air)), quat)
        # A hit's momentum, over this one frame.
        force, torque = self._impulse / dt, self._angular / dt
        self._impulse, self._angular = np.zeros(3), np.zeros(3)
        u = Plant.inputs(cmd, self.weather.wind, force, torque)

        nxt, imu = self.plant.advance(s, u, dt / substeps, substeps, self._params)
        self.battery.step(frame, nxt[ROTORS], dt)
        self.t += dt
        if self.damage.pack_dead(self.t):  # the flight controller reads a dead pack
            self.battery.voltage = self.battery.current = 0.0

        gyro = _vec(FLIP * imu[3:])
        accel = _vec(FLIP * imu[:3])
        if self.phase == "flying":
            self.s = nxt
            if self._height(nxt) <= 0:
                down_speed = -float(nxt[V][2])
                tilt = math.degrees(_tilt(ned_quat(nxt[Q])))
                if down_speed > CRASH_SPEED or tilt > CRASH_TILT_DEG:
                    self.phase = "crashed"
                    log.warning("crashed: %.1f m/s into the ground, %.0f° over", down_speed, tilt)
                else:
                    self.phase = "ground"
                    log.info("on the ground at %.1f m/s", down_speed)
        else:
            held, accel = self._contact(s, nxt, imu, dt)
            if self.phase == "ground" and self._height(held) > 0 and held[V][2] > 0:
                self.phase = "flying"  # its thrust has lifted it off its legs
                log.info("off the ground")
                self.s = nxt
                accel = _vec(FLIP * imu[:3])
            else:
                self.s = held
                gyro = (0.0, 0.0, 0.0)
        s = self.s
        self.last = Fdm(
            t=self.t,
            gyro=gyro,
            accel=accel,
            position=_vec(FLIP * s[X]),
            quat=ned_quat(s[Q]),
            velocity=_vec(FLIP * s[V]),
            battery_v=self.battery.voltage,
            battery_a=self.battery.current,
        )
        return self.last

    def _height(self, s: np.ndarray) -> float:
        """How far above where its legs touch the ground it is, m (below: they give)."""
        return float(s[X][2]) - (CONTACT_HEIGHT - self._ground_down())

    def _contact(
        self, before: np.ndarray, nxt: np.ndarray, imu: np.ndarray, dt: float
    ) -> tuple[np.ndarray, tuple[float, float, float]]:
        """A frame on the ground from `before`, the plant having taken it to `nxt` with the IMU
        `imu` there: level, its heading kept, sliding to a stop, up and down on its legs, the
        rotors as the plant turned them. The state after, and the accelerometer's reading."""
        g = self.frame.grav
        # What the thrust, drag and gravity alone do, NED, from the plant.
        force = rotate(ned_quat(nxt[Q]), _vec(FLIP * imu[:3]))
        free_up = -(force[2] + g)
        # The legs: a critically damped spring, which only pushes.
        omega = 2 * math.pi * CONTACT_HZ
        h, v_up = self._height(before), float(before[V][2])
        push = max(0.0, -omega * omega * h - 2 * omega * v_up) if h < 0 else 0.0
        a_up = free_up + push
        v_up += a_up * dt
        h += v_up * dt
        # Sliding friction, horizontally.
        vn, ve = float(before[V][0]), -float(before[V][1])
        speed = math.hypot(vn, ve)
        slowed = max(0.0, speed - FRICTION * g * dt)
        k = slowed / speed if speed > 0 else 0.0
        a_n, a_e = (vn * (k - 1) / dt, ve * (k - 1) / dt)
        vn, ve = vn * k, ve * k
        yaw = to_euler(ned_quat(before[Q]))[2]
        level = yaw_quat(math.degrees(yaw))
        held = before.copy()
        held[X] = np.array(
            [
                float(before[X][0]) + vn * dt,
                float(before[X][1]) - ve * dt,
                CONTACT_HEIGHT - self._ground_down() + h,
            ]
        )
        held[V] = np.array([vn, -ve, v_up])
        held[Q] = skyflow_quat(level)
        held[W] = 0.0
        held[ROTORS] = nxt[ROTORS]
        specific = (a_n, a_e, -a_up - g)
        return held, unrotate(level, specific)
