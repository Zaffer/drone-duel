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
"""

import logging
import math
import threading
from typing import Any

import numpy as np

from drone_duels.bridges.ardupilot_json import Fdm
from drone_duels.physics.core import weather as wx
from drone_duels.physics.core.rigidbody import Quat, rotate, to_euler, unrotate
from drone_duels.physics.quad.esc import Battery, Esc, motor_throttles
from drone_duels.physics.quad.plant import (
    FLIP,
    MOTOR_OF_ROTOR,
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
        wind = wind or {}
        self.weather = wx.from_settings(
            float(wind.get("speed_ms", 0.0)),
            float(wind.get("from_deg", 0.0)),
            str(wind.get("turbulence", "none")),
            bool(wind.get("gusts", False)),
            span=WIND_SPAN,
            seed=seed,
        )
        self._lock = threading.Lock()
        self.ground_alt = origin[2]
        """The ground under the quad, m above sea level."""
        self.t = 0.0
        self.phase = "ground"
        self.battery = Battery()
        self.s = self._start_state()
        self.last: Fdm | None = None

    # Setters, from any thread.

    def reset(self) -> None:
        """Back to the origin, still, on the ground, a fresh pack."""
        with self._lock:
            self.t = 0.0
            self.ground_alt = self.origin[2]
            self.phase = "ground"
            self.battery = Battery()
            self.s = self._start_state()
            self.last = None

    def set_ground(self, alt_amsl: float) -> None:
        """The ground under the quad is at `alt_amsl`. On the ground, the quad moves with it (the
        map's ground is finer than the location's); flying, it meets it where it is."""
        with self._lock:
            if self.phase != "flying":
                self.s[X.start + 2] += alt_amsl - self.ground_alt
            self.ground_alt = alt_amsl

    def set_wind(
        self, speed_ms: float, from_deg: float, turbulence: str, gusts: bool, gust_now: bool
    ) -> None:
        with self._lock:
            wx.update(self.weather, speed_ms, from_deg, turbulence, gusts, gust_now)

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
        cmd = self.esc.command(throttles, self.battery.voltage)

        quat = ned_quat(s[Q])
        position = FLIP * s[X]
        velocity = FLIP * s[V]
        height = self._ground_down() - position[2]
        air = velocity - np.asarray(self.weather.wind)
        self.weather.step(dt, max(0.0, height), float(np.linalg.norm(air)), quat)
        u = Plant.inputs(cmd, self.weather.wind)

        nxt, imu = self.plant.advance(s, u, dt / substeps, substeps)
        self.battery.step(frame, nxt[ROTORS], dt)
        self.t += dt

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
