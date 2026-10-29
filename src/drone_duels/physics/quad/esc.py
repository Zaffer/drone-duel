"""The ESCs and the pack: ArduPilot's motor outputs to commanded rotor speeds, and the 6S LiPo
behind them. Harness-side, as SkyFlow-Dynamics leaves them (its README: command path and battery
belong to the consuming simulator).

The ESC map is SkyFlow-Dynamics' verified throttle curve (spec.motor.throttle_to_speed, through
the backend's throttle_to_speed_fn): with u the motor output's share of its PWM range,

    Ω_c = (Ω_top − Ω_min) √(k u² + (1 − k) u) + Ω_min,

k = 0.5 as SkyDreamer identified it for a 5 inch racer, Ω_min racer5in's floor and Ω_top its
ceiling scaled by the pack's voltage under load over a full pack's (a motor's top speed is its KV
times the volts it gets; spec.motor.voltage_to_rpm's form). At u = 0 (disarmed, or the ESC's
stop) the rotor is unpowered and Ω_c is 0. Thrust goes as Ω², so with Ω_min ≪ Ω_top thrust is
very nearly (1 − k) u + k u² of full: ArduPilot's own thrust model with MOT_THST_EXPO = k. The
floor bends it a little; `fit_expo` fits MOT_THST_EXPO to the curve itself, and
tests/test_physics_quad.py holds copter-quad5.parm to it.

The pack: six cells on a typical LiPo discharge curve (resting volts against charge left), sagging
through its internal resistance under the current the motors draw. That current comes from the
rotors' shaft power, Σ Q Ω = Σ c_Q Ω³, over the motors' and ESCs' efficiency, plus the
electronics'; the loaded voltage then solves V = OCV − R P / V.
"""

import math
from dataclasses import dataclass

import numpy as np

from drone_duels.physics.quad.plant import MOTOR_OF_ROTOR, Airframe, N

PWM_MIN = 1000
PWM_MAX = 2000
"""ArduPilot's MOT_PWM_MIN and MAX in SITL (the ESC's calibrated range)."""
CURVE_K = 0.5
"""The throttle curve's k: SkyDreamer's (arXiv:2510.14783) identified value for a 5 inch racer."""

CELLS = 6
CELL_FULL = 4.2
CELL_EMPTY = 3.3
"""ArduPilot's MOT_BAT_VOLT_MAX and MIN per cell (copter-quad5.parm)."""
LIPO_CELL = (
    (0.00, 3.27),
    (0.05, 3.61),
    (0.10, 3.69),
    (0.20, 3.73),
    (0.30, 3.77),
    (0.40, 3.80),
    (0.50, 3.84),
    (0.60, 3.87),
    (0.70, 3.95),
    (0.80, 4.02),
    (0.90, 4.11),
    (1.00, 4.20),
)
"""EST: a LiPo cell's resting voltage against the charge left (the common hobby chart's, as the
wing's pack, physics/wing/airframe.py)."""
CAPACITY_MAH = 1300.0
"""EST: a 6S 1300 mAh pack, the usual 5 inch freestyle pack (BATT_CAPACITY)."""
PACK_RESISTANCE = 0.035
"""EST: ohms, six cells of about 4 mΩ and the leads and connector."""
EFFICIENCY = 0.75
"""EST: shaft power over electrical, motors and ESCs together."""
AVIONICS_W = 5.0
"""EST: flight controller, receiver, camera and VTX."""


def cell_voltage(charge: float) -> float:
    """A cell's resting voltage with `charge` (0 to 1) left."""
    charge = max(0.0, min(1.0, charge))
    for (c0, v0), (c1, v1) in zip(LIPO_CELL, LIPO_CELL[1:], strict=False):
        if charge <= c1:
            return v0 + (v1 - v0) * (charge - c0) / (c1 - c0)
    return LIPO_CELL[-1][1]


def throttle_curve(u: np.ndarray, w_min: float, w_top: float, k: float = CURVE_K) -> np.ndarray:
    """spec.motor.throttle_to_speed in NumPy, for the fits (the ESC itself runs the backend's)."""
    return (w_top - w_min) * np.sqrt(k * u * u + (1 - k) * u) + w_min


def motor_throttles(pwm: tuple[int, ...] | list[int]) -> np.ndarray:
    """SERVO1-4's pulse widths as each of racer5in's rotors' throttle, 0 to 1."""
    out = np.zeros(N)
    for rotor, motor in enumerate(MOTOR_OF_ROTOR):
        width = pwm[motor] if motor < len(pwm) else 0
        out[rotor] = min(1.0, max(0.0, (width - PWM_MIN) / (PWM_MAX - PWM_MIN)))
    return out


class Esc:
    """The four ESCs: `command` turns the motor outputs into commanded rotor speeds."""

    def __init__(self, frame: Airframe) -> None:
        import jax
        from skyflow_dynamics.backends import jax as backend

        self.frame = frame
        self._curve = jax.jit(backend.throttle_to_speed_fn())

    def top_speed(self, pack_v: float) -> float:
        """The fastest the motors turn on `pack_v` volts, rad/s."""
        return self.frame.rotor_max * min(1.0, max(0.0, pack_v / (CELLS * CELL_FULL)))

    def command(self, throttles: np.ndarray, pack_v: float) -> np.ndarray:
        """Each rotor's commanded speed (racer5in's order) at `throttles` (motor_throttles)."""
        top = self.top_speed(pack_v)
        w = np.asarray(self._curve(throttles, self.frame.rotor_min, top, CURVE_K))
        return np.where(throttles > 0, w, 0.0)


@dataclass
class Battery:
    """The pack, drained and sagging by the rotors' power."""

    capacity_mah: float = CAPACITY_MAH
    used_mah: float = 0.0
    voltage: float = CELLS * CELL_FULL
    current: float = 0.0

    @property
    def resting(self) -> float:
        return CELLS * cell_voltage(1 - self.used_mah / self.capacity_mah)

    def step(self, frame: Airframe, rotor_speeds: np.ndarray, dt: float) -> None:
        """`dt` s at `rotor_speeds` (rad/s): the pack's loaded voltage and current, and the
        charge drawn."""
        shaft = float(np.sum(frame.cq2 * np.abs(rotor_speeds) ** 3))
        power = shaft / EFFICIENCY + AVIONICS_W
        ocv = self.resting
        disc = ocv * ocv - 4 * PACK_RESISTANCE * power
        self.voltage = (ocv + math.sqrt(disc)) / 2 if disc > 0 else ocv / 2
        self.current = power / self.voltage
        self.used_mah = min(self.capacity_mah, self.used_mah + self.current * dt / 3.6)


def thrust_share(frame: Airframe, u: np.ndarray, pack_v: float = CELLS * CELL_FULL) -> np.ndarray:
    """Thrust at motor output `u` (0..1) over thrust at full output, on `pack_v` volts."""
    top = frame.rotor_max * min(1.0, pack_v / (CELLS * CELL_FULL))
    w = throttle_curve(u, frame.rotor_min, top)
    return w * w / (top * top)


def fit_expo(frame: Airframe, spin_min: float, spin_max: float) -> float:
    """MOT_THST_EXPO for this ESC map: ArduPilot takes thrust between MOT_SPIN_MIN and
    MOT_SPIN_MAX as (1 − e) x + e x² of x, the output's share of that range. Least squares."""
    x = np.linspace(0.0, 1.0, 201)
    u = spin_min + (spin_max - spin_min) * x
    t = thrust_share(frame, u)
    y = (t - t[0]) / (t[-1] - t[0])
    # y - x = e (x² - x)
    a = x * x - x
    return float(np.dot(a, y - x) / np.dot(a, a))


def hover_output(frame: Airframe, pack_v: float) -> float:
    """The motor output (0..1 of the PWM range) at which the quad hovers on `pack_v` volts."""
    top = frame.rotor_max * min(1.0, pack_v / (CELLS * CELL_FULL))
    s = (frame.hover_speed - frame.rotor_min) / (top - frame.rotor_min)
    # k u² + (1 - k) u = s²
    k = CURVE_K
    return (-(1 - k) + math.sqrt((1 - k) ** 2 + 4 * k * s * s)) / (2 * k)


def hover_thrust(frame: Airframe, spin_min: float, spin_max: float, expo: float,
                 loaded_v: float, resting_v: float) -> float:
    """MOT_THST_HOVER: ArduPilot's throttle (in its thrust model with `expo`) that hovers, the
    pack at `resting_v` sagging to `loaded_v`. ArduPilot asks output x on voltage ratio v for
    thrust f(x v) / f(v), f(r) = (1 − e) r + e r², v the resting voltage over MOT_BAT_VOLT_MAX
    (AP_Motors_Thrust_Linearization.cpp, apply_thrust_curve_and_volt_scaling)."""
    x = (hover_output(frame, loaded_v) - spin_min) / (spin_max - spin_min)
    v = min(1.0, resting_v / (CELLS * CELL_FULL))

    def f(r: float) -> float:
        return (1 - expo) * r + expo * r * r

    return f(x * v) / f(v)
