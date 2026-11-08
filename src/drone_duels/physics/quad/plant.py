"""The quad's plant: SkyFlow-Dynamics' multirotor model on its own JAX backend, as published
(DESIGN.md §6.2), on the CPU.

SkyFlow-Dynamics' frames (skyflow_dynamics/spec/frames.py): the world W is right-handed with z
up, the body B is forward, left, up (FLU); the state is (x_W, v_W, q_WB wxyz, ω_B, Ω). We take W
as north, west, up (NWU), so W and NED differ by a half turn about north, and B and FRD by a half
turn about forward: each is diag(1, -1, -1) of the other, and q (NED ← FRD) is q (W ← B) with y
and z negated (`ned_quat`).

racer5in (`RACER_5IN`) orders its rotors front-left, front-right, rear-right, rear-left
(rotor_pos), spinning CW, CCW, CW, CCW seen from above (spin -1, +1, -1, +1: SkyFlow's spin is
the physical sign about +z_B). ArduPilot's quad X (AP_MotorsMatrix.cpp, MOTOR_FRAME_TYPE_X)
drives motor 1 front-right CCW, 2 rear-left CCW, 3 front-left CW, 4 rear-right CW, on SERVO1-4:
`MOTOR_OF_ROTOR` maps one onto the other, spins and all.

Each firmware frame is one jitted call: `substeps` exact-exponential steps (rotor lag in closed
form, RK4 for the rest; one a frame at SITL's 1.2 kHz, more for a slower frame) through the
backend's make_rollout, which renormalises the quaternion and clips the rotor speeds after each,
then the backend's exact IMU at the end. JAX is configured for float64 on the CPU, never the GPU
(Chrome's), with its persistent compilation cache in logs/jax-cache so later spawns skip most of
the jit.
"""

import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

os.environ["JAX_PLATFORMS"] = "cpu"  # before jax loads: never take the GPU from Chrome

import jax  # noqa: E402
import jax.numpy as jnp  # noqa: E402
import numpy as np  # noqa: E402
from skyflow_dynamics.backends import jax as backend  # noqa: E402
from skyflow_dynamics.spec.parameters import RACER_5IN  # noqa: E402

ROOT = Path(__file__).resolve().parents[4]
JAX_CACHE = ROOT / "logs" / "jax-cache"

jax.config.update("jax_enable_x64", True)
jax.config.update("jax_compilation_cache_dir", str(JAX_CACHE))
jax.config.update("jax_persistent_cache_min_compile_time_secs", 0.0)

N = 4
AIRFRAMES: dict[str, dict[str, Any]] = {"racer5in": RACER_5IN}
"""SkyFlow-Dynamics' reference vehicles we fly, by the vehicle file's physics.airframe."""

MOTOR_OF_ROTOR = (2, 0, 3, 1)
"""For each of racer5in's rotors (FL, FR, RR, RL), the index of the ArduPilot motor (SERVO
output, from 0) that drives it: FL is motor 3, FR motor 1, RR motor 4, RL motor 2."""
ROTOR_OF_MOTOR = tuple(MOTOR_OF_ROTOR.index(m) for m in range(N))
"""For each ArduPilot motor 1-4, racer5in's rotor."""

FLIP = np.array([1.0, -1.0, -1.0])
"""diag(1, -1, -1): NWU ⇄ NED and FLU ⇄ FRD."""

STATE = backend.state_slices(N)
X, V, Q, W, ROTORS = (STATE[k] for k in ("x", "v", "q_wxyz", "w", "rotor_speeds"))
INPUT = backend.input_slices(N)


def ned_quat(q_wxyz: np.ndarray) -> tuple[float, float, float, float]:
    """q (W ← B), SkyFlow's, as q (NED ← FRD)."""
    w, x, y, z = (float(c) for c in q_wxyz)
    return (w, x, -y, -z)


def skyflow_quat(q: tuple[float, float, float, float]) -> np.ndarray:
    """q (NED ← FRD) as q (W ← B)."""
    w, x, y, z = q
    return np.array([w, x, -y, -z])


def yaw_quat(heading_deg: float) -> tuple[float, float, float, float]:
    """Level, nose on `heading_deg` true: q (NED ← FRD)."""
    half = math.radians(heading_deg) / 2
    return (math.cos(half), 0.0, 0.0, math.sin(half))


@dataclass(frozen=True)
class Airframe:
    name: str
    values: dict[str, Any]
    """SkyFlow-Dynamics' parameter dict (spec.parameters), its harness-side `limits` too."""

    @property
    def mass(self) -> float:
        return float(self.values["mass"])

    @property
    def grav(self) -> float:
        return float(self.values["grav"])

    @property
    def rotor_min(self) -> float:
        """The slowest a powered rotor turns, rad/s (the `limits` floor)."""
        return float(self.values["limits"]["rotor_speed_min"])

    @property
    def rotor_max(self) -> float:
        """The fastest, rad/s (the `limits` ceiling: racer5in's 8.5 N per motor)."""
        return float(self.values["limits"]["rotor_speed_max"])

    @property
    def ct2(self) -> float:
        return float(self.values["ct2"][0])

    @property
    def cq2(self) -> float:
        return float(self.values["cq2"][0])

    def thrust(self, omega: float) -> float:
        """One rotor's static thrust at `omega` rad/s, N (ct0 and ct1 are 0 for racer5in)."""
        return self.ct2 * omega * omega

    @property
    def hover_speed(self) -> float:
        """Each rotor's speed to hover, rad/s."""
        return math.sqrt(self.mass * self.grav / (N * self.ct2))


def airframe(name: str) -> Airframe:
    try:
        return Airframe(name, AIRFRAMES[name])
    except KeyError:
        raise ValueError(f"no quad airframe {name!r}: one of {sorted(AIRFRAMES)}") from None


def with_payload(frame: Airframe, mass_kg: float, length_m: float, diameter_m: float,
                 below_m: float) -> Airframe:
    """`frame` carrying a payload (content's PayloadSpec): a solid tube along the body, its axis
    `below_m` under the centre of mass. Its mass adds to the frame's, and its inertia (its own
    as a solid cylinder, and the parallel axis' m d² for roll and pitch) to the frame's. The
    centre of mass moves down by m d / (M + m), about a centimetre for a 0.4 kg charge 3 cm under
    a 5 inch: not modelled (SkyFlow-Dynamics' rotors are placed about it), nor the tube's drag."""
    m, r, d = mass_kg, diameter_m / 2, below_m
    along = 0.5 * m * r * r
    across = m * (3 * r * r + length_m * length_m) / 12
    inertia = [list(row) for row in frame.values["inertia"]]
    inertia[0][0] += along + m * d * d
    inertia[1][1] += across + m * d * d
    inertia[2][2] += across
    values = {**frame.values, "mass": frame.mass + m, "inertia": inertia}
    return Airframe(f"{frame.name}+payload", values)


class Plant:
    """The airframe's dynamics, jitted: `advance` one firmware frame, `statedot` and `imu` for
    the tests. Arrays are SkyFlow's flat state (13 + 4) and inputs (9 + 4)."""

    def __init__(self, frame: Airframe) -> None:
        self.frame = frame
        values = {k: v for k, v in frame.values.items() if k != "limits"}
        self.params = backend.pack_params(values)
        self._statedot = jax.jit(backend.statedot_fn(N))
        imu = backend.imu_fn(N)
        step = backend.exact_exp_step_fn(N)
        # Powered rotors never turn slower than the limits' floor (the ESC map starts there);
        # an unpowered one stops, so the clip's floor is 0.
        rollout = backend.make_rollout(step, 0.0, frame.rotor_max)
        eye = jnp.eye(3).ravel()
        zero = jnp.zeros(3)

        def advance(
            s: jax.Array, u: jax.Array, p: jax.Array, dt: float, substeps: int
        ) -> tuple[jax.Array, jax.Array]:
            states = rollout(s, jnp.tile(u, (substeps, 1)), p, dt)
            end = states[-1]
            return end, imu(end, u, p, zero, eye)

        self._advance = jax.jit(advance, static_argnums=(4,))
        self._imu = jax.jit(lambda s, u, p: imu(s, u, p, zero, eye))

    def warm(self, substeps: int = 1) -> None:
        """Trace and compile `advance` now (or load it from the cache), not on the first frame:
        for `substeps` a frame, one at SITL's 1.2 kHz (firmware/ardupilot/sitl.parm)."""
        s = self.rest_state((0.0, 0.0, 0.0), 0.0)
        self.advance(s, self.inputs(np.zeros(N)), 1 / 1200, substeps)

    def advance(
        self,
        s: np.ndarray,
        u: np.ndarray,
        dt: float,
        substeps: int,
        params: jax.Array | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """`substeps` steps of `dt` s with the inputs held: the state after, and the IMU there
        (specific force, then rates, body FLU). `params`: the airframe as damage has left it
        (`damaged_params`), else as built."""
        p = self.params if params is None else params
        end, imu = self._advance(s, u, p, dt, substeps)
        return np.asarray(end), np.asarray(imu)

    def damaged_params(self, rotor_share: np.ndarray) -> jax.Array:
        """The airframe's parameters with each rotor's (racer5in's order) thrust and drag-torque
        coefficients (ct2, cq2) scaled by `rotor_share`: 0 for a rotor shot away. Packed as
        the built ones are, the same shape, so `advance` is not traced again."""
        values = {k: v for k, v in self.frame.values.items() if k != "limits"}
        for name in ("ct2", "cq2"):
            values[name] = [float(c) * float(k) for c, k in zip(values[name], rotor_share,
                                                                  strict=True)]
        return backend.pack_params(values)

    def statedot(self, s: np.ndarray, u: np.ndarray, params: jax.Array | None = None) -> np.ndarray:
        return np.asarray(self._statedot(s, u, self.params if params is None else params))

    def imu(self, s: np.ndarray, u: np.ndarray) -> np.ndarray:
        return np.asarray(self._imu(s, u, self.params))

    @staticmethod
    def inputs(
        rotor_cmd: np.ndarray,
        wind_ned: tuple[float, float, float] = (0.0, 0.0, 0.0),
        force_ned: np.ndarray | None = None,
        torque_frd: np.ndarray | None = None,
    ) -> np.ndarray:
        """Commanded rotor speeds (racer5in's order), the wind (NED, m/s), and an external
        force (NED, N) and torque (body FRD, N·m), as SkyFlow's inputs (its F_ext is in W, its
        τ_ext in B: NWU and FLU here)."""
        u = np.zeros(9 + N)
        u[:N] = rotor_cmd
        u[INPUT["v_wind"]] = FLIP * np.asarray(wind_ned)
        if force_ned is not None:
            u[INPUT["F_ext"]] = FLIP * np.asarray(force_ned)
        if torque_frd is not None:
            u[INPUT["tau_ext"]] = FLIP * np.asarray(torque_frd)
        return u

    @staticmethod
    def rest_state(ned: tuple[float, float, float], heading_deg: float) -> np.ndarray:
        """Still, level, facing `heading_deg`, at `ned` m, motors stopped."""
        s = np.zeros(13 + N)
        s[X] = FLIP * np.asarray(ned)
        s[Q] = skyflow_quat(yaw_quat(heading_deg))
        return s
