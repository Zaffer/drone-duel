"""Reference geometry and air data: what the aerodynamics (strips.py) are measured against."""

import math
from dataclasses import dataclass

from drone_duels.physics.core.rigidbody import Vec, norm


@dataclass(frozen=True)
class Geometry:
    """The reference wing: span and area, m and m²."""

    span: float
    area: float

    @property
    def chord(self) -> float:
        """Mean chord, S / b."""
        return self.area / self.span

    @property
    def aspect_ratio(self) -> float:
        return self.span * self.span / self.area


@dataclass(frozen=True)
class AirData:
    airspeed: float
    alpha: float
    beta: float
    dynamic_pressure: float


def air_data(v_body: Vec, rho: float) -> AirData:
    """Airspeed, angle of attack and sideslip of body-frame air-relative velocity `v_body`."""
    u, v, w = v_body
    speed = norm(v_body)
    if speed < 1e-6:
        return AirData(0.0, 0.0, 0.0, 0.0)
    return AirData(
        speed,
        math.atan2(w, u),
        math.asin(max(-1.0, min(1.0, v / speed))),
        0.5 * rho * speed * speed,
    )
