"""6-DOF rigid-body state and its integration.

World: north-east-down (NED), metres, from the launch point. Body: forward-right-down (FRD).
Attitude: unit quaternion (w, x, y, z) turning body vectors into world ones. Angular rates
(p, q, r) are body-frame, rad/s: roll right, nose up, nose right positive.

Plain tuples and floats: at a thousand steps a second, faster than numpy on 3-vectors.
"""

import math
from dataclasses import dataclass

Vec = tuple[float, float, float]
Quat = tuple[float, float, float, float]


def add(a: Vec, b: Vec) -> Vec:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Vec, b: Vec) -> Vec:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a: Vec, k: float) -> Vec:
    return (a[0] * k, a[1] * k, a[2] * k)


def dot(a: Vec, b: Vec) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Vec, b: Vec) -> Vec:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def norm(a: Vec) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def rotate(q: Quat, v: Vec) -> Vec:
    """Body vector `v` in the world."""
    w, x, y, z = q
    tx = 2 * (y * v[2] - z * v[1])
    ty = 2 * (z * v[0] - x * v[2])
    tz = 2 * (x * v[1] - y * v[0])
    return (
        v[0] + w * tx + y * tz - z * ty,
        v[1] + w * ty + z * tx - x * tz,
        v[2] + w * tz + x * ty - y * tx,
    )


def unrotate(q: Quat, v: Vec) -> Vec:
    """World vector `v` in the body."""
    return rotate((q[0], -q[1], -q[2], -q[3]), v)


def from_euler(roll: float, pitch: float, yaw: float) -> Quat:
    """Aerospace Z-Y-X Euler angles (radians) to a quaternion."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def to_euler(q: Quat) -> Vec:
    """Roll, pitch, yaw (radians; yaw from north, clockwise) of a quaternion."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return (roll, pitch, yaw)


def integrate_attitude(q: Quat, rates: Vec, dt: float) -> Quat:
    """`q` turned by body rates `rates` for `dt`, exactly for constant rates."""
    p, qq, r = rates
    angle = math.sqrt(p * p + qq * qq + r * r) * dt
    if angle < 1e-12:
        return q
    k = math.sin(angle / 2) / (angle / dt)
    dw, dx, dy, dz = math.cos(angle / 2), p * k, qq * k, r * k
    w, x, y, z = q
    out = (
        w * dw - x * dx - y * dy - z * dz,
        w * dx + x * dw + y * dz - z * dy,
        w * dy - x * dz + y * dw + z * dx,
        w * dz + x * dy - y * dx + z * dw,
    )
    n = math.sqrt(sum(c * c for c in out))
    return (out[0] / n, out[1] / n, out[2] / n, out[3] / n)


@dataclass(frozen=True)
class Inertia:
    """Mass (kg) and inertia about the centre of mass (kg m², body axes; xz the only product)."""

    mass: float
    ixx: float
    iyy: float
    izz: float
    ixz: float = 0.0

    def angular_acceleration(self, moment: Vec, rates: Vec) -> Vec:
        """Euler's equations with the xz product, body frame."""
        p, q, r = rates
        roll, pitch, yaw = moment
        ixx, iyy, izz, ixz = self.ixx, self.iyy, self.izz, self.ixz
        # The rigid body's own gyroscopic moments, then I^-1 for an inertia with only Ixz.
        gl = roll - (q * r * (izz - iyy) - p * q * ixz)
        gm = pitch - (r * p * (ixx - izz) + (p * p - r * r) * ixz)
        gn = yaw - (p * q * (iyy - ixx) + q * r * ixz)
        det = ixx * izz - ixz * ixz
        return ((izz * gl + ixz * gn) / det, gm / iyy, (ixz * gl + ixx * gn) / det)


@dataclass
class BodyState:
    position: Vec = (0.0, 0.0, 0.0)
    velocity: Vec = (0.0, 0.0, 0.0)
    attitude: Quat = (1.0, 0.0, 0.0, 0.0)
    rates: Vec = (0.0, 0.0, 0.0)

    def step(self, inertia: Inertia, force: Vec, moment: Vec, gravity: float, dt: float) -> None:
        """Advance by `dt` under body-frame `force` and `moment` (excluding gravity),
        semi-implicitly: the new velocities move the body."""
        accel = add(scale(rotate(self.attitude, force), 1 / inertia.mass), (0.0, 0.0, gravity))
        self.velocity = add(self.velocity, scale(accel, dt))
        self.position = add(self.position, scale(self.velocity, dt))
        alpha = inertia.angular_acceleration(moment, self.rates)
        self.rates = add(self.rates, scale(alpha, dt))
        self.attitude = integrate_attitude(self.attitude, self.rates, dt)
