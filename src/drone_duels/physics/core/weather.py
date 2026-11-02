"""The air the wing flies through: the wind, its shear near the ground, turbulence and gusts.

MIL-F-8785C's low-altitude model (MIL-HDBK-1797's, as JSBSim and MATLAB's Aerospace Blockset
have it), from the wind at 20 ft (6.1 m), W20:
  - the mean wind falls off toward the ground as the log profile, ln(h / z0) / ln(20 ft / z0),
    z0 0.15 ft (its terminal flight phase's: near the ground is where this wing flies);
  - Dryden turbulence, frozen in the air and met at the airspeed: σ_w = 0.1 W20,
    σ_u = σ_v = σ_w / (0.177 + 0.000823 h)^0.4, L_w = h, L_u = L_v = h / (0.177 + 0.000823 h)^1.2,
    h in feet held to 10..1000; u along the mean wind, v across it, w down (the low-altitude
    axes), with 8785C's spectra, u's first order and v's and w's second;
  - the turbulence's rotation, how its velocity varies over the wing (8785C's forms, b the
    span): about x, the spanwise slope of w, a process of its own; about y and z, how w and v
    change along the flight path, met as the wing flies on, -(s / V) w and (s / V) v, each through
    1 / (1 + (4b / πV) s) (y) and 1 / (1 + (3b / πV) s) (z);
  - the turbulence's velocity as the airframe feels it (`felt`): averaged over its span, through
    1 / (1 + (4b / πV) s), 8785C's own form for the span's averaging (its q filter). 8785C's
    spectra are a point's, and the physics steps at 1 kHz: unfiltered, eddies far smaller than
    the wing shook it at tens of Hz, a buzz no wing of this size feels;
  - `scale` stretches the scale lengths (larger, slower and smoother air; 1 is 8785C's);
  - gusts on top, as a weather report gives them: now and then (every `gust_every` s on
    average) the wind rises along its own direction (within 30°) by up to `gusts` and falls
    back, in 8785C's 1 - cos shape, over about 3 s (the WMO's gust is a 3 s mean).
Each filter is stepped exactly for its time constant, so the variances hold at any airspeed and
step (tests/test_physics_wing.py).
"""

import math
import random
from collections.abc import Mapping
from typing import Any

from drone_duels.physics.core.rigidbody import Quat, Vec, add, scale, sub, unrotate

FOOT = 0.3048
GROUND_ROUGHNESS = 0.15 * FOOT
"""z0, m: MIL-F-8785C's for terminal flight phases."""
REFERENCE_HEIGHT = 20 * FOOT
MIN_AIRSPEED = 2.0
"""The turbulence is met at no less than this, m/s: held in the hand or hanging in a tail slide,
the wind still carries it past (EST)."""
GUST_EVERY = 10.0
"""Mean time between gusts, s (EST)."""
GUST_TIME = (2.0, 5.0)
"""Each gust's rise and fall, s: from, to (EST, about the WMO's 3 s gust)."""
GUST_SPREAD = math.radians(30)
"""How far off the wind's direction a gust can blow (EST)."""
GUST_LEAST = 0.4
"""The weakest gust, as a share of the strongest (EST)."""


class Weather:
    """The wind of `speed` m/s at 20 ft from `from_deg` (true), its MIL-F-8785C turbulence times
    `turbulence` with scale lengths times `scale`, and gusts up to `gusts` m/s above it every
    `gust_every` s or so, met by a wing of `span` m.

    `step` advances it; then `wind` is the air's velocity at the wing (NED, m/s), and `rates` the
    turbulence's rotation in the body frame, rad/s: the aircraft's rates through the air are its
    own less these."""

    def __init__(
        self,
        speed: float = 0.0,
        from_deg: float = 0.0,
        gusts: float = 0.0,
        span: float = 1.0,
        turbulence: float = 1.0,
        scale: float = 1.0,
        gust_every: float = GUST_EVERY,
        seed: int | None = None,
    ) -> None:
        self.speed = speed
        self.from_deg = from_deg
        self.gusts = gusts
        self.span = span
        self.turbulence = turbulence
        self.scale = scale
        self.gust_every = gust_every
        self.rng = random.Random(seed)
        self.wind: Vec = (0.0, 0.0, 0.0)
        self.rates: Vec = (0.0, 0.0, 0.0)
        self.gust: Vec = (0.0, 0.0, 0.0)
        self.eddies: Vec = (0.0, 0.0, 0.0)
        """The turbulence at a point, 8785C's, NED, m/s."""
        self.felt: Vec = (0.0, 0.0, 0.0)
        """The turbulence over the span, as the airframe feels it, NED, m/s."""
        self._gust_next: float | None = None
        """A gust asked for now (`gust_now`), m/s above the wind."""
        self._u = 0.0
        self._v = (0.0, 0.0)
        self._w = (0.0, 0.0)
        self._p = 0.0
        self._gust_in = self.rng.expovariate(1 / gust_every)
        self._gust: tuple[float, float, Vec] | None = None
        """The gust under way: for how long so far, how long in all, its peak (NED)."""

    @property
    def downwind(self) -> Vec:
        """Unit, NED: where the wind blows to."""
        to = math.radians(self.from_deg + 180)
        return (math.cos(to), math.sin(to), 0.0)

    def shear(self, height: float) -> float:
        """The mean wind at `height` m above the ground over the wind at 20 ft."""
        if height <= GROUND_ROUGHNESS:
            return 0.0
        return math.log(height / GROUND_ROUGHNESS) / math.log(REFERENCE_HEIGHT / GROUND_ROUGHNESS)

    def step(self, dt: float, height: float, airspeed: float, attitude: Quat) -> None:
        """On by `dt` s, the wing `height` m above the ground at `airspeed`, attitude `attitude`."""
        along = self.downwind
        across = (-along[1], along[0], 0.0)
        mean = scale(along, self.speed * self.shear(height))
        sigma_w = 0.1 * self.speed * self.turbulence
        if sigma_w > 0:
            h = min(1000.0, max(10.0, height / FOOT))
            airspeed = max(airspeed, MIN_AIRSPEED)
            before = self.eddies
            self.eddies = self._turbulence(dt, h, sigma_w, airspeed, along, across)
            self.rates = self._rotation(dt, h, sigma_w, airspeed, attitude, before)
            e = math.exp(-dt * math.pi * airspeed / (4 * self.span))
            self.felt = add(scale(self.felt, e), scale(self.eddies, 1 - e))
        else:
            self.eddies = self.felt = self.rates = (0.0, 0.0, 0.0)
        self.gust = self._gusts(dt, height)
        self.wind = add(add(mean, self.felt), self.gust)

    def reschedule(self, gust_every: float) -> None:
        """Gusts every `gust_every` s on average from now: a shorter wait comes sooner."""
        self.gust_every = gust_every
        self._gust_in = min(self._gust_in, self.rng.expovariate(1 / gust_every))

    def gust_now(self, peak: float) -> None:
        """A gust of up to `peak` m/s starts at the next step (the one under way ends)."""
        self._gust = None
        self._gust_next = peak

    def _turbulence(
        self, dt: float, h: float, sigma_w: float, airspeed: float, along: Vec, across: Vec
    ) -> Vec:
        """The turbulence's velocity, NED, `h` ft above the ground."""
        k = 0.177 + 0.000823 * h
        sigma_u = sigma_w / k**0.4
        long_scale = h / k**1.2 * FOOT * self.scale
        self._u = self._lag(self._u, long_scale / airspeed, dt)
        self._v = self._lag2(self._v, long_scale / airspeed, dt)
        self._w = self._lag2(self._w, h * FOOT * self.scale / airspeed, dt)
        u = sigma_u * self._u
        v = sigma_u * _shaped(self._v)
        w = sigma_w * _shaped(self._w)
        return add(add(scale(along, u), scale(across, v)), (0.0, 0.0, w))

    def _lag(self, z: float, tau: float, dt: float) -> float:
        """A unit-variance first-order (Gauss-Markov) process of time constant `tau`, s."""
        e = math.exp(-dt / tau)
        return e * z + math.sqrt(1 - e * e) * self.rng.gauss(0.0, 1.0)

    def _lag2(self, z: tuple[float, float], tau: float, dt: float) -> tuple[float, float]:
        """`_lag`, and that again through a first-order lag: Dryden's second-order spectrum's
        states (`_shaped`)."""
        first = self._lag(z[0], tau, dt)
        return first, z[1] + (1 - math.exp(-dt / tau)) * (first - z[1])

    def _rotation(
        self, dt: float, h: float, sigma_w: float, airspeed: float, attitude: Quat, before: Vec
    ) -> Vec:
        """The turbulence's rotation, body frame, the turbulence having been `before` a step
        ago. σ_p is 8785C's spectrum of p integrated: σ_w² 0.1 π² (π L_w / 4b)^⅓ / (b L_w)."""
        b = self.span
        long_w = h * FOOT * self.scale
        sigma_p = sigma_w * math.sqrt(
            0.1 * math.pi**2 * (math.pi * long_w / (4 * b)) ** (1 / 3) / (b * long_w)
        )
        self._p = self._lag(self._p, 4 * b / (math.pi * airspeed), dt)
        _, dv, dw = unrotate(attitude, sub(self.eddies, before))
        _, q, r = self.rates
        eq = math.exp(-dt * math.pi * airspeed / (4 * b))
        er = math.exp(-dt * math.pi * airspeed / (3 * b))
        q = eq * q - (1 - eq) * dw / (airspeed * dt)
        r = er * r + (1 - er) * dv / (airspeed * dt)
        return sigma_p * self._p, q, r

    def _gusts(self, dt: float, height: float) -> Vec:
        if self.gusts <= 0 and self._gust is None and self._gust_next is None:
            return (0.0, 0.0, 0.0)
        if self._gust is None:
            self._gust_in -= dt
            if self._gust_in > 0 and self._gust_next is None:
                return (0.0, 0.0, 0.0)
            if self.speed > 0:
                heading = math.radians(self.from_deg + 180)
                heading += self.rng.uniform(-GUST_SPREAD, GUST_SPREAD)
            else:
                heading = self.rng.uniform(-math.pi, math.pi)
            if self._gust_next is not None:
                peak, self._gust_next = self._gust_next, None
            else:
                peak = self.gusts * self.rng.uniform(GUST_LEAST, 1.0)
            self._gust = (
                0.0,
                self.rng.uniform(*GUST_TIME),
                (peak * math.cos(heading), peak * math.sin(heading), 0.0),
            )
            self._gust_in = self.rng.expovariate(1 / self.gust_every)
        t, duration, peak = self._gust
        t += dt
        if t >= duration:
            self._gust = None
            return (0.0, 0.0, 0.0)
        self._gust = (t, duration, peak)
        return scale(peak, 0.5 * (1 - math.cos(2 * math.pi * t / duration)) * self.shear(height))


def _shaped(z: tuple[float, float]) -> float:
    """Dryden's second-order spectrum, (1 + √3 T s) / (1 + T s)², at unit variance, from
    `_lag2`'s states: (z2 + √3 (z1 - z2)) / √2."""
    return (z[1] + math.sqrt(3) * (z[0] - z[1])) / math.sqrt(2)


CALM = {
    "speed_ms": 0.0,
    "from_deg": 0.0,
    "turbulence": 1.0,
    "eddy_scale": 1.0,
    "gust_factor": 1.0,
    "gust_every_s": GUST_EVERY,
}
"""The game's wind settings (content.schema.Wind; the Wind options tab) for a key left out:
still air, and MIL-F-8785C's turbulence for whatever wind is set."""
GUST_NOW_LEAST = 0.5
"""A gust asked for (G) rises at least this share of the mean wind above it..."""
GUST_LEAST_MS = 2.0
"""...and at least this much, so a light wind still gusts."""


def gust_peak(speed: float, gust_factor: float) -> float:
    """The `gusts` (m/s above the mean) a Weather gets for a gust factor (the strongest gust
    over the mean wind, as a weather report's) at mean wind `speed`."""
    return max(0.0, gust_factor - 1.0) * speed


def from_settings(
    wind: Mapping[str, Any], span: float = 1.0, seed: int | None = None
) -> Weather:
    """A Weather from the game's wind settings (content.schema.Wind, as a dict)."""
    w = {**CALM, **wind}
    return Weather(
        speed=float(w["speed_ms"]),
        from_deg=float(w["from_deg"]),
        gusts=gust_peak(float(w["speed_ms"]), float(w["gust_factor"])),
        span=span,
        turbulence=float(w["turbulence"]),
        scale=float(w["eddy_scale"]),
        gust_every=float(w["gust_every_s"]),
        seed=seed,
    )


def update(weather: Weather, wind: Mapping[str, Any], gust_now: bool = False) -> None:
    """Change `weather` live, keeping its turbulence's state; `gust_now` starts a gust at once."""
    w = {**CALM, **wind}
    speed = float(w["speed_ms"])
    weather.speed = speed
    weather.from_deg = float(w["from_deg"])
    weather.turbulence = float(w["turbulence"])
    weather.scale = float(w["eddy_scale"])
    weather.gusts = gust_peak(speed, float(w["gust_factor"]))
    if float(w["gust_every_s"]) != weather.gust_every:
        weather.reschedule(float(w["gust_every_s"]))
    if gust_now:
        weather.gust_now(max(weather.gusts, GUST_NOW_LEAST * speed, GUST_LEAST_MS))
