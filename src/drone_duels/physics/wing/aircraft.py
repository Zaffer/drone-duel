"""The whole wing: in the hand, thrown, flying, and back on the ground.

`Aircraft.step` advances it by one physics step under `Controls`: the throttle and each elevon's
servo, as the flight controller drives them. Between steps it keeps what an IMU, a pitot and a
GPS would read (specific force, rates, airspeed, position), for the flight controller, and the
forces and moments that moved it, piece by piece (`Forces`), to look into the model.
"""

import math
from dataclasses import dataclass, field
from enum import StrEnum

from drone_duels.physics.core.rigidbody import (
    BodyState,
    Inertia,
    Vec,
    add,
    cross,
    from_euler,
    scale,
    sub,
    to_euler,
    unrotate,
)
from drone_duels.physics.core.weather import Weather
from drone_duels.physics.wing.aero import AirData, air_data
from drone_duels.physics.wing.propulsion import PowerTrain, PowerTrainState, momentum_drag
from drone_duels.physics.wing.strips import Flow, Loads, Wing, forces_and_moments

STANDARD_GRAVITY = 9.80665
SEA_LEVEL_DENSITY = 1.225
SERVO_RATE = math.radians(600)
"""How fast an elevon servo moves, rad/s (EST: a typical 0.1 s per 60 degrees; TBS's servo's
own is unknown)."""
BELLY = 0.04
"""The belly's height below the centre of mass, m: where the wing touches the ground (EST)."""
OBSTACLE = 0.3
"""Found this far, m, below the ground in one step, the wing has flown into something that rose
in its way, not come down onto it (EST)."""


class Phase(StrEnum):
    HAND = "hand"
    THROW = "throw"
    FLYING = "flying"
    LANDED = "landed"
    CRASHED = "crashed"


@dataclass(frozen=True)
class Launch:
    height: float
    """Above the ground, m."""
    speed: float
    """At release, m/s."""
    climb: float
    """The throw's path above the horizon, rad."""
    pitch: float
    """The nose above the horizon, in the hand and at release, rad."""
    duration: float
    """Of the arm's swing, s."""


@dataclass(frozen=True)
class GroundContact:
    landing_descent: float
    """Coming down slower than this, m/s, is a landing."""
    landing_bank: float
    landing_speed: float
    """Along the ground, m/s."""
    friction: float


@dataclass(frozen=True)
class Airframe:
    wing: Wing
    inertia: Inertia
    power_train: PowerTrain
    elevon_max: float
    """Each elevon's deflection at full servo travel, rad."""
    launch: Launch
    ground: GroundContact


@dataclass(frozen=True)
class Forces:
    """One step's forces (N) and moments (N m, about the centre of mass), body frame, by where
    they come from; gravity aside. Nothing but in flight."""

    aero: Vec = (0.0, 0.0, 0.0)
    """The strips' and the pod's."""
    aero_moment: Vec = (0.0, 0.0, 0.0)
    prop: Vec = (0.0, 0.0, 0.0)
    """Its thrust, and the air crossing its disc."""
    prop_moment: Vec = (0.0, 0.0, 0.0)
    """The prop's force off the centre of mass, and the motor's reaction."""
    gyroscopic: Vec = (0.0, 0.0, 0.0)
    """The spinning rotor's."""
    loads: Loads = field(default_factory=Loads)
    """The aerodynamics piece by piece."""

    @property
    def total(self) -> Vec:
        return add(self.aero, self.prop)

    @property
    def total_moment(self) -> Vec:
        return add(add(self.aero_moment, self.prop_moment), self.gyroscopic)


STILL = Forces()


@dataclass
class Controls:
    throttle: float = 0.0
    """0..1."""
    left: float = 0.0
    right: float = 0.0
    """Each elevon's servo, -1..1 of full travel, trailing edge down positive."""


@dataclass
class Aircraft:
    airframe: Airframe
    heading: float = 0.0
    """Faces this way in the hand, rad from north, clockwise."""
    wind: Vec = (0.0, 0.0, 0.0)
    """NED, m/s: steady, or the `weather`'s where the wing is."""
    weather: Weather | None = None
    """The wind's shear, turbulence and gusts (default: just `wind`)."""
    rho: float = SEA_LEVEL_DENSITY
    gravity: float = STANDARD_GRAVITY
    body: BodyState = field(default_factory=BodyState)
    power: PowerTrainState = field(default_factory=PowerTrainState)
    flow: Flow = field(default_factory=Flow)
    """Each strip's flow, separating and reattaching."""
    phase: Phase = Phase.HAND
    elevons: tuple[float, float] = (0.0, 0.0)
    """Where the elevons are, rad, trailing edge down positive."""
    servos: tuple[float, float] = (0.0, 0.0)
    """Where the flight controller has sent them, rad: the servos follow at their own speed."""
    forces: Forces = STILL
    specific_force: Vec = (0.0, 0.0, 0.0)
    """What an accelerometer at the centre of mass reads, body frame, m/s²."""
    air: AirData = field(default_factory=lambda: AirData(0.0, 0.0, 0.0, 0.0))
    impact: tuple[float, float] = (0.0, 0.0)
    """Descent and ground speed, m/s, when it last came down."""
    _throw_t: float = 0.0

    def __post_init__(self) -> None:
        self.reset(self.heading)

    def reset(
        self,
        heading: float | None = None,
        ground_down: float = 0.0,
        at: tuple[float, float] | None = None,
    ) -> None:
        """Back in the hand, facing `heading` (or as before), over the ground at NED down
        `ground_down`, at `at` (north, east; or where it is)."""
        if heading is not None:
            self.heading = heading
        north, east = at or self.body.position[:2]
        launch = self.airframe.launch
        self.body = BodyState(
            position=(north, east, ground_down - launch.height),
            attitude=from_euler(0.0, launch.pitch, self.heading),
        )
        self.phase = Phase.HAND
        self.elevons = (0.0, 0.0)
        self.flow.forget()
        self._held()

    def throw(self) -> None:
        if self.phase == Phase.HAND:
            self.phase = Phase.THROW
            self._throw_t = 0.0

    @property
    def euler(self) -> Vec:
        """Roll, pitch, yaw, rad."""
        return to_euler(self.body.attitude)

    def step(
        self, controls: Controls, dt: float, ground_down: float = 0.0, detail: bool = False
    ) -> None:
        """Advance by `dt` s. `ground_down`: the ground under the wing, NED down, m. `detail`:
        find the aerodynamics piece by piece too (`Forces.loads`), which costs a little; else
        the last found stay."""
        af = self.airframe
        self._move_elevons(controls, dt)
        height = ground_down - self.body.position[2]
        if self.weather is not None:
            self.weather.step(dt, height, self.air.airspeed, self.body.attitude)
            self.wind = self.weather.wind
        v_air_body = unrotate(self.body.attitude, sub(self.body.velocity, self.wind))
        v_prop = add(v_air_body, cross(self.body.rates, af.power_train.prop_position))
        self.power.step(af.power_train, controls.throttle, v_prop[0], self.rho, dt)
        if self.phase != Phase.FLYING:
            self.forces = STILL
        if self.phase == Phase.HAND:
            self._held()
        elif self.phase == Phase.THROW:
            self._swing(dt)
        elif self.phase == Phase.FLYING:
            self._fly(dt, v_air_body, height, detail)
            depth = self.body.position[2] - (ground_down - BELLY)
            if depth >= 0:
                self._touch_down(ground_down, struck=depth > OBSTACLE)
        elif self.phase == Phase.LANDED:
            self._slide(dt, ground_down)
        else:
            self.body.velocity = (0.0, 0.0, 0.0)
            self.body.rates = (0.0, 0.0, 0.0)
            self._held()

    def _move_elevons(self, controls: Controls, dt: float) -> None:
        travel = SERVO_RATE * dt
        full = self.airframe.elevon_max
        self.servos = (controls.left * full, controls.right * full)

        def toward(now: float, servo: float) -> float:
            return now + max(-travel, min(travel, servo - now))

        self.elevons = (
            toward(self.elevons[0], self.servos[0]),
            toward(self.elevons[1], self.servos[1]),
        )

    def _held(self) -> None:
        """Still: the accelerometer reads the ground's or hand's push against gravity."""
        self.specific_force = unrotate(self.body.attitude, (0.0, 0.0, -self.gravity))
        self.air = air_data(
            unrotate(self.body.attitude, sub(self.body.velocity, self.wind)), self.rho
        )

    def _swing(self, dt: float) -> None:
        """The throw: the arm accelerates it along the throw's path, nose held, until release."""
        launch = self.airframe.launch
        path = (
            math.cos(launch.climb) * math.cos(self.heading),
            math.cos(launch.climb) * math.sin(self.heading),
            -math.sin(launch.climb),
        )
        accel = launch.speed / launch.duration
        self._throw_t += dt
        b = self.body
        b.velocity = add(b.velocity, scale(path, accel * dt))
        b.position = add(b.position, scale(b.velocity, dt))
        self.specific_force = unrotate(
            b.attitude, sub(scale(path, accel), (0.0, 0.0, self.gravity))
        )
        self.air = air_data(unrotate(b.attitude, sub(b.velocity, self.wind)), self.rho)
        if self._throw_t >= launch.duration:
            b.velocity = scale(path, launch.speed)
            self.phase = Phase.FLYING

    def _fly(self, dt: float, v_air_body: Vec, height: float, detail: bool) -> None:
        af = self.airframe
        b = self.body
        # Through the air: the turbulence turns with it.
        rates = b.rates if self.weather is None else sub(b.rates, self.weather.rates)
        up = unrotate(b.attitude, (0.0, 0.0, -1.0))
        loads = Loads() if detail else None
        aero, aero_moment, self.air = forces_and_moments(
            af.wing, v_air_body, rates, self.elevons, self.rho, self.flow, dt, (height, up), loads
        )
        pt = af.power_train
        v_prop = add(v_air_body, cross(rates, pt.prop_position))
        side, normal = momentum_drag(pt, self.power, v_prop[0], (v_prop[1], v_prop[2]), self.rho)
        prop_force = (self.power.thrust, side, normal)
        prop_moment = add(cross(pt.prop_position, prop_force), (self.power.torque, 0.0, 0.0))
        # The spinning rotor's gyroscopic moment, -ω × H: pitching yaws it, yawing pitches it.
        spin = -pt.torque_sign * pt.rotor_inertia * self.power.omega
        _, q, r = b.rates
        gyroscopic = (0.0, -r * spin, q * spin)
        loads = loads or self.forces.loads
        forces = Forces(aero, aero_moment, prop_force, prop_moment, gyroscopic, loads)
        self.forces = forces
        force = forces.total
        self.specific_force = scale(force, 1 / af.inertia.mass)
        b.step(af.inertia, force, forces.total_moment, self.gravity, dt)

    def _touch_down(self, ground_down: float, struck: bool = False) -> None:
        """On the ground; `struck`, into something that rose above it (a roof, a tree): a crash
        however gently it was coming down."""
        g = self.airframe.ground
        b = self.body
        descent = b.velocity[2]
        speed = math.hypot(b.velocity[0], b.velocity[1])
        roll, pitch, yaw = self.euler
        self.impact = (descent, speed)
        gentle = (
            not struck
            and descent <= g.landing_descent
            and abs(roll) <= g.landing_bank
            and speed <= g.landing_speed
        )
        self.phase = Phase.LANDED if gentle else Phase.CRASHED
        b.position = (b.position[0], b.position[1], ground_down - BELLY)
        b.velocity = (b.velocity[0], b.velocity[1], 0.0) if gentle else (0.0, 0.0, 0.0)
        b.rates = (0.0, 0.0, 0.0)
        b.attitude = from_euler(0.0, 0.0, yaw) if gentle else b.attitude
        self._held()

    def _slide(self, dt: float, ground_down: float) -> None:
        """On its belly: friction stops it."""
        b = self.body
        vn, ve, _ = b.velocity
        speed = math.hypot(vn, ve)
        decel = min(self.airframe.ground.friction * self.gravity, speed / dt)
        along = (vn / speed, ve / speed, 0.0) if speed > 1e-6 else (0.0, 0.0, 0.0)
        b.velocity = sub((vn, ve, 0.0), scale(along, decel * dt))
        b.position = (
            b.position[0] + b.velocity[0] * dt,
            b.position[1] + b.velocity[1] * dt,
            ground_down - BELLY,
        )
        self._held()
        self.specific_force = sub(self.specific_force, unrotate(b.attitude, scale(along, decel)))
