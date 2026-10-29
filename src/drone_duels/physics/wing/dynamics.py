"""The flight model at work, to look into as it flies: every force and moment by where it comes
from, each strip's flow and load, the air, the elevons and the power train.

`Dynamics.of(aircraft)` reads it from the aircraft's last step: degrees, m/s, N, N m, Pa, in the
body frame (forward, right, down), the wind in NED. Rounded, for the 3D viewer's wing.html; what
has no value (a centre of pressure with no lift) is None.
"""

import math
from collections.abc import Iterable
from dataclasses import dataclass

from drone_duels.physics.core.rigidbody import Vec, add, cross, dot, scale, sub, unrotate
from drone_duels.physics.wing.aircraft import Aircraft
from drone_duels.physics.wing.strips import Strip, Wing, forces_and_moments

PLACES = 3
NEUTRAL_STEP = math.radians(0.5)
"""The angle of attack's step either way, rad, to find the neutral point."""
REACH = 0.5
"""A centre of pressure or neutral point further than this, m, from the centre of mass is shown
as none: near zero lift the centre of pressure runs off to infinity."""


def _r(x: float) -> float:
    return round(x, PLACES)


def _v(v: Iterable[float], k: float = 1.0) -> tuple[float, ...]:
    return tuple(round(c * k, PLACES) for c in v)


def _deg(v: Iterable[float]) -> tuple[float, ...]:
    return _v(v, math.degrees(1))


@dataclass(frozen=True)
class StripDynamics:
    """One strip: where it is, its flow and its load."""

    at: tuple[float, ...]
    """Its quarter chord from the centre of mass, m: where its force acts."""
    up: tuple[float, ...]
    """Unit: its lift side."""
    elevon: int
    """Which elevon it carries, 0 left, 1 right; -1 none."""
    alpha: float
    """Its own angle of attack, the twist included, deg."""
    downwash: float
    """Taken off it by the rest of the wing, deg."""
    seen: float
    """What its section flies at, deg."""
    stall: float
    """Where its section stalls, its flap's shift included, deg."""
    lead: float
    """How far its stall lags a changing angle of attack (dynamic stall), deg."""
    ground_effect: float
    """The share of the downwash the ground leaves."""
    flap: float
    """Its elevon's effective deflection, deg."""
    attached: float
    """The share of its flow attached."""
    cl: float
    cd: float
    cm: float
    q: float
    """Its dynamic pressure, Pa."""
    force: tuple[float, ...]
    corners: tuple[tuple[float, ...], ...]
    """Its outline seen along the flow: leading edge at either end, then the trailing edge back
    the other way, m."""
    circulation: float
    """Its bound vortex's, m²/s: ½ CL c V."""
    air: tuple[float, ...]
    """The air coming past it, m/s: its own motion and the aircraft's rotation, turned by the
    downwash (strips.StripLoad.air)."""


@dataclass(frozen=True)
class Dynamics:
    """Everything the flight model is doing, from its last step."""

    airspeed: float
    alpha: float
    beta: float
    dynamic_pressure: float
    coefficients: tuple[float, ...]
    """CL, CD, CY (stability axes), Cl, Cm, Cn (body): on the reference area, span and chord."""
    wind: tuple[float, ...]
    """NED, m/s: all of it at the wing."""
    wind_mean: tuple[float, ...]
    """The mean wind there, after the shear."""
    turbulence: tuple[float, ...]
    gust: tuple[float, ...]
    turbulence_rates: tuple[float, ...]
    """The turbulence's rotation, deg/s, body frame."""
    aero: tuple[float, ...]
    """N, body frame: the strips' and the pod's."""
    pod_drag: tuple[float, ...]
    prop: tuple[float, ...]
    """Thrust, and the air crossing the disc."""
    gravity: tuple[float, ...]
    total: tuple[float, ...]
    load: tuple[float, ...]
    """What an accelerometer at the centre of mass reads, g: z -1 in level flight, so the load
    factor is -z."""
    aero_moment: tuple[float, ...]
    """N m about the centre of mass: the strips' and the pod's, Munk's included."""
    munk: tuple[float, ...]
    prop_moment: tuple[float, ...]
    """The prop's force off the centre of mass, and the motor's reaction."""
    gyroscopic: tuple[float, ...]
    total_moment: tuple[float, ...]
    rates: tuple[float, ...]
    """deg/s."""
    angular_acceleration: tuple[float, ...]
    """deg/s²."""
    servos: tuple[float, ...]
    """Where the flight controller sends each elevon, left then right, deg, trailing edge down +."""
    elevons: tuple[float, ...]
    """Where the servos have got them to."""
    effective: tuple[float, ...]
    """What the air feels once the foam wing twists under them."""
    flex: float
    """The share of the elevons' effect the twist leaves."""
    rpm: float
    advance_ratio: float
    ct: float
    cp: float
    thrust: float
    motor_torque: float
    """N m, its reaction on the airframe."""
    prop_torque: float
    motor_amps: float
    pack_volts: float
    pack_amps: float
    used_mah: float
    prop_at: tuple[float, ...]
    strips: tuple[StripDynamics, ...]
    wake: tuple[tuple[tuple[float, ...], float], ...]
    """The trailing vortices, from the wing's strips' ends along the span, left to right: where
    each leaves the wing (m), and its strength, m²/s, the difference of the strips' circulations
    either side of it (the tips': all of the outermost strip's)."""
    centre_of_pressure: float | None
    """Where along the body's x axis the aerodynamic force would give its pitching moment, m
    from the centre of mass, forward +."""
    neutral_point: float | None
    """Where a change of angle of attack's lift acts: behind the centre of mass, nose-down to
    an upset, stable."""
    static_margin: float | None
    """The neutral point's distance behind the centre of mass, over the mean chord."""
    height: float
    """Above where it started, m."""
    energy_height: float
    """Its height and its speed's: h + V² / 2g, m (ground speed)."""
    energy_rate: float
    """How fast that grows, m/s: the thrust's and the air's work on it, over its weight."""

    @staticmethod
    def of(aircraft: Aircraft) -> "Dynamics":
        ac = aircraft
        af = ac.airframe
        wing = af.wing
        pt = af.power_train
        forces = ac.forces
        loads = forces.loads
        air = ac.air
        mass = af.inertia.mass
        gravity = unrotate(ac.body.attitude, (0.0, 0.0, mass * ac.gravity))
        total = add(forces.total, gravity)
        moment = forces.total_moment
        alpha, beta = air.alpha, air.beta
        # Stability axes: drag against the flow, lift square to it in the plane of symmetry.
        flow = (math.cos(alpha) * math.cos(beta), math.sin(beta), math.sin(alpha) * math.cos(beta))
        lift = forces.aero[0] * math.sin(alpha) - forces.aero[2] * math.cos(alpha)
        qs = air.dynamic_pressure * wing.reference.area
        span, chord = wing.reference.span, wing.reference.chord
        k = 1 / qs if qs > 1e-3 else 0.0
        weather = ac.weather
        if weather is not None:
            turbulence, gust, eddy_rates = weather.eddies, weather.gust, weather.rates
        else:
            turbulence = gust = eddy_rates = (0.0, 0.0, 0.0)
        flex = 1 / (1 + air.dynamic_pressure / wing.flex_pa) if wing.flex_pa > 0 else 1.0
        rps = ac.power.omega / (2 * math.pi)
        ct, cp = pt.coefficients(ac.power.advance_ratio) if rps > 1 else (0.0, 0.0)
        strips = tuple(
            StripDynamics(
                at=_v(s.quarter_chord),
                up=_v(s.up),
                elevon=s.elevon if s.elevon_fraction > 0 else -1,
                alpha=_r(math.degrees(load.alpha)),
                downwash=_r(math.degrees(load.downwash)),
                seen=_r(math.degrees(load.alpha - load.downwash)),
                stall=_r(
                    math.degrees(
                        s.section.alpha_stall
                        + s.stall_offset
                        - s.section.flap_stall * s.section.flap_lift * load.flap
                    )
                ),
                lead=_r(math.degrees(load.lead)),
                ground_effect=_r(load.ground_effect),
                flap=_r(math.degrees(load.flap)),
                attached=_r(load.attached),
                cl=_r(load.cl),
                cd=_r(load.cd),
                cm=_r(load.cm),
                q=_r(load.dynamic_pressure),
                force=_v(load.force),
                corners=_outline(s),
                circulation=_r(_circulation(s, load.cl, load.dynamic_pressure, ac.rho)),
                air=_v(load.air),
            )
            for s, load in zip(wing.strips, loads.strips, strict=False)
        )
        circulations = [s.circulation for s in strips]
        centre = _pitch_centre(forces.aero, forces.aero_moment)
        neutral = _neutral_point(wing, air.airspeed, alpha, beta, ac.elevons, ac.rho)
        v_body = unrotate(ac.body.attitude, ac.body.velocity)
        height = -ac.body.position[2]
        return Dynamics(
            airspeed=_r(air.airspeed),
            alpha=_r(math.degrees(alpha)),
            beta=_r(math.degrees(beta)),
            dynamic_pressure=_r(air.dynamic_pressure),
            coefficients=_v(
                (
                    lift * k,
                    -dot(forces.aero, flow) * k,
                    forces.aero[1] * k,
                    forces.aero_moment[0] * k / span,
                    forces.aero_moment[1] * k / chord,
                    forces.aero_moment[2] * k / span,
                )
            ),
            wind=_v(ac.wind),
            wind_mean=_v(sub(sub(ac.wind, turbulence), gust)),
            turbulence=_v(turbulence),
            gust=_v(gust),
            turbulence_rates=_deg(eddy_rates),
            aero=_v(forces.aero),
            pod_drag=_v(loads.pod_drag),
            prop=_v(forces.prop),
            gravity=_v(gravity),
            total=_v(total),
            load=_v(ac.specific_force, 1 / ac.gravity),
            aero_moment=_v(forces.aero_moment),
            munk=_v(loads.munk),
            prop_moment=_v(forces.prop_moment),
            gyroscopic=_v(forces.gyroscopic),
            total_moment=_v(moment),
            rates=_deg(ac.body.rates),
            angular_acceleration=_deg(af.inertia.angular_acceleration(moment, ac.body.rates)),
            servos=_deg(ac.servos),
            elevons=_deg(ac.elevons),
            effective=_deg(loads.elevons),
            flex=_r(flex),
            rpm=_r(rps * 60),
            advance_ratio=_r(ac.power.advance_ratio),
            ct=_r(ct),
            cp=_r(cp),
            thrust=_r(ac.power.thrust),
            motor_torque=_r(ac.power.torque),
            prop_torque=_r(ac.power.prop_torque),
            motor_amps=_r(ac.power.motor_current),
            pack_volts=_r(ac.power.pack_voltage),
            pack_amps=_r(ac.power.pack_current),
            used_mah=_r(ac.power.used_mah),
            prop_at=_v(pt.prop_position),
            strips=strips,
            wake=_wake(wing, circulations),
            centre_of_pressure=None if centre is None else _r(centre),
            neutral_point=None if neutral is None else _r(neutral),
            static_margin=None if neutral is None else _r(-neutral / chord),
            height=_r(height),
            energy_height=_r(height + dot(v_body, v_body) / (2 * ac.gravity)),
            energy_rate=_r(dot(forces.total, v_body) / (mass * ac.gravity)),
        )


def _span(s: Strip) -> Vec:
    """Unit, along its quarter chord, toward +y: its span."""
    span = cross(s.up, s.forward)
    return span if span[1] >= 0 or abs(span[1]) < 1e-9 else scale(span, -1)


def _outline(s: Strip) -> tuple[tuple[float, ...], ...]:
    """Streamwise, as the wing is seen from above: its chord along the body's x axis."""
    span = _span(s)
    half = scale(span, s.width / 2)
    # Its area over its streamwise width: the chord along the flow (a fin's own is that).
    chord = s.chord / abs(span[1]) if abs(span[1]) > 0.1 else s.chord
    x = (1.0, 0.0, 0.0)
    le = add(s.quarter_chord, scale(x, 0.25 * chord))
    te = sub(s.quarter_chord, scale(x, 0.75 * chord))
    return tuple(_v(p) for p in (sub(le, half), add(le, half), add(te, half), sub(te, half)))


def _circulation(s: Strip, cl: float, q: float, rho: float) -> float:
    return 0.5 * cl * s.chord * math.sqrt(2 * max(q, 0.0) / rho)


def _is_wing(s: Strip) -> bool:
    return abs(s.up[1]) < 0.5


def _wake(wing: Wing, circulations: list[float]) -> tuple[tuple[tuple[float, ...], float], ...]:
    """Each strip a horseshoe vortex: where two meet, what is left of their trailing legs."""
    on = sorted(
        ((s, g) for s, g in zip(wing.strips, circulations, strict=False) if _is_wing(s)),
        key=lambda sg: sg[0].quarter_chord[1],
    )
    if not on:
        return ()
    ends = [
        (
            sub(s.quarter_chord, scale(_span(s), s.width / 2)),
            add(s.quarter_chord, scale(_span(s), s.width / 2)),
        )
        for s, _ in on
    ]
    wake = [(_v(ends[0][0]), _r(-on[0][1]))]
    for k in range(len(on) - 1):
        at = scale(add(ends[k][1], ends[k + 1][0]), 0.5)
        wake.append((_v(at), _r(on[k][1] - on[k + 1][1])))
    wake.append((_v(ends[-1][1]), _r(on[-1][1])))
    return tuple(wake)


def _pitch_centre(force: Vec, moment: Vec) -> float | None:
    """x along the body where `force`'s z part gives `moment`'s y part: M_y = -x F_z."""
    if abs(force[2]) < 0.3:
        return None
    x = -moment[1] / force[2]
    return x if abs(x) <= REACH else None


def _neutral_point(
    wing: Wing,
    airspeed: float,
    alpha: float,
    beta: float,
    elevons: tuple[float, float],
    rho: float,
) -> float | None:
    """Where the change in lift acts as the angle of attack changes, the elevons held: the
    settled flow's, a half degree either way."""
    if airspeed < 3:
        return None

    def at(a: float) -> tuple[Vec, Vec]:
        v = (
            airspeed * math.cos(a) * math.cos(beta),
            airspeed * math.sin(beta),
            airspeed * math.sin(a) * math.cos(beta),
        )
        force, moment, _ = forces_and_moments(wing, v, (0.0, 0.0, 0.0), elevons, rho)
        return force, moment

    (f0, m0), (f1, m1) = at(alpha - NEUTRAL_STEP), at(alpha + NEUTRAL_STEP)
    lift = f1[2] - f0[2]
    if abs(lift) < 0.05:
        return None
    x = -(m1[1] - m0[1]) / lift
    return x if abs(x) <= REACH else None
