"""The Chupito's flight model (drone_duels.physics.wing), held to what is known of the real one.

The targets come from TBS's specification and one pilot's INAV blackbox log;
drone_duels.physics.wing.airframe says which is which.
"""

import json
import math
from dataclasses import asdict

import pytest

from drone_duels.physics.core.rigidbody import from_euler, rotate, to_euler, unrotate
from drone_duels.physics.core.weather import FOOT, Weather
from drone_duels.physics.wing.aircraft import Aircraft, Controls, Phase
from drone_duels.physics.wing.airframe import CHUPITO, SECTION
from drone_duels.physics.wing.dynamics import Dynamics
from drone_duels.physics.wing.propulsion import PowerTrainState
from drone_duels.physics.wing.strips import (
    Flow,
    Loads,
    Planform,
    Section,
    forces_and_moments,
    lifting_line,
    wing_strips,
)

DT = 0.001
ROLL_TRAVEL = 585 / 500
"""LOG's full rolls: the servo 585 µs off its centre, past the 500 µs of full travel."""
PULL_TRAVEL = 203 / 500
"""LOG's full pitch command near 29 m/s: 203 µs of pitch on the servo."""


def hold_level(ac: Aircraft, throttle: float, seconds: float, roll: float = 0.0) -> None:
    """Fly level for `seconds` at `throttle`, an elevator loop holding the height, the aileron
    holding `roll` (rad, right wing down positive)."""
    target = ac.body.position[2]
    climb_i = 0.0
    for _ in range(round(seconds / DT)):
        b = ac.body
        bank, pitch, _ = to_euler(b.attitude)
        climb = -b.velocity[2] - 0.5 * (b.position[2] - target)  # NED: down positive
        climb_i += climb * DT
        # Trailing edge down noses down: climbing, or pitching up, asks for more of it.
        elevator = 0.08 * climb + 0.04 * climb_i + 0.15 * b.rates[1] + 0.3 * pitch
        aileron = -1.5 * (bank - roll) - 0.1 * b.rates[0]
        elevator = max(-1.0, min(1.0, elevator))
        aileron = max(-1.0, min(1.0, aileron))
        ac.step(Controls(throttle, elevator + aileron, elevator - aileron), DT, ground_down=1e6)


def flying(speed: float) -> Aircraft:
    """Thrown, then settled level near `speed` m/s by trading throttle for it."""
    ac = Aircraft(CHUPITO)
    ac.throw()
    for _ in range(round(CHUPITO.launch.duration / DT) + 1):
        ac.step(Controls(1.0), DT, ground_down=1e6)
    throttle = 0.5
    for _ in range(40):
        hold_level(ac, throttle, 0.5)
        throttle = max(0.0, min(1.0, throttle + 0.03 * (speed - ac.air.airspeed)))
    return ac


def test_frames() -> None:
    q = from_euler(0.3, -0.2, 2.0)
    assert to_euler(q) == pytest.approx((0.3, -0.2, 2.0))
    v = (1.0, -2.0, 0.5)
    assert unrotate(q, rotate(q, v)) == pytest.approx(v)
    # Nose up 90 degrees: the nose points up, NED's -z.
    assert rotate(from_euler(0, math.pi / 2, 0), (1.0, 0.0, 0.0)) == pytest.approx(
        (0, 0, -1), abs=1e-9
    )


def test_controls_move_it_the_right_way() -> None:
    af = CHUPITO
    v = (20.0, 0.0, 0.0)
    _, m0, _ = forces_and_moments(af.wing, v, (0, 0, 0), (0.0, 0.0), 1.225)
    _, m_down, _ = forces_and_moments(af.wing, v, (0, 0, 0), (0.1, 0.1), 1.225)
    _, m_roll, _ = forces_and_moments(af.wing, v, (0, 0, 0), (0.1, -0.1), 1.225)
    assert m_down[1] < m0[1], "trailing edges down pitch the nose down"
    assert m_roll[0] > 0, "left elevon down, right up, rolls right"
    # Weathervane: air from the right (positive sideslip) yaws the nose right.
    _, m_slip, _ = forces_and_moments(af.wing, (20.0, 2.0, 0.0), (0, 0, 0), (0, 0), 1.225)
    assert m_slip[2] > 0


def test_in_the_hand_the_accelerometer_reads_one_g_up() -> None:
    ac = Aircraft(CHUPITO)
    ac.step(Controls(), DT)
    up = rotate(ac.body.attitude, ac.specific_force)
    assert up == pytest.approx((0, 0, -9.80665), abs=1e-6)
    assert ac.phase == Phase.HAND


def test_the_throw_reads_as_a_launch_and_lets_go_at_launch_speed() -> None:
    ac = Aircraft(CHUPITO)
    ac.throw()
    peak = 0.0
    while ac.phase == Phase.THROW:
        ac.step(Controls(1.0), DT)
        peak = max(peak, ac.specific_force[0])
    assert peak > 2 * 9.81, "INAV detects a throw at about 1.9 g forward"
    assert ac.air.airspeed == pytest.approx(CHUPITO.launch.speed, rel=0.02)


def test_it_flies_away_from_the_throw() -> None:
    """With the wings held level and the nose up 20 degrees, as INAV's launch mode does: hands
    off, the motor's torque rolls it over, and the thrust line, above the centre of mass, noses
    it down."""
    ac = Aircraft(CHUPITO)
    ac.throw()
    for _ in range(3000):
        bank, pitch, _ = ac.euler
        p, q, _ = ac.body.rates
        aileron = max(-1.0, min(1.0, -1.5 * bank - 0.1 * p))
        elevator = max(-1.0, min(1.0, 1.5 * (pitch - math.radians(20)) + 0.1 * q))
        ac.step(Controls(1.0, elevator + aileron, elevator - aileron), DT)
    assert ac.phase == Phase.FLYING
    assert -ac.body.position[2] > CHUPITO.launch.height + 5, "climbs out at full throttle"


def test_top_speed_and_power_match_the_log() -> None:
    ac = flying(60)  # asks for more than it has: full throttle
    ac.power.used_mah = 300  # the log's pack, part used: 20.8 V under this load
    hold_level(ac, 1.0, 5)
    assert ac.power.pack_voltage * ac.power.pack_current == pytest.approx(540, rel=0.15)
    assert ac.air.airspeed == pytest.approx(42, abs=3)


def test_its_launch_draws_the_logs_current() -> None:
    """LOG's launch, at 65 % throttle (1650 µs): 16 A slow, 14 A at 25 m/s, on a full pack."""
    pt = CHUPITO.power_train
    for speed, logged in ((5.0, 16.0), (25.0, 14.0)):
        power = PowerTrainState()
        for _ in range(1500):
            power.step(pt, 0.65, speed, 1.225, DT)
        assert power.pack_current == pytest.approx(logged, rel=0.15)


def test_the_pack_sags_at_once_then_slowly_and_recovers() -> None:
    """LOG's pack: 0.050-0.062 ohm at once, then 0.07 more over 5 to 10 s, back as it eases."""
    pt = CHUPITO.power_train
    power = PowerTrainState()

    def run(throttle: float, seconds: float) -> None:
        for _ in range(round(seconds / DT)):
            power.step(pt, throttle, 0.0, 1.225, DT)

    def sag() -> float:
        """Below the pack's voltage at rest with the charge it has now."""
        rest = pt.cells * pt.cell_voltage(1 - power.used_mah / pt.capacity_mah)
        return rest - power.pack_voltage

    run(1.0, 0.3)  # the prop spun up
    assert sag() / power.pack_current == pytest.approx(pt.pack_resistance, rel=0.1)
    run(1.0, 30)
    assert sag() / power.pack_current == pytest.approx(pt.pack_resistance + 0.07, rel=0.05)
    run(0.0, DT)
    cut = sag()
    run(0.0, pt.polarization_time)
    assert sag() == pytest.approx(cut / math.e, rel=0.05), "recovers over its time"


def test_a_lipo_holds_its_voltage_through_the_middle() -> None:
    cell = CHUPITO.power_train.cell_voltage
    assert cell(1.0) == 4.2
    middle, top = (cell(0.6) - cell(0.3)) / 0.3, (cell(1.0) - cell(0.8)) / 0.2
    assert top > 2 * middle, "falls quickly from full, then flat"
    assert cell(0.05) - cell(0.0) > 0.3, "and drops off a cliff at the end"
    assert cell(-0.1) == cell(0.0)


def test_it_cruises_as_tbs_says() -> None:
    """TBS: cruise 55-70 km/h, at its 35 % cruise throttle."""
    ac = Aircraft(CHUPITO)
    ac.throw()
    for _ in range(round(CHUPITO.launch.duration / DT) + 1):
        ac.step(Controls(1.0), DT, ground_down=1e6)
    hold_level(ac, 0.35, 25)
    assert 55 / 3.6 * 0.85 < ac.air.airspeed < 70 / 3.6 * 1.15


def coefficients(alpha: float, elevator: float = 0.0, speed: float = 15.0) -> tuple[float, float]:
    """Lift and pitching moment coefficients at `alpha`, the elevons both at `elevator` (rad)."""
    v = (speed * math.cos(alpha), 0.0, speed * math.sin(alpha))
    wing = CHUPITO.wing
    force, moment, _ = forces_and_moments(wing, v, (0, 0, 0), (elevator, elevator), 1.225)
    q = 0.5 * 1.225 * speed * speed * wing.reference.area
    lift = force[0] * math.sin(alpha) - force[2] * math.cos(alpha)
    return lift / q, moment[1] / (q * wing.reference.chord)


def trimmed_lift(alpha: float) -> float:
    """The lift at `alpha` with the elevons holding it there, as a tailless wing must."""
    lo, hi = -CHUPITO.elevon_max, CHUPITO.elevon_max
    for _ in range(40):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if coefficients(alpha, mid)[1] > 0 else (lo, mid)
    return coefficients(alpha, lo)[0]


def test_stall_speed_is_tbs() -> None:
    """TBS's 35 km/h: the most lift the elevons can hold, a tailless wing's trim costing some."""
    cl_max = max(trimmed_lift(math.radians(a / 2)) for a in range(16, 60))
    weight = CHUPITO.inertia.mass * 9.80665
    stall = math.sqrt(2 * weight / (1.225 * CHUPITO.wing.reference.area * cl_max))
    assert stall * 3.6 == pytest.approx(35, abs=2)


@pytest.mark.parametrize(("speed", "logged"), [(23, 650), (45, 700)])
def test_full_elevon_roll_rate_matches_the_log(speed: float, logged: float) -> None:
    ac = flying(speed)
    peak = 0.0
    for _ in range(400):
        ac.step(Controls(0.6, ROLL_TRAVEL, -ROLL_TRAVEL), DT, ground_down=1e6)
        peak = max(peak, math.degrees(ac.body.rates[0]))
    assert peak == pytest.approx(logged, rel=0.15)


def test_hands_off_it_does_not_diverge_in_pitch() -> None:
    ac = flying(25)
    throttle = 0.5
    hold_level(ac, throttle, 2)
    trim = Controls(throttle, *[e / CHUPITO.elevon_max for e in ac.elevons])
    for _ in range(5000):
        ac.step(trim, DT, ground_down=1e6)
    _, pitch, _ = ac.euler
    assert abs(math.degrees(pitch)) < 35


def test_a_dive_into_the_ground_is_a_crash_and_a_glide_on_is_a_landing() -> None:
    ac = flying(25)
    ground = ac.body.position[2] + 3
    for _ in range(3000):
        ac.step(Controls(1.0, 0.6, 0.6), DT, ground_down=ground)
    assert ac.phase == Phase.CRASHED

    ac = flying(15)
    b = ac.body
    ground = b.position[2] + 0.2
    b.velocity = (b.velocity[0], b.velocity[1], 0.8)  # sinking gently, wings level
    trim = Controls(0.0, *[e / CHUPITO.elevon_max for e in ac.elevons])
    for _ in range(5000):
        ac.step(trim, DT, ground_down=ground)
    assert ac.phase == Phase.LANDED
    assert math.hypot(ac.body.velocity[0], ac.body.velocity[1]) < 0.1, "slid to a stop"


def test_the_vortex_lattice_agrees_with_lifting_surface_theory() -> None:
    """A rectangular wing of the Chupito's aspect ratio against Helmbold's lift slope; sweep of
    either sign, the Chupito's 12 degrees, hardly changes it."""
    section = Section(
        lift_slope=2 * math.pi,
        zero_lift=0.0,
        cm0=0.0,
        cd0=0.0,
        alpha_stall=1.0,
        alpha_stall_neg=1.0,
        stall_sharpness=30.0,
        cd90=1.3,
    )
    cuts = tuple(0.4 * i / 8 for i in range(9))

    def lift_slope(sweep_deg: float) -> float:
        dx = 0.4 * math.tan(math.radians(sweep_deg))
        plan = Planform(stations=((0.0, 0.092, -0.092), (0.4, 0.092 + dx, -0.092 + dx)), cuts=cuts)
        strips = wing_strips(plan, section, (0.0, 0.0), (0.0, 0.0, 0.01), 1.0)
        loading = lifting_line(strips)
        area = sum(s.area for s in strips)
        return (
            2
            * math.pi
            * sum(sum(row) * s.area for row, s in zip(loading, strips, strict=True))
            / area
        )

    aspect = 0.8**2 / (0.8 * 0.184)
    helmbold = 2 * math.pi * aspect / (2 + math.sqrt(aspect**2 + 4))
    assert lift_slope(0) == pytest.approx(helmbold, rel=0.06)
    assert lift_slope(12) == pytest.approx(lift_slope(0), rel=0.04)
    assert lift_slope(-12) == pytest.approx(lift_slope(0), rel=0.04)


def test_a_full_pitch_pull_matches_the_log() -> None:
    """LOG: at full pitch command (203 µs on the servo) near 29 m/s, 7.3 g (6.1 to 9.1) at a CL
    of 0.66 (0.47 to 0.81), the wing still flying."""
    ac = flying(29)
    trim = sum(ac.elevons) / 2 / CHUPITO.elevon_max
    g, cl, alpha = [], [], []
    for i in range(500):
        ac.step(Controls(0.7, trim - PULL_TRAVEL, trim - PULL_TRAVEL), DT, ground_down=1e6)
        if i >= 100:
            q = 0.5 * 1.225 * ac.air.airspeed**2 * CHUPITO.wing.reference.area
            g.append(-ac.specific_force[2] / 9.80665)
            cl.append(g[-1] * CHUPITO.inertia.mass * 9.80665 / q)
            alpha.append(math.degrees(ac.air.alpha))
    assert 6.0 < sorted(g)[len(g) // 2] < 9.1
    assert 0.47 < sorted(cl)[len(cl) // 2] < 0.81
    assert max(alpha) < 20, "pulls hard, short of the stall"


def test_a_steady_pull_takes_the_logs_elevon_a_g() -> None:
    """LOG: moderate pulls take 1.0 to 1.4 deg of elevon a g (at TBS's throws): what a static
    margin over the elevons' power comes to."""
    ac = flying(29)
    trim = sum(ac.elevons) / 2
    pull = (trim - math.radians(3)) / CHUPITO.elevon_max
    g = []
    for i in range(400):
        aileron = -1.5 * to_euler(ac.body.attitude)[0] - 0.1 * ac.body.rates[0]
        ac.step(Controls(0.6, pull + aileron, pull - aileron), DT, ground_down=1e6)
        if i >= 200:
            g.append(-ac.specific_force[2] / 9.80665)
    assert 1.0 < 3 / (sum(g) / len(g) - 1) < 1.5


def test_let_go_in_a_stall_it_recovers() -> None:
    """INAV's full up elevon at low speed, power off: it mushes, stalled, without dropping a wing
    hard; let go, it flies again: no locked deep stall."""
    ac = flying(15)
    for _ in range(3000):
        ac.step(Controls(0.0, -0.6, -0.6), DT, ground_down=1e6)
    assert math.degrees(ac.air.alpha) > 12, "stalled"
    assert abs(math.degrees(ac.euler[0])) < 30, "a gentle stall: no hard wing drop"
    for _ in range(3000):
        ac.step(Controls(0.0), DT, ground_down=1e6)
    assert math.degrees(ac.air.alpha) < 8
    assert ac.air.airspeed > 12


def test_a_tail_slide_flips_it_over() -> None:
    """Straight up, motor off: it stops, slides back tail first, and the reversed flow flips it
    nose down (Selig's tail slide)."""
    ac = Aircraft(CHUPITO)
    ac.phase = Phase.FLYING
    ac.body.position = (0.0, 0.0, -300.0)
    ac.body.attitude = from_euler(0.0, math.radians(89), 0.0)
    ac.body.velocity = (0.0, 0.0, -12.0)
    least = 90.0
    for _ in range(5000):
        ac.step(Controls(0.0), DT, ground_down=1e6)
        least = min(least, math.degrees(ac.euler[1]))
    assert least < -30, "flipped nose down"
    assert ac.air.airspeed > 10 and abs(math.degrees(ac.air.alpha)) < 20, "and flies again"


def test_the_wing_damps_roll_with_no_airspeed() -> None:
    """Hanging still, rolling: each strip meets the air of its own motion (strip theory)."""
    _, moment, _ = forces_and_moments(CHUPITO.wing, (0.0, 0.0, 0.0), (5.0, 0.0, 0.0), (0, 0), 1.225)
    assert moment[0] < -0.01


def test_punching_the_throttle_kicks_it_in_roll() -> None:
    """The airframe takes the motor's torque: spinning the rotor up, more than the prop's drag."""
    ac = flying(20)
    ac.power.omega = 300.0
    ac.step(Controls(1.0), DT, ground_down=1e6)
    kick = abs(ac.power.torque)
    for _ in range(1000):
        ac.step(Controls(1.0), DT, ground_down=1e6)
    assert kick > 1.5 * abs(ac.power.torque)


def test_flying_into_a_roof_is_a_crash_however_gently() -> None:
    """Climbing gently, low, into something taller than it is: not a landing."""
    ac = flying(16)
    b = ac.body
    b.velocity = (b.velocity[0], b.velocity[1], -1.0)
    roof = b.position[2] - 3  # 3 m above it
    ac.step(Controls(0.5), DT, ground_down=roof)
    assert ac.phase == Phase.CRASHED


def _lift(
    alpha: float,
    flow: Flow | None = None,
    dt: float = 0.0,
    ground: tuple[float, tuple[float, float, float]] | None = None,
    speed: float = 12.0,
) -> tuple[float, float, tuple[float, float, float]]:
    """The whole wing's lift and drag coefficients at `alpha`, and its moment, N m."""
    wing = CHUPITO.wing
    v = (speed * math.cos(alpha), 0.0, speed * math.sin(alpha))
    force, moment, _ = forces_and_moments(wing, v, (0, 0, 0), (0, 0), 1.225, flow, dt, ground)
    q = 0.5 * 1.225 * speed**2 * wing.reference.area
    lift = force[0] * math.sin(alpha) - force[2] * math.cos(alpha)
    drag = -force[0] * math.cos(alpha) - force[2] * math.sin(alpha)
    return lift / q, drag / q, moment


def _sweep(degrees: list[float], rate: float, flow: Flow) -> dict[float, float]:
    """Lift coefficients met at `degrees`, swept through at `rate` deg/s."""
    dt, met = 0.002, {}
    for a, b in zip(degrees, degrees[1:], strict=False):
        steps = max(1, round(abs(b - a) / rate / dt))
        for i in range(steps):
            alpha = a + (b - a) * i / steps
            met[round(alpha, 1)] = _lift(math.radians(alpha), flow, dt)[0]
    return met


def test_the_stall_has_hysteresis() -> None:
    """Swept slowly up past the stall and back, the flow reattaches only below the angle it
    separated at (low Reynolds numbers' laminar bubble), and lower down it is the same again."""
    flow = Flow()
    up = _sweep([0.0, 30.0], 2.0, flow)
    down = _sweep([30.0, 0.0], 2.0, flow)
    past = round(math.degrees(SECTION.alpha_stall)) + 5.0  # the whole wing's loop, past its root's
    assert up[past] - down[past] > 0.15
    assert up[10.0] == pytest.approx(down[10.0], abs=0.02)
    assert up[10.0] == pytest.approx(_lift(math.radians(10))[0], abs=0.02), "slowly: static"


def test_a_quick_pull_outruns_the_stall() -> None:
    """Dynamic stall: pitched up fast, the flow separates late and the wing lifts past its static
    maximum for a moment (Lorber and Carta's ramps, 30 to 70 % at these rates)."""
    static = max(_lift(math.radians(a / 10))[0] for a in range(0, 300, 5))
    slow = max(_sweep([0.0, 30.0], 5.0, Flow()).values())
    quick = max(_sweep([0.0, 30.0], 200.0, Flow()).values())
    assert slow == pytest.approx(static, rel=0.05)
    assert 1.3 * static < quick < 1.8 * static


def test_ground_effect_lifts_more_for_less_drag() -> None:
    """McCormick's: at a tenth of the span up, the weaker downwash lifts more at the same angle
    of attack for less drag; a span up, it is gone."""
    alpha, up = math.radians(6), (0.0, 0.0, -1.0)
    cl_free, cd_free, _ = _lift(alpha)
    cl_low, cd_low, _ = _lift(alpha, ground=(0.1 * 0.8, up))
    cl_high, _, _ = _lift(alpha, ground=(0.8, up))
    assert 1.08 * cl_free < cl_low < 1.25 * cl_free
    assert cd_low < cd_free
    assert cl_high == pytest.approx(cl_free, rel=0.01)


def test_ground_effect_rolls_a_low_wing_back_up() -> None:
    """Banked low over the ground, the lower tip feels it most."""
    bank = math.radians(10)
    up = (0.0, -math.sin(bank), -math.cos(bank))  # right wing down: the ground's up, body frame
    _, _, moment = _lift(math.radians(6), ground=(0.1, up))
    assert moment[0] < -0.01, "rolls left, back toward level"


def test_turbulence_is_mil_spec_dryden() -> None:
    """MIL-F-8785C's low-altitude intensities at 100 ft in a 10 m/s wind, and its rotational
    spectra integrated: σ_q 0.215, σ_r 0.193 rad/s at 15 m/s."""
    weather = Weather(speed=10.0, span=0.8, seed=7)
    w_, v_, p_, q_, r_ = [], [], [], [], []
    for _ in range(100_000):
        weather.step(0.01, 100 * FOOT, 15.0, (1.0, 0.0, 0.0, 0.0))
        w_.append(weather.eddies[2])
        v_.append(weather.eddies[1])
        p, q, r = weather.rates
        p_.append(p)
        q_.append(q)
        r_.append(r)

    def sd(x: list[float]) -> float:
        m = sum(x) / len(x)
        return math.sqrt(sum((a - m) ** 2 for a in x) / len(x))

    k = 0.177 + 0.000823 * 100
    assert sd(w_) == pytest.approx(1.0, rel=0.1)
    assert sd(v_) == pytest.approx(1.0 / k**0.4, rel=0.15)
    long_w = 100 * FOOT
    sigma_p = math.sqrt(0.1 * math.pi**2 * (math.pi * long_w / 3.2) ** (1 / 3) / (0.8 * long_w))
    assert sd(p_) == pytest.approx(sigma_p, rel=0.1)
    assert sd(q_) == pytest.approx(0.215, rel=0.1)
    assert sd(r_) == pytest.approx(0.193, rel=0.1)


def _zero_crossings_hz(x: list[float], seconds: float) -> float:
    m = sum(x) / len(x)
    return sum(1 for a, b in zip(x, x[1:], strict=False) if (a - m) * (b - m) < 0) / 2 / seconds


def test_the_wing_feels_the_turbulence_over_its_span() -> None:
    """At the physics' 1 kHz, 8785C's point turbulence crosses its mean several times a second;
    over the span, a wing of 0.8 m feels it as air, not a buzz: under 2 Hz, and nearly as
    strong."""
    weather = Weather(speed=5.0, span=0.8, seed=4)
    point, felt = [], []
    for _ in range(60_000):
        weather.step(0.001, 20.0, 15.0, (1.0, 0.0, 0.0, 0.0))
        point.append(weather.eddies[2])
        felt.append(weather.felt[2])
    assert _zero_crossings_hz(point, 60) > 4
    assert _zero_crossings_hz(felt, 60) < 2
    m = sum(felt) / len(felt)
    sd = math.sqrt(sum((a - m) ** 2 for a in felt) / len(felt))
    assert 0.3 < sd < 0.5, "σ_w 0.5: the span takes off only the smallest eddies"


def test_larger_eddies_are_slower() -> None:
    crossings = []
    for scale in (0.5, 1.0, 4.0):
        weather = Weather(speed=5.0, span=0.8, scale=scale, seed=4)
        w = []
        for _ in range(40_000):
            weather.step(0.005, 20.0, 15.0, (1.0, 0.0, 0.0, 0.0))
            w.append(weather.felt[2])
        crossings.append(_zero_crossings_hz(w, 200))
    assert crossings[0] > crossings[1] > crossings[2]


def test_no_turbulence_is_still_air_at_once() -> None:
    weather = Weather(speed=8.0, span=0.8, seed=1)
    weather.step(0.01, 20.0, 15.0, (1.0, 0.0, 0.0, 0.0))
    assert weather.felt != (0.0, 0.0, 0.0)
    weather.turbulence = 0.0
    weather.step(0.01, 20.0, 15.0, (1.0, 0.0, 0.0, 0.0))
    assert weather.felt == weather.rates == (0.0, 0.0, 0.0)


def test_a_gust_asked_for_comes_once() -> None:
    weather = Weather(speed=5.0, turbulence=0.0, seed=3)
    weather.gust_now(3.0)
    peaks = []
    for _ in range(6_000):  # a minute
        weather.step(0.01, 20 * FOOT, 15.0, (1.0, 0.0, 0.0, 0.0))
        peaks.append(math.hypot(*weather.gust))
    assert 2.5 < max(peaks) <= 3.0
    assert peaks[600:] == [0.0] * len(peaks[600:]), "gusts off: one, and no more"


def test_the_wind_falls_off_toward_the_ground() -> None:
    weather = Weather(speed=8.0, from_deg=270.0, turbulence=0.0)
    assert weather.shear(20 * FOOT) == pytest.approx(1.0)
    assert weather.shear(0.02) == 0.0
    assert 0.6 < weather.shear(1.0) < 0.8
    weather.step(0.01, 20 * FOOT, 15.0, (1.0, 0.0, 0.0, 0.0))
    assert weather.wind == pytest.approx((0.0, 8.0, 0.0), abs=1e-9), "from the west: to the east"


def test_gusts_come_and_go() -> None:
    weather = Weather(speed=5.0, from_deg=0.0, gusts=6.0, turbulence=0.0, seed=3)
    peaks = []
    for _ in range(12_000):
        weather.step(0.01, 20 * FOOT, 15.0, (1.0, 0.0, 0.0, 0.0))
        peaks.append(math.hypot(*weather.gust))
    assert 2.0 < max(peaks) <= 6.0
    assert peaks.count(0.0) > len(peaks) / 3, "and between them, the steady wind"


def test_it_flies_through_rough_air() -> None:
    """A strong wind's turbulence and gusts toss it about; it keeps flying."""
    ac = flying(16)
    ac.weather = Weather(speed=12.0, from_deg=90.0, gusts=8.0, span=0.8, seed=5)
    speeds = []
    for _ in range(20):
        hold_level(ac, 0.5, 0.5)
        speeds.append(ac.air.airspeed)
        assert abs(math.degrees(ac.euler[0])) < 60
    assert ac.phase == Phase.FLYING
    assert max(speeds) - min(speeds) > 1.0, "tossed about"


def test_the_dynamics_add_up() -> None:
    """What the dynamics view shows is what moved the wing: the strips and the pod make the
    aerodynamic force, it, the prop and gravity the net force; level, it pulls one g. In rough
    air too, where the wind is its mean, the turbulence and a gust."""
    ac = flying(20)
    for weather in (None, Weather(speed=6.0, from_deg=90.0, gusts=3.0, span=0.8, seed=2)):
        ac.weather = weather
        hold_level(ac, 0.45, 2.0)
        ac.step(Controls(0.45), DT, ground_down=1e6, detail=True)
        d = Dynamics.of(ac)
        assert len(d.strips) == len(CHUPITO.wing.strips)
        pieces = [sum(s.force[k] for s in d.strips) + d.pod_drag[k] for k in range(3)]
        assert pieces == pytest.approx(d.aero, abs=0.01)
        net = [d.aero[k] + d.prop[k] + d.gravity[k] for k in range(3)]
        assert net == pytest.approx(d.total, abs=0.01)
        moment = [d.aero_moment[k] + d.prop_moment[k] + d.gyroscopic[k] for k in range(3)]
        assert moment == pytest.approx(d.total_moment, abs=0.002)
        wind = [d.wind_mean[k] + d.turbulence[k] + d.gust[k] for k in range(3)]
        assert wind == pytest.approx(d.wind, abs=0.002)
        json.dumps(asdict(d), allow_nan=False)  # as the viewer gets it: no NaN
        if weather is None:
            assert -d.load[2] == pytest.approx(1.0, abs=0.15)
            weight = CHUPITO.inertia.mass * ac.gravity
            lift = d.coefficients[0] * d.dynamic_pressure * CHUPITO.wing.reference.area
            assert lift == pytest.approx(weight, rel=0.15)
            assert all(s.attached > 0.9 and s.seen < s.stall for s in d.strips), "none stalled"
            assert 0 < d.flex < 1 and abs(d.effective[0]) <= abs(d.elevons[0])


def test_the_dynamics_show_its_balance_wake_and_energy() -> None:
    """The neutral point gives the calibrated static margin; trimmed, the centre of pressure
    sits near the centre of mass; the trailing vortices sum to nothing, strongest at the tips;
    the energy height is the height and the speed's, and gliding it falls."""
    ac = flying(25)
    ac.step(Controls(0.4), DT, ground_down=1e6, detail=True)
    d = Dynamics.of(ac)
    assert d.static_margin == pytest.approx(0.031, abs=0.01)
    assert d.centre_of_pressure is not None and abs(d.centre_of_pressure) < 0.01
    strengths = [w[1] for w in d.wake]
    assert sum(strengths) == pytest.approx(0.0, abs=1e-3)
    assert strengths[-1] == pytest.approx(-strengths[0], rel=0.05)
    assert abs(strengths[-1]) == max(abs(g) for g in strengths)
    speed2 = sum(v * v for v in ac.body.velocity)
    assert d.energy_height == pytest.approx(d.height + speed2 / (2 * ac.gravity), abs=0.01)
    for _ in range(500):
        ac.step(Controls(0.0), DT, ground_down=1e6, detail=True)
    assert Dynamics.of(ac).energy_rate < -0.5, "the drag takes its energy"


def test_looking_into_it_changes_nothing() -> None:
    ac, looked = Aircraft(CHUPITO), Aircraft(CHUPITO)
    for a in (ac, looked):
        a.throw()
    for i in range(3000):
        controls = Controls(0.6, -0.1 + 0.2 * math.sin(i / 300), -0.1)
        ac.step(controls, DT, ground_down=1e6)
        looked.step(controls, DT, ground_down=1e6, detail=i % 4 == 0)
    assert looked.body == ac.body
    assert looked.forces.loads.strips and not ac.forces.loads.strips


def test_each_strip_meets_its_own_air() -> None:
    """Level at 20 m/s the air comes at every strip from ahead; hanging still and rolling right,
    the right tip, going down, meets it from below and the left tip from above."""
    wing = CHUPITO.wing
    loads = Loads()
    forces_and_moments(wing, (20.0, 0.0, 0.0), (0, 0, 0), (0, 0), 1.225, loads=loads)
    for load in loads.strips[:-1]:  # the wing's; the fin's is edge on
        assert load.air == pytest.approx((-20, 0, 0), abs=0.5)
    loads = Loads()
    forces_and_moments(wing, (0.0, 0.0, 0.0), (5.0, 0.0, 0.0), (0, 0), 1.225, loads=loads)
    pairs = zip(wing.strips, loads.strips, strict=True)
    tips = [(s.quarter_chord[1], load.air[2]) for s, load in pairs]
    right, left = max(tips), min(tips)
    assert right[1] < -1.5, "the right tip meets air coming up (body z is down)"
    assert left[1] > 1.5
