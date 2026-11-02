"""The 5" quad's physics: SkyFlow-Dynamics' racer5in through our harness (physics/quad).

The golden vectors in tests/fixtures/skyflow are SkyFlow-Dynamics' own (github.com/Synetic-Labs/
SkyFlow-Dynamics, golden/ at the commit pyproject.toml pins, c2f6478): rotorpy_base_no_aero and
rotorpy_aero_default_wind (RotorPy's executed model, with wind), skydreamer_forces (SkyDreamer's
5 inch racer, force side and rotor lag), and checkdata/neurobem_racer.json (racer5in's
identification from the NeuroBEM flight data). They hold the plant as we call it (our parameter
packing, state and input layout, the jitted step) to the references, and racer5in to its source.
"""

import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from drone_duels.physics.core.rigidbody import rotate, to_euler
from drone_duels.physics.quad import esc
from drone_duels.physics.quad.model import CONTACT_HEIGHT, CRASH_SPEED, QuadModel
from drone_duels.physics.quad.plant import (
    FLIP,
    MOTOR_OF_ROTOR,
    ROTORS,
    Airframe,
    Plant,
    airframe,
    ned_quat,
    skyflow_quat,
    yaw_quat,
)
from drone_duels.server.protocol import POSE_FIELDS

FIXTURES = Path(__file__).parent / "fixtures" / "skyflow"
GOLDEN = ["rotorpy_base_no_aero", "rotorpy_aero_default_wind", "skydreamer_forces"]
BLOCKS = {
    "xdot": slice(0, 3),
    "vdot": slice(3, 6),
    "qdot_wxyz": slice(6, 10),
    "wdot": slice(10, 13),
    "Wdot": slice(13, 17),
}
ORIGIN = (43.63093, -79.38945, 77.0)
HZ = 1200
DT = 1 / HZ
PARM = Path(__file__).parents[1] / "firmware" / "ardupilot" / "copter-quad5.parm"


def load(name: str) -> dict[str, Any]:
    return json.loads((FIXTURES / f"{name}.json").read_text())


def case_arrays(case: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    st, inp = case["state"], case["inputs"]
    s = np.concatenate([st["x"], st["v"], st["q_wxyz"], st["w"], st["rotor_speeds"]])
    u = np.concatenate([inp["cmd_rotor_speeds"], inp["v_wind"], inp["F_ext"], inp["tau_ext"]])
    return s, u


def check(got: np.ndarray, expected: np.ndarray, tol: float, what: str) -> None:
    scale = max(1.0, float(np.abs(expected).max()))
    np.testing.assert_allclose(got, expected, rtol=tol, atol=tol * scale, err_msg=what)


@pytest.fixture(scope="module")
def plant() -> Plant:
    return Plant(airframe("racer5in"))


@pytest.fixture(scope="module")
def golden_plants() -> dict[str, Plant]:
    return {name: Plant(Airframe(name, load(name)["params"])) for name in GOLDEN}


# The plant against SkyFlow-Dynamics' golden vectors.


@pytest.mark.parametrize("name", GOLDEN)
def test_statedot_matches_golden(name: str, golden_plants: dict[str, Plant]) -> None:
    doc = load(name)
    plant = golden_plants[name]
    tol = doc.get("tolerance", 1e-9)
    for k, case in enumerate(doc["cases"]):
        s, u = case_arrays(case)
        out = plant.statedot(s, u)
        for block in doc.get("compare", list(BLOCKS)):
            check(
                out[BLOCKS[block]],
                np.asarray(case["expected"][block]),
                tol,
                f"{name} case {k} {block}",
            )


@pytest.mark.parametrize("name", GOLDEN)
def test_jitted_step_follows_golden_derivative(name: str, golden_plants: dict[str, Plant]) -> None:
    """Our per-frame call (exact-exp steps through make_rollout, the IMU after) moves the state
    along the golden derivative: one tiny step is the derivative times the step."""
    doc = load(name)
    plant = golden_plants[name]
    h = 1e-7
    for k, case in enumerate(doc["cases"]):
        s, u = case_arrays(case)
        nxt, _ = plant.advance(s, u, h, 1)
        rate = (nxt - s) / h
        for block in doc.get("compare", list(BLOCKS)):
            expected = np.asarray(case["expected"][block])
            check(
                rate[BLOCKS[block]],
                expected,
                max(1e-4, doc.get("tolerance", 0.0)),
                f"{name} case {k} {block}",
            )


def test_racer5in_is_neurobems_identified_quad() -> None:
    ident = load("neurobem_racer")
    values = airframe("racer5in").values
    fit = ident["force_fit"]
    assert values["mass"] == ident["fixed"]["mass"]
    assert [values["inertia"][i][i] for i in range(3)] == ident["fixed"]["inertia_diag"]
    assert math.hypot(*values["rotor_pos"][0][:2]) == pytest.approx(ident["fixed"]["arm"])
    assert values["ct2"][0] == pytest.approx(fit["c_T"], rel=1e-12)
    assert values["k_d"] == pytest.approx(fit["k_d"], rel=1e-12)
    assert values["c_D"][2] == pytest.approx(fit["c_Dz"], rel=1e-12)
    assert values["k_z"] == pytest.approx(ident["k_z_bem"], rel=1e-12)


def test_hover_imu_reads_one_g_up(plant: Plant) -> None:
    s = Plant.rest_state((0.0, 0.0, -10.0), 0.0)
    s[ROTORS] = plant.frame.hover_speed
    u = Plant.inputs(np.full(4, plant.frame.hover_speed))
    nxt, imu = plant.advance(s, u, DT, 1)
    np.testing.assert_allclose(FLIP * imu[:3], [0.0, 0.0, -plant.frame.grav], atol=1e-9)
    np.testing.assert_allclose(imu[3:], 0.0, atol=1e-12)
    np.testing.assert_allclose(nxt[:13], s[:13], atol=1e-9)


# Frames and the motor map.


def test_frames_ned_and_frd() -> None:
    q = yaw_quat(90.0)
    assert rotate(q, (1.0, 0.0, 0.0)) == pytest.approx((0.0, 1.0, 0.0))  # nose east
    s = Plant.rest_state((1.0, 2.0, -3.0), 90.0)
    np.testing.assert_allclose(s[0:3], [1.0, -2.0, 3.0])  # NWU: north, west, up
    assert ned_quat(skyflow_quat(q)) == pytest.approx(q)
    # SkyFlow's body x is forward in both frames; its y is left, so it points west of a quad
    # facing north... and north of one facing east.
    w, x, y, z = s[6:10]
    left_world = (2 * (x * y - w * z), w * w - x * x + y * y - z * z, 2 * (y * z + w * x))
    assert left_world == pytest.approx((1.0, 0.0, 0.0))  # body left is north: nose east


def test_motor_map_matches_ardupilot_quad_x() -> None:
    """racer5in: FL, FR, RR, RL spinning CW, CCW, CW, CCW. ArduPilot's quad X: motor 1 front
    right CCW, 2 rear left CCW, 3 front left CW, 4 rear right CW."""
    values = airframe("racer5in").values
    ardupilot = {0: ((1, -1), +1), 1: ((-1, 1), +1), 2: ((1, 1), -1), 3: ((-1, -1), -1)}
    for rotor, motor in enumerate(MOTOR_OF_ROTOR):
        (fwd, left), spin = ardupilot[motor]
        x, y, _ = values["rotor_pos"][rotor]
        assert (math.copysign(1, x), math.copysign(1, y)) == (fwd, left)
        assert values["spin"][rotor] == spin


def test_esc_map() -> None:
    frame = airframe("racer5in")
    t = esc.motor_throttles([1000, 1500, 2000, 1250] + [1500] * 12)
    np.testing.assert_allclose(t, [1.0, 0.0, 0.25, 0.5])  # FL is motor 3, FR 1, RR 4, RL 2
    e = esc.Esc(frame)
    full = esc.CELLS * esc.CELL_FULL
    w = e.command(np.array([0.0, 1e-6, 0.5, 1.0]), full)
    assert w[0] == 0.0
    assert w[1] == pytest.approx(frame.rotor_min, rel=0.1)
    assert w[3] == pytest.approx(frame.rotor_max)
    assert w[1] < w[2] < w[3]
    assert e.command(np.ones(4), full * 0.9)[0] == pytest.approx(frame.rotor_max * 0.9)


def test_parameters_come_from_the_plant() -> None:
    """copter-quad5.parm's thrust curve and hover throttle are the ESC map's."""
    params = {}
    for line in PARM.read_text().splitlines():
        words = line.split("#")[0].split()
        if len(words) == 2:
            params[words[0]] = float(words[1])
    frame = airframe("racer5in")
    lo, hi = params["MOT_SPIN_MIN"], params["MOT_SPIN_MAX"]
    expo = esc.fit_expo(frame, lo, hi)
    assert params["MOT_THST_EXPO"] == pytest.approx(expo, abs=0.01)
    pack = esc.Battery()
    pack.step(frame, np.full(4, frame.hover_speed), 0.01)
    hover = esc.hover_thrust(frame, lo, hi, params["MOT_THST_EXPO"], pack.voltage, pack.resting)
    assert params["MOT_THST_HOVER"] == pytest.approx(hover, abs=0.015)
    assert params["MOT_BAT_VOLT_MAX"] == pytest.approx(esc.CELLS * esc.CELL_FULL)
    assert params["BATT_CAPACITY"] == esc.CAPACITY_MAH


# The harness: ArduPilot's outputs in, its frames out.


def airborne(plant: Plant, wind: dict[str, Any] | None = None, up: float = 20.0) -> QuadModel:
    model = QuadModel(plant.frame, ORIGIN, 0.0, wind, plant=plant, seed=1)
    s = Plant.rest_state((0.0, 0.0, -up), 0.0)
    s[ROTORS] = plant.frame.hover_speed
    model.s = s
    model.phase = "flying"
    return model


def hover_pwm(model: QuadModel) -> int:
    return round(esc.PWM_MIN + 1000 * esc.hover_output(model.frame, model.battery.voltage))


def fly(model: QuadModel, pwm: list[int], seconds: float) -> list[Any]:
    return [model.step(DT, pwm + [1000] * 12) for _ in range(round(seconds * HZ))]


@pytest.mark.parametrize(
    ("axis", "more"),
    [
        (0, (1, 2)),  # roll right: the left motors (2 rear left, 3 front left) push harder
        (1, (0, 2)),  # pitch up: the front motors (1, 3)
        (2, (0, 1)),  # yaw right: the CCW motors (1, 2), whose drag turns the frame CW
    ],
)
def test_mixing_turns_the_right_way(plant: Plant, axis: int, more: tuple[int, int]) -> None:
    model = airborne(plant)
    base = hover_pwm(model)
    pwm = [base + (40 if m in more else -40) for m in range(4)]
    out = fly(model, pwm, 0.15)[-1]
    rates = out.gyro
    assert rates[axis] > 0.5
    for other in {0, 1, 2} - {axis}:
        assert abs(rates[other]) < 0.05 * rates[axis]
    if axis < 2:
        assert to_euler(out.quat)[axis] > 0.01


def test_full_throttle_climbs_and_motors_stop_at_1000(plant: Plant) -> None:
    model = airborne(plant)
    out = fly(model, [1900] * 4, 0.5)[-1]
    assert out.velocity[2] < -3.0  # up, NED
    assert out.accel[2] < -2 * plant.frame.grav
    fly(model, [1000] * 4, 0.3)
    assert np.all(model.s[ROTORS] < 1.0)


def test_wind_blows_the_quad_downwind(plant: Plant) -> None:
    from_north = {"speed_ms": 6.0, "from_deg": 0.0, "turbulence": 0.0}
    model = airborne(plant, from_north)
    out = fly(model, [hover_pwm(model)] * 4, 2.0)[-1]
    assert out.velocity[0] < -0.5
    assert abs(out.velocity[1]) < 0.1 * abs(out.velocity[0])
    from_west = {"speed_ms": 6.0, "from_deg": 270.0, "turbulence": 0.0}
    model = airborne(plant, from_west)
    out = fly(model, [hover_pwm(model)] * 4, 2.0)[-1]
    assert out.velocity[1] > 0.5


def test_the_pack_sags_and_drains(plant: Plant) -> None:
    frame = plant.frame
    rest, hover, full = esc.Battery(), esc.Battery(), esc.Battery()
    rest.step(frame, np.zeros(4), 1.0)
    hover.step(frame, np.full(4, frame.hover_speed), 1.0)
    full.step(frame, np.full(4, frame.rotor_max), 1.0)
    assert full.voltage < hover.voltage < rest.voltage <= 25.2
    assert 3.0 < hover.current < 10.0  # a 5 inch hovers on a few amps at 6S
    assert 40.0 < full.current < 80.0
    assert full.used_mah == pytest.approx(full.current / 3.6)


# The ground.


def settle(model: QuadModel, pwm: int, seconds: float) -> list[Any]:
    return fly(model, [pwm] * 4, seconds)


@pytest.mark.parametrize("pwm", [1000, 1020])
def test_at_rest_on_the_ground_nothing_moves(plant: Plant, pwm: int) -> None:
    """Ten seconds still, disarmed or idling: every output constant, as the EKF and the land
    detector need."""
    model = QuadModel(plant.frame, ORIGIN, 270.0, None, plant=plant)
    outs = settle(model, pwm, 10.0)
    assert model.phase == "ground"
    later = outs[HZ:]  # idling, the rotors spin up in the first second
    for field in ("accel", "gyro", "position", "quat", "velocity"):
        values = np.array([getattr(o, field) for o in later])
        assert np.ptp(values, axis=0).max() < 1e-6, field
    last = outs[-1]
    assert last.accel == pytest.approx((0.0, 0.0, -plant.frame.grav), abs=0.01)
    assert last.gyro == (0.0, 0.0, 0.0)
    assert abs(np.dot(last.quat, yaw_quat(270.0))) == pytest.approx(1.0)
    assert -0.03 < last.position[2] < 0.0


def test_at_rest_in_turbulence_barely_moves(plant: Plant) -> None:
    wind = {"speed_ms": 3.0, "from_deg": 250.0, "turbulence": 1.0, "gust_factor": 1.5}
    model = QuadModel(plant.frame, ORIGIN, 0.0, wind, plant=plant, seed=3)
    outs = settle(model, 1000, 10.0)
    accel = np.array([o.accel for o in outs[HZ:]])
    assert np.ptp(accel, axis=0).max() < 0.05
    assert outs[-1].position[:2] == (0.0, 0.0)


def test_takes_off_and_lands_softly(plant: Plant) -> None:
    model = QuadModel(plant.frame, ORIGIN, 0.0, None, plant=plant)
    settle(model, 1000, 0.5)
    fly(model, [1700] * 4, 0.6)
    assert model.phase == "flying"
    # Down at a metre or two a second: below hover.
    outs = []
    while model.phase == "flying" and len(outs) < 20 * HZ:
        outs += fly(model, [hover_pwm(model) - 40] * 4, 0.5)
    outs += fly(model, [1000] * 4, 1.0)
    assert model.phase == "ground"
    touch = next(i for i, o in enumerate(outs) if o.position[2] >= -CONTACT_HEIGHT)
    # What the accelerometer felt over the landing adds up to how the velocity changed: the
    # EKF, integrating it, stops where the quad does.
    window = outs[touch - HZ // 10 : touch + HZ // 2]
    felt = sum(rotate(o.quat, o.accel)[2] + plant.frame.grav for o in window) * DT
    assert felt == pytest.approx(window[-1].velocity[2] - window[0].velocity[2], abs=0.05)
    assert window[0].velocity[2] > 0.3


def test_a_hard_impact_is_a_crash(plant: Plant) -> None:
    model = airborne(plant, up=12.0)
    fly(model, [1000] * 4, 3.0)
    assert model.phase == "crashed"
    assert math.sqrt(2 * plant.frame.grav * 12.0) > CRASH_SPEED
    outs = fly(model, [2000] * 4, 1.0)  # the firmware asks for full power: the motors are dead
    assert model.phase == "crashed"
    assert np.all(model.s[ROTORS] < 1e-6)
    assert outs[-1].velocity == pytest.approx((0.0, 0.0, 0.0), abs=1e-3)


def test_the_map_moves_the_ground(plant: Plant) -> None:
    model = QuadModel(plant.frame, ORIGIN, 0.0, None, plant=plant)
    settle(model, 1000, 0.5)
    model.set_ground(ORIGIN[2] - 1.5)  # the tiles put the pad lower than the location said
    out = settle(model, 1000, 0.5)[-1]
    assert out.position[2] == pytest.approx(1.5 - 0.03 + 0.0155, abs=0.002)
    assert model.phase == "ground"


def test_pose_and_reset(plant: Plant) -> None:
    model = QuadModel(plant.frame, ORIGIN, 90.0, None, plant=plant)
    assert model.pose() is None
    model.step(DT, [1000] * 16)
    model.s[ROTORS] = [10.0, 20.0, 30.0, 40.0]  # FL, FR, RR, RL
    pose = model.pose()
    assert pose is not None and len(pose) == len(POSE_FIELDS)
    fields: dict[str, float] = dict(zip(POSE_FIELDS, pose, strict=True))
    assert (fields["lat"], fields["lon"]) == pytest.approx(ORIGIN[:2])
    assert fields["alt"] == pytest.approx(ORIGIN[2] + 0.0145, abs=1e-3)
    assert (fields["qw"], fields["qz"]) == pytest.approx(yaw_quat(90.0)[::3])
    assert [fields[f"a{i}"] for i in range(4)] == [20.0, 40.0, 10.0, 30.0]  # motors 1-4
    airborne_state = model.s.copy()
    airborne_state[2] = 50.0
    model.s, model.phase = airborne_state, "flying"
    model.reset()
    assert model.phase == "ground" and model.t == 0.0 and model.pose() is None
    assert float(model.s[2]) == pytest.approx(0.0145, abs=1e-3)
