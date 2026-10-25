"""swarm.wingsim's link to INAV SITL: X-Plane's protocol, the server, and INAV itself flying it."""

import math
import socket
import time
from pathlib import Path

import pytest

from swarm.wingrun import MapGround, WingView
from swarm.wingsim import inav, xplane
from swarm.wingsim.aircraft import Aircraft, Phase
from swarm.wingsim.airframe import CHUPITO
from swarm.wingsim.rigidbody import from_euler
from swarm.wingsim.server import THROTTLE, YOKES, XPlaneServer

ORIGIN = (45.319287, -75.758596, 77.0)
INAV_BINARY = Path(__file__).resolve().parents[1] / "external" / "inav" / "inav_SITL"


def test_protocol_round_trip() -> None:
    sub = xplane.parse(xplane.request_packet(7, 100, "sim/flightmodel/position/phi"))
    assert sub == xplane.Subscribe(7, 100, "sim/flightmodel/position/phi")
    assert len(xplane.request_packet(7, 100, "x")) == 413, "INAV's RREF request size"
    write = xplane.parse(xplane.write_packet("sim/joystick/yoke_roll_ratio", -0.25))
    assert write == xplane.Write("sim/joystick/yoke_roll_ratio", -0.25)
    assert len(xplane.write_packet("x", 0)) == 509, "INAV's DREF write size"
    packet = xplane.values_packet({1: 2.5, 9: -1.0})
    assert packet[:5] == b"RREF,"
    assert [xplane.PAIR.unpack_from(packet, 5 + 8 * i) for i in range(2)] == [(1, 2.5), (9, -1.0)]
    assert xplane.parse(b"junk") is None


def test_readings_are_in_inavs_conventions() -> None:
    rc = [1700.0, 1400.0, 1250.0, 1500.0, 2000.0, 1000.0, 1500.0, 1000.0]
    ac = Aircraft(CHUPITO)
    server = XPlaneServer(ac, ORIGIN, rc=lambda: rc)
    # Flying north-east, banked right and nose up, climbing.
    ac.phase = Phase.FLYING
    ac.body.position = (100.0, 50.0, -30.0)
    ac.body.velocity = (15.0, 15.0, -2.0)
    ac.body.attitude = from_euler(math.radians(20), math.radians(5), math.radians(45))
    ac.body.rates = (0.1, -0.2, 0.3)
    ac.specific_force = (1.0, 0.5, -9.80665)
    r = server.readings()
    assert r["sim/flightmodel/position/latitude"] > ORIGIN[0]
    assert r["sim/flightmodel/position/longitude"] > ORIGIN[1]
    assert r["sim/flightmodel/position/elevation"] == pytest.approx(107.0)
    assert r["sim/flightmodel/position/y_agl"] == pytest.approx(30.0)
    assert r["sim/flightmodel/position/phi"] == pytest.approx(20.0)
    assert r["sim/flightmodel/position/theta"] == pytest.approx(5.0)
    assert r["sim/flightmodel/position/psi"] == pytest.approx(45.0)
    assert r["sim/flightmodel/position/hpath"] == pytest.approx(45.0)
    assert r["sim/flightmodel/position/groundspeed"] == pytest.approx(math.hypot(15, 15))
    assert r["sim/flightmodel/position/local_vy"] == pytest.approx(2.0), "X-Plane's y is up"
    # INAV's accelerometer: forward = -g_axil, up = g_nrml; 1 g up in level flight.
    assert r["sim/flightmodel/forces/g_nrml"] == pytest.approx(1.0)
    assert r["sim/flightmodel/forces/g_axil"] == pytest.approx(-1.0 / 9.80665)
    assert r["sim/flightmodel/position/Q"] == pytest.approx(math.degrees(-0.2))
    assert 29.5 < r["sim/weather/barometer_current_inhg"] < 30.0
    # The RC as INAV's joystick: roll and pitch -1..1, throttle and switches 0..1.
    assert r["sim/joystick/joy_mapped_axis_value[2]"] == pytest.approx(0.4)
    assert r["sim/joystick/joy_mapped_axis_value[1]"] == pytest.approx(-0.2)
    assert r["sim/joystick/joy_mapped_axis_value[57]"] == pytest.approx(0.25)
    assert r["sim/joystick/joy_mapped_axis_value[58]"] == pytest.approx(1.0)
    assert r["sim/joystick/joy_mapped_axis_value[60]"] == pytest.approx(0.5)


def test_the_server_answers_inav_and_takes_its_outputs() -> None:
    ac = Aircraft(CHUPITO)
    server = XPlaneServer(ac, ORIGIN, rc=lambda: [1500.0] * 8, port=49170)
    server.start()
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.settimeout(2)
    try:
        client.sendto(
            xplane.request_packet(3, 100, "sim/flightmodel/position/elevation"),
            ("127.0.0.1", 49170),
        )
        # Servo 1 (left elevon) at -0.5: trailing edge down (TBS's mirrored servos).
        client.sendto(xplane.write_packet(YOKES[0], -0.5), ("127.0.0.1", 49170))
        client.sendto(xplane.write_packet(YOKES[1], -0.5), ("127.0.0.1", 49170))
        client.sendto(xplane.write_packet(THROTTLE, 0.3), ("127.0.0.1", 49170))
        packet, _ = client.recvfrom(1024)
        index, value = xplane.PAIR.unpack_from(packet, 5)
        assert (index, value) == (3, pytest.approx(ORIGIN[2] + CHUPITO.launch.height))
        time.sleep(0.2)
        left, right = ac.elevons
        assert left == pytest.approx(0.5 * CHUPITO.elevon_max)
        assert right == pytest.approx(-0.5 * CHUPITO.elevon_max)
        assert ac.power.omega > 0, "the motor spins at 30 %"
    finally:
        client.close()
        server.stop()


@pytest.mark.skipif(not INAV_BINARY.exists(), reason="no INAV SITL: scripts/setup.sh")
def test_inav_arms_launches_and_flies_it(tmp_path: Path) -> None:
    """INAV SITL with the Chupito's settings: arm, raise the throttle, throw; INAV's launch mode
    runs the motor and climbs; then ANGLE mode holds it level."""
    sim_port, cli_port = 49180, 5980
    rc = [1500.0, 1500.0, 1000.0, 1500.0, 1000.0, 1000.0, 1000.0, 1000.0]
    ac = Aircraft(CHUPITO)
    server = XPlaneServer(ac, ORIGIN, rc=lambda: rc, port=sim_port)
    server.start()
    proc = inav.start(INAV_BINARY, tmp_path, sim_port, cli_port)
    msp = None
    try:
        deadline = time.monotonic() + 20
        while server.packets == 0:
            assert time.monotonic() < deadline, "INAV did not connect"
            time.sleep(0.1)
        assert inav.configure(tmp_path, inav.CHUPITO_SETTINGS, cli_port)
        time.sleep(2)
        msp = inav.Msp(cli_port + 1)
        deadline = time.monotonic() + 20
        while msp.status().blockers:
            assert time.monotonic() < deadline, f"cannot arm: {msp.status().blockers}"
            time.sleep(0.5)
        rc[4] = 2000.0
        time.sleep(1)
        assert msp.status().armed
        rc[2] = 1800.0
        time.sleep(1)
        server.throw()
        time.sleep(6)
        snap = server.snapshot()
        assert snap.phase == "flying"
        assert snap.agl > 40, "INAV's launch climbs out at full throttle"
        rc[2] = 1600.0
        rc[1] = 1300.0  # stick back: launch mode ends; ANGLE holds the nose up
        time.sleep(3)
        assert server.snapshot().pitch > 5
        rc[1] = 1500.0
        time.sleep(3)
        snap = server.snapshot()
        assert abs(snap.roll) < 5 and abs(snap.pitch) < 5, "ANGLE levels it"
    finally:
        if msp:
            msp.close()
        proc.terminate()
        proc.wait(5)
        server.stop()


def test_the_viewer_sees_the_wing_and_gives_it_the_maps_ground() -> None:
    ac = Aircraft(CHUPITO)
    ground = MapGround(ORIGIN)
    server = XPlaneServer(ac, ORIGIN, rc=lambda: [1500.0] * 8, ground=ground)
    view = WingView(server, ground)
    assert view.origin == ORIGIN
    (tick, frame), *_ = view.latest().values()
    assert frame.blockers == ("starting",)
    assert frame.wing.agl == pytest.approx(CHUPITO.launch.height)
    view.status = inav.Status(armed=True, blockers=())
    view.ground(ORIGIN[0], ORIGIN[1], ORIGIN[2] + 10)  # a roof under it
    time.sleep(0.05)
    (later, frame), *_ = view.latest().values()
    assert later != tick and frame.armed
    assert frame.wing.agl == pytest.approx(CHUPITO.launch.height - 10)
    server.readings()  # as sent to INAV
    time.sleep(0.05)
    (_, frame), *_ = view.latest().values()
    assert frame.rc == (1500.0,) * 8
    view.reset()  # back in the hand: above the roof
    server._run_commands()  # pyright: ignore[reportPrivateUsage]
    assert server.snapshot().agl == pytest.approx(CHUPITO.launch.height)


def test_the_maps_ground_is_the_report_nearest_where_the_wing_is() -> None:
    ground = MapGround(ORIGIN)
    assert ground(ORIGIN[0], ORIGIN[1]) == ORIGIN[2], "flat at the start until a page reports"
    ground.report(ORIGIN[0], ORIGIN[1], 76.0)  # the start
    ground.report(ORIGIN[0] + 0.001, ORIGIN[1], 90.0)  # a roof 111 m north
    ground.report(ORIGIN[0] + 0.002, ORIGIN[1], 80.0)  # sampled ahead, the wing not there yet
    assert ground(ORIGIN[0] + 0.001, ORIGIN[1]) == 90.0, "the report from where the wing is"
    assert ground(ORIGIN[0] + 0.005, ORIGIN[1]) == 80.0, "nothing near: the latest"
    assert ground(ORIGIN[0], ORIGIN[1]) == 76.0, "the start keeps its own"


def test_back_in_the_hand_is_back_at_the_start() -> None:
    ac = Aircraft(CHUPITO)
    server = XPlaneServer(ac, ORIGIN, rc=lambda: [1500.0] * 8)
    ac.phase = Phase.CRASHED
    ac.body.position = (250.0, -40.0, 5.0)
    server.reset()
    server._run_commands()  # pyright: ignore[reportPrivateUsage]
    assert ac.phase == Phase.HAND
    assert ac.body.position == pytest.approx((0.0, 0.0, -CHUPITO.launch.height))
