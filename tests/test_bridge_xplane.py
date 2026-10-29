"""The wing's link to INAV SITL: X-Plane's protocol, the server playing it, INAV's own process,
its MSP and its OSD, and INAV flying it."""

import math
import signal
import socket
import struct
import subprocess
import threading
import time
from pathlib import Path

import pytest

from drone_duels import ports
from drone_duels.bridges import xplane_packets as xplane
from drone_duels.bridges.xplane import THROTTLE, YOKES, XPlaneServer
from drone_duels.firmware import inav
from drone_duels.physics.core.rigidbody import from_euler
from drone_duels.physics.wing.aircraft import Aircraft, Phase
from drone_duels.physics.wing.airframe import CHUPITO
from drone_duels.physics.wing.ground import MapGround
from drone_duels.server.protocol import OsdMsg, TelemetryMsg
from drone_duels.session.base import FIRMWARE_FILES
from drone_duels.video.osd import MSP_DISPLAYPORT

ORIGIN = (45.319287, -75.758596, 77.0)
CHUPITO_CLI = FIRMWARE_FILES / "inav" / "chupito.cli"
SLOT = ports.slot(2)
"""The SITL tests' slot: clear of slot 0, which ArduPilot's tests use."""


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
        assert server.heard == 0, "nothing from INAV yet"
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
        assert server.heard > 0
        assert server.time > 0.1, "the model's own clock, one period a tick"
    finally:
        client.close()
        server.stop()


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


# INAV's MSP and OSD, against stand-ins.


def _displayport(payload: bytes) -> bytes:
    """An MSP v1 reply carrying `payload` as MSP_DISPLAYPORT."""
    checksum = len(payload) ^ MSP_DISPLAYPORT
    for b in payload:
        checksum ^= b
    return b"$M>" + bytes([len(payload), MSP_DISPLAYPORT]) + payload + bytes([checksum])


def test_the_air_unit_polls_inav_and_keeps_its_osd() -> None:
    """INAV draws its OSD only to an air unit polling it: this one polls, then keeps the screen
    INAV drew, on Betaflight-compatible HD's 53 x 20, and hands it on as the page's OsdMsg."""
    listener = socket.create_server(("127.0.0.1", 0))
    drawn: list[dict[str, object]] = []
    unit = inav.AirUnit(listener.getsockname()[1], lambda s: drawn.append(inav.osd_message(s)))
    unit.start()
    try:
        inav_side, _ = listener.accept()
        inav_side.settimeout(2)
        assert inav_side.recv(64).startswith(b"$M<"), "it asks first, as an air unit does"
        screen = (
            _displayport(bytes([5, 0, 3]))  # options: Betaflight-compatible HD
            + _displayport(bytes([2]))  # clear
            + _displayport(bytes([3, 17, 6, 0]) + b"ANGL")
            + _displayport(bytes([3, 1, 6, 0, 0x96]) + b"0.00")
            + _displayport(bytes([4]))  # draw
        )
        inav_side.sendall(screen)
        deadline = time.monotonic() + 2
        while not drawn and time.monotonic() < deadline:
            time.sleep(0.01)
        osd = unit.screen
        assert osd is not None
        assert (osd.cols, osd.rows) == (53, 20)
        assert osd.text[17][6:10] == "ANGL"
        assert osd.text[1][6:11] == "\x960.00"
        msg = OsdMsg.model_validate(drawn[0])
        assert (msg.font, msg.cols, msg.rows) == ("betaflight", 53, 20)
        assert len(msg.codes) == 53 * 20
        assert bytes(msg.codes[17 * 53 + 6 : 17 * 53 + 10]) == b"ANGL"
        # Drawn again and again, faster than the page needs: no more than 20 a second.
        t0 = time.monotonic()
        for _ in range(50):
            inav_side.sendall(_displayport(bytes([4])))
            time.sleep(0.004)
        time.sleep(0.15)
        later = len(drawn) - 1
        assert 1 <= later <= (time.monotonic() - t0) * 20 + 1
        inav_side.close()
    finally:
        unit.stop()
        listener.close()


def _msp_reply(command: int, payload: bytes) -> bytes:
    body = struct.pack("<BHH", 0, command, len(payload)) + payload
    return b"$X>" + body + bytes([inav._crc8_dvb_s2(body)])  # pyright: ignore[reportPrivateUsage]


class FakeInav:
    """INAV's MSP side, answering from `replies` (command → payload)."""

    def __init__(self, replies: dict[int, bytes]) -> None:
        self.replies = replies
        self.listener = socket.create_server(("127.0.0.1", 0))
        self.port = self.listener.getsockname()[1]
        threading.Thread(target=self._serve, daemon=True).start()

    def _serve(self) -> None:
        conn, _ = self.listener.accept()
        with conn:
            while True:
                head = conn.recv(8)
                if len(head) < 8:
                    return
                _, command, size = struct.unpack("<BHH", head[3:])
                conn.recv(size + 1)
                conn.sendall(_msp_reply(command, self.replies[command]))


def _status(arming: int, modes: int) -> bytes:
    return struct.pack("<HHHHBI", 1000, 0, 0, 5, 0, arming) + modes.to_bytes(8, "little") + b"\0"


def test_inavs_msp_is_what_a_ground_station_sees() -> None:
    # Boxes in INAV's order: ARM, ANGLE, MANUAL, NAV RTH, NAV LAUNCH (permanent ids).
    boxes = bytes([0, 1, 12, 10, 36])
    fake = FakeInav(
        {
            inav.MSP2_INAV_STATUS: _status(inav.ARMED, 0b10011),  # armed, ANGLE, LAUNCH
            inav.MSP_BOXIDS: boxes,
            inav.MSP_ATTITUDE: struct.pack("<hhh", 125, -50, 271),
            inav.MSP_ALTITUDE: struct.pack("<ihi", 4210, -150, 0),
            inav.MSP_RAW_GPS: struct.pack(
                "<BBiihHHH", 2, 16, 436309300, -793894500, 119, 1550, 2710, 100
            ),
            inav.MSP2_INAV_ANALOG: struct.pack("<BHHIIIIBH", 0x40, 1612, 820, 0, 120, 0, 0, 87, 0),
            inav.MSP2_INAV_AIR_SPEED: struct.pack("<I", 1720),
            inav.MSP_MOTOR: struct.pack("<8H", 1650, *[0] * 7),
        }
    )
    msp = inav.Msp(fake.port)
    try:
        t = TelemetryMsg.model_validate(msp.telemetry())
    finally:
        msp.close()
    assert t.armed and t.mode == "LAUNCH" and t.blockers == []
    assert (t.lat, t.lon, t.alt_amsl) == (pytest.approx(43.63093), pytest.approx(-79.38945), 119)
    assert (t.roll_deg, t.pitch_deg, t.yaw_deg) == (12.5, 5.0, 271.0), "INAV's pitch: nose down"
    assert (t.alt_rel, t.climb) == (42.1, -1.5)
    assert (t.groundspeed, t.airspeed) == (15.5, 17.2)
    assert (t.battery_v, t.battery_pct, t.gps_sats) == (16.12, 87.0, 16)
    assert t.throttle_pct == 65.0


def test_inavs_mode_names() -> None:
    boxes = (0, 1, 12, 10, 27)
    assert inav.mode_name(0b00001, boxes) == "ACRO", "armed, no mode: ACRO"
    assert inav.mode_name(0b00011, boxes) == "ANGLE"
    assert inav.mode_name(0b00101, boxes) == "MANUAL"
    assert inav.mode_name(0b01011, boxes) == "RTH"
    assert inav.mode_name(0b11011, boxes) == "FAILSAFE"


def _layout(path: Path) -> dict[int, tuple[int, int, str]]:
    """osd_layout 0 ITEM COL ROW V|H, by item."""
    out = {}
    for line in path.read_text().splitlines():
        if line.startswith("osd_layout 0 "):
            _, _, item, col, row, shown = line.split()
            out[int(item)] = (int(col), int(row), shown)
    return out


def test_inav_gets_tbss_osd() -> None:
    """TBS's OSD layout, for a DJI air unit on UART3 in Betaflight-compatible HD, on digital
    video; on analog, the same elements re-fitted to Betaflight-compatible SD as NTSC shows it."""
    common = CHUPITO_CLI.read_text().splitlines()
    assert not any(line.startswith(("osd_", "set osd_", "serial 2")) for line in common)
    # INAV 9.1 SITL's quirks: no GPS blocks arming; no battery kills it on disarming.
    assert "feature GPS" in common and "set vbat_meter_type = FAKE" in common
    assert "set failsafe_procedure = RTH" in common, "a lost link brings it home"
    hd, sd = (CHUPITO_CLI.with_name(f"chupito.{v}.cli") for v in ("digital", "analog"))
    files = inav.settings_files([CHUPITO_CLI], "analog")
    assert files == [CHUPITO_CLI, sd]
    assert inav.settings_files([CHUPITO_CLI], "digital") == [CHUPITO_CLI, hd]
    for path, system in ((hd, "BFHDCOMPAT"), (sd, "BF43COMPAT")):
        lines = path.read_text().splitlines()
        assert f"set osd_video_system = {system}" in lines
        assert "serial 2 33554432 115200 115200 0 115200" in lines  # MSP DisplayPort, UART3
    digital, analog = _layout(hd), _layout(sd)
    assert len(digital) == 27
    assert {i: v[2] for i, v in analog.items()} == {i: v[2] for i, v in digital.items()}
    widest = {30: 28}  # the messages
    for item, (col, row, shown) in analog.items():
        assert 0 <= row < 13 and 0 <= col, f"item {item} off NTSC's 13 rows"
        if shown == "V":
            assert col + widest.get(item, 6) <= 30, f"item {item} runs off 30 columns"
    shown = {(c, r) for c, r, v in analog.values() if v == "V"}
    assert len(shown) == sum(v == "V" for _, _, v in analog.values()), "none on another"


def test_the_analog_osd_is_ntscs_13_rows() -> None:
    """On analog video, INAV draws Betaflight-compatible SD (30 x 16); the picture shows 13."""
    listener = socket.create_server(("127.0.0.1", 0))
    drawn: list[dict[str, object]] = []
    unit = inav.AirUnit(
        listener.getsockname()[1], lambda s: drawn.append(inav.osd_message(s)), video="analog"
    )
    unit.start()
    try:
        inav_side, _ = listener.accept()
        inav_side.sendall(
            _displayport(bytes([5, 0, 0]))  # options: SD_3016
            + _displayport(bytes([2]))
            + _displayport(bytes([3, 11, 1, 0]) + b"ANGL")
            + _displayport(bytes([3, 14, 1, 0]) + b"GONE")  # below NTSC's picture
            + _displayport(bytes([4]))
        )
        deadline = time.monotonic() + 2
        while not drawn and time.monotonic() < deadline:
            time.sleep(0.01)
        msg = OsdMsg.model_validate(drawn[0])
        assert (msg.cols, msg.rows, len(msg.codes)) == (30, 13, 30 * 13)
        assert bytes(msg.codes[11 * 30 + 1 : 11 * 30 + 5]) == b"ANGL"
        assert b"GONE" not in bytes(msg.codes)
        inav_side.close()
    finally:
        unit.stop()
        listener.close()


def test_inav_ignores_a_client_hanging_up(tmp_path: Path) -> None:
    """SITL writes its TCP UARTs with a bare send(): it must run with SIGPIPE ignored, or a
    client hanging up as it writes kills it without a word."""
    binary = tmp_path / "inav_SITL"
    binary.write_text("#!/bin/sh\ngrep SigIgn /proc/$$/status\n")
    binary.chmod(0o755)
    proc = inav.start(binary, tmp_path, 49190, 6990)
    proc.wait(5)
    ignored = int((tmp_path / "inav.log").read_text().split()[-1], 16)
    assert ignored & 1 << (signal.SIGPIPE - 1)
    assert inav.ended(proc) == "exit code 0"


def test_how_inav_ended() -> None:
    killed = subprocess.Popen(["sh", "-c", "kill -PIPE $$"])
    failed = subprocess.Popen(["sh", "-c", "exit 1"])
    killed.wait(5)
    failed.wait(5)
    assert inav.ended(killed) == "killed by SIGPIPE"
    assert inav.ended(failed) == "exit code 1"


def test_inav_is_stopped_with_sigint() -> None:
    """INAV SITL handles only SIGINT; one that will not go is killed."""
    proc = subprocess.Popen(["sh", "-c", "trap 'exit 3' INT; while :; do sleep 0.05; done"])
    time.sleep(0.2)
    inav.interrupt(proc)
    assert inav.ended(proc) == "exit code 3"
    stubborn = subprocess.Popen(["sh", "-c", "trap '' INT; while :; do sleep 0.05; done"])
    time.sleep(0.2)
    inav.interrupt(stubborn, timeout=0.5)
    assert inav.ended(stubborn) == "killed by SIGKILL"


# INAV itself.


@pytest.mark.sitl
@pytest.mark.skipif(not inav.BINARY.exists(), reason="no INAV SITL: scripts/setup.sh")
def test_inav_arms_launches_and_flies_it(tmp_path: Path) -> None:
    """INAV SITL with the Chupito's settings: arm, raise the throttle, throw; INAV's launch mode
    runs the motor and climbs; then ANGLE mode holds it level."""
    sim_port, tcp_base = SLOT.inav_xplane, SLOT.inav_tcp_base
    rc = [1500.0, 1500.0, 1000.0, 1500.0, 1000.0, 1000.0, 1000.0, 1000.0]
    ac = Aircraft(CHUPITO)
    server = XPlaneServer(ac, ORIGIN, rc=lambda: rc, port=sim_port)
    server.start()
    proc = inav.start(inav.BINARY, tmp_path, sim_port, tcp_base)
    msp = None
    try:
        deadline = time.monotonic() + 20
        while server.packets == 0:
            assert time.monotonic() < deadline, "INAV did not connect"
            time.sleep(0.1)
        assert inav.configure(tmp_path, CHUPITO_CLI.read_text(), tcp_base)
        assert not inav.configure(tmp_path, CHUPITO_CLI.read_text(), tcp_base), "held already"
        time.sleep(2)
        msp = inav.Msp(tcp_base + 1)
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
        inav.interrupt(proc)
        server.stop()
