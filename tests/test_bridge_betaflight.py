"""Betaflight SITL's simulator protocol (bridges/betaflight.py), its MSP and its settings
(firmware/betaflight.py), without Betaflight."""

import math
import re
import socket
import struct
import threading
from pathlib import Path

import pytest

from drone_duels.bridges import betaflight as bridge
from drone_duels.bridges.ardupilot_json import Fdm
from drone_duels.firmware import betaflight
from drone_duels.firmware.inav import (  # pyright: ignore[reportPrivateUsage]
    MSP_BOXIDS,
    _crc8_dvb_s2,
)
from drone_duels.physics.core.rigidbody import from_euler, unrotate
from drone_duels.server.protocol import TelemetryMsg
from drone_duels.session.base import FIRMWARE_FILES, FirmwareError, settings_files

G = 9.80665
ATTITUDES = [(0, 0, 0), (30, 0, 0), (0, 20, 0), (0, 0, 90), (-25, 15, 200), (10, -40, 315)]
"""Roll right, pitch nose up, heading, degrees."""


# Betaflight's own harness (src/test/sitl/sitl_harness.py), which drives the default SITL build:
# its packets are the reference. Its model's angles: roll right, pitch nose down, yaw compass.


def harness_quat_from_euler_bf(roll: float, pitch: float, yaw: float) -> bridge.Quat:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(-yaw / 2), math.sin(-yaw / 2)
    return (
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        cy * sp * cr + sy * cp * sr,
        sy * cp * cr - cy * sp * sr,
    )


def harness_packet_quat(q_nwu: bridge.Quat) -> bridge.Quat:
    q = bridge._mul((math.sqrt(0.5), 0.0, 0.0, -math.sqrt(0.5)), q_nwu)  # pyright: ignore[reportPrivateUsage]
    return (q[0], q[1], -q[2], -q[3])


def harness_rotate_inv(q: bridge.Quat, v: tuple[float, float, float]) -> tuple[float, ...]:
    qc = (q[0], -q[1], -q[2], -q[3])
    mul = bridge._mul  # pyright: ignore[reportPrivateUsage]
    p = mul(mul(qc, (0.0, *v)), q)
    return p[1:]


def same_rotation(a: tuple[float, ...], b: tuple[float, ...]) -> bool:
    dot = abs(sum(x * y for x, y in zip(a, b, strict=True)))
    return dot == pytest.approx(1.0, abs=1e-9)


def fdm(
    quat: bridge.Quat = (1.0, 0.0, 0.0, 0.0),
    gyro: tuple[float, float, float] = (0.0, 0.0, 0.0),
    accel: tuple[float, float, float] = (0.0, 0.0, -G),
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0),
    position: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Fdm:
    return Fdm(
        t=0.0,
        gyro=gyro,
        accel=accel,
        position=position,
        quat=quat,
        velocity=velocity,
        battery_v=25.0,
        battery_a=1.0,
    )


PAD = (43.63093, -79.38945)


def decode(packet: bytes) -> dict[str, tuple[float, ...]]:
    v = bridge.FDM.unpack(packet)
    return {
        "t": v[0:1],
        "gyro": v[1:4],
        "acc": v[4:7],
        "quat": v[7:11],
        "vel": v[11:14],
        "pos": v[14:17],
        "pressure": v[17:18],
    }


@pytest.mark.parametrize(("roll", "pitch", "heading"), ATTITUDES)
def test_the_attitude_is_the_harness(roll: float, pitch: float, heading: float) -> None:
    q_ned = from_euler(math.radians(roll), math.radians(pitch), math.radians(heading))
    sent = decode(bridge.encode_state(fdm(quat=q_ned), 1.0, *PAD, 80.0, PAD))["quat"]
    theirs = harness_packet_quat(
        harness_quat_from_euler_bf(math.radians(roll), math.radians(-pitch), math.radians(heading))
    )
    assert same_rotation(sent, theirs)


@pytest.mark.parametrize(("roll", "pitch", "heading"), ATTITUDES)
def test_the_accelerometer_is_the_harness(roll: float, pitch: float, heading: float) -> None:
    """Gravity and an acceleration north-east-up, felt on the body as the harness feels it."""
    q_ned = from_euler(math.radians(roll), math.radians(pitch), math.radians(heading))
    a_ned = (1.5, -2.0, -0.7)
    f_ned = (a_ned[0], a_ned[1], a_ned[2] - G)
    f_frd = unrotate(q_ned, f_ned)
    sent = decode(bridge.encode_state(fdm(quat=q_ned, accel=f_frd), 1.0, *PAD, 80.0, PAD))["acc"]
    q_nwu = harness_quat_from_euler_bf(
        math.radians(roll), math.radians(-pitch), math.radians(heading)
    )
    f_nwu = (f_ned[0], -f_ned[1], -f_ned[2])
    theirs = tuple(-c for c in harness_rotate_inv(q_nwu, f_nwu))
    assert sent == pytest.approx(theirs, abs=1e-9)


def test_level_at_rest_reads_one_g_up() -> None:
    sent = decode(bridge.encode_state(fdm(), 1.0, *PAD, 80.0, PAD))
    assert sent["acc"] == pytest.approx((0.0, 0.0, -G)), "Betaflight negates it: +1 g up"


def test_rates_velocity_and_position() -> None:
    """Gyro: roll right, pitch up, yaw anticlockwise; velocity ENU; lat/lon mirrored about the
    pad, which Betaflight mirrors back about its first fix (taken at the pad)."""
    lat, lon = PAD[0] + 0.001, PAD[1] - 0.002
    sent = decode(bridge.encode_state(
        fdm(gyro=(0.1, 0.2, 0.3), velocity=(4.0, -3.0, -1.0)), 7.5, lat, lon, 95.0, PAD
    ))  # fmt: skip
    assert sent["t"] == (7.5,)
    assert sent["gyro"] == pytest.approx((0.1, 0.2, -0.3))
    assert sent["vel"] == pytest.approx((-3.0, 4.0, 1.0))
    pkt_lon, pkt_lat, alt = sent["pos"]
    first_lon, first_lat = 2 * PAD[1] - PAD[1], 2 * PAD[0] - PAD[0]  # at the pad: the pad
    assert (2 * first_lat - pkt_lat, 2 * first_lon - pkt_lon) == pytest.approx((lat, lon))
    assert alt == 95.0
    assert sent["pressure"][0] == pytest.approx(100184, abs=5)  # ISA at 95 m


def test_motors_come_in_ardupilots_order() -> None:
    # Betaflight's: rear right, front right, rear left, front left.
    data = bridge.encode_motors((1100.0, 1200.0, 1300.0, 1400.0))
    assert len(data) == 68
    # ArduPilot's: front right, rear left, front left, rear right.
    assert bridge.decode_motors(data) == (1200, 1300, 1400, 1100)
    assert bridge.decode_motors(data[:-4]) is None
    assert bridge.decode_motors(bridge.encode_motors((1100.0, 1200.0))) is None


class StepModel:
    def __init__(self) -> None:
        self.steps: list[tuple[float, tuple[int, ...]]] = []

    def step(self, dt: float, pwm: tuple[int, ...]) -> Fdm:
        self.steps.append((dt, pwm))
        return fdm(position=(10.0, -5.0, -3.0))


def test_a_frame_steps_with_betaflights_last_motors() -> None:
    model = StepModel()
    b = bridge.BetaflightBridge(model, 0, 0, (PAD[0], PAD[1], 77.0), frame_hz=500)
    first = decode(b.frame())
    assert model.steps[-1] == (0.002, (1000, 1000, 1000, 1000)), "nothing heard: stopped"
    assert not b.connected()
    b.take(bridge.encode_motors((1500.0, 1600.0, 1700.0, 1800.0)))
    second = decode(b.frame())
    assert b.connected()
    assert model.steps[-1] == (0.002, (1600, 1700, 1800, 1500))
    assert second["t"][0] == pytest.approx(first["t"][0] + 0.002)
    lon, lat, alt = second["pos"]
    assert alt == pytest.approx(80.0)
    assert lat < PAD[0] and lon > PAD[1], "10 m north, 5 m west: mirrored, south and east"


# MSP.


def _reply(command: int, payload: bytes) -> bytes:
    body = struct.pack("<BHH", 0, command, len(payload)) + payload
    return b"$X>" + body + bytes([_crc8_dvb_s2(body)])


class FakeBetaflight:
    """Betaflight's MSP side, answering from `replies` (command → payload)."""

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
                conn.sendall(_reply(command, self.replies[command]))


def status_ex(boxes_on: int, arming: int, extra: bytes = b"") -> bytes:
    """MSP_STATUS_EX as Betaflight writes it (msp/msp.c)."""
    return (
        struct.pack("<HHHIBHBBB", 125, 0, 0x2F, boxes_on & 0xFFFFFFFF, 0, 20, 4, 0, len(extra))
        + extra
        + struct.pack("<BIB", 30, arming, 0)
    )


def test_betaflights_msp_is_what_a_ground_station_sees() -> None:
    # Boxes in Betaflight's order (permanent ids): ARM, ANGLE, HORIZON, ALTHOLD, ..., FAILSAFE.
    boxes = bytes([0, 1, 2, 3, 6, 27])
    fake = FakeBetaflight(
        {
            betaflight.MSP_STATUS_EX: status_ex(0b000011, 0),  # armed, ANGLE
            MSP_BOXIDS: boxes,
            betaflight.MSP_ATTITUDE: struct.pack("<hhh", 125, -50, 271),
            betaflight.MSP_ALTITUDE: struct.pack("<ihh", 4210, -150, 0),
            betaflight.MSP_RAW_GPS: struct.pack(
                "<BBiiHHHHHHB", 1, 12, 436309300, -793894500, 119, 1550, 2710, 100, 0, 0, 0
            ),
            betaflight.MSP_ANALOG: struct.pack("<BHHhH", 0, 0, 0, 0, 0),
            betaflight.MSP_MOTOR: struct.pack("<8H", 1400, 1500, 1600, 1700, *[0] * 4),
        }
    )
    msp = betaflight.BetaflightMsp(fake.port)
    try:
        t = TelemetryMsg.model_validate(msp.telemetry())
    finally:
        msp.close()
    assert t.armed and t.mode == "ANGLE" and t.blockers == []
    assert (t.lat, t.lon, t.alt_amsl) == (pytest.approx(43.63093), pytest.approx(-79.38945), 119)
    assert (t.roll_deg, t.pitch_deg, t.yaw_deg) == (12.5, 5.0, 271.0), "pitch: nose down"
    assert (t.alt_rel, t.climb, t.groundspeed) == (42.1, -1.5, 15.5)
    assert t.battery_v is None, "SITL has no battery"
    assert t.throttle_pct == 55.0 and t.gps_sats == 12


def test_mode_names_and_blockers() -> None:
    boxes = (0, 1, 2, 27, 46)
    assert betaflight.mode_name(set()) == "ACRO"
    on = betaflight._status(status_ex(0b00101, 0), boxes)[0]  # pyright: ignore[reportPrivateUsage]
    assert on == {0, 2} and betaflight.mode_name(on) == "HORIZON"
    assert betaflight.mode_name({0, 1, 27}) == "FAILSAFE"
    # Boxes past the first 32 come in the extra bytes.
    many = tuple(range(40))
    on = betaflight._status(status_ex(1, 0, extra=b"\x80"), many)[0]  # pyright: ignore[reportPrivateUsage]
    assert on == {0, 39}
    rxloss, bootgrace, arm_switch = 1 << 2, 1 << 9, 1 << 29
    blocked = betaflight._status(status_ex(0, rxloss | bootgrace | arm_switch), boxes)[1]  # pyright: ignore[reportPrivateUsage]
    assert blocked == ("no RC link", "booting", "arm switch")


# Settings.


def test_cli_comments_go() -> None:
    settings = "# a comment\n\nset a = 1    # 3, 4\n  aux 0 0 0 1700 2100 0 0\n"
    assert betaflight.commands(settings) == ["set a = 1", "aux 0 0 0 1700 2100 0 0"]


def fake_sitl(tmp_path: Path, output: str) -> Path:
    """A stand-in for SITL's --config run: prints `output` and saves an eeprom."""
    script = tmp_path / "sitl"
    script.write_text(f"#!/bin/sh\nprintf '%s\\n' '{output}'\nhead -c 64 /dev/zero > eeprom.bin\n")
    script.chmod(0o755)
    return script


def test_configure_types_settings_in_once(tmp_path: Path) -> None:
    sitl = fake_sitl(tmp_path, "[CONFIG] Config file processed, EEPROM saved")
    work = tmp_path / "work"
    work.mkdir()
    assert betaflight.configure(sitl, work, "set a = 1\n", 9605, 7760)
    assert (work / "settings.cli").read_text() == "set a = 1\n"
    assert not betaflight.configure(sitl, work, "set a = 1   # the same\n", 9605, 7760)
    assert betaflight.configure(sitl, work, "set a = 2\n", 9605, 7760)


def test_configure_says_what_betaflight_rejected(tmp_path: Path) -> None:
    sitl = fake_sitl(tmp_path, "###ERROR IN set: INVALID NAME: osd_bogus_pos = 1###")
    work = tmp_path / "work"
    work.mkdir()
    with pytest.raises(FirmwareError, match="INVALID NAME: osd_bogus_pos"):
        betaflight.configure(sitl, work, "set osd_bogus_pos = 1\n", 9605, 7760)
    assert not (work / "settings.hash").exists()


@pytest.mark.parametrize("video", ["analog", "digital"])
def test_the_osd_fits_its_grid(video: str) -> None:
    """Every element the layout shows is on the grid Betaflight draws for the video system."""
    files = settings_files([FIRMWARE_FILES / "betaflight" / "quad5.cli"], video)  # pyright: ignore[reportArgumentType]
    assert len(files) == 2
    cols, rows = betaflight.OSD_GRIDS[video][0]  # pyright: ignore[reportArgumentType]
    shown = 0
    for line in betaflight.commands(files[1].read_text()):
        m = re.fullmatch(r"set osd_(\w+)_pos = (\d+)", line)
        if not m:
            continue
        pos = int(m[2])
        x = (pos & 31) | ((pos >> 5) & 32)
        y = (pos >> 5) & 31
        assert pos & 0x0800, f"{m[1]} is not shown"
        assert x < cols and y < rows, f"{m[1]} at {x}, {y} is off the {cols} x {rows} grid"
        shown += 1
    assert shown >= 15


def _stick_factor(throttle: float) -> float:
    """Betaflight's altitude hold, reading the throttle stick (flight/alt_hold_multirotor.c's
    altHoldUpdateTargetAltitude): the share of its climb rate asked, -1 to 1."""
    hover, deadband = betaflight.HOVER_US, betaflight.ALT_HOLD_DEADBAND
    low = hover - deadband * (hover - 1000)
    high = hover + deadband * (2000 - hover)
    if throttle < low:
        return (throttle - 1000) / (low - 1000) - 1
    if throttle > high:
        return (throttle - high) / (2000 - high)
    return 0.0


@pytest.mark.parametrize("climb", [-2.0, -0.7, -0.1, 0.0, 0.1, 0.5, 3.0, 5.0])
def test_the_throttle_asks_for_the_climb(climb: float) -> None:
    us = betaflight.throttle_for(climb)
    assert _stick_factor(us) * betaflight.ALT_HOLD_CLIMB_MS == pytest.approx(climb, abs=0.03)
    assert us > 1050  # never "throttle low", where altitude hold takes no stick


def test_the_pilot_flies_by_the_settings() -> None:
    """The constants the game flies Betaflight's altitude hold by are what quad5.cli sets, and
    its switches are where quad5.cli puts the modes."""
    sets: dict[str, str] = {}
    aux: dict[int, tuple[int, int, int]] = {}
    for line in betaflight.commands((FIRMWARE_FILES / "betaflight" / "quad5.cli").read_text()):
        if m := re.fullmatch(r"set (\w+) = (\S+)", line):
            sets[m[1]] = m[2]
        elif m := re.fullmatch(r"aux \d+ (\d+) (\d+) (\d+) (\d+) 0 0", line):
            aux[int(m[1])] = (int(m[2]), int(m[3]), int(m[4]))
    assert int(sets["ap_hover_throttle"]) == betaflight.HOVER_US
    assert int(sets["alt_hold_deadband"]) / 100 == betaflight.ALT_HOLD_DEADBAND
    assert int(sets["alt_hold_climb_rate"]) / 10 == betaflight.ALT_HOLD_CLIMB_MS

    def on(box: int, channel: int, us: int) -> bool:
        ch, start, end = aux[box]
        return ch == channel and start <= us < end

    arm, angle, althold, poshold = 0, 1, 3, 11  # permanent ids
    assert on(arm, 0, betaflight.ARM_HIGH) and not on(arm, 0, betaflight.IDLE_US[4])
    assert on(angle, 1, betaflight.MODE_ANGLE) and on(angle, 1, betaflight.IDLE_US[5])
    assert on(althold, 1, betaflight.MODE_POSHOLD) and on(poshold, 1, betaflight.MODE_POSHOLD)
    assert not on(althold, 1, betaflight.MODE_ANGLE)
