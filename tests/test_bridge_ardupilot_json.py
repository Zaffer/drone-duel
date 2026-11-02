"""ArduPilot SITL's JSON physics backend, the physics' side (bridges/ardupilot_json.py)."""

import json
import re
import socket

import pytest

from drone_duels.bridges.ardupilot_json import (
    MAGIC16,
    MAGIC32,
    SERVO16,
    Fdm,
    JsonBridge,
    decode_servo,
    encode_servo,
    encode_state,
)

FDM = Fdm(
    t=12.5,
    gyro=(0.1, -0.2, 0.3),
    accel=(-0.5, 0.25, -9.81),
    position=(1.5, -2.25, -10.0),
    quat=(0.9, 0.1, -0.2, 0.3),
    velocity=(4.0, -5.0, -0.5),
    battery_v=24.9,
    battery_a=6.5,
)

KEYTABLE = [
    ("", "timestamp", 1),
    ("imu", "gyro", 3),
    ("imu", "accel_body", 3),
    ("", "position", 3),
    ("", "attitude", 3),
    ("", "quaternion", 4),
    ("", "velocity", 3),
    ("", "velocity_wind", 3),
    ("", "airspeed", 1),
    ("rc", "rc_1", 1),
    ("battery", "voltage", 1),
    ("battery", "current", 1),
]
"""SIM_JSON.h's keytable, as much as matters: (section, key, how many numbers)."""


def parse_like_sitl(line: str) -> dict[str, list[float]]:
    """SIM_JSON::parse_sensors: strstr for the section, then for the key after it, then the
    numbers from two characters past the key's end."""
    out: dict[str, list[float]] = {}
    for section, key, count in KEYTABLE:
        p = line.find(section)
        if p < 0:
            continue
        p += len(section) + 1 if section else 1
        q = line.find(key, p if section else 0)
        if q < 0:
            continue
        rest = line[q + len(key) + 2 :]
        numbers = re.findall(r"-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?", rest)[:count]
        out[f"{section}/{key}"] = [float(n) for n in numbers]
    return out


def test_servo_packets_both_sizes() -> None:
    pwm16 = list(range(1000, 1016))
    pkt = decode_servo(encode_servo(400, 7, pwm16))
    assert pkt is not None
    assert (pkt.frame_rate, pkt.frame_count, list(pkt.pwm)) == (400, 7, pwm16)
    pwm32 = list(range(1100, 1132))
    pkt = decode_servo(encode_servo(1200, 2**31, pwm32))
    assert pkt is not None and len(pkt.pwm) == 32 and pkt.frame_count == 2**31
    assert SERVO16.size == 40


def test_bad_servo_packets_are_ignored() -> None:
    good = encode_servo(400, 1, [1500] * 16)
    assert decode_servo(good[:-1]) is None
    assert decode_servo(b"\x00\x00" + good[2:]) is None
    assert decode_servo(MAGIC32.to_bytes(2, "little") + good[2:]) is None
    assert MAGIC16 != MAGIC32


def test_state_line_reads_back_as_sitl_parses_it() -> None:
    data = encode_state(FDM)
    assert data.startswith(b"\n") and data.endswith(b"\n") and data.count(b"\n") == 2
    body = json.loads(data)
    assert set(body) == {"timestamp", "imu", "position", "quaternion", "velocity", "battery"}
    got = parse_like_sitl(data.decode())
    assert got["/timestamp"] == [12.5]
    assert got["imu/gyro"] == list(FDM.gyro)
    assert got["imu/accel_body"] == list(FDM.accel)
    assert got["/position"] == list(FDM.position)
    assert got["/quaternion"] == list(FDM.quat)
    assert got["/velocity"] == list(FDM.velocity)
    assert got["battery/voltage"] == [24.9]
    assert got["battery/current"] == [6.5]
    for absent in ("/attitude", "/velocity_wind", "/airspeed", "rc/rc_1"):
        assert absent not in got


class FakeModel:
    def __init__(self) -> None:
        self.steps: list[tuple[float, tuple[int, ...]]] = []
        self.resets = 0
        self.t = 0.0

    def step(self, dt: float, pwm: tuple[int, ...]) -> Fdm:
        self.t += dt
        self.steps.append((dt, pwm))
        return Fdm(
            self.t,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, -9.81),
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            25.2,
            0.5,
        )

    def reset(self) -> None:
        self.resets += 1
        self.t = 0.0


def test_lockstep_one_step_a_frame_resends_and_restarts() -> None:
    model = FakeModel()
    bridge = JsonBridge(model, 0)
    pwm = [1100] * 4 + [1500] * 12
    first = bridge.handle(encode_servo(1200, 0, pwm))
    assert first is not None and model.steps == [(1 / 1200, tuple(pwm))]
    assert bridge.handle(encode_servo(1200, 0, pwm)) == first  # resent: answered, unstepped
    assert len(model.steps) == 1
    bridge.handle(encode_servo(400, 1, pwm))
    assert model.steps[-1][0] == pytest.approx(1 / 400)
    bridge.handle(encode_servo(400, 5, pwm))  # frames skipped: still one step
    assert len(model.steps) == 3
    bridge.handle(encode_servo(400, 0, pwm))  # SITL restarted
    assert model.resets == 1 and len(model.steps) == 4
    assert bridge.handle(b"junk") is None
    assert bridge.handle(encode_servo(0, 9, pwm)) is None


def test_answers_sitl_over_udp() -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    model = FakeModel()
    bridge = JsonBridge(model, port)
    bridge.start()
    try:
        assert not bridge.connected()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sitl:
            sitl.settimeout(2)
            for count in range(3):
                sitl.sendto(encode_servo(1200, count, [1000] * 16), ("127.0.0.1", port))
                line = json.loads(sitl.recv(4096))
                assert line["timestamp"] == pytest.approx((count + 1) / 1200)
        assert bridge.connected() and bridge.frames == 3
    finally:
        bridge.stop()
