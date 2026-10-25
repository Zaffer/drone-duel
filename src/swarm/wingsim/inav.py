"""INAV SITL: the process, and its settings over its CLI.

INAV SITL keeps its settings in an eeprom file and serves MSP and its CLI on TCP (UART1 at the
base port, 5760; UART2 at the next). `configure` types settings into the CLI and saves them;
saving restarts INAV, which then connects to the simulator again. A hash beside the eeprom file
says which settings it holds, so unchanged ones are not typed in again. `Msp` reads INAV's state
over UART2: armed or not, and what stops it arming.
"""

import hashlib
import logging
import socket
import struct
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path

log = logging.getLogger(__name__)

CHANMAP = "M01-01,S01-02,S02-03"
"""Motor 1 to the X-Plane throttle, servos 1 and 2 (the elevons) to the first two yokes."""
CLI_PORT = 5760
MSP_PORT = CLI_PORT + 1
PROMPT = b"\n# "

CHUPITO_SETTINGS = """
# The simulator's sensors and receiver. SITL's defaults leave the GPS off.
feature GPS
set gps_provider = FAKE
set acc_hardware = FAKE
set baro_hardware = FAKE
set mag_hardware = FAKE
set pitot_hardware = VIRTUAL
set receiver_type = SIM (SITL)
# Over X-Plane's protocol INAV SITL reads a fixed 16.8 V: a full 4S pack, not the Chupito's 6S.
set bat_cells = 4

# TBS's Chupito INAV CLI suggestions (INAV 8.0.1), less what is the real board's.
feature FW_LAUNCH
set platform_type = AIRPLANE
mmix reset
mmix 0 1.000 0.000 0.000 0.000
smix reset
smix 0 1 0 -80 0 -1
smix 1 1 1 -60 0 -1
smix 2 2 0 -80 0 -1
smix 3 2 1 60 0 -1
set failsafe_procedure = RTH
set small_angle = 180
set airmode_type = STICK_CENTER_ONCE
set nav_wp_radius = 1000
set nav_wp_max_safe_distance = 500
set nav_fw_soaring_motor_stop = ON
set nav_rth_allow_landing = FS_ONLY
set nav_rth_altitude = 12000
set nav_rth_home_altitude = 6500
set nav_fw_bank_angle = 45
set nav_fw_manual_climb_rate = 500
set nav_fw_climb_angle = 45
set nav_fw_dive_angle = 30
set nav_fw_loiter_radius = 15000
set nav_fw_control_smoothness = 2
set nav_fw_launch_max_angle = 180
set nav_fw_launch_motor_delay = 0
set nav_fw_launch_spinup_time = 250
set nav_fw_launch_max_altitude = 15000
set nav_fw_launch_climb_angle = 45
set fw_p_pitch = 15
set fw_i_pitch = 5
set fw_d_pitch = 5
set fw_ff_pitch = 70
set fw_p_roll = 15
set fw_i_roll = 3
set fw_d_roll = 7
set fw_p_yaw = 20
set fw_i_yaw = 0
set fw_ff_yaw = 100
set max_angle_inclination_rll = 550
set dterm_lpf_hz = 10
set fw_turn_assist_pitch_gain = 0.300
set nav_fw_pos_z_p = 35
set nav_fw_pos_xy_p = 70
set rc_expo = 30
set rc_yaw_expo = 30
set roll_rate = 18
set pitch_rate = 12
set yaw_rate = 4
set manual_rc_expo = 60
set throttle_idle = 5.000
set nav_fw_cruise_thr = 1350
set nav_fw_min_thr = 1100
set nav_fw_launch_thr = 2000

# Modes: CH5 high arms; CH6 low ANGLE, middle ACRO (no mode), high MANUAL; CH7 high RTH.
aux 0 0 0 1700 2100
aux 1 1 1 900 1300
aux 2 12 1 1700 2100
aux 3 10 2 1700 2100
"""


def start(
    binary: Path, workdir: Path, sim_port: int, cli_port: int = CLI_PORT
) -> subprocess.Popen[bytes]:
    """INAV SITL flying the simulator on UDP `sim_port`, its eeprom and log in `workdir`. It
    waits for the simulator to answer before it serves its CLI."""
    workdir.mkdir(parents=True, exist_ok=True)
    args = [
        str(binary.resolve()),
        f"--path={workdir.resolve() / 'eeprom.bin'}",
        "--sim=xp",
        "--simip=127.0.0.1",
        f"--simport={sim_port}",
        f"--chanmap={CHANMAP}",
        f"--tcpbaseport={cli_port}",
    ]
    out = (workdir / "inav.log").open("ab")
    log.info("starting %s", " ".join(args))
    return subprocess.Popen(args, cwd=workdir, stdout=out, stderr=subprocess.STDOUT)


def _commands(settings: str) -> list[str]:
    return [
        line.strip() for line in settings.splitlines() if line.strip() and not line.startswith("#")
    ]


def settings_hash(settings: str) -> str:
    return hashlib.sha256("\n".join(_commands(settings)).encode()).hexdigest()[:16]


def configure(workdir: Path, settings: str, cli_port: int = CLI_PORT, timeout: float = 30) -> bool:
    """Type `settings` into INAV's CLI and save, unless the eeprom already holds them. True when
    it saved (and so restarted). Raises on a command INAV rejects."""
    marker = workdir / "settings.hash"
    digest = settings_hash(settings)
    if marker.exists() and marker.read_text().strip() == digest:
        return False
    with _connect(cli_port, timeout) as cli:
        cli.sendall(b"#")
        _read_to_prompt(cli)
        for command in _commands(settings):
            cli.sendall(command.encode() + b"\n")
            reply = _read_to_prompt(cli).decode(errors="replace")
            if "ERROR" in reply or "Invalid" in reply or "Unknown" in reply:
                raise RuntimeError(f"INAV rejected {command!r}: {reply.strip()}")
        cli.sendall(b"save\n")
        time.sleep(1.0)  # INAV saves, then restarts itself
    marker.write_text(digest)
    log.info("INAV configured; it restarts")
    return True


def _connect(port: int, timeout: float) -> socket.socket:
    deadline = time.monotonic() + timeout
    while True:
        try:
            sock = socket.create_connection(("127.0.0.1", port), timeout=5)
            sock.settimeout(5)
            return sock
        except OSError:
            if time.monotonic() > deadline:
                raise
            time.sleep(0.5)


def _read_to_prompt(sock: socket.socket) -> bytes:
    data = b""
    while not data.endswith(PROMPT) and not data.endswith(b"\r\n# "):
        chunk = sock.recv(4096)
        if not chunk:
            break
        data += chunk
    return data


MSP_ATTITUDE = 108
MSP2_INAV_STATUS = 0x2000
ARMED = 1 << 2
ARMING_BLOCKERS = {
    6: "geozone",
    7: "failsafe",
    8: "not level",
    9: "calibrating",
    10: "overloaded",
    11: "navigation unsafe",
    12: "compass",
    13: "accelerometer",
    14: "arm switch",
    15: "hardware failure",
    16: "failsafe mode",
    18: "no RC link",
    19: "throttle",
    20: "CLI",
    21: "CMS menu",
    22: "OSD menu",
    23: "roll/pitch not centred",
    24: "servo autotrim",
    25: "out of memory",
    26: "invalid setting",
    27: "PWM output",
    28: "no prearm",
    29: "DShot beeper",
    30: "landing detected",
}
"""INAV's armingFlags bits (fc/runtime_config.h) that block arming."""


@dataclass(frozen=True)
class Status:
    armed: bool
    blockers: tuple[str, ...]


def _crc8_dvb_s2(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class Msp:
    """An MSP v2 link to INAV over TCP, one request at a time from any thread."""

    def __init__(self, port: int = MSP_PORT) -> None:
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=2)
        self._lock = threading.Lock()

    def close(self) -> None:
        self.sock.close()

    def request(self, command: int, payload: bytes = b"") -> bytes:
        with self._lock:
            return self._request(command, payload)

    def _request(self, command: int, payload: bytes) -> bytes:
        body = struct.pack("<BHH", 0, command, len(payload)) + payload
        self.sock.sendall(b"$X<" + body + bytes([_crc8_dvb_s2(body)]))
        head = self._read(8)  # $X> flag command size
        if head[:3] != b"$X>":
            raise OSError(f"MSP: unexpected reply {head!r}")
        _, cmd, size = struct.unpack("<BHH", head[3:])
        data = self._read(size + 1)
        if cmd != command or _crc8_dvb_s2(head[3:] + data[:-1]) != data[-1]:
            raise OSError("MSP: bad reply")
        return data[:-1]

    def _read(self, n: int) -> bytes:
        data = b""
        while len(data) < n:
            chunk = self.sock.recv(n - len(data))
            if not chunk:
                raise OSError("MSP: closed")
            data += chunk
        return data

    def status(self) -> Status:
        reply = self.request(MSP2_INAV_STATUS)
        (flags,) = struct.unpack_from("<I", reply, 9)
        blockers = tuple(name for bit, name in ARMING_BLOCKERS.items() if flags & (1 << bit))
        return Status(bool(flags & ARMED), blockers)

    def attitude(self) -> tuple[float, float, float]:
        """INAV's estimate: roll (right wing down), pitch (nose up) and heading, degrees."""
        roll, pitch, heading = struct.unpack_from("<hhh", self.request(MSP_ATTITUDE))
        return roll / 10, -pitch / 10, float(heading)
