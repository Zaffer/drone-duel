"""INAV SITL: the process, its settings over its CLI, MSP, its OSD, and the session's Firmware.

INAV SITL plays against a simulator over X-Plane's protocol (the physics, drone_duels.physics.
wing.backend), keeps its settings in an eeprom file, and serves TCP UARTs from a base port
(ports.SlotPorts.inav_tcp_base, clear of ArduPilot's 5760): UART1 the CLI and MSP, UART2 (+1)
MSP, UART3 (+2) the OSD over MSP DisplayPort, as TBS sets it for the DJI air unit.

`configure` types settings (firmware/inav/*.cli, each with its video system's beside it, which
sets the OSD's grid: `settings_files`) into the CLI and saves them; saving restarts
INAV (it execs itself again), which then connects to the simulator again. A hash beside the
eeprom file says which settings it holds, so unchanged ones are not typed in again. `Msp` reads
INAV's state over UART2, as a ground station would; `AirUnit` reads its OSD as the DJI air unit
does. `Inav` puts them together for the session (DESIGN.md §6.1, §6.4).

INAV 9.1 SITL's quirks, met here: its defaults leave the GPS off, which blocks arming as
"navigation unsafe" (the settings turn it on); just booted, it shows no arming blockers for a
moment before calibrating starts, so readiness needs two clear polls in a row; it sends its OSD
only once something polls it over MSP, as a DJI unit does; it handles only SIGINT; and it writes
its TCP UARTs with a bare send(), so it runs with SIGPIPE ignored.
"""

import hashlib
import logging
import select
import signal
import socket
import struct
import subprocess
import threading
import time
from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from drone_duels.server.protocol import VideoSystem
from drone_duels.session.base import (
    EXTERNAL,
    Events,
    Firmware,
    FirmwareError,
    PhysicsProcess,
    SpawnContext,
)
from drone_duels.video.osd import MSP_DISPLAYPORT, DisplayPort, MspReader, OsdScreen

log = logging.getLogger(__name__)

BINARY = EXTERNAL / "inav" / "inav_SITL"
CHANMAP = "M01-01,S01-02,S02-03"
"""Motor 1 to the X-Plane throttle, servos 1 and 2 (the elevons) to the first two yokes."""
PROMPT = b"\n# "


def start(binary: Path, workdir: Path, sim_port: int, tcp_base: int) -> subprocess.Popen[bytes]:
    """INAV SITL flying the simulator on UDP `sim_port`, its UARTs from TCP `tcp_base`, its
    eeprom and log (inav.log) in `workdir`. It waits for the simulator to answer before it
    serves its CLI."""
    workdir.mkdir(parents=True, exist_ok=True)
    args = [
        str(binary.resolve()),
        f"--path={workdir.resolve() / 'eeprom.bin'}",
        "--sim=xp",
        "--simip=127.0.0.1",
        f"--simport={sim_port}",
        f"--chanmap={CHANMAP}",
        f"--tcpbaseport={tcp_base}",
    ]
    log.info("starting %s", " ".join(args))
    # SITL writes its TCP UARTs with a bare send(), so a client hanging up as it writes would
    # kill it with SIGPIPE, silently. It keeps Python's SIG_IGN instead and gets EPIPE.
    with (workdir / "inav.log").open("ab") as out:
        return subprocess.Popen(
            args, cwd=workdir, stdout=out, stderr=subprocess.STDOUT, restore_signals=False
        )


def ended(proc: subprocess.Popen[bytes]) -> str:
    """How INAV SITL ended: its exit code, or the signal that killed it."""
    code = proc.returncode
    if code is None:
        return "still running"
    if code < 0:
        try:
            return f"killed by {signal.Signals(-code).name}"
        except ValueError:
            return f"killed by signal {-code}"
    return f"exit code {code}"


def interrupt(proc: subprocess.Popen[bytes], timeout: float = 3.0) -> None:
    """Stop INAV SITL as it expects to be stopped, with SIGINT (the only signal it handles),
    then kill it if it will not go."""
    if proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout)


def settings_files(files: Iterable[Path], video: VideoSystem) -> list[Path]:
    """Each settings file, then its video system's beside it, if there is one: chupito.cli,
    then chupito.analog.cli (the OSD for an analog picture) or chupito.digital.cli."""
    out: list[Path] = []
    for path in files:
        out.append(path)
        extra = path.with_name(f"{path.stem}.{video}{path.suffix}")
        if extra.exists():
            out.append(extra)
    return out


def load_settings(files: Iterable[Path]) -> str:
    """The CLI settings in `files`, one after another."""
    return "\n".join(path.read_text() for path in files)


def _commands(settings: str) -> list[str]:
    return [
        line.strip() for line in settings.splitlines() if line.strip() and not line.startswith("#")
    ]


def settings_hash(settings: str) -> str:
    return hashlib.sha256("\n".join(_commands(settings)).encode()).hexdigest()[:16]


def configure(workdir: Path, settings: str, cli_port: int, timeout: float = 30) -> bool:
    """Type `settings` into INAV's CLI and save, unless the eeprom already holds them. True when
    it saved (and so restarted). Raises RuntimeError on a command INAV rejects, OSError when its
    CLI does not answer."""
    marker = workdir / "settings.hash"
    digest = settings_hash(settings)
    held = marker.exists() and marker.read_text().strip() == digest
    if held and (workdir / "eeprom.bin").exists():
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


# MSP.

MSP_MOTOR = 104
MSP_RAW_GPS = 106
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_BOXIDS = 119
MSP2_INAV_STATUS = 0x2000
MSP2_INAV_ANALOG = 0x2002
MSP2_INAV_AIR_SPEED = 0x2009
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
MODES = (
    (27, "FAILSAFE"),
    (10, "RTH"),
    (28, "WP"),
    (36, "LAUNCH"),
    (11, "LOITER"),
    (53, "CRUISE"),
    (45, "COURSE"),
    (3, "ALTHOLD"),
    (12, "MANUAL"),
    (1, "ANGLE"),
    (2, "HORIZON"),
    (64, "ANGLEHOLD"),
)
"""INAV's flight modes by their boxes' permanent ids (fc/fc_msp_box.c), the one shown first
first; none of them is ACRO."""
GPS_FIX_2D = 1


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

    def __init__(self, port: int) -> None:
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=2)
        self._lock = threading.Lock()
        self._boxes: tuple[int, ...] | None = None

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
        return _status(self.request(MSP2_INAV_STATUS))

    def attitude(self) -> tuple[float, float, float]:
        """INAV's estimate: roll (right wing down), pitch (nose up) and heading, degrees."""
        roll, pitch, heading = struct.unpack_from("<hhh", self.request(MSP_ATTITUDE))
        return roll / 10, -pitch / 10, float(heading)

    def boxes(self) -> tuple[int, ...]:
        """The permanent ids of INAV's boxes, in the order its mode flags give them."""
        if self._boxes is None:
            self._boxes = tuple(self.request(MSP_BOXIDS))
        return self._boxes

    def telemetry(self) -> dict[str, Any]:
        """What a ground station sees, as protocol.TelemetryMsg (with its `ch`)."""
        status = self.request(MSP2_INAV_STATUS)
        roll, pitch, heading = self.attitude()
        alt_cm, vario_cms = struct.unpack_from("<ih", self.request(MSP_ALTITUDE))
        fix, sats, lat, lon, gps_alt, speed = struct.unpack_from(
            "<BBiihH", self.request(MSP_RAW_GPS)
        )
        # MSP2_INAV_ANALOG: flags (u8), volts (u16, 10 mV), amps (u16), power, mAh, mWh,
        # capacity left (u32 each), percent (u8), RSSI (u16).
        analog = self.request(MSP2_INAV_ANALOG)
        _, vbat = struct.unpack_from("<BH", analog)
        percent = analog[21] if len(analog) > 21 else None
        (airspeed_cms,) = struct.unpack_from("<I", self.request(MSP2_INAV_AIR_SPEED))
        (motor,) = struct.unpack_from("<H", self.request(MSP_MOTOR))
        st = _status(status)
        has_fix = fix >= GPS_FIX_2D
        return {
            "ch": "telemetry",
            "armed": st.armed,
            "mode": mode_name(_mode_bits(status), self.boxes()),
            "lat": lat / 1e7 if has_fix else None,
            "lon": lon / 1e7 if has_fix else None,
            "alt_amsl": float(gps_alt) if has_fix else None,
            "alt_rel": alt_cm / 100,
            "roll_deg": roll,
            "pitch_deg": pitch,
            "yaw_deg": heading,
            "groundspeed": speed / 100,
            "airspeed": airspeed_cms / 100,
            "climb": vario_cms / 100,
            "throttle_pct": max(0.0, min(100.0, (motor - 1000) / 10)),
            "battery_v": vbat / 100,
            "battery_pct": float(percent) if percent is not None else None,
            "gps_sats": sats,
            "blockers": list(st.blockers),
        }


def _status(reply: bytes) -> Status:
    """MSP2_INAV_STATUS: cycle time, I2C errors, sensors, load (u16 each), profiles (u8),
    armingFlags (u32), the box mode flags, the mixer profile (u8)."""
    (flags,) = struct.unpack_from("<I", reply, 9)
    blockers = tuple(name for bit, name in ARMING_BLOCKERS.items() if flags & (1 << bit))
    return Status(bool(flags & ARMED), blockers)


def _mode_bits(status: bytes) -> int:
    return int.from_bytes(status[13:-1], "little")


def mode_name(bits: int, boxes: Sequence[int]) -> str:
    """The flight mode, short, as INAV's OSD would name it: bit i of `bits` is box `boxes[i]`
    (permanent ids) active."""
    active = {box for i, box in enumerate(boxes) if bits >> i & 1}
    for box, name in MODES:
        if box in active:
            return name
    return "ACRO"


# The OSD.


OSD_GRIDS: dict[VideoSystem, dict[int, tuple[int, int]]] = {
    "analog": {0: (30, 16)},
    "digital": {3: (53, 20)},
}
"""The grid INAV's MSP DisplayPort draws on, by the resolution it names (io/displayport_msp_osd.c):
Betaflight-compatible SD (BF43COMPAT, SD_3016) for analog video, cut to NTSC's 13 rows as the
picture shows them; Betaflight-compatible HD (BFHDCOMPAT, HD_5320) for digital."""


class AirUnit:
    """The DJI air unit's side of INAV's OSD (or, on analog video, the OSD chip's). INAV sends
    its OSD over MSP DisplayPort on the UART TBS gives it (UART3, SITL's TCP `port`), but only
    to an air unit it hears: the unit polls the flight controller over MSP, and so does this
    (MSP_FC_VARIANT and MSP_STATUS, ten times a second). `screen`: the last INAV drew, on
    `video`'s grid (OSD_GRIDS); `on_screen` gets each screen drawn, no more than `max_hz` a
    second (the latest)."""

    POLL = b"$M<\x00\x02\x02$M<\x00\x65\x65"
    POLL_S = 0.1

    def __init__(
        self,
        port: int,
        on_screen: Callable[[OsdScreen], None] | None = None,
        max_hz: float = 20.0,
        video: VideoSystem = "digital",
    ) -> None:
        self.port = port
        self.video: VideoSystem = video
        self.on_screen = on_screen
        self.min_gap = 1 / max_hz
        self.screen: OsdScreen | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="air-unit", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                with socket.create_connection(("127.0.0.1", self.port), timeout=2) as sock:
                    self._listen(sock)
            except OSError:
                pass  # INAV not up yet, or restarting with its settings
            self._stop.wait(0.5)

    def _listen(self, sock: socket.socket) -> None:
        reader, display = MspReader(), DisplayPort(
            OSD_GRIDS[self.video], analog=self.video == "analog"
        )
        polled = sent = 0.0
        fresh = False
        while not self._stop.is_set():
            now = time.monotonic()
            if now - polled >= self.POLL_S:
                sock.sendall(self.POLL)
                polled = now
            if fresh and now - sent >= self.min_gap and self.screen is not None:
                fresh, sent = False, now
                if self.on_screen is not None:
                    self.on_screen(self.screen)
            wait = self.min_gap / 2 if fresh else self.POLL_S
            if not select.select([sock], [], [], wait)[0]:
                continue
            data = sock.recv(65536)
            if not data:
                return
            for command, payload in reader.feed(data):
                if command == MSP_DISPLAYPORT and display.handle(payload):
                    self.screen = display.screen
                    fresh = True


def osd_message(screen: OsdScreen) -> dict[str, Any]:
    """protocol.OsdMsg for a screen INAV drew: Betaflight's glyphs (BFHDCOMPAT)."""
    return {
        "ch": "osd",
        "font": "betaflight",
        "cols": screen.cols,
        "rows": screen.rows,
        "codes": [ord(c) for row in screen.text for c in row],
    }


# The session's firmware.

READY_S = 60.0
"""From start to ready to arm, at most, s: INAV boots, connects, takes its settings, restarts,
calibrates."""
TELEMETRY_HZ = 10.0
POLL_S = 0.5
"""Between readiness polls, s."""


class Inav(Firmware):
    """INAV SITL for a vehicle, against its physics' X-Plane port, with the vehicle's CLI
    settings for the spawn's video system. Ready once INAV could arm (all but the arm switch) on
    two polls in a row; then it publishes its telemetry at TELEMETRY_HZ and its OSD screens as
    they are drawn. When the page's radio goes quiet, the physics stops giving INAV a joystick,
    so INAV's receiver loses its link and INAV's own failsafe flies (RTH, in chupito.cli)."""

    name = "INAV"

    def __init__(self, ctx: SpawnContext, events: Events, physics: PhysicsProcess) -> None:
        super().__init__(ctx, events, physics)
        self.binary = BINARY
        self.proc: subprocess.Popen[bytes] | None = None
        self.msp: Msp | None = None
        self.air_unit: AirUnit | None = None
        self.settings = ""
        self._stop = threading.Event()
        self._telemetry: threading.Thread | None = None

    @property
    def log_path(self) -> Path:
        return self.ctx.workdir / "inav.log"

    def start(self) -> None:
        ctx = self.ctx
        if not self.binary.exists():
            raise FirmwareError(f"no INAV SITL at {self.binary}: run scripts/setup.sh")
        files = settings_files(
            [ctx.firmware_file(name) for name in ctx.vehicle.firmware.params], ctx.video
        )
        missing = [str(f) for f in files if not f.exists()]
        if missing:
            raise FirmwareError(f"INAV settings missing: {', '.join(missing)}")
        self.settings = load_settings(files)
        self.proc = start(self.binary, ctx.workdir, ctx.ports.inav_xplane, ctx.ports.inav_tcp_base)
        self.air_unit = AirUnit(ctx.ports.inav_osd, self._osd, video=ctx.video)
        self.air_unit.start()

    def wait_ready(self, step: Callable[[str], None]) -> None:
        ctx = self.ctx
        deadline = time.monotonic() + READY_S
        step("waiting for INAV to connect")
        self._until(self._connected, deadline, "INAV did not connect to the physics")
        step("configuring INAV")
        try:
            saved = configure(
                ctx.workdir,
                self.settings,
                ctx.ports.inav_tcp_base,
                timeout=max(1.0, deadline - time.monotonic()),
            )
        except RuntimeError as exc:
            raise FirmwareError(f"INAV's settings: {exc}") from None
        except OSError as exc:
            self._check(deadline, "INAV's CLI")
            raise FirmwareError(f"INAV's CLI: {exc}: see {self.log_path}") from None
        if saved:
            step("INAV restarting with its settings")
            self._pause(2.0, deadline)
            self._until(self._connected, deadline, "INAV did not come back after saving")
        step("connecting to INAV")
        self.msp = self._open_msp(deadline)
        clear = 0
        while clear < 2:
            self._check(deadline, "INAV is not ready to arm")
            try:
                status = self.msp.status()
            except OSError as exc:
                self._check(deadline, "INAV's MSP")
                raise FirmwareError(f"INAV's MSP: {exc}: see {self.log_path}") from None
            ready = not status.armed and set(status.blockers) <= {"arm switch"}
            clear = clear + 1 if ready else 0
            if not ready:
                step("INAV: " + (", ".join(status.blockers) or "armed"))
            self._pause(POLL_S, deadline)
        self._telemetry = threading.Thread(target=self._publish, name="inav-msp", daemon=True)
        self._telemetry.start()

    def send_rc(self, us: Sequence[int]) -> None:
        self.physics.send({"t": "rc", "us": list(us)})

    def rc_lost(self) -> None:
        """Nothing: the physics stops hearing sticks, and its own timeout gives INAV what a
        receiver gives with no link."""

    def alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def stop(self) -> None:
        self._stop.set()
        if self._telemetry is not None and self._telemetry is not threading.current_thread():
            self._telemetry.join(2)
        self._telemetry = None
        if self.air_unit is not None:
            self.air_unit.stop()
            self.air_unit = None
        if self.msp is not None:
            self.msp.close()
            self.msp = None
        if self.proc is not None:
            interrupt(self.proc)
            log.info("INAV SITL stopped: %s", ended(self.proc))

    # Inside.

    def _connected(self) -> bool:
        return bool(self.physics.status.get("connected"))

    def _check(self, deadline: float, what: str) -> None:
        self.ctx.check()
        if self.proc is not None and self.proc.poll() is not None:
            raise FirmwareError(f"INAV SITL stopped ({ended(self.proc)}): see {self.log_path}")
        if not self.physics.alive():
            raise FirmwareError(
                f"the physics stopped: see {self.ctx.workdir / 'physics.log'}"
            )
        if time.monotonic() > deadline:
            raise FirmwareError(f"{what} in {READY_S:.0f} s: see {self.log_path}")

    def _until(self, done: Callable[[], bool], deadline: float, what: str) -> None:
        while not done():
            self._check(deadline, what)
            time.sleep(0.1)

    def _pause(self, seconds: float, deadline: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._check(deadline, "INAV")
            time.sleep(0.1)

    def _open_msp(self, deadline: float) -> Msp:
        while True:
            try:
                return Msp(self.ctx.ports.inav_msp)
            except OSError:
                self._check(deadline, "INAV's MSP did not answer")
                time.sleep(0.5)

    def _publish(self) -> None:
        period = 1 / TELEMETRY_HZ
        failed = False
        while not self._stop.wait(period):
            if not self.alive():
                return  # the session's watchdog says so
            msp = self.msp
            try:
                if msp is None:
                    msp = self.msp = Msp(self.ctx.ports.inav_msp)
                self.events.publish(msp.telemetry())
                failed = False
            except (OSError, struct.error) as exc:
                if not failed:
                    log.warning("INAV telemetry: %s", exc)
                    failed = True
                if msp is not None:
                    msp.close()
                self.msp = None

    def _osd(self, screen: OsdScreen) -> None:
        self.events.publish(osd_message(screen))
