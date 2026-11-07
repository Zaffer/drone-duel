"""Betaflight SITL for the Betaflight 5" quad: the process, its settings, its RC input, MSP, its
OSD, and the session's Firmware (DESIGN.md §6.1, §6.4).

Betaflight SITL is built by scripts/setup.sh with scripts/betaflight-sitl.patch: its ports are
arguments (ports.SlotPorts.betaflight_*), it listens on this machine only, and it has an OSD. It
flies the physics over its simulator protocol (bridges/betaflight.py), in real time: the
physics paces itself and Betaflight's clock follows.

Settings: the vehicle's CLI files (firmware/betaflight/*.cli, each followed by its video
system's: the OSD's grid and layout) are typed into a fresh eeprom by SITL itself (`--config`:
load, save, exit) before it boots. A hash beside the eeprom says which settings it holds, so
unchanged ones are not typed in again. `--config` exits 0 whatever it met: a setting it
rejects shows only in its output (###ERROR ...###), which is read for one. It listens on ports
of its own meanwhile (ports.SlotPorts.betaflight_config_sim_base): the physics' state arriving
as it types the settings in crashes it.

RC: SITL reads packets of a timestamp and 16 pulse widths on its RC port (RX_UDP). Until the
page sends sticks a receiver sitting idle is played (throttle low, arm switch low, the rest
centred) at 50 Hz; then only the page's sticks go. When the page goes quiet nothing more is sent:
Betaflight's receiver loses its link (RXLOSS) and its own failsafe flies (quad5.cli: it lands
where it is); the page's next sticks bring the link back.

Reading it: MSP on UART2, as a ground station would (`BetaflightMsp`); its OSD over MSP
DisplayPort on UART3, as a DJI air unit (firmware.inav.AirUnit, which polls for it).

Flying itself (`hover`, `land`: a target to shoot at). Betaflight takes no commands from a
ground station: its modes are on the radio's switches. So the game flies it as a pilot on its
own modes would (the receiver's source is then "auto"), reading it over MSP: the mode switch
centred (ANGLE), the arm switch low then high; the mode switch to POSHOLD (altitude and
position hold, on the GPS: Betaflight arms only with it off) and the throttle up, past
Betaflight's airmode threshold, which the hold modes wait for. In altitude hold the throttle
stick asks for a climb or a descent, nothing within its deadband about the hover throttle
(alt_hold_deadband, ap_hover_throttle in quad5.cli); its height target trails a fast climb, so
the climb slows near the height asked, and the pilot goes on nudging it there. Landing, it
comes down slowly in POSHOLD until Betaflight's touchdown detection disarms it (or it sits
still on the ground: the arm switch goes low). A reset (`reset_in_place`) puts the arm switch
low.
"""

import hashlib
import logging
import shutil
import socket
import struct
import subprocess
import threading
import time
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any

from drone_duels.firmware.inav import AirUnit, MspLink, ended, load_settings, osd_message
from drone_duels.server.protocol import VideoSystem
from drone_duels.session.base import (
    EXTERNAL,
    Events,
    Firmware,
    FirmwareError,
    PhysicsProcess,
    SpawnContext,
    settings_files,
)
from drone_duels.video.osd import OsdScreen

log = logging.getLogger(__name__)

BINARY = EXTERNAL / "betaflight" / "obj" / "main" / "betaflight_SITL.elf"
CHANNELS = 16
IDLE_US = (1500, 1500, 1000, 1500, 1000) + (1500,) * (CHANNELS - 5)
"""A receiver linked but its sticks let go: throttle low, arm switch (channel 5) low, the rest
centred."""
RC_IDLE_HZ = 50.0
RC = struct.Struct(f"<d{CHANNELS}H")
CONFIG_S = 30.0
"""How long `--config` may take to type the settings in."""
READY_S = 60.0
"""From start to ready to arm, at most, s: Betaflight boots, finds the physics, waits out its
boot grace, calibrates."""
POLL_S = 0.5
TELEMETRY_HZ = 10.0

# Flying itself (the module's docstring): the sticks, µs, as quad5.cli sets them.
ARM_HIGH = 2000
MODE_POSHOLD, MODE_ANGLE = 1000, 1500
THROTTLE_LOW = 1000
HOVER_US = 1275
"""ap_hover_throttle: Betaflight's default, and where this quad hovers in ANGLE (about 1280)."""
ALT_HOLD_DEADBAND = 0.20
"""alt_hold_deadband (20 %): the share of the stick's travel either side of the hover throttle
that asks for no climb."""
ALT_HOLD_CLIMB_MS = 5.0
"""alt_hold_climb_rate (50): the climb or descent asked at full or (nearly) no throttle, m/s."""
CLIMB_GAIN = 0.6
"""Climb asked per metre below the height, 1/s."""
CLIMB_MAX_MS, SINK_MAX_MS = 3.0, 2.0
HELD_M = 0.3
"""Within this of the height asked, the throttle stays at the hover throttle."""
LAND_MS = 0.7
"""Descent asked near the ground, landing, m/s (faster higher up, up to SINK_MAX_MS)."""
STILL_S = 1.5
"""On the ground, armed and still this long, landing: the arm switch goes low."""
PILOT_HZ = 10.0
ARM_S = 5.0
FLY_TRY_S = 30.0

OSD_GRIDS: dict[VideoSystem, dict[int, tuple[int, int]]] = {
    "analog": {0: (30, 13)},
    "digital": {0: (53, 20)},
}
"""The grid Betaflight's MSP DisplayPort draws on (io/displayport_msp.c): it names none, so the
video system's: NTSC's 30 x 13 for analog (vcd_video_system NTSC), its HD canvas for digital."""

# MSP.

MSP_MOTOR = 104
MSP_RAW_GPS = 106
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ANALOG = 110
MSP_STATUS_EX = 150
BOX_ARM = 0
ARMING_BLOCKERS = (
    "no gyro",
    "failsafe",
    "no RC link",
    "not disarmed",
    "failsafe switch",
    "runaway",
    "crash",
    "throttle",
    "not level",
    "booting",
    "no prearm",
    "overloaded",
    "calibrating",
    "CLI",
    "CMS menu",
    "BST",
    "MSP",
    "paralyzed",
    "GPS",
    "rescue switch",
    "DShot telemetry",
    "reboot required",
    "DShot bitbang",
    "accelerometer not calibrated",
    "motor protocol",
    "crash flip switch",
    "altitude hold switch",
    "position hold switch",
    "autopilot switch",
    "arm switch",
)
"""Betaflight's arming disable flags, bit by bit (fc/runtime_config.c's armingDisableFlagNames)."""
ARM_SWITCH = "arm switch"
MODES = (
    (27, "FAILSAFE"),
    (46, "RESCUE"),
    (56, "AUTOPILOT"),
    (11, "POSHOLD"),
    (3, "ALTHOLD"),
    (1, "ANGLE"),
    (2, "HORIZON"),
)
"""Betaflight's flight modes by their boxes' permanent ids (msp/msp_box.c), the one shown first
first; with none of them it flies ACRO."""
GPS_FIX = 1


def throttle_for(climb_ms: float) -> int:
    """The throttle stick, µs, that asks Betaflight's altitude hold for `climb_ms` (m/s, up
    positive): its stick factor (flight/alt_hold_multirotor.c), turned round. 0: the hover
    throttle, in the deadband."""
    if climb_ms == 0:
        return HOVER_US
    factor = max(-1.0, min(1.0, climb_ms / ALT_HOLD_CLIMB_MS))
    if factor > 0:
        high = HOVER_US + ALT_HOLD_DEADBAND * (2000 - HOVER_US)
        return round(high + factor * (2000 - high))
    low = HOVER_US - ALT_HOLD_DEADBAND * (HOVER_US - 1000)
    # Never quite low: at the bottom of the stick altitude hold takes no stick at all.
    return max(1060, round(low + factor * (low - 1000)))


def blockers(flags: int) -> tuple[str, ...]:
    return tuple(name for bit, name in enumerate(ARMING_BLOCKERS) if flags >> bit & 1)


def mode_name(active: set[int]) -> str:
    """The flight mode, short, from the permanent ids of the boxes active."""
    for box, name in MODES:
        if box in active:
            return name
    return "ACRO"


class BetaflightMsp(MspLink):
    """Betaflight over MSP: its state, as a ground station reads it."""

    def status(self) -> tuple[set[int], tuple[str, ...]]:
        """The boxes active (permanent ids) and what stops it arming."""
        return _status(self.request(MSP_STATUS_EX), self.boxes())

    def telemetry(self) -> dict[str, Any]:
        """What a ground station sees, as protocol.TelemetryMsg (with its `ch`)."""
        active, blocked = self.status()
        roll, pitch, heading = struct.unpack_from("<hhh", self.request(MSP_ATTITUDE))
        alt_cm, vario_cms = struct.unpack_from("<ih", self.request(MSP_ALTITUDE))
        fix, sats, lat, lon, gps_alt, speed = struct.unpack_from(
            "<BBiiHH", self.request(MSP_RAW_GPS)
        )
        # MSP_ANALOG: volts (u8, 0.1 V), mAh, RSSI, amps (u16 each), volts (u16, 0.01 V). SITL
        # has no battery: 0.
        vbat = struct.unpack_from("<H", self.request(MSP_ANALOG), 7)[0]
        motors = self.request(MSP_MOTOR)
        outputs = struct.unpack_from("<4H", motors) if len(motors) >= 8 else ()
        throttle = sum(outputs) / len(outputs) if outputs else 1000
        has_fix = fix >= GPS_FIX
        return {
            "ch": "telemetry",
            "armed": BOX_ARM in active,
            "mode": mode_name(active),
            "lat": lat / 1e7 if has_fix else None,
            "lon": lon / 1e7 if has_fix else None,
            "alt_amsl": float(gps_alt) if has_fix else None,
            "alt_rel": alt_cm / 100,
            "roll_deg": roll / 10,
            "pitch_deg": -pitch / 10,
            "yaw_deg": float(heading),
            "groundspeed": speed / 100,
            "airspeed": None,
            "climb": vario_cms / 100,
            "throttle_pct": max(0.0, min(100.0, (throttle - 1000) / 10)),
            "battery_v": vbat / 100 if vbat else None,
            "battery_pct": None,
            "gps_sats": sats,
            "blockers": [b for b in blocked if b != ARM_SWITCH],
        }


def _status(reply: bytes, boxes: Sequence[int]) -> tuple[set[int], tuple[str, ...]]:
    """MSP_STATUS_EX: cycle time, I2C errors, sensors (u16 each), the first 32 boxes' flags
    (u32), PID profile (u8), load (u16), PID profile count, rate profile, the count of further
    flag bytes (u8 each), those bytes, the count of arming flags (u8), arming flags (u32)."""
    (low,) = struct.unpack_from("<I", reply, 6)
    extra = reply[15]
    bits = low | int.from_bytes(reply[16 : 16 + extra], "little") << 32
    (flags,) = struct.unpack_from("<I", reply, 16 + extra + 1)
    active = {box for i, box in enumerate(boxes) if bits >> i & 1}
    return active, blockers(flags)


# Settings.


def commands(settings: str) -> list[str]:
    """The CLI's commands in `settings`: no comments, no blank lines."""
    out = []
    for line in settings.splitlines():
        command = line.split("#", 1)[0].strip()
        if command:
            out.append(command)
    return out


def settings_hash(settings: str) -> str:
    return hashlib.sha256("\n".join(commands(settings)).encode()).hexdigest()[:16]


def sitl_args(binary: Path, sim_base: int, tcp_base: int) -> list[str]:
    args = [str(binary.resolve()), "--simport", str(sim_base), "--tcpbaseport", str(tcp_base)]
    # Line-buffered, so its log is whole when it dies.
    stdbuf = shutil.which("stdbuf")
    return [stdbuf, "-oL", "-eL", *args] if stdbuf else args


def configure(binary: Path, workdir: Path, settings: str, sim_base: int, tcp_base: int) -> bool:
    """Type `settings` into a fresh eeprom in `workdir`, unless it already holds them. True when
    it did. Raises FirmwareError on a setting Betaflight rejects."""
    marker = workdir / "settings.hash"
    eeprom = workdir / "eeprom.bin"
    digest = settings_hash(settings)
    if marker.exists() and marker.read_text().strip() == digest and eeprom.exists():
        return False
    eeprom.unlink(missing_ok=True)
    marker.unlink(missing_ok=True)
    cli = workdir / "settings.cli"
    cli.write_text("\n".join(commands(settings)) + "\n")
    args = [*sitl_args(binary, sim_base, tcp_base), "--config", cli.name]
    try:
        run = subprocess.run(
            args,
            cwd=workdir,
            capture_output=True,
            text=True,
            timeout=CONFIG_S,
            errors="replace",
            check=False,
        )
    except subprocess.TimeoutExpired:
        raise FirmwareError(f"Betaflight took over {CONFIG_S:.0f} s to take its settings") from None
    (workdir / "config.log").write_text(
        "".join(
            line
            for line in (run.stdout + run.stderr).splitlines(keepends=True)
            if not line.startswith("[FLASH_ProgramWord]")
        )
    )
    errors = [line.strip("#") for line in run.stdout.splitlines() if line.startswith("###ERROR")]
    if errors:
        raise FirmwareError(f"Betaflight rejected its settings: {'; '.join(errors)}")
    if run.returncode != 0 or not eeprom.exists():
        raise FirmwareError(
            f"Betaflight could not save its settings ({run.returncode}): see "
            f"{workdir / 'config.log'}"
        )
    marker.write_text(digest)
    log.info("Betaflight configured")
    return True


# The session's firmware.


class Betaflight(Firmware):
    """Betaflight SITL for a vehicle, flying its physics, with the vehicle's CLI settings for the
    spawn's video system. Ready once Betaflight could arm (all but the arm switch) on two polls
    in a row; then it publishes its telemetry at TELEMETRY_HZ and its OSD screens as they are
    drawn."""

    name = "Betaflight"
    # Betaflight has no EKF to upset: the quad jumping back to the pad is only a GPS jump, and
    # its attitude settles on the accelerometer at once.
    resets_in_place = True

    def __init__(self, ctx: SpawnContext, events: Events, physics: PhysicsProcess) -> None:
        super().__init__(ctx, events, physics)
        self.binary = BINARY
        self.proc: subprocess.Popen[bytes] | None = None
        self.msp: BetaflightMsp | None = None
        self.air_unit: AirUnit | None = None
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._rc_sock: socket.socket | None = None
        self._rc_lock = threading.Lock()
        self._rc_source = "idle"
        """Who feeds Betaflight's receiver: "idle" (nobody yet), "page", "lost", or "auto" (the
        game, flying it as a target)."""
        self._auto_us: list[int] = list(IDLE_US)
        """The sticks while it flies itself."""
        self._fly_lock = threading.Lock()
        self._fly_gen = 0
        self._telemetry: dict[str, Any] | None = None
        """The last telemetry read, as published."""
        self._t0 = time.monotonic()
        self._stopped = False

    @property
    def log_path(self) -> Path:
        return self.ctx.workdir / "betaflight.log"

    def start(self) -> None:
        ctx = self.ctx
        if not self.binary.exists():
            raise FirmwareError(f"no Betaflight SITL at {self.binary}: run scripts/setup.sh")
        files = settings_files(
            [ctx.firmware_file(name) for name in ctx.vehicle.firmware.params], ctx.video
        )
        missing = [str(f) for f in files if not f.exists()]
        if missing:
            raise FirmwareError(f"Betaflight settings missing: {', '.join(missing)}")
        ports = ctx.ports
        ctx.workdir.mkdir(parents=True, exist_ok=True)
        configure(
            self.binary,
            ctx.workdir,
            load_settings(files),
            ports.betaflight_config_sim_base,
            ports.betaflight_tcp_base,
        )
        args = sitl_args(self.binary, ports.betaflight_sim_base, ports.betaflight_tcp_base)
        log.info("Betaflight: %s", " ".join(args))
        with self.log_path.open("wb") as out:
            # SIGPIPE stays ignored (as Python has it): SITL writes its TCP UARTs with send().
            self.proc = subprocess.Popen(
                args,
                cwd=ctx.workdir,
                stdout=out,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                restore_signals=False,
            )
        self._rc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._thread(self._rc_idle_loop, "betaflight-rc")
        self.air_unit = AirUnit(ports.betaflight_osd, self._osd, video=ctx.video, grids=OSD_GRIDS)
        self.air_unit.start()

    def wait_ready(self, step: Callable[[str], None]) -> None:
        deadline = time.monotonic() + READY_S
        step("waiting for Betaflight to fly the physics")
        while not self.physics.status.get("connected"):
            self._check(deadline, "Betaflight did not connect to the physics")
            time.sleep(0.1)
        step("connecting to Betaflight")
        self.msp = self._open_msp(deadline)
        clear, said = 0, ""
        while clear < 2:
            self._check(deadline, "Betaflight is not ready to arm")
            try:
                active, blocked = self.msp.status()
            except (OSError, struct.error) as exc:
                self._check(deadline, "Betaflight's MSP")
                raise FirmwareError(f"Betaflight's MSP: {exc}: see {self.log_path}") from None
            ready = BOX_ARM not in active and set(blocked) <= {ARM_SWITCH}
            clear = clear + 1 if ready else 0
            what = "Betaflight: " + (", ".join(blocked) or "armed")
            if not ready and what != said:
                said = what
                step(what)
            time.sleep(POLL_S)
        self._thread(self._publish, "betaflight-msp")

    # RC.

    def send_rc(self, us: Sequence[int]) -> None:
        with self._rc_lock:
            self._rc_source = "page"
            self._send_rc(us)

    def rc_lost(self) -> None:
        """Only the page's link can be lost: before it sends (a respawn the moment the radio went
        quiet, say) the idle receiver plays on, or Betaflight would never be ready to arm."""
        with self._rc_lock:
            if self._rc_source == "page":
                self._rc_source = "lost"

    def _send_rc(self, us: Sequence[int]) -> None:
        sock = self._rc_sock
        if sock is None:
            return
        widths = [int(max(800, min(2200, w))) for w in us[:CHANNELS]]
        widths += IDLE_US[len(widths) :]
        try:
            sock.sendto(
                RC.pack(time.monotonic() - self._t0, *widths),
                ("127.0.0.1", self.ctx.ports.betaflight_rc),
            )
        except OSError:
            pass

    def _rc_idle_loop(self) -> None:
        while not self._stop.wait(1 / RC_IDLE_HZ):
            with self._rc_lock:
                if self._rc_source == "idle":
                    self._send_rc(IDLE_US)
                elif self._rc_source == "auto":
                    self._send_rc(self._auto_us)

    # Flying itself (the module's docstring).

    def hover(self, height_m: float) -> None:
        self._fly(lambda gen: self._hover(gen, height_m), "betaflight-hover")

    def land(self) -> None:
        self._fly(self._land, "betaflight-land")

    def reset_in_place(self) -> None:
        with self._fly_lock:
            self._fly_gen += 1
        with self._rc_lock:
            if self._rc_source == "auto":
                self._auto_us = list(IDLE_US)

    def _fly(self, run: Callable[[int], None], name: str) -> None:
        with self._fly_lock:
            self._fly_gen += 1
            gen = self._fly_gen
        with self._rc_lock:
            if self._rc_source != "auto":
                self._rc_source = "auto"
                self._auto_us = list(IDLE_US)
        self._thread(lambda: run(gen), name)

    def _current(self, gen: int) -> bool:
        """Still the latest hover or land, with the radio still the game's, and running."""
        return gen == self._fly_gen and self._rc_source == "auto" and not self._stop.is_set()

    def _sticks(self, **channels: int) -> None:
        index = {"throttle": 2, "arm": 4, "mode": 5}
        with self._rc_lock:
            for name, us in channels.items():
                self._auto_us[index[name]] = us

    def _say(self, level: str, text: str) -> None:
        getattr(log, "warning" if level == "warn" else level)("%s", text)
        self.events.publish({"ch": "log", "level": level, "source": "session", "text": text})

    def _armed(self) -> bool:
        t = self._telemetry
        return bool(t and t["armed"])

    def _arm(self, gen: int) -> bool:
        """Arm it on the switches (ANGLE, throttle low); False if it would not before FLY_TRY_S,
        or the flying was overtaken."""
        deadline = time.monotonic() + FLY_TRY_S
        said = ""
        while self._current(gen):
            self._sticks(arm=IDLE_US[4], mode=MODE_ANGLE, throttle=THROTTLE_LOW)
            self._stop.wait(0.5)  # the arm switch seen low first, as Betaflight wants
            self._sticks(arm=ARM_HIGH)
            until = time.monotonic() + ARM_S
            while self._current(gen) and time.monotonic() < until and not self._armed():
                self._stop.wait(1 / PILOT_HZ)
            if self._armed():
                return True
            blocked = (self._telemetry or {}).get("blockers") or []
            why = blocked[0] if blocked else "no answer"
            if time.monotonic() > deadline:
                self._say("error", f"could not arm: {why}")
                return False
            if why != said:
                said = why
                self._say("warn", f"arming: {why}; trying again")
        return False

    def _hover(self, gen: int, height_m: float) -> None:
        if not self._armed():
            if not self._arm(gen):
                return
            self._say("info", f"taking off to {height_m:.0f} m above the pad")
        else:
            self._say("info", f"holding {height_m:.0f} m above the pad")
        self._sticks(mode=MODE_POSHOLD)
        flew = False
        while self._current(gen):
            t = self._telemetry
            if t is None:
                pass  # MSP is back in a moment: the sticks stay as they are
            elif not t["armed"]:
                if flew:  # Betaflight disarmed it (a crash): let go
                    self._sticks(arm=IDLE_US[4], throttle=THROTTLE_LOW, mode=MODE_ANGLE)
                    return
            else:
                flew = True
                error = height_m - float(t["alt_rel"] or 0.0)
                climb = 0.0 if abs(error) < HELD_M else CLIMB_GAIN * error
                self._sticks(throttle=throttle_for(max(-SINK_MAX_MS, min(CLIMB_MAX_MS, climb))))
            self._stop.wait(1 / PILOT_HZ)

    def _land(self, gen: int) -> None:
        if not self._armed():
            self._sticks(arm=IDLE_US[4], throttle=THROTTLE_LOW, mode=MODE_ANGLE)
            return
        self._say("info", "landing")
        self._sticks(mode=MODE_POSHOLD)
        still_since: float | None = None
        while self._current(gen):
            t = self._telemetry
            if t is None:
                self._stop.wait(1 / PILOT_HZ)
                continue
            if not t["armed"]:
                break
            height = float(t["alt_rel"] or 0.0)
            self._sticks(throttle=throttle_for(-max(LAND_MS, min(SINK_MAX_MS, 0.3 * height))))
            if height < 0.5 and abs(t["climb"]) < 0.1:
                still_since = still_since or time.monotonic()
                if time.monotonic() - still_since > STILL_S:
                    break
            else:
                still_since = None
            self._stop.wait(1 / PILOT_HZ)
        if self._current(gen):
            self._sticks(arm=IDLE_US[4], throttle=THROTTLE_LOW, mode=MODE_ANGLE)

    # Stop.

    def alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def stop(self) -> None:
        if self._stopped:
            return
        self._stopped = True
        self._stop.set()
        for t in self._threads:
            if t is not threading.current_thread():
                t.join(2)
        if self.air_unit is not None:
            self.air_unit.stop()
            self.air_unit = None
        if self.msp is not None:
            self.msp.close()
            self.msp = None
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(2)
        if self.proc is not None:
            log.info("Betaflight SITL stopped: %s", ended(self.proc))
        if self._rc_sock is not None:
            self._rc_sock.close()
            self._rc_sock = None

    # Inside.

    def _thread(self, target: Callable[[], None], name: str) -> None:
        t = threading.Thread(target=target, name=name, daemon=True)
        t.start()
        self._threads.append(t)

    def _check(self, deadline: float, what: str) -> None:
        self.ctx.check()
        if self.proc is not None and self.proc.poll() is not None:
            raise FirmwareError(
                f"Betaflight SITL stopped ({ended(self.proc)}): see {self.log_path}"
            )
        if not self.physics.alive():
            raise FirmwareError(f"the physics stopped: see {self.ctx.workdir / 'physics.log'}")
        if time.monotonic() > deadline:
            raise FirmwareError(f"{what} in {READY_S:.0f} s: see {self.log_path}")

    def _open_msp(self, deadline: float) -> BetaflightMsp:
        while True:
            try:
                return BetaflightMsp(self.ctx.ports.betaflight_msp)
            except OSError:
                self._check(deadline, "Betaflight's MSP did not answer")
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
                    msp = self.msp = BetaflightMsp(self.ctx.ports.betaflight_msp)
                telemetry = msp.telemetry()
                self._telemetry = telemetry
                self.events.publish(telemetry)
                failed = False
            except (OSError, struct.error) as exc:
                if not failed:
                    log.warning("Betaflight telemetry: %s", exc)
                    failed = True
                if msp is not None:
                    msp.close()
                self.msp = None
                self._telemetry = None

    def _osd(self, screen: OsdScreen) -> None:
        self.events.publish(osd_message(screen))
