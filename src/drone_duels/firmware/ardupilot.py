"""ArduCopter SITL for the 5" quad: the process, mavlink-router in front of it, its RC input, its
OSD and its telemetry, as the session's Firmware (DESIGN.md §6.2, §6.4).

ArduCopter flies `--model JSON` against our physics (drone_duels.physics.quad over
bridges/ardupilot_json.py): SITL instance = the slot, so its ports are ports.SlotPorts's. It
starts from its parameter files every spawn (the eeprom is wiped): ArduPilot's SITL copter
defaults, the vehicle's files (firmware/ardupilot/*.parm, each followed by its video system's,
copter-quad5.analog.parm or .digital.parm: the OSD's layout) and the video system's OSD grid
(OSD1_TXT_RES: 0 the analog chip's 30 x 16, 1 HD's 50 x 18). A switch of video system sets the
grid and the layout in flight: ArduPilot reads them each time it draws the screen. Its SERIAL0
is MAVLink on TCP, which mavlink-router connects to and shares out: UDP 14550 to QGroundControl
(system 255) and UDP 14570 + slot for the game (system 254, telemetry.mavlink.VehicleLink).
SERIAL5 sends its OSD over MSP DisplayPort to the server (video.osd), standing in for the OSD
chip.

RC: SITL reads packets of 16 little-endian pulse widths on its RC port and keeps the last values
it got for ever. Until the page sends sticks, a receiver sitting idle is played (throttle low,
arm switch low, the rest centred) at 50 Hz; then only the page's sticks go. When the page goes
quiet (`rc_lost`), nothing more is sent and SIM_RC_FAIL 1 makes SITL's receiver stop its pulses,
so ArduPilot's own radio failsafe runs after RC_FS_TIMEOUT, as on a real lost link; the page's
next sticks put it back (SIM_RC_FAIL 0).

Flying itself (`hover`, `land`: a target to shoot at), the game flies it as a ground station
would, over MAVLink, with nobody on the sticks: the idle receiver plays on, and nothing about
the RC changes. GUIDED (MAV_CMD_DO_SET_MODE), arm (MAV_CMD_COMPONENT_ARM_DISARM: GUIDED allows
arming from a GCS, ModeGuided::allows_arming; the arm switch, RC5_OPTION 153, acts only when it
moves, RC_Channel::init_position_on_first_radio_read, so held low it never disarms it), then
MAV_CMD_NAV_TAKEOFF to the height above home, the pad (Mode::do_user_takeoff_U_m: armed, landed,
upwards). Each waits for its COMMAND_ACK, and the whole is tried again until telemetry shows it
climbing. Up already, GUIDED and a position target at the height. Landing is LAND. The idle
receiver's 1000 µs throttle sits above FS_THR_VALUE (975), so no throttle failsafe; the mode
switch (1500 µs, STABILIZE) acts only when it moves, so GUIDED stays. A takeoff is not
landed, so the auto-disarm (DISARM_DELAY, landed and throttle low) does not cut in; if it
does not leave the ground, ArduPilot disarms it after 10 s and the next try arms it again.
"""

import logging
import socket
import struct
import subprocess
import threading
import time
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any

from pymavlink.dialects.v20 import ardupilotmega as apm

from drone_duels import ports
from drone_duels.server.protocol import VideoSystem
from drone_duels.session.base import (
    EXTERNAL,
    ROOT,
    Events,
    Firmware,
    FirmwareError,
    PhysicsProcess,
    SpawnContext,
    settings_files,
)
from drone_duels.telemetry.mavlink import VehicleLink
from drone_duels.video.osd import MSP_DISPLAYPORT, DisplayPort, MspReader, OsdScreen

log = logging.getLogger(__name__)

ARDUCOPTER = EXTERNAL / "ardupilot" / "build" / "sitl" / "bin" / "arducopter"
COPTER_DEFAULTS = EXTERNAL / "ardupilot" / "Tools" / "autotest" / "default_params" / "copter.parm"
ROUTER = EXTERNAL / "mavlink-router" / "build" / "src" / "mavlink-routerd"
SYSID = 1
READY_S = 90.0
RC_IDLE_HZ = 50.0
CHANNELS = 16
IDLE_US = (1500, 1500, 1000, 1500, 1000) + (1500,) * (CHANNELS - 5)
"""A receiver linked but its sticks let go: throttle low, arm switch (channel 5) low, the rest
centred."""
OSD_MIN_GAP_S = 0.05
"""At most 20 OSD screens a second to the page (ArduPilot draws 10)."""
PREARM_POLL_S = 5.0
FLY_TRY_S = 60.0
"""How long a hover or land keeps trying before it says it could not."""
CLIMB_SEEN_S = 6.0
"""After the takeoff is accepted, how long to wait for telemetry to show it climbing."""
CLIMB_SEEN_M = 0.5
"""Above home by this, m (telemetry's alt_rel), it has left the pad; or climbing CLIMB_SEEN_MS."""
CLIMB_SEEN_MS = 0.3
RETRY_S = 1.0

ROUTER_CONF = ROOT / "router" / "sitl.conf"
"""mavlink-router's configuration for slot 0; `router_conf` moves it to a slot's port."""


def _result(result: int | None) -> str:
    """A MAV_RESULT, as a ground station would say it."""
    if result is None:
        return "no answer"
    entry = apm.enums["MAV_RESULT"].get(result)
    return entry.name.removeprefix("MAV_RESULT_").lower() if entry else str(result)


def _climbing(link: VehicleLink) -> bool:
    s = link.state
    return (s.alt_rel or 0.0) > CLIMB_SEEN_M or s.climb > CLIMB_SEEN_MS


def rc_packet(us: Sequence[int]) -> bytes:
    """SITL's RC input: 16 pulse widths, little-endian uint16, µs; channels not given idle."""
    widths = [int(max(800, min(2200, w))) for w in us[:CHANNELS]]
    widths += IDLE_US[len(widths) :]
    return struct.pack(f"<{CHANNELS}H", *widths)


def osd_message(screen: OsdScreen) -> dict[str, Any]:
    """protocol.OsdMsg for a screen ArduPilot drew, in its OSD_FONT (served by number)."""
    return {
        "ch": "osd",
        "font": str(screen.font),
        "cols": screen.cols,
        "rows": screen.rows,
        "codes": [ord(c) for row in screen.text for c in row],
    }


def router_conf(slot: int) -> str:
    """router/sitl.conf for `slot`: its copter endpoint on that slot's SITL and the game's on the
    slot's own port. QGroundControl's is the same for every slot (ports.ROUTER_QGC)."""
    text = ROUTER_CONF.read_text()
    base, mine = ports.slot(0), ports.slot(slot)
    for was, now in ((base.ardupilot_mavlink, mine.ardupilot_mavlink),
                     (base.router_game, mine.router_game)):
        line = f"Port = {was}\n"
        if text.count(line) != 1:
            raise FirmwareError(f"{ROUTER_CONF}: no single endpoint on {line.strip()}")
        text = text.replace(line, f"Port = {now}\n")
    return text


def txt_res(video: str) -> int:
    """The OSD grid for the video system: the analog chip's SD, or HD for a digital goggle."""
    return 0 if video == "analog" else 1


def read_params(path: Path) -> dict[str, float]:
    """A parameter file's values: `NAME value` lines, # comments."""
    out: dict[str, float] = {}
    for line in path.read_text().splitlines():
        words = line.split("#", 1)[0].split()
        if len(words) >= 2:
            out[words[0]] = float(words[1])
    return out


def video_params(video: str) -> str:
    return (
        "# The video system's OSD grid (drone_duels.firmware.ardupilot).\n"
        f"OSD1_TXT_RES {txt_res(video)}\n"
    )


def _stop_process(proc: subprocess.Popen[bytes] | None, name: str) -> None:
    if proc is None or proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(5)
    except subprocess.TimeoutExpired:
        log.warning("%s did not stop: killing it", name)
        proc.kill()
        proc.wait(2)


class ArduCopter(Firmware):
    name = "ArduCopter"

    def __init__(self, ctx: SpawnContext, events: Events, physics: PhysicsProcess) -> None:
        super().__init__(ctx, events, physics)
        self.sitl: subprocess.Popen[bytes] | None = None
        self.router: subprocess.Popen[bytes] | None = None
        self.link: VehicleLink | None = None
        self.log_path = ctx.workdir / "arducopter.log"
        self.router_log = ctx.workdir / "router.log"
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._rc_sock: socket.socket | None = None
        self._osd_sock: socket.socket | None = None
        self._rc_lock = threading.Lock()
        self._rc_source = "idle"
        """Who feeds SITL's receiver: "idle" (nobody yet), "page", or "lost"."""
        self._stopped = False
        self._fly_lock = threading.Lock()
        self._fly_gen = 0
        """Each hover or land is one more: an older one still running sees it is overtaken."""
        self._display = DisplayPort(analog=ctx.video == "analog")
        self.osd_screens = 0

    # Start.

    def start(self) -> None:
        ctx = self.ctx
        for path in (ARDUCOPTER, ROUTER, COPTER_DEFAULTS):
            if not path.exists():
                raise FirmwareError(f"{path} is missing: run scripts/setup.sh")
        work = ctx.workdir
        work.mkdir(parents=True, exist_ok=True)
        (work / "eeprom.bin").unlink(missing_ok=True)
        video = work / "video.parm"
        video.write_text(video_params(ctx.video))
        conf = work / "router.conf"
        conf.write_text(router_conf(ctx.ports.slot))
        defaults = [COPTER_DEFAULTS, *self._param_files(ctx.video), video]

        self._osd_sock = self._bind_udp(ctx.ports.ardupilot_osd, "the OSD")
        self._rc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._thread(self._osd_loop, "ardupilot-osd")
        self._thread(self._rc_idle_loop, "ardupilot-rc")

        lat, lon, alt = ctx.origin
        args = [
            str(ARDUCOPTER),
            "--model",
            "JSON:127.0.0.1",
            "--speedup",
            "1",
            f"-I{ctx.ports.slot}",
            "--sysid",
            str(SYSID),
            "--home",
            f"{lat:.8f},{lon:.8f},{alt:.2f},{ctx.pad.heading_deg:.1f}",
            "--defaults",
            ",".join(str(d.resolve()) for d in defaults),
            "--serial5",
            f"udpclient:127.0.0.1:{ctx.ports.ardupilot_osd}",
        ]
        log.info("ArduCopter: %s", " ".join(args))
        with self.log_path.open("wb") as out:
            self.sitl = subprocess.Popen(
                args, cwd=work, stdout=out, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL
            )
        with self.router_log.open("wb") as out:
            self.router = subprocess.Popen(
                [str(ROUTER), "-c", str(conf)], stdout=out, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
            )
        self.link = VehicleLink(f"udpout:127.0.0.1:{ctx.ports.router_game}", self.events.publish,
                                sysid=SYSID)
        self.link.start()

    def _bind_udp(self, port: int, what: str) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("127.0.0.1", port))
        except OSError as exc:
            sock.close()
            raise FirmwareError(f"{what}'s UDP port {port} is in use: {exc}") from None
        sock.settimeout(0.2)
        return sock

    def _thread(self, target: Callable[[], None], name: str) -> None:
        t = threading.Thread(target=target, name=name, daemon=True)
        t.start()
        self._threads.append(t)

    # Ready.

    def wait_ready(self, step: Callable[[str], None]) -> None:
        link = self.link
        assert link is not None
        deadline = time.monotonic() + READY_S
        said = ""
        poll = 0.0
        while True:
            self._check(deadline)
            s = link.state
            if not link.heard():
                what = "waiting for ArduCopter"
            elif not s.gps_ok:
                what = "waiting for GPS"
            elif not s.ekf_ok:
                what = "waiting for the EKF"
            elif not s.prearm_ok:
                blockers = s.blocker_list(time.monotonic())
                what = "pre-arm checks" + (f": {blockers[0]}" if blockers else "")
                if time.monotonic() - poll > PREARM_POLL_S:
                    poll = time.monotonic()
                    link.run_prearm_checks()
            else:
                return
            if what != said:
                said = what
                step(what)
            link.wait(link.ready, 0.25)

    def _check(self, deadline: float) -> None:
        self.ctx.check()
        if self.sitl is not None and self.sitl.poll() is not None:
            raise FirmwareError(
                f"ArduCopter SITL exited ({self.sitl.returncode}): see {self.log_path}"
            )
        if self.router is not None and self.router.poll() is not None:
            raise FirmwareError(
                f"mavlink-router exited ({self.router.returncode}): see {self.router_log}"
            )
        if not self.physics.alive():
            raise FirmwareError(
                f"the physics stopped: see {self.ctx.workdir / 'physics.log'}"
            )
        if time.monotonic() > deadline:
            raise FirmwareError(
                f"ArduCopter was not ready to arm within {READY_S:.0f} s: see {self.log_path}"
            )

    # RC.

    def send_rc(self, us: Sequence[int]) -> None:
        with self._rc_lock:
            if self._rc_source == "lost" and self.link is not None:
                self.link.set_param("SIM_RC_FAIL", 0)
            self._rc_source = "page"
            self._send_rc(us)

    def rc_lost(self) -> None:
        with self._rc_lock:
            if self._rc_source == "lost":
                return
            self._rc_source = "lost"
            if self.link is not None:
                self.link.set_param("SIM_RC_FAIL", 1)

    def _send_rc(self, us: Sequence[int]) -> None:
        sock = self._rc_sock
        if sock is None:
            return
        try:
            sock.sendto(rc_packet(us), ("127.0.0.1", self.ctx.ports.ardupilot_rc))
        except OSError:
            pass

    def _rc_idle_loop(self) -> None:
        while not self._stop.wait(1 / RC_IDLE_HZ):
            with self._rc_lock:
                if self._rc_source == "idle":
                    self._send_rc(IDLE_US)

    # Flying itself, as a ground station would (the module's docstring).

    def hover(self, height_m: float) -> None:
        self._fly(lambda gen: self._hover(gen, height_m), "ardupilot-hover")

    def land(self) -> None:
        self._fly(self._land, "ardupilot-land")

    def go(
        self,
        point: tuple[float, float, float],
        vel_ned: tuple[float, float, float] | None,
        yaw_deg: float | None,
    ) -> None:
        # In GUIDED since the hover; a later hover or land takes it from here.
        if self.link is not None:
            self.link.set_target(point, vel_ned, yaw_deg)

    def _fly(self, run: Callable[[int], None], name: str) -> None:
        with self._fly_lock:
            self._fly_gen += 1
            gen = self._fly_gen
        self._thread(lambda: run(gen), name)

    def _current(self, gen: int) -> bool:
        """Still the latest hover or land, and running."""
        return gen == self._fly_gen and not self._stop.is_set()

    def _say(self, level: str, text: str) -> None:
        getattr(log, "warning" if level == "warn" else level)("%s", text)
        self.events.publish({"ch": "log", "level": level, "source": "session", "text": text})

    def _hover(self, gen: int, height_m: float) -> None:
        link = self.link
        if link is None:
            return
        deadline = time.monotonic() + FLY_TRY_S
        why = ""
        said = ""
        while self._current(gen):
            if time.monotonic() > deadline:
                self._say("error", f"could not take off: {why or 'no answer'}")
                return
            if why and why != said:
                said = why
                self._say("warn", f"taking off: {why}; trying again")
            s = link.state
            up = s.armed and (s.alt_rel or 0.0) > CLIMB_SEEN_M
            result = link.set_mode("GUIDED")
            if result != apm.MAV_RESULT_ACCEPTED:
                why = f"GUIDED refused ({_result(result)})"
                self._stop.wait(RETRY_S)
                continue
            if not self._current(gen):
                return
            if up:
                assert s.lat is not None and s.lon is not None
                link.go_to(s.lat, s.lon, height_m)
                self._say("info", f"holding {height_m:.0f} m above the pad")
                return
            result = link.command_ack(apm.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            if result != apm.MAV_RESULT_ACCEPTED:
                blockers = s.blocker_list(time.monotonic())
                why = f"arming refused ({_result(result)})" + (
                    f": {blockers[0]}" if blockers else ""
                )
                self._stop.wait(RETRY_S)
                continue
            if not self._current(gen):
                return
            result = link.command_ack(apm.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, height_m)
            if result != apm.MAV_RESULT_ACCEPTED:
                why = f"takeoff refused ({_result(result)})"
                self._stop.wait(RETRY_S)
                continue
            if link.wait(lambda: not self._current(gen) or _climbing(link), CLIMB_SEEN_S):
                if self._current(gen):
                    self._say("info", f"taking off to {height_m:.0f} m above the pad")
                return
            why = "it did not climb"

    def _land(self, gen: int) -> None:
        link = self.link
        if link is None:
            return
        deadline = time.monotonic() + FLY_TRY_S
        while self._current(gen):
            result = link.set_mode("LAND")
            if result == apm.MAV_RESULT_ACCEPTED:
                self._say("info", "landing")
                return
            if time.monotonic() > deadline:
                self._say("error", f"could not land: LAND refused ({_result(result)})")
                return
            self._stop.wait(RETRY_S)

    # OSD.

    def _param_files(self, video: VideoSystem) -> list[Path]:
        paths = [self.ctx.firmware_file(name) for name in self.ctx.vehicle.firmware.params]
        for path in paths:
            if not path.exists():
                raise FirmwareError(f"{self.ctx.vehicle.id}'s parameter file {path} is missing")
        return settings_files(paths, video)

    def set_video(self, video: VideoSystem) -> bool:
        link = self.link
        if link is None:
            return False
        common = set(self._param_files("analog")) & set(self._param_files("digital"))
        for path in self._param_files(video):
            if path not in common:
                for name, value in read_params(path).items():
                    link.set_param(name, value)
        link.set_param("OSD1_TXT_RES", txt_res(video))
        self._display.analog = video == "analog"
        return True

    def _osd_loop(self) -> None:
        sock = self._osd_sock
        assert sock is not None
        reader = MspReader()
        display = self._display
        sent = 0.0
        while not self._stop.is_set():
            try:
                data = sock.recv(4096)
            except TimeoutError:
                continue
            except OSError:
                return
            drawn = False
            for command, payload in reader.feed(data):
                if command == MSP_DISPLAYPORT and display.handle(payload):
                    drawn = True
            now = time.monotonic()
            if drawn and display.screen is not None and now - sent >= OSD_MIN_GAP_S:
                sent = now
                self.osd_screens += 1
                self.events.publish(osd_message(display.screen))

    # Stop.

    def alive(self) -> bool:
        return all(p is not None and p.poll() is None for p in (self.sitl, self.router))

    def stop(self) -> None:
        if self._stopped:
            return
        self._stopped = True
        self._stop.set()
        if self.link is not None:
            self.link.stop()
        _stop_process(self.router, "mavlink-router")
        _stop_process(self.sitl, "ArduCopter SITL")
        for t in self._threads:
            t.join(2)
        for sock in (self._rc_sock, self._osd_sock):
            if sock is not None:
                sock.close()
        self._rc_sock = self._osd_sock = None
        log.info("ArduCopter stopped")

