"""Launch the simulation stack: N ArduCopter SITL instances, mavlink-router, and optionally
QGroundControl.

The SITL binaries are started directly, headless, with the same arguments that
`sim_vehicle.py` would use. Each instance runs in its own directory under logs/sitl/.
"""

import logging
import shutil
import signal
import socket
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from types import FrameType

from swarm.elevation import ElevationError, ground_elevation_m
from swarm.geo import LocalFrame
from swarm.osd import osd_port

log = logging.getLogger(__name__)

ROOT = Path(__file__).resolve().parents[2]
EXTERNAL = ROOT / "external"
ARDUCOPTER = EXTERNAL / "ardupilot" / "build" / "sitl" / "bin" / "arducopter"
DEFAULT_PARAMS = EXTERNAL / "ardupilot" / "Tools" / "autotest" / "default_params" / "copter.parm"
SWARM_PARAMS = ROOT / "sitl" / "swarm.parm"
FRAME_MODEL = ROOT / "sitl" / "fpv5.json"
"""The airframe SITL flies: a typical 5 inch quad X (SIM_Frame model). Copied into each
instance's directory: SITL opens files only under its working directory."""
FRAME_PARAMS = ROOT / "sitl" / "fpv5.parm"
"""That airframe's tune, between ArduPilot's SITL defaults and the mission's swarm.parm."""
ROUTER = EXTERNAL / "mavlink-router" / "build" / "src" / "mavlink-routerd"
ROUTER_CONFS = {"sitl": ROOT / "router" / "sitl.conf", "field": ROOT / "router" / "field.conf"}
"""sitl: the router connects to each SITL's TCP port. field: each SITL sends its MAVLink over
UDP to the laptop, as the drones' Wi-Fi bridges do, through the field laptop's own config."""
FIELD_DRONE_BASE_PORT = 14550
"""router/field.conf: drone N sends to UDP 14550 + N."""
WATCHER_UDP_PORT = 14560
"""The router's watcher endpoint, bound in both configs."""
QGC = EXTERNAL / "qgc" / "AppRun"
"""QGroundControl unpacked from its AppImage by scripts/setup.sh."""
LOGS = ROOT / "logs"


@dataclass(frozen=True)
class SimField:
    """Where the mission flies (mission.toml). Copter 1 spawns at `home`, the others in a line
    to the east.
    The pilot flies the swarm to A, holds it there, then flies each drone on to B and lands it
    (README, The mission). The 3D viewer marks both."""

    home: LocalFrame
    home_alt_m: float | None
    """Ground elevation AMSL at home; None looks it up when the sim starts (swarm.elevation)."""
    point_a: tuple[float, float]
    """Where the swarm flies in formation and holds."""
    point_b: tuple[float, float]
    """Where each drone is flown on to and landed, one by one."""

    def moved_to(self, lat_deg: float, lon_deg: float, alt_m: float | None) -> "SimField":
        """The same field at another home: A and B keep their offsets from home."""
        home = LocalFrame(lat_deg, lon_deg)
        return SimField(
            home,
            alt_m,
            home.to_latlon(*self.home.to_ne(*self.point_a)),
            home.to_latlon(*self.home.to_ne(*self.point_b)),
        )

    def with_points(
        self, point_a: tuple[float, float] | None, point_b: tuple[float, float] | None
    ) -> "SimField":
        """This field with A or B, or both, put elsewhere."""
        return SimField(
            self.home, self.home_alt_m, point_a or self.point_a, point_b or self.point_b
        )


SITL_BASE_PORT = 5760
SITL_PORT_STRIDE = 10


class SimStackError(RuntimeError):
    pass


@dataclass
class SimStack:
    sim_field: SimField
    count: int = 5
    spawn_spacing_m: float = 5.0
    """Copter 1 starts at home, the others in a line to the east, this far apart."""
    sitl_params: dict[str, float] = field(default_factory=dict[str, float])
    """Parameters for every copter on top of the parameter files, such as the wind."""
    start_qgc: bool = True
    router: str = "sitl"
    """Which router config the copters connect through (ROUTER_CONFS)."""
    wipe_params: bool = True
    """Delete each instance's eeprom.bin so every run starts from the parameter files."""
    _procs: list[subprocess.Popen[bytes]] = field(default_factory=list, init=False)
    """mavlink-router and QGroundControl."""
    _copters: dict[int, subprocess.Popen[bytes]] = field(default_factory=dict, init=False)
    """Each copter's SITL, by instance index."""
    _copters_lock: threading.Lock = field(default_factory=threading.Lock, init=False)
    _home_alt_m: float = field(default=0.0, init=False)

    @property
    def home_alt_m(self) -> float:
        """Ground elevation AMSL at home, as the copters were started with."""
        return self._home_alt_m

    def __enter__(self) -> "SimStack":
        self.start()
        return self

    def __exit__(self, *exc: object) -> None:
        self.stop()

    def start(self) -> None:
        for path in (ARDUCOPTER, ROUTER):
            if not path.exists():
                raise SimStackError(f"{path} missing; run scripts/setup.sh")
        # Before any log or parameter file is touched: they belong to the stack running there.
        sitl_ports = [SITL_BASE_PORT + SITL_PORT_STRIDE * i for i in range(self.count)]
        busy = [f"TCP {p}" for p in ports_in_use(sitl_ports)]
        busy += [f"UDP {p}" for p in ports_in_use([WATCHER_UDP_PORT], socket.SOCK_DGRAM)]
        if busy:
            raise SimStackError(
                f"{', '.join(busy)} in use: another sim stack (swarm sim, fly or rehearse) or "
                "mavlink-router is running; stop it first"
            )
        if self.router not in ROUTER_CONFS:
            raise SimStackError(f"router config {self.router!r}: one of {sorted(ROUTER_CONFS)}")
        self._home_alt_m = self._ground_alt_m()
        LOGS.mkdir(exist_ok=True)
        for i in range(self.count):
            self._start_copter(i)
        router_log = (LOGS / "router.log").open("wb")
        self._procs.append(
            subprocess.Popen(
                [str(ROUTER), "-c", str(ROUTER_CONFS[self.router])],
                stdout=router_log,
                stderr=router_log,
            )
        )
        qgc = self._start_qgc()
        self._wait_for_router()
        if qgc is not None and qgc.poll() is not None:
            log.warning("QGroundControl exited with %d; see logs/qgc.log", qgc.returncode)

    def _ground_alt_m(self) -> float:
        if self.sim_field.home_alt_m is not None:
            return self.sim_field.home_alt_m
        home = self.sim_field.home
        try:
            return ground_elevation_m(home.origin_lat_deg, home.origin_lon_deg)
        except ElevationError as exc:
            raise SimStackError(str(exc)) from None

    def _start_qgc(self) -> subprocess.Popen[bytes] | None:
        if not self.start_qgc or _process_running("QGroundControl"):
            return None
        if not QGC.exists():
            log.warning("%s missing; run scripts/setup.sh", QGC)
            return None
        qgc_log = (LOGS / "qgc.log").open("wb")
        qgc = subprocess.Popen([str(QGC)], stdout=qgc_log, stderr=qgc_log)
        self._procs.append(qgc)
        return qgc

    def destroy(self, sysid: int) -> bool:
        """Stop copter `sysid` dead, as if destroyed: its link goes quiet where it was. False
        when there is no such copter, or it is down already."""
        with self._copters_lock:
            proc = self._copters.get(sysid - 1)
            if proc is None or proc.poll() is not None:
                return False
            proc.kill()
            proc.wait()
        log.info("copter %d destroyed", sysid)
        return True

    def reset(self) -> None:
        """Restart every copter on the ground where it started, destroyed ones too. Each keeps
        the parameters and fence the stack gave it; mavlink-router reconnects to it."""
        with self._copters_lock:
            for proc in self._copters.values():
                if proc.poll() is None:
                    proc.terminate()
            for index, proc in list(self._copters.items()):
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
                self._start_copter(index, wipe=False)
        log.info("copters reset to their starts")

    def _start_copter(self, index: int, wipe: bool | None = None) -> None:
        """Start copter `index`'s SITL; `wipe` its parameters, by default as `wipe_params`."""
        instance_dir = LOGS / "sitl" / str(index)
        instance_dir.mkdir(parents=True, exist_ok=True)
        if self.wipe_params if wipe is None else wipe:
            (instance_dir / "eeprom.bin").unlink(missing_ok=True)
        shutil.copyfile(FRAME_MODEL, instance_dir / FRAME_MODEL.name)
        defaults = [DEFAULT_PARAMS, FRAME_PARAMS, SWARM_PARAMS]
        if self.sitl_params:
            settings = instance_dir / "settings.parm"
            settings.write_text("".join(f"{k} {v}\n" for k, v in self.sitl_params.items()))
            defaults.append(settings)
        lat, lon = self.sim_field.home.to_latlon(0.0, self.spawn_spacing_m * index)
        cmd = [
            str(ARDUCOPTER),
            "--model",
            f"X:{FRAME_MODEL.name}",
            "--speedup",
            "1",
            "--slave",
            "0",
            "--sim-address=127.0.0.1",
            f"-I{index}",
            "--sysid",
            str(index + 1),
            "--home",
            f"{lat:.8f},{lon:.8f},{self._home_alt_m},0",
            "--defaults",
            ",".join(str(d) for d in defaults),
            # The true state, every physics step, for the camera feed (swarm.physics).
            "--enable-fgview",
            # The OSD over MSP DisplayPort, standing in for the OSD chip (swarm.osd).
            "--serial5",
            f"udpclient:127.0.0.1:{osd_port(index + 1)}",
        ]
        if self.router == "field":
            # As the drone's Wi-Fi bridge: MAVLink out over UDP to the laptop.
            link = f"udpclient:127.0.0.1:{FIELD_DRONE_BASE_PORT + index + 1}"
            cmd += ["--serial0", link]
        else:
            link = f"tcp:127.0.0.1:{SITL_BASE_PORT + SITL_PORT_STRIDE * index}"
        out = (instance_dir / "arducopter.log").open("wb")
        self._copters[index] = subprocess.Popen(cmd, cwd=instance_dir, stdout=out, stderr=out)
        log.info("copter %d: %s", index + 1, link)

    def _wait_for_router(self, timeout_s: float = 60.0) -> None:
        """sitl: the router connects to each SITL TCP port, and SITL boots once that
        connection exists. field: the router opens a UDP port per drone."""
        deadline = time.monotonic() + timeout_s
        router_log = LOGS / "router.log"
        opened_line = "Opened TCP Client" if self.router == "sitl" else "]drone"
        while time.monotonic() < deadline:
            self._check_alive()
            opened = router_log.read_text(errors="replace").count(opened_line)
            if opened >= self.count:
                log.info("router (%s.conf) open to %d copters", self.router, self.count)
                return
            time.sleep(0.5)
        raise SimStackError(f"mavlink-router ({self.router}.conf) did not open every copter")

    def _check_alive(self) -> None:
        for p in [*self._copters.values(), *self._procs]:
            if p.poll() is not None and p.args and str(p.args[0]) != str(QGC):  # type: ignore[index]
                raise SimStackError(f"{Path(str(p.args[0])).name} exited with {p.returncode}")  # type: ignore[index]

    def wait(self) -> None:
        """Block until Ctrl-C."""
        stop = False

        def on_signal(signum: int, frame: FrameType | None) -> None:
            nonlocal stop
            stop = True

        signal.signal(signal.SIGINT, on_signal)
        signal.signal(signal.SIGTERM, on_signal)
        while not stop:
            time.sleep(0.5)

    def stop(self) -> None:
        with self._copters_lock:
            procs = [*reversed(self._procs), *self._copters.values()]
            for p in procs:
                if p.poll() is None:
                    p.terminate()
            for p in procs:
                try:
                    p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    p.kill()
            self._procs.clear()
            self._copters.clear()


def ports_in_use(ports: list[int], kind: int = socket.SOCK_STREAM) -> list[int]:
    """The TCP (or UDP) ports among `ports` something already binds (SITL binds 0.0.0.0)."""
    busy = []
    for port in ports:
        with socket.socket(socket.AF_INET, kind) as sock:
            try:
                sock.bind(("0.0.0.0", port))
            except OSError:
                busy.append(port)
    return busy


def _process_running(name: str) -> bool:
    if shutil.which("pgrep") is None:
        return False
    return subprocess.run(["pgrep", "-x", name], capture_output=True).returncode == 0
