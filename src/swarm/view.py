"""Serve the 3D mission viewer: the watcher's telemetry, relayed to browsers.

The watcher publishes one JSON snapshot per step to UDP `DEFAULT_PORT` (swarm.telemetry),
plus each vehicle's plan (swarm.mission: mission, fence, rally points) when it changes.
ViewServer listens there, or replays a telemetry log, turns each snapshot into a geodetic
frame and streams frames and plans to every open page as Server-Sent Events. In the sim it
also streams every copter's physics (swarm.physics) to a page showing a camera feed, each new
state as it comes, unbuffered: what a camera would see, not the telemetry; and every copter's
ArduPilot OSD (swarm.osd), each screen as it is drawn, to lay over it. The page is
swarm/web/, with its drone and truck models (swarm.fpvmodel, swarm.truckmodel), and
drive.html beside it, a truck to drive on the map's 3D tiles (`swarm drive`). The two pages
meet in the duel peer to peer, browser to browser (duel.js), not through this server. The
libraries they load (CesiumJS, milsymbol, mgrs, Rapier, Trystero) are in external/web/,
fetched by scripts/setup.sh. Map keys come from the environment or .env.local.
"""

import json
import logging
import os
import queue
import socket
import threading
import time
from collections import deque
from collections.abc import Iterator
from dataclasses import asdict, dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Protocol, TypedDict

from swarm.geo import LocalFrame
from swarm.launch import LEADER_RADIUS_M
from swarm.osd import OsdFeed, font_png
from swarm.physics import PhysicsFeed
from swarm.telemetry import DEFAULT_PORT

log = logging.getLogger(__name__)

ROOT = Path(__file__).resolve().parents[2]
WEB = Path(__file__).resolve().parent / "web"
VENDOR = ROOT / "external" / "web"
ENV_FILE = ROOT / ".env.local"
DEFAULT_HTTP_PORT = 8600
HISTORY_S = 120.0
"""Seconds of past frames a newly opened page receives, so trails survive a reload."""
HISTORY_HZ = 2.0
KEEPALIVE_S = 15.0
FEED_POLL_S = 0.002
"""How often a feed stream (/physics, /osd) looks for new states: at most this much added
delay."""
RADIO_POLL_S = 0.01
"""How often /radio looks at the radio: the truck's controls, 100 times a second at most."""
MAP_KEYS = ("CESIUM_ION_TOKEN", "GOOGLE_MAPS_API_KEY")
"""Optional map provider keys, read from the environment or .env.local."""
CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".mjs": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json",
    ".glb": "model/gltf-binary",
    ".gltf": "model/gltf+json",
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".svg": "image/svg+xml",
    ".wasm": "application/wasm",
    ".xml": "application/xml",
    ".ktx2": "image/ktx2",
}


class SimControl(Protocol):
    """The sim's copters, for the duel: a crashed drone is destroyed, and every one comes back
    for its pilot's next turn (swarm.sim.SimStack)."""

    def destroy(self, sysid: int) -> bool: ...

    def reset(self) -> None: ...


class RadioSource(Protocol):
    """The radio's channels for the drive page's truck (swarm.radio.RadioReader)."""

    def channels(self) -> list[float] | None: ...

    def stop(self) -> None: ...


class Feed(Protocol):
    def latest(self) -> dict[int, tuple[int, Any]]:
        """Per copter: how many states so far, and the last, a dataclass."""
        ...


class Marker(TypedDict):
    name: str
    lat: float
    lon: float


class ViewVehicle(TypedDict):
    sysid: int
    link: bool
    """Its link is up: heard from lately. A destroyed drone's last report stays, link down."""
    nav: bool
    """Its EKF can navigate: a sim copter just restarted cannot take off until it can."""
    mode: str
    armed: bool
    pilot: bool
    """A human is flying it: armed in a pilot mode."""
    role: str
    lat: float
    lon: float
    alt: float
    """Height above the vehicle's home, metres."""
    vn: float
    ve: float
    vd: float
    heading: float
    roll: float
    pitch: float
    battery: int | None
    battery_v: float | None
    sticks: list[int]
    """RC input of channels 1-4 (roll, pitch, throttle, yaw) in µs; empty until reported."""
    slot: list[float] | None
    """The formation slot ArduPilot FOLLOW flies to, as [lat, lon]; None when not following."""
    wp: int | None
    """Current mission item (MISSION_CURRENT seq)."""
    mission_state: int


class ViewFrame(TypedDict):
    t: float
    phase: str
    leader: int | None
    origin: list[float]
    vehicles: list[ViewVehicle]
    target: list[float] | None
    """Point B as [lat, lon], when the tracker is reporting."""


def to_view_frame(snap: dict[str, Any], fallback_origin: LocalFrame) -> ViewFrame:
    """Georeference a telemetry snapshot. Logs written before snapshots carried their
    origin are placed at `fallback_origin`."""
    frame = LocalFrame(
        snap.get("origin_lat", fallback_origin.origin_lat_deg),
        snap.get("origin_lon", fallback_origin.origin_lon_deg),
    )
    vehicles: list[ViewVehicle] = []
    for v in snap["vehicles"]:
        lat, lon = frame.to_latlon(v["n"], v["e"])
        slot_n, slot_e = v.get("slot_n"), v.get("slot_e")
        vehicles.append(
            ViewVehicle(
                sysid=v["sysid"],
                link=v.get("link", True),
                nav=v.get("nav", True),
                mode=v["mode"],
                armed=v["armed"],
                pilot=v.get("pilot", False),
                role=v["role"],
                lat=lat,
                lon=lon,
                alt=-v["d"],
                vn=v["vn"],
                ve=v["ve"],
                vd=v["vd"],
                heading=v["heading"],
                roll=v.get("roll", 0.0),
                pitch=v.get("pitch", 0.0),
                battery=v.get("battery"),
                battery_v=v.get("battery_v"),
                sticks=v.get("sticks", []),
                slot=None
                if slot_n is None or slot_e is None
                else list(frame.to_latlon(slot_n, slot_e)),
                wp=v.get("wp"),
                mission_state=v.get("mission_state", 0),
            )
        )
    tn, te = snap.get("target_n"), snap.get("target_e")
    return ViewFrame(
        t=snap["t"],
        phase=snap["phase"],
        leader=snap["leader"],
        origin=[frame.origin_lat_deg, frame.origin_lon_deg],
        vehicles=vehicles,
        target=None if tn is None or te is None else list(frame.to_latlon(tn, te)),
    )


def read_env_file(path: Path) -> dict[str, str]:
    """KEY=VALUE lines; blank lines, comments and malformed lines are skipped."""
    if not path.exists():
        return {}
    values: dict[str, str] = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key.strip()] = value.strip().strip("\"'")
    return values


def replay_frames(path: Path, speed: float = 1.0) -> Iterator[tuple[float, dict[str, Any]]]:
    """(delay before this snapshot in seconds, snapshot) for each line of a telemetry log."""
    last_t: float | None = None
    with path.open() as f:
        for line in f:
            if not line.strip():
                continue
            snap: dict[str, Any] = json.loads(line)
            t = float(snap["t"])
            delay = 0.0 if last_t is None else max(0.0, t - last_t) / speed
            last_t = t
            yield delay, snap


Event = tuple[str, str]
"""(SSE event name, JSON data). "frame" goes out as a plain message."""


class Hub:
    """Fans frames and plans out to connected pages. Keeps a short, thinned history of frames
    and the latest plan per vehicle for pages that open later."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._clients: set[queue.Queue[Event]] = set()
        self._history: deque[tuple[float, str]] = deque()
        self._plans: dict[int, str] = {}
        self._last_kept = 0.0

    def publish(self, frame: ViewFrame) -> None:
        data = json.dumps(frame)
        now = time.monotonic()
        with self._lock:
            if now - self._last_kept >= 1.0 / HISTORY_HZ:
                self._last_kept = now
                self._history.append((now, data))
            while self._history and now - self._history[0][0] > HISTORY_S:
                self._history.popleft()
            clients = list(self._clients)
        self._fan_out(clients, ("frame", data))

    def publish_plan(self, plan: dict[str, Any]) -> None:
        data = json.dumps(plan)
        with self._lock:
            self._plans[int(plan["sysid"])] = data
            clients = list(self._clients)
        self._fan_out(clients, ("plan", data))

    @staticmethod
    def _fan_out(clients: list[queue.Queue[Event]], event: Event) -> None:
        for q in clients:
            if q.full():
                try:
                    q.get_nowait()
                except queue.Empty:
                    pass
            q.put_nowait(event)

    def subscribe(self) -> tuple[queue.Queue[Event], list[str], list[str]]:
        """A queue of live events, the frame history, and the latest plan per vehicle."""
        q: queue.Queue[Event] = queue.Queue(maxsize=50)
        with self._lock:
            self._clients.add(q)
            history = [d for _, d in self._history]
            plans = list(self._plans.values())
        return q, history, plans

    def unsubscribe(self, q: queue.Queue[Event]) -> None:
        with self._lock:
            self._clients.discard(q)

    def clear_history(self) -> None:
        with self._lock:
            self._history.clear()
            self._plans.clear()


@dataclass
class ViewConfig:
    markers: list[Marker]
    land_radius_m: float = 25.0
    leader_radius_m: float = LEADER_RADIUS_M
    """Followers lift off only this close to the leader; drawn around it."""
    point_b: tuple[float, float] | None = None
    """Where the duel's truck starts, [lat, lon]; the telemetry's point B wins once it comes."""
    fence: list[tuple[float, float]] = field(default_factory=list[tuple[float, float]])
    """The fence's corners, [lat, lon] in order round; empty with the fence off. The duel's
    truck must keep inside it."""
    vtx_mw: float = 25.0
    """The camera feed's video transmitter power (web/analog.js)."""
    host: str = "127.0.0.1"
    http_port: int = DEFAULT_HTTP_PORT
    telemetry_port: int = DEFAULT_PORT


class ViewServer:
    """HTTP + SSE server for the viewer, fed from live UDP telemetry or a replayed log."""

    def __init__(
        self,
        config: ViewConfig,
        fallback_origin: LocalFrame,
        replay: Path | None = None,
        replay_speed: float = 1.0,
        replay_loop: bool = False,
    ) -> None:
        self.config = config
        self.fallback_origin = fallback_origin
        self.replay = replay
        self.replay_speed = replay_speed
        self.replay_loop = replay_loop
        self.hub = Hub()
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._httpd: ThreadingHTTPServer | None = None
        self.physics: PhysicsFeed | None = None
        """Every copter's physics in the sim, for /physics; None elsewhere."""
        self.osd: OsdFeed | None = None
        """Every copter's OSD in the sim, for /osd; None elsewhere."""
        self.sim: SimControl | None = None
        """The sim's copters, for the duel's /sim/destroy and /sim/reset; None elsewhere."""
        self.radio: RadioSource | None = None
        """The radio, for /radio: read from the first page that asks (radio_source)."""
        self._radio_lock = threading.Lock()

    @property
    def url(self) -> str:
        host = "localhost" if self.config.host in ("127.0.0.1", "0.0.0.0") else self.config.host
        return f"http://{host}:{self.config.http_port}/"

    @property
    def running(self) -> bool:
        return not self._stop.is_set()

    def page_config(self) -> dict[str, Any]:
        env = read_env_file(ENV_FILE)
        env.update({k: v for k, v in os.environ.items() if k in MAP_KEYS})
        return {
            "cesiumIonToken": env.get("CESIUM_ION_TOKEN") or None,
            "googleMapsApiKey": env.get("GOOGLE_MAPS_API_KEY") or None,
            "markers": self.config.markers,
            "landRadiusM": self.config.land_radius_m,
            "leaderRadiusM": self.config.leader_radius_m,
            "pointB": self.config.point_b,
            "fence": self.config.fence,
            "vtxMw": self.config.vtx_mw,
            "replay": self.replay.name if self.replay else None,
            "sim": self.sim is not None,
        }

    def start(self) -> None:
        """Bind the HTTP and telemetry ports (OSError if either is taken), then serve."""
        sock: socket.socket | None = None
        if self.replay is None:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(("127.0.0.1", self.config.telemetry_port))
        try:
            self._httpd = ThreadingHTTPServer(
                (self.config.host, self.config.http_port), _handler_for(self)
            )
        except OSError:
            if sock is not None:
                sock.close()
            raise
        self._httpd.daemon_threads = True
        source = self._replay if sock is None else lambda: self._listen(sock)
        for target, name in ((self._httpd.serve_forever, "view-http"), (source, "view-source")):
            t = threading.Thread(target=target, name=name, daemon=True)
            t.start()
            self._threads.append(t)
        libraries = ("cesium", "milsymbol", "mgrs", "trystero")
        missing = [p for p in libraries if not (VENDOR / p).exists()]
        if missing:
            log.warning("viewer libraries missing (%s); run scripts/setup.sh", ", ".join(missing))
        log.info("3D viewer: %s", self.url)

    def radio_source(self) -> RadioSource:
        """The radio, read from now on if it is not yet."""
        with self._radio_lock:
            if self.radio is None:
                from swarm.radio import RadioReader

                reader = RadioReader()
                reader.start()
                self.radio = reader
            return self.radio

    def stop(self) -> None:
        self._stop.set()
        if self.radio is not None:
            self.radio.stop()
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
        for t in self._threads:
            t.join(timeout=2.0)

    def _publish(self, snap: dict[str, Any]) -> None:
        if snap.get("kind") == "plan":
            if isinstance(snap.get("sysid"), int):
                self.hub.publish_plan(snap)
            return
        try:
            frame = to_view_frame(snap, self.fallback_origin)
        except (KeyError, TypeError, ValueError) as exc:
            log.debug("skipping malformed snapshot: %s", exc)
            return
        self.hub.publish(frame)

    def _listen(self, sock: socket.socket) -> None:
        sock.settimeout(0.5)
        log.info("viewer listening for telemetry on udp:127.0.0.1:%d", self.config.telemetry_port)
        while not self._stop.is_set():
            try:
                data = sock.recv(65535)
            except TimeoutError:
                continue
            try:
                self._publish(json.loads(data))
            except json.JSONDecodeError:
                continue
        sock.close()

    def _replay(self) -> None:
        assert self.replay is not None
        while not self._stop.is_set():
            log.info("replaying %s at %gx", self.replay, self.replay_speed)
            self.hub.clear_history()
            for delay, snap in replay_frames(self.replay, self.replay_speed):
                if self._stop.wait(delay):
                    return
                self._publish(snap)
            if not self.replay_loop:
                log.info("replay finished")
                return
            self._stop.wait(3.0)


def _handler_for(server: ViewServer) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        # Each event out at once, not held back to share a packet with the next.
        disable_nagle_algorithm = True

        def log_message(self, format: str, *args: object) -> None:
            log.debug("http: " + format, *args)

        def do_GET(self) -> None:  # noqa: N802
            path = self.path.split("?", 1)[0]
            if path == "/events":
                self._events()
            elif path == "/physics":
                self._feed(server.physics)
            elif path == "/osd":
                self._feed(server.osd)
            elif path == "/radio":
                self._radio()
            elif path.startswith("/osd/font/") and path.endswith(".png"):
                index = path.removeprefix("/osd/font/").removesuffix(".png")
                png = font_png(int(index)) if index.isdigit() else None
                if png:
                    self._send(png, ".png")
                else:
                    self.send_error(HTTPStatus.NOT_FOUND)
            elif path == "/config.json":
                self._send(json.dumps(server.page_config()).encode(), ".json")
            elif path.startswith("/vendor/"):
                self._file(VENDOR, path.removeprefix("/vendor/"))
            else:
                self._file(WEB, path.lstrip("/") or "index.html")

        def do_POST(self) -> None:  # noqa: N802
            """The duel's sim controls. Only from this machine, and only as JSON: a web page
            elsewhere cannot send JSON here without a CORS preflight, which is refused."""
            path = self.path.split("?", 1)[0]
            sim = server.sim
            if path not in ("/sim/destroy", "/sim/reset") or sim is None:
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            loopback = self.client_address[0] in ("127.0.0.1", "::1")
            json_body = self.headers.get("Content-Type", "").startswith("application/json")
            if not loopback or not json_body:
                self.send_error(HTTPStatus.FORBIDDEN)
                return
            length = int(self.headers.get("Content-Length") or 0)
            try:
                body: Any = json.loads(self.rfile.read(length) or b"{}")
            except json.JSONDecodeError:
                self.send_error(HTTPStatus.BAD_REQUEST)
                return
            if path == "/sim/destroy":
                sysid = body.get("sysid") if isinstance(body, dict) else None
                if not isinstance(sysid, int):
                    self.send_error(HTTPStatus.BAD_REQUEST)
                    return
                done = sim.destroy(sysid)
            else:
                # SITL takes a few seconds to stop and start again: answer at once.
                threading.Thread(target=sim.reset, name="sim-reset", daemon=True).start()
                done = True
            self._send(json.dumps({"ok": done}).encode(), ".json")

        def _send(self, body: bytes, suffix: str) -> None:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", CONTENT_TYPES.get(suffix, "application/octet-stream"))
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-cache" if suffix != ".glb" else "max-age=3600")
            self.end_headers()
            self.wfile.write(body)

        def _file(self, root: Path, rel: str) -> None:
            path = (root / rel).resolve()
            if not path.is_relative_to(root.resolve()) or not path.is_file():
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            self._send(path.read_bytes(), path.suffix)

        def _events(self) -> None:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            q, history, plans = server.hub.subscribe()
            try:
                if history:
                    self.wfile.write(f"event: history\ndata: [{','.join(history)}]\n\n".encode())
                for plan in plans:
                    self.wfile.write(f"event: plan\ndata: {plan}\n\n".encode())
                while server.running:
                    try:
                        name, data = q.get(timeout=KEEPALIVE_S)
                        prefix = "" if name == "frame" else f"event: {name}\n"
                        self.wfile.write(f"{prefix}data: {data}\n\n".encode())
                    except queue.Empty:
                        self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                server.hub.unsubscribe(q)

        def _radio(self) -> None:
            """The radio's channels, each change as it comes: [-1..1, ...] from channel 1, or
            null with no radio."""
            radio = server.radio_source()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            sent: object = ()
            idle_s = 0.0
            try:
                while server.running:
                    channels = radio.channels()
                    if channels != sent:
                        sent = channels
                        idle_s = 0.0
                        self.wfile.write(f"data: {json.dumps(channels)}\n\n".encode())
                        self.wfile.flush()
                    elif idle_s >= KEEPALIVE_S:
                        idle_s = 0.0
                        self.wfile.write(b": keepalive\n\n")
                        self.wfile.flush()
                    time.sleep(RADIO_POLL_S)
                    idle_s += RADIO_POLL_S
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _feed(self, feed: Feed | None) -> None:
            """Every copter's state from `feed`, each new one as soon as it arrives: one event
            of {sysid: state} for the copters that moved on since the last."""
            if feed is None:
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            sent: dict[int, int] = {}
            idle_s = 0.0
            try:
                while server.running:
                    new = {
                        sysid: (count, state)
                        for sysid, (count, state) in feed.latest().items()
                        if sent.get(sysid) != count
                    }
                    if new:
                        sent.update({sysid: count for sysid, (count, _) in new.items()})
                        idle_s = 0.0
                        data = json.dumps({sysid: asdict(st) for sysid, (_, st) in new.items()})
                        self.wfile.write(f"data: {data}\n\n".encode())
                        self.wfile.flush()
                    elif idle_s >= KEEPALIVE_S:
                        idle_s = 0.0
                        self.wfile.write(b": keepalive\n\n")
                        self.wfile.flush()
                    time.sleep(FEED_POLL_S)
                    idle_s += FEED_POLL_S
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler
