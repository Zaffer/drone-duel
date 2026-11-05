"""The game's server: the page, its REST API and the /ws stream (DESIGN.md §3, §8).

FastAPI on uvicorn, one process. The session (drone_duels.session) runs its vehicles on threads;
the Hub carries their streams onto the event loop and out to every open page. A page that falls
behind loses JSON messages past QUEUE_MAX and every pose but the newest, never the loop.
"""

import asyncio
import json
import logging
import os
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import ValidationError

from drone_duels import __version__, ports
from drone_duels.content.store import Content
from drone_duels.content.store import load as load_content
from drone_duels.rc.pilot import Pilot
from drone_duels.server import prefs
from drone_duels.server.protocol import (
    ConfigMsg,
    ContentMsg,
    DynamicsMsg,
    GroundMsg,
    MapKeys,
    RadioSourceMsg,
    RcMsg,
    SessionMsg,
    VideoMsg,
    WindMsg,
    client_msg,
)
from drone_duels.session.session import Session
from drone_duels.video.osd import BETAFLIGHT_FONT, font_png, mcm_png

log = logging.getLogger(__name__)

ROOT = Path(__file__).resolve().parents[3]
WEB = ROOT / "web"
DIST = WEB / "dist"
PUBLIC = WEB / "public"
CESIUM = WEB / "node_modules" / "cesium" / "Build" / "Cesium"
VENDOR = ROOT / "external" / "web"
ENV_FILE = ROOT / ".env.local"
QUEUE_MAX = 512
"""JSON messages a page may fall behind by before it misses some."""
CACHED = ("status", "telemetry", "osd", "plan", "physics")
"""Channels whose last message a page gets as it connects."""


class Client:
    def __init__(self, ws: WebSocket) -> None:
        self.ws = ws
        self.queue: asyncio.Queue[str] = asyncio.Queue(QUEUE_MAX)
        self.pose: bytes | None = None
        self.wake = asyncio.Event()

    def put(self, text: str) -> None:
        try:
            self.queue.put_nowait(text)
        except asyncio.QueueFull:
            pass
        self.wake.set()

    def put_pose(self, frame: bytes) -> None:
        self.pose = frame
        self.wake.set()

    async def pump(self) -> None:
        while True:
            await self.wake.wait()
            self.wake.clear()
            while not self.queue.empty():
                await self.ws.send_text(self.queue.get_nowait())
            pose, self.pose = self.pose, None
            if pose is not None:
                await self.ws.send_bytes(pose)


class Hub:
    """Session events, from any thread, to every page (session.base.Events)."""

    def __init__(self) -> None:
        self.loop: asyncio.AbstractEventLoop | None = None
        self.clients: set[Client] = set()
        self.last: dict[str, str] = {}

    def publish(self, msg: dict[str, Any]) -> None:
        text = json.dumps(msg, separators=(",", ":"))
        ch = msg.get("ch", "")
        if ch in CACHED:
            self.last[ch] = text
        if ch == "status" and msg.get("state") != "ready":
            for stale in ("telemetry", "osd", "plan"):
                self.last.pop(stale, None)
        if self.loop is not None:
            self.loop.call_soon_threadsafe(self._fan_out, text)

    def publish_pose(self, frame: bytes) -> None:
        if self.loop is not None:
            self.loop.call_soon_threadsafe(self._fan_out_pose, frame)

    def _fan_out(self, text: str) -> None:
        for c in self.clients:
            c.put(text)

    def _fan_out_pose(self, frame: bytes) -> None:
        for c in self.clients:
            c.put_pose(frame)


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


def local_origin(origin: str | None) -> bool:
    """Whether a WebSocket comes from the game's own page (or from no page at all: a test). A web
    page elsewhere could otherwise open /ws from the player's browser and fly their sim."""
    if origin is None:
        return True
    from urllib.parse import urlsplit

    return urlsplit(origin).hostname in ("localhost", "127.0.0.1", "::1")


def map_keys() -> MapKeys:
    env = read_env_file(ENV_FILE)
    env.update({k: v for k, v in os.environ.items() if k.endswith(("_API_KEY", "_TOKEN"))})
    return MapKeys(
        google_maps_api_key=env.get("GOOGLE_MAPS_API_KEY") or None,
        cesium_ion_token=env.get("CESIUM_ION_TOKEN") or None,
    )


def create_app(
    content: Content | None = None, session: Session | None = None, slot: int = 0
) -> FastAPI:
    """The app; its vehicles fly in `slot` (a second game on the same machine takes another)."""
    content = content or load_content()
    hub = Hub()
    session = session or Session(content, hub, slot=slot)
    session.events = hub
    pilot = Pilot(session, hub.publish, content, prefs.load())

    @asynccontextmanager
    async def lifespan(_app: FastAPI) -> AsyncIterator[None]:
        hub.loop = asyncio.get_running_loop()
        session.start()
        hub.publish(session.status())
        try:
            yield
        finally:
            await asyncio.to_thread(session.close)
            pilot.close()

    app = FastAPI(title="Drone Duels", version=__version__, lifespan=lifespan)
    app.state.hub = hub
    app.state.session = session

    @app.get("/api/content")
    def get_content() -> ContentMsg:
        return ContentMsg(
            locations=content.locations,
            vehicles=content.vehicles,
            courses=content.courses,
            controllers=content.controllers,
        )

    @app.get("/api/config")
    def get_config() -> ConfigMsg:
        return ConfigMsg(map_keys=map_keys(), qgc_port=ports.ROUTER_QGC, version=__version__)

    @app.get("/api/prefs")
    def get_prefs() -> dict[str, Any]:
        return prefs.load()

    @app.put("/api/prefs")
    async def put_prefs(request: Request) -> dict[str, Any]:
        try:
            data = await request.json()
        except ValueError:
            raise HTTPException(400, "prefs must be JSON") from None
        if not isinstance(data, dict):
            raise HTTPException(400, "prefs must be a JSON object")
        prefs.save(data)
        pilot.set_prefs(data)
        return data

    @app.get("/osd/font/{name}.png")
    def osd_font(name: str) -> Response:
        png = mcm_png(BETAFLIGHT_FONT) if name == "betaflight" else (
            font_png(int(name)) if name.isdigit() else None
        )
        if png is None:
            raise HTTPException(404, f"no OSD font {name}")
        return Response(png, media_type="image/png", headers={"Cache-Control": "max-age=3600"})

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket) -> None:
        if not local_origin(ws.headers.get("origin")):
            await ws.close(code=1008)  # another site's page in the same browser
            return
        await ws.accept()
        client = Client(ws)
        hub.clients.add(client)
        for text in hub.last.values():
            client.put(text)
        pump = asyncio.create_task(client.pump())
        try:
            while True:
                text = await ws.receive_text()
                msg = handle(session, text)
                if isinstance(msg, RadioSourceMsg):
                    pilot.want(client, msg.on, msg.fly)
        except WebSocketDisconnect:
            pass
        finally:
            hub.clients.discard(client)
            pilot.want(client, False)
            pump.cancel()

    if (CESIUM / "Cesium.js").exists():
        app.mount("/cesium", StaticFiles(directory=CESIUM), name="cesium")
    if VENDOR.is_dir():
        app.mount("/vendor", StaticFiles(directory=VENDOR), name="vendor")
    app.mount("/models", StaticFiles(directory=PUBLIC / "models"), name="models")

    @app.get("/")
    def index() -> FileResponse:
        page = DIST / "index.html"
        if not page.exists():
            raise HTTPException(503, "the page is not built: run `cd web && bun run build`")
        return FileResponse(page, headers={"Cache-Control": "no-cache"})

    if DIST.is_dir():
        app.mount("/", StaticFiles(directory=DIST), name="dist")
    return app


def handle(session: Session, text: str) -> object:
    """One page → server message; returns it (None if it was not valid)."""
    try:
        msg = client_msg.validate_json(text)
    except ValidationError as exc:
        log.warning("bad message from the page: %s", exc.errors()[:1])
        # Said on the page too: a page newer than its server (rebuilt under it) is otherwise a
        # silent hang, its start never answered.
        session.events.publish(
            {
                "ch": "log",
                "level": "error",
                "source": "server",
                "text": "The server did not understand the page: restart `uv run duels`.",
            }
        )
        return None
    match msg:
        case RcMsg():
            session.send_rc(msg.us)
        case GroundMsg():
            session.set_ground(msg.lat, msg.lon, msg.alt)
        case SessionMsg():
            session.command(msg.action, msg.location, msg.vehicle, msg.pad)
        case WindMsg():
            session.set_wind(msg.wind, msg.gust_now)
        case VideoMsg():
            session.set_video(msg.system)
        case DynamicsMsg():
            session.set_dynamics(msg.on)
        case RadioSourceMsg():
            pass  # the connection's own (ws_endpoint)
    return msg
