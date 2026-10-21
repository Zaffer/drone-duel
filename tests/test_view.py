"""The 3D viewer server: snapshot georeferencing, replay, history and HTTP routes."""

import json
import time
import urllib.error
import urllib.request
from collections.abc import Iterator
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from pymavlink.dialects.v20 import ardupilotmega as apm

from swarm.geo import LocalFrame
from swarm.mission import PlanReader
from swarm.state import VehicleState
from swarm.telemetry import Snapshot
from swarm.view import (
    HISTORY_HZ,
    Hub,
    Marker,
    ViewConfig,
    ViewServer,
    read_env_file,
    replay_frames,
    to_view_frame,
)
from swarm.watcher import Watcher

FRAME = LocalFrame(45.663099, -77.587167)
FALLBACK = LocalFrame(45.0, -77.0)


class FakeLink:
    def __init__(self, vehicles: list[VehicleState]) -> None:
        self.vehicles = {v.sysid: v for v in vehicles}
        self.target: VehicleState | None = None
        self.plans: dict[int, PlanReader] = {}

    def poll(self) -> None:
        pass

    def set_mode(self, sysid: int, mode: int) -> None:
        pass

    def send_velocity(self, sysid: int, velocity_ned: np.ndarray, yaw_rad: float) -> None:
        pass


def _flying(sysid: int, mode: int, north: float = 0.0, east: float = 0.0) -> VehicleState:
    now = time.monotonic()
    lat, lon = FRAME.to_latlon(north, east)
    return VehicleState(
        sysid=sysid,
        mode=mode,
        armed=True,
        gps_fix_type=3,
        ekf_flags=apm.EKF_POS_HORIZ_ABS,
        landed_state=apm.MAV_LANDED_STATE_IN_AIR,
        lat_deg=lat,
        lon_deg=lon,
        rel_alt_m=15.0,
        last_heartbeat_s=now,
        last_position_s=now,
    )


def _snapshot(**overrides: object) -> dict[str, Any]:
    snap: dict[str, Any] = {
        "t": 1.0,
        "origin_lat": FRAME.origin_lat_deg,
        "origin_lon": FRAME.origin_lon_deg,
        "leader": 1,
        "phase": "formation",
        "vehicles": [
            {
                "sysid": 1,
                "mode": "GUIDED",
                "armed": True,
                "role": "follower",
                "n": 100.0,
                "e": -50.0,
                "d": -15.0,
                "vn": 1.0,
                "ve": 2.0,
                "vd": 0.0,
                "heading": 0.5,
                "roll": 0.1,
                "pitch": -0.2,
                "battery": 80,
                "battery_v": 25.0,
                "slot_n": 90.0,
                "slot_e": -40.0,
            }
        ],
        "target_n": 300.0,
        "target_e": 20.0,
    }
    snap.update(overrides)
    return snap


class RecordingPublisher:
    def __init__(self) -> None:
        self.snapshots: list[Snapshot] = []

    def publish(self, snapshot: Snapshot) -> None:
        self.snapshots.append(snapshot)


def test_watcher_snapshot_georeferences_back_to_the_vehicle_position() -> None:
    v = _flying(1, apm.COPTER_MODE_LOITER, north=40.0, east=-25.0)
    v.roll_rad, v.pitch_rad, v.battery_pct, v.battery_v = 0.05, -0.1, 77, 24.8
    link = FakeLink([_flying(2, apm.COPTER_MODE_GUIDED), v])
    publisher = RecordingPublisher()
    coord = Watcher(link, publisher)  # type: ignore[arg-type]
    coord.step()
    coord.step()
    frame = to_view_frame(dict(publisher.snapshots[-1]), FALLBACK)
    row = next(r for r in frame["vehicles"] if r["sysid"] == 1)
    assert row["lat"] == pytest.approx(v.lat_deg, abs=1e-9)
    assert row["lon"] == pytest.approx(v.lon_deg, abs=1e-9)
    assert row["alt"] == pytest.approx(15.0)
    assert (row["roll"], row["pitch"], row["battery"], row["battery_v"]) == (0.05, -0.1, 77, 24.8)


def test_snapshot_marks_pilot_flown_drones_and_carries_their_sticks() -> None:
    pilot = _flying(1, apm.COPTER_MODE_LOITER)
    pilot.rc_channels_us = (1500, 1400, 1650, 1500, 1800, 1000)
    grounded = _flying(3, apm.COPTER_MODE_STABILIZE, east=10.0)
    grounded.armed = False
    link = FakeLink([pilot, _flying(2, apm.COPTER_MODE_GUIDED, east=5.0), grounded])
    publisher = RecordingPublisher()
    coord = Watcher(link, publisher)  # type: ignore[arg-type]
    coord.step()
    coord.step()
    frame = to_view_frame(dict(publisher.snapshots[-1]), FALLBACK)
    rows = {r["sysid"]: r for r in frame["vehicles"]}
    # A disarmed drone in a pilot mode is nobody's to fly yet.
    assert [rows[s]["pilot"] for s in (1, 2, 3)] == [True, False, False]
    assert rows[1]["sticks"] == [1500, 1400, 1650, 1500]
    assert rows[2]["sticks"] == []


def test_view_frame_converts_slot_and_target_to_lat_lon() -> None:
    frame = to_view_frame(_snapshot(), FALLBACK)
    v = frame["vehicles"][0]
    assert frame["origin"] == [FRAME.origin_lat_deg, FRAME.origin_lon_deg]
    assert (v["lat"], v["lon"]) == pytest.approx(FRAME.to_latlon(100.0, -50.0))
    assert v["alt"] == 15.0
    assert v["slot"] == pytest.approx(list(FRAME.to_latlon(90.0, -40.0)))
    assert frame["target"] == pytest.approx(list(FRAME.to_latlon(300.0, 20.0)))


def test_view_frame_without_slot_or_target() -> None:
    snap = _snapshot(target_n=None, target_e=None)
    snap["vehicles"][0].update(slot_n=None, slot_e=None)
    frame = to_view_frame(snap, FALLBACK)
    assert frame["target"] is None
    assert frame["vehicles"][0]["slot"] is None


def test_old_logs_without_origin_or_attitude_use_the_fallback() -> None:
    snap = _snapshot()
    del snap["origin_lat"], snap["origin_lon"]
    for key in ("roll", "pitch", "battery", "battery_v"):
        del snap["vehicles"][0][key]
    frame = to_view_frame(snap, FALLBACK)
    v = frame["vehicles"][0]
    assert (v["lat"], v["lon"]) == pytest.approx(FALLBACK.to_latlon(100.0, -50.0))
    assert (v["roll"], v["pitch"], v["battery"], v["battery_v"]) == (0.0, 0.0, None, None)
    assert (v["pilot"], v["sticks"]) == (False, [])


def test_env_file_parsing(tmp_path: Path) -> None:
    env = tmp_path / ".env.local"
    env.write_text('# keys\nCESIUM_ION_TOKEN="abc"\n\nnot a pair\nGOOGLE_MAPS_API_KEY = xyz\n')
    assert read_env_file(env) == {"CESIUM_ION_TOKEN": "abc", "GOOGLE_MAPS_API_KEY": "xyz"}
    assert read_env_file(tmp_path / "missing") == {}


def test_replay_delays_follow_snapshot_time_and_speed(tmp_path: Path) -> None:
    log = tmp_path / "run.jsonl"
    log.write_text("".join(json.dumps(_snapshot(t=t)) + "\n" for t in (10.0, 10.5, 12.5)) + "\n")
    delays = [d for d, _ in replay_frames(log, speed=2.0)]
    assert delays == [0.0, 0.25, 1.0]


def test_hub_sends_live_frames_and_thinned_history() -> None:
    hub = Hub()
    frame = to_view_frame(_snapshot(), FALLBACK)
    for _ in range(5):
        hub.publish(frame)
    q, history, _ = hub.subscribe()
    assert len(history) == 1, f"history is kept at {HISTORY_HZ} Hz"
    hub.publish(frame)
    name, data = q.get_nowait()
    assert name == "frame"
    assert json.loads(data)["phase"] == "formation"
    hub.unsubscribe(q)
    hub.publish(frame)
    assert q.empty()


def test_slow_client_drops_oldest_frames() -> None:
    hub = Hub()
    q, _, _ = hub.subscribe()
    for t in range(q.maxsize + 10):
        hub.publish(to_view_frame(_snapshot(t=float(t)), FALLBACK))
    assert q.full()
    assert json.loads(q.get_nowait()[1])["t"] == 10.0


@pytest.fixture
def server(tmp_path: Path) -> Iterator[str]:
    log = tmp_path / "run.jsonl"
    log.write_text(json.dumps(_snapshot()) + "\n")
    config = ViewConfig(markers=[Marker(name="A", lat=1.0, lon=2.0)], http_port=0)
    s = ViewServer(config, FALLBACK, replay=log)
    s.start()
    assert s._httpd is not None  # pyright: ignore[reportPrivateUsage]
    port = s._httpd.server_address[1]  # pyright: ignore[reportPrivateUsage]
    yield f"http://127.0.0.1:{port}"
    s.stop()


def test_config_and_page_are_served(server: str) -> None:
    with urllib.request.urlopen(f"{server}/config.json") as r:
        config = json.load(r)
    assert config["markers"] == [{"name": "A", "lat": 1.0, "lon": 2.0}]
    assert config["replay"] == "run.jsonl"
    with urllib.request.urlopen(f"{server}/") as r:
        assert r.headers["Content-Type"].startswith("text/html")
        assert b"app.js" in r.read()
    with urllib.request.urlopen(f"{server}/drive.html") as r:
        assert b"drive.js" in r.read()


def test_config_carries_the_duels_field() -> None:
    fence = [(1.0, 2.0), (1.0, 2.1), (1.1, 2.1)]
    config = ViewConfig(markers=[], point_b=(1.05, 2.05), fence=fence)
    page = ViewServer(config, FALLBACK).page_config()
    assert json.loads(json.dumps(page))["pointB"] == [1.05, 2.05]
    assert json.loads(json.dumps(page))["fence"] == [[1.0, 2.0], [1.0, 2.1], [1.1, 2.1]]
    assert ViewServer(ViewConfig(markers=[]), FALLBACK).page_config()["fence"] == []


class FakeSim:
    def __init__(self) -> None:
        self.destroyed: list[int] = []
        self.resets = 0

    def destroy(self, sysid: int) -> bool:
        self.destroyed.append(sysid)
        return True

    def reset(self) -> None:
        self.resets += 1


def _post(url: str, body: object, content_type: str = "application/json") -> int:
    request = urllib.request.Request(
        url, data=json.dumps(body).encode(), headers={"Content-Type": content_type}
    )
    try:
        with urllib.request.urlopen(request, timeout=5) as r:
            return r.status
    except urllib.error.HTTPError as err:
        return err.code


def test_the_duel_destroys_and_resets_the_sims_copters(tmp_path: Path) -> None:
    log = tmp_path / "run.jsonl"
    log.write_text(json.dumps(_snapshot()) + "\n")
    s = ViewServer(ViewConfig(markers=[], http_port=0), FALLBACK, replay=log)
    s.start()
    assert s._httpd is not None  # pyright: ignore[reportPrivateUsage]
    url = f"http://127.0.0.1:{s._httpd.server_address[1]}"  # pyright: ignore[reportPrivateUsage]
    try:
        assert s.page_config()["sim"] is False
        assert _post(f"{url}/sim/destroy", {"sysid": 2}) == 404  # not a sim
        sim = FakeSim()
        s.sim = sim
        assert s.page_config()["sim"] is True
        assert _post(f"{url}/sim/destroy", {"sysid": 2}) == 200
        assert _post(f"{url}/sim/destroy", {"sysid": "2"}) == 400
        assert _post(f"{url}/sim/destroy", {"sysid": 3}, "text/plain") == 403  # a form post
        assert _post(f"{url}/sim/reset", {}) == 200
        deadline = time.monotonic() + 2
        while sim.resets == 0 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert (sim.destroyed, sim.resets) == ([2], 1)
    finally:
        s.stop()


def test_old_logs_have_every_link_up_and_navigating() -> None:
    vehicle = to_view_frame(_snapshot(), FALLBACK)["vehicles"][0]
    assert (vehicle["link"], vehicle["nav"]) == (True, True)
    snap = _snapshot()
    snap["vehicles"][0].update(link=False, nav=False)
    vehicle = to_view_frame(snap, FALLBACK)["vehicles"][0]
    assert (vehicle["link"], vehicle["nav"]) == (False, False)


def test_files_outside_the_web_roots_are_not_served(server: str) -> None:
    for path in ("/../view.py", "/vendor/../../pyproject.toml", "/%2e%2e/view.py", "/nope.js"):
        with pytest.raises(urllib.error.HTTPError) as err:
            urllib.request.urlopen(f"{server}{path}")
        assert err.value.code == 404, path


def test_events_stream_replayed_frames(server: str) -> None:
    deadline = time.monotonic() + 5
    with urllib.request.urlopen(f"{server}/events", timeout=5) as r:
        while time.monotonic() < deadline:
            line = r.readline().decode()
            if line.startswith("data: "):
                payload = json.loads(line.removeprefix("data: "))
                frames = payload if isinstance(payload, list) else [payload]
                assert frames[0]["phase"] == "formation"
                return
    pytest.fail("no frame on /events")


def test_latest_plan_per_vehicle_is_kept_for_new_pages() -> None:
    hub = Hub()
    for t in (0.0, 1.0):
        hub.publish_plan({"kind": "plan", "sysid": 2, "t": t})
    q, _, plans = hub.subscribe()
    assert [json.loads(p)["t"] for p in plans] == [1.0]
    hub.publish_plan({"kind": "plan", "sysid": 3, "t": 2.0})
    assert q.get_nowait()[0] == "plan"


def test_events_stream_carries_plans_from_the_log(tmp_path: Path) -> None:
    log = tmp_path / "run.jsonl"
    plan = {"kind": "plan", "t": 1.0, "sysid": 1, "items": []}
    log.write_text(json.dumps(plan) + "\n" + json.dumps(_snapshot(t=1.5)) + "\n")
    s = ViewServer(ViewConfig(markers=[], http_port=0), FALLBACK, replay=log)
    s.start()
    assert s._httpd is not None  # pyright: ignore[reportPrivateUsage]
    port = s._httpd.server_address[1]  # pyright: ignore[reportPrivateUsage]
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/events", timeout=5) as r:
            event = None
            for _ in range(50):
                line = r.readline().decode().strip()
                if line.startswith("event: "):
                    event = line.removeprefix("event: ")
                elif line.startswith("data: ") and event == "plan":
                    assert json.loads(line.removeprefix("data: "))["sysid"] == 1
                    return
        pytest.fail("no plan event on /events")
    finally:
        s.stop()
