"""SITL's physics for the camera feed: FlightGear FDM packets in, the viewer's /physics out."""

import json
import math
import socket
import struct
import time
import urllib.error
import urllib.request
from pathlib import Path

import pytest

from swarm.geo import LocalFrame
from swarm.physics import FDM_HEAD, FDM_VERSION, PhysicsFeed, fg_view_port, parse_fdm
from swarm.view import ViewConfig, ViewServer

FDM_SIZE = 408
"""sizeof(FGNetFDM) as SITL sends it."""
SYSID = 3000
"""High enough that its port is clear of a sim stack running beside the tests."""


def fdm_packet(lat: float, lon: float, alt: float, roll: float, pitch: float, yaw: float) -> bytes:
    head = FDM_HEAD.pack(
        FDM_VERSION, 0, math.radians(lon), math.radians(lat), alt, alt, roll, pitch, yaw
    )
    return head + bytes(FDM_SIZE - len(head))


def test_fdm_packet_is_read_in_degrees_and_metres_above_home() -> None:
    state = parse_fdm(fdm_packet(45.3, -75.7, 112.5, 0.1, -0.2, 3.0), home_alt_m=100.0)
    assert state is not None
    assert state.lat == pytest.approx(45.3)
    assert state.lon == pytest.approx(-75.7)
    assert state.alt == pytest.approx(12.5)
    assert (state.roll, state.pitch, state.yaw) == pytest.approx((0.1, -0.2, 3.0))


def test_short_or_foreign_packets_are_ignored() -> None:
    assert parse_fdm(b"\0" * 10, 0.0) is None
    assert parse_fdm(struct.pack(">I", 0x17) + bytes(FDM_SIZE - 4), 0.0) is None


def test_ports_follow_sitl_instances() -> None:
    assert [fg_view_port(s) for s in (1, 2, 5)] == [5503, 5513, 5543]


def _send(packet: bytes) -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as out:
        out.sendto(packet, ("127.0.0.1", fg_view_port(SYSID)))


def test_feed_keeps_the_newest_state() -> None:
    feed = PhysicsFeed([SYSID], home_alt_m=0.0)
    feed.start()
    try:
        assert feed.latest() == {}
        for alt in (1.0, 2.0, 3.0):
            _send(fdm_packet(45.0, -75.0, alt, 0, 0, 0))
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            _, state = feed.latest().get(SYSID, (0, None))
            if state is not None and state.alt == pytest.approx(3.0):
                return
            time.sleep(0.01)
        pytest.fail(f"newest state never read: {feed.latest()}")
    finally:
        feed.stop()


def test_physics_stream_sends_each_copter_as_it_comes(tmp_path: Path) -> None:
    log = tmp_path / "run.jsonl"
    log.write_text("")
    s = ViewServer(ViewConfig(markers=[], http_port=0), LocalFrame(45.0, -75.0), replay=log)
    feed = PhysicsFeed([SYSID], home_alt_m=100.0)
    feed.start()
    s.physics = feed
    s.start()
    assert s._httpd is not None  # pyright: ignore[reportPrivateUsage]
    url = f"http://127.0.0.1:{s._httpd.server_address[1]}"  # pyright: ignore[reportPrivateUsage]
    try:
        with urllib.request.urlopen(f"{url}/physics", timeout=5) as r:
            _send(fdm_packet(45.1, -75.2, 110.0, 0.5, 0, 0))
            line = r.readline().decode()
            state = json.loads(line.removeprefix("data: "))[str(SYSID)]
            assert state["alt"] == pytest.approx(10.0)
            assert state["roll"] == pytest.approx(0.5)
    finally:
        s.stop()
        feed.stop()


def test_no_physics_stream_without_the_sim(tmp_path: Path) -> None:
    log = tmp_path / "run.jsonl"
    log.write_text("")
    s = ViewServer(ViewConfig(markers=[], http_port=0), LocalFrame(45.0, -75.0), replay=log)
    s.start()
    assert s._httpd is not None  # pyright: ignore[reportPrivateUsage]
    url = f"http://127.0.0.1:{s._httpd.server_address[1]}"  # pyright: ignore[reportPrivateUsage]
    try:
        with pytest.raises(urllib.error.HTTPError) as err:
            urllib.request.urlopen(f"{url}/physics")
        assert err.value.code == 404
    finally:
        s.stop()
