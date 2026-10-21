"""ArduPilot's OSD over MSP DisplayPort: frames in, drawn screens out, the viewer's /osd."""

import json
import socket
import time
import urllib.request
from pathlib import Path

import pytest

from swarm.geo import LocalFrame
from swarm.osd import (
    CLEAR,
    DRAW,
    MSP_DISPLAYPORT,
    OPTIONS,
    WRITE,
    DisplayPort,
    MspReader,
    OsdFeed,
    osd_port,
)
from swarm.view import ViewConfig, ViewServer

SYSID = 3000
"""High enough that its port is clear of a sim stack running beside the tests."""


def frame(payload: bytes, cmd: int = MSP_DISPLAYPORT) -> bytes:
    """An MSP v1 frame from the flight controller, as AP_MSP sends it."""
    checksum = len(payload) ^ cmd
    for b in payload:
        checksum ^= b
    return b"$M>" + bytes([len(payload), cmd]) + payload + bytes([checksum])


def write(row: int, col: int, text: bytes) -> bytes:
    return frame(bytes([WRITE, row, col, 0]) + text)


def osd_frame(*writes: bytes) -> bytes:
    """One OSD refresh: options, clear, the writes, draw."""
    return (
        frame(bytes([OPTIONS, 0, 0]))
        + frame(bytes([CLEAR]))
        + b"".join(writes)
        + frame(bytes([DRAW]))
    )


def test_frames_split_across_reads_and_bad_ones_skipped() -> None:
    reader = MspReader()
    data = b"noise" + frame(b"\x02") + frame(b"\x03\x00\x00\x00AB")
    bad = bytearray(frame(b"\x04"))
    bad[-1] ^= 0xFF
    data += bytes(bad) + frame(b"\x04")
    got = [p for i in range(0, len(data), 3) for _, p in reader.feed(data[i : i + 3])]
    assert got == [b"\x02", b"\x03\x00\x00\x00AB", b"\x04"]


def test_screen_is_drawn_only_on_draw() -> None:
    port = DisplayPort()
    reader = MspReader()
    drawn = [port.handle(p) for _, p in reader.feed(osd_frame(write(2, 3, b"STAB\x8a")))]
    assert drawn[-1] and not any(drawn[:-1])
    assert port.screen is not None
    assert (port.screen.cols, port.screen.rows) == (30, 16)
    assert port.screen.text[2][3:8] == "STAB\x8a"
    assert port.screen.text[0] == " " * 30


def test_writes_past_the_edge_are_cut_and_options_set_the_grid() -> None:
    port = DisplayPort()
    port.handle(bytes([OPTIONS, 1, 1]))
    port.handle(bytes([WRITE, 17, 48, 0]) + b"XYZ")
    port.handle(bytes([WRITE, 30, 0, 0]) + b"gone")
    port.handle(bytes([DRAW]))
    assert port.screen is not None
    assert (port.screen.cols, port.screen.rows, port.screen.font) == (50, 18, 1)
    assert port.screen.text[17][48:] == "XY"


def test_feed_and_osd_stream(tmp_path: Path) -> None:
    log = tmp_path / "run.jsonl"
    log.write_text("")
    s = ViewServer(ViewConfig(markers=[], http_port=0), LocalFrame(45.0, -75.0), replay=log)
    feed = OsdFeed([SYSID])
    feed.start()
    s.osd = feed
    s.start()
    assert s._httpd is not None  # pyright: ignore[reportPrivateUsage]
    url = f"http://127.0.0.1:{s._httpd.server_address[1]}"  # pyright: ignore[reportPrivateUsage]
    try:
        with urllib.request.urlopen(f"{url}/osd", timeout=5) as r:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as out:
                out.sendto(osd_frame(write(0, 0, b"LOIT")), ("127.0.0.1", osd_port(SYSID)))
            line = r.readline().decode()
            screen = json.loads(line.removeprefix("data: "))[str(SYSID)]
            assert screen["text"][0].startswith("LOIT")
    finally:
        s.stop()
        feed.stop()


def test_feed_counts_each_drawn_screen() -> None:
    feed = OsdFeed([SYSID])
    feed.start()
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as out:
            for _ in range(2):
                out.sendto(osd_frame(), ("127.0.0.1", osd_port(SYSID)))
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            if feed.latest().get(SYSID, (0, None))[0] == 2:
                return
            time.sleep(0.01)
        pytest.fail(f"screens not counted: {feed.latest()}")
    finally:
        feed.stop()
