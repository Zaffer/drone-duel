"""Each SITL copter's ArduPilot OSD, as its analog NTSC video would show it.

An analog FPV drone has the OSD chip on its flight controller (MAX7456, OSD_TYPE 1) lay the
OSD into the camera's video on its way to the video transmitter: a grid of 30 characters by
16 rows in ArduPilot's font (OSD_FONT), of which NTSC shows the top 13 (NTSC_ROWS).

SITL has no OSD chip. It sends the same screen over MSP DisplayPort instead (SERIALn_PROTOCOL
42, OSD_TYPE 5, OSD1_TXT_RES 0: 30 x 16): ten times a second ArduPilot clears the screen,
writes each OSD item as a string at a row and column, and draws. sitl/fpv5.parm sets it up on
SERIAL5 and `swarm sim` points each SITL's SERIAL5 at UDP OSD_BASE_PORT + sysid. OsdFeed
parses the MSP frames and keeps each copter's last drawn screen, cut to NTSC's rows, for the
3D viewer's camera feed, which draws it in the chip's font (font_png). The layout is the
drone's OSD1_* parameters.

Sim only. On hardware the chip draws the OSD into the video itself.
"""

import logging
import select
import socket
import threading
from collections.abc import Iterator
from dataclasses import dataclass
from functools import cache
from io import BytesIO
from pathlib import Path

from PIL import Image

log = logging.getLogger(__name__)

ROOT = Path(__file__).resolve().parents[2]
FONT_DIR = ROOT / "external/ardupilot/libraries/AP_OSD/fonts"
FONTS = ["clarity", "clarity_medium", "bfstyle", "bold", "digital"]
"""The OSD chip's fonts, by OSD_FONT, as ArduPilot builds them (fonts/mcm_all.py)."""
GLYPH_W, GLYPH_H = 12, 18
"""A MAX7456 character, in the chip's pixels."""
NTSC_ROWS = 13
"""The rows of the 16 an NTSC picture shows (PAL shows all 16)."""
OSD_BASE_PORT = 14700

MSP_DISPLAYPORT = 182
HEARTBEAT, RELEASE, CLEAR, WRITE, DRAW, OPTIONS = range(6)
GRIDS = {0: (30, 16), 1: (50, 18), 2: (60, 22)}
"""OSDn_TXT_RES: columns and rows."""
BLANK = 0x20


def osd_port(sysid: int) -> int:
    return OSD_BASE_PORT + sysid


@cache
def font_png(index: int) -> bytes | None:
    """OSD_FONT `index` as a PNG: its 256 glyphs of GLYPH_W x GLYPH_H in one column, white with
    a black edge on clear. None when that font is not there."""
    if not 0 <= index < len(FONTS):
        return None
    path = FONT_DIR / f"{FONTS[index]}.mcm"
    if not path.is_file():
        return None
    # A .mcm: "MAX7456", then 64 lines of 8 bits a glyph, the first 54 its 18 rows of 3 bytes.
    # Each pixel is 2 bits: 00 black, 10 white, 01 and 11 clear.
    lines = path.read_text().split()[1:]
    img = Image.new("RGBA", (GLYPH_W, GLYPH_H * 256))
    colours = {0b00: (0, 0, 0, 255), 0b10: (255, 255, 255, 255)}
    for glyph in range(256):
        data = lines[glyph * 64 : glyph * 64 + 54]
        for i, bits in enumerate(data):
            byte = int(bits, 2)
            for k in range(4):
                colour = colours.get((byte >> (6 - 2 * k)) & 0b11)
                if colour:
                    img.putpixel(((i % 3) * 4 + k, glyph * GLYPH_H + i // 3), colour)
    out = BytesIO()
    img.save(out, "PNG")
    return out.getvalue()


@dataclass(frozen=True)
class OsdScreen:
    cols: int
    rows: int
    font: int
    """OSDn_FONT: which of the chip's fonts the screen asks for."""
    text: list[str]
    """Each row's character codes, 0-255, as a Latin-1 string."""


class MspReader:
    """MSP v1 frames from the flight controller ($M>), out of a byte stream."""

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> Iterator[tuple[int, bytes]]:
        """(command, payload) for each whole, valid frame in what has arrived so far."""
        buf = self._buf
        buf += data
        while True:
            start = buf.find(b"$M>")
            if start < 0:
                del buf[:-2]  # keep a partial "$M" for the next read
                return
            del buf[:start]
            if len(buf) < 5:
                return
            size, cmd = buf[3], buf[4]
            if len(buf) < 6 + size:
                return
            payload = bytes(buf[5 : 5 + size])
            checksum = size ^ cmd
            for b in payload:
                checksum ^= b
            if checksum == buf[5 + size]:
                del buf[: 6 + size]
                yield cmd, payload
            else:
                del buf[:3]


class DisplayPort:
    """The far side of MSP DisplayPort: a screen ArduPilot writes, then draws."""

    def __init__(self) -> None:
        self.cols, self.rows = GRIDS[0]
        self.font = 0
        self._back = self._blank()
        self.screen: OsdScreen | None = None
        """The last screen drawn."""

    def _blank(self) -> list[bytearray]:
        return [bytearray([BLANK] * self.cols) for _ in range(self.rows)]

    def handle(self, payload: bytes) -> bool:
        """Take one MSP_DISPLAYPORT payload. True when it drew a new screen."""
        if not payload:
            return False
        sub = payload[0]
        if sub == OPTIONS and len(payload) >= 3:
            self.font = payload[1]
            grid = GRIDS.get(payload[2], GRIDS[0])
            if grid != (self.cols, self.rows):
                self.cols, self.rows = grid
                self._back = self._blank()
        elif sub == CLEAR:
            self._back = self._blank()
        elif sub == WRITE and len(payload) >= 4:
            row, col, text = payload[1], payload[2], payload[4:]
            if row < self.rows:
                line = self._back[row]
                for i, ch in enumerate(text[: max(0, self.cols - col)]):
                    line[col + i] = ch
        elif sub == DRAW:
            # The analog chip's 30 x 16, as NTSC shows it; another grid as it is.
            shown = NTSC_ROWS if (self.cols, self.rows) == GRIDS[0] else self.rows
            self.screen = OsdScreen(
                cols=self.cols,
                rows=shown,
                font=self.font,
                text=[bytes(line).decode("latin-1") for line in self._back[:shown]],
            )
            return True
        return False


class OsdFeed:
    """Receives every copter's DisplayPort in a thread; `latest` from any."""

    def __init__(self, sysids: list[int], host: str = "127.0.0.1") -> None:
        self._socks: dict[socket.socket, int] = {}
        self._readers: dict[int, tuple[MspReader, DisplayPort]] = {}
        for sysid in sysids:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setblocking(False)
            try:
                sock.bind((host, osd_port(sysid)))
            except OSError as exc:
                sock.close()
                log.warning("no OSD for copter %d: UDP %d (%s)", sysid, osd_port(sysid), exc)
                continue
            self._socks[sock] = sysid
            self._readers[sysid] = (MspReader(), DisplayPort())
        self._lock = threading.Lock()
        self._latest: dict[int, tuple[int, OsdScreen]] = {}
        """Per copter: how many screens drawn so far, and the last."""
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="osd", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)
        for sock in self._socks:
            sock.close()

    def latest(self) -> dict[int, tuple[int, OsdScreen]]:
        """Per copter heard from: how many screens it has drawn, and its last."""
        with self._lock:
            return dict(self._latest)

    def _run(self) -> None:
        socks = list(self._socks)
        while not self._stop.is_set() and socks:
            ready, _, _ = select.select(socks, [], [], 0.5)
            for sock in ready:
                sysid = self._socks[sock]
                reader, port = self._readers[sysid]
                try:
                    while True:
                        data = sock.recv(4096)
                        for cmd, payload in reader.feed(data):
                            if cmd == MSP_DISPLAYPORT and port.handle(payload):
                                self._drawn(sysid, port)
                except BlockingIOError:
                    pass
                except OSError:
                    continue

    def _drawn(self, sysid: int, port: DisplayPort) -> None:
        assert port.screen is not None
        with self._lock:
            count = self._latest.get(sysid, (0, port.screen))[0]
            self._latest[sysid] = (count + 1, port.screen)
