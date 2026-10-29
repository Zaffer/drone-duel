"""ArduPilot's OSD over MSP DisplayPort: frames in, drawn screens out."""

from io import BytesIO

import pytest
from PIL import Image

from drone_duels.video.osd import (
    CLEAR,
    DRAW,
    FONT_DIR,
    GLYPH_H,
    GLYPH_W,
    MSP_DISPLAYPORT,
    OPTIONS,
    WRITE,
    DisplayPort,
    MspReader,
    font_png,
)


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
    assert (port.screen.cols, port.screen.rows) == (30, 13)
    assert port.screen.text[2][3:8] == "STAB\x8a"
    assert port.screen.text[0] == " " * 30


def test_ntsc_shows_the_top_13_rows() -> None:
    port = DisplayPort()
    port.handle(bytes([WRITE, 12, 0, 0]) + b"SHOWN")
    port.handle(bytes([WRITE, 13, 0, 0]) + b"CUT")
    port.handle(bytes([DRAW]))
    assert port.screen is not None
    assert len(port.screen.text) == 13
    assert port.screen.text[12].startswith("SHOWN")


@pytest.mark.skipif(not (FONT_DIR / "clarity.mcm").is_file(), reason="no ArduPilot checkout")
def test_font_is_the_chips_glyphs_in_one_column() -> None:
    png = font_png(0)
    assert png is not None
    img = Image.open(BytesIO(png))
    assert img.size == (GLYPH_W, GLYPH_H * 256)
    a = img.crop((0, ord("A") * GLYPH_H, GLYPH_W, (ord("A") + 1) * GLYPH_H))
    colours = {c for _, c in a.getcolors() or []}
    # White, edged in black, on clear.
    assert {(255, 255, 255, 255), (0, 0, 0, 255), (0, 0, 0, 0)} <= colours
    assert font_png(99) is None


def test_writes_past_the_edge_are_cut_and_options_set_the_grid() -> None:
    port = DisplayPort()
    port.handle(bytes([OPTIONS, 1, 1]))
    port.handle(bytes([WRITE, 17, 48, 0]) + b"XYZ")
    port.handle(bytes([WRITE, 30, 0, 0]) + b"gone")
    port.handle(bytes([DRAW]))
    assert port.screen is not None
    assert (port.screen.cols, port.screen.rows, port.screen.font) == (50, 18, 1)
    assert port.screen.text[17][48:] == "XY"
