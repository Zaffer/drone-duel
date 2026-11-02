"""content/*.toml: every file loads, and a typo or a broken reference is an error naming it."""

from pathlib import Path

import pytest

from drone_duels.content.store import CONTENT, ContentError, load


def test_the_shipped_content_loads() -> None:
    c = load()
    assert {"billy-bishop", "area-xo"} <= set(c.locations)
    assert {"quad5", "chupito", "truck"} <= set(c.vehicles)
    assert c.vehicles["quad5"].firmware.kind == "ardupilot-copter"
    assert c.vehicles["chupito"].firmware.kind == "inav"
    for course in c.courses.values():
        assert len(course.gates) >= 2


def _copy(tmp: Path) -> Path:
    for sub in CONTENT.iterdir():
        (tmp / sub.name).mkdir()
        for f in sub.glob("*.toml"):
            (tmp / sub.name / f.name).write_text(f.read_text())
    return tmp


def test_an_unknown_key_names_the_file_and_key(tmp_path: Path) -> None:
    root = _copy(tmp_path)
    f = root / "locations" / "billy-bishop.toml"
    text = f.read_text().replace("heading_deg = 270.0\n", "heading_deg = 270.0\nheadng = 1\n", 1)
    f.write_text(text)
    with pytest.raises(ContentError, match=r"billy-bishop.toml(.|\n)*headng"):
        load(root)


def test_a_missing_reference_is_an_error(tmp_path: Path) -> None:
    root = _copy(tmp_path)
    f = root / "courses" / "areaxo-loop.toml"
    f.write_text(f.read_text().replace('location = "area-xo"', 'location = "atlantis"'))
    with pytest.raises(ContentError, match="no location 'atlantis'"):
        load(root)
