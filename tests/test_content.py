"""content/*.toml: every file loads, and a typo or a broken reference is an error naming it."""

from pathlib import Path

import pytest

from drone_duels.content.store import CONTENT, ContentError, load


def test_the_shipped_content_loads() -> None:
    c = load()
    assert {"billy-bishop", "area-xo"} <= set(c.locations)
    assert {"quad5", "quad5-betaflight", "chupito", "truck"} <= set(c.vehicles)
    assert c.vehicles["quad5"].firmware.kind == "ardupilot-copter"
    assert c.vehicles["quad5-betaflight"].firmware.kind == "betaflight"
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


def test_a_payload_is_fitted_by_a_mode_not_a_vehicle(tmp_path: Path) -> None:
    """Shoot Down's charge: fitted to whichever quad it starts, with that firmware's files after
    the vehicle's; a vehicle's own file cannot carry one."""
    c = load()
    for vehicle_id in ("quad5", "quad5-betaflight"):
        vehicle = c.vehicle(vehicle_id)
        assert vehicle.payload is None
        fitted = c.fitted(vehicle_id, "charge")
        assert fitted.payload == c.payload("charge").spec()
        assert fitted.payload is not None and fitted.payload.lethal_m == 5.0
        extra = c.payload("charge").params.get(vehicle.firmware.kind, [])
        assert fitted.firmware.params == [*vehicle.firmware.params, *extra]
    assert c.fitted("quad5", "charge").firmware.params[-1] == "ardupilot/copter-quad5-charge.parm"
    assert c.fitted("quad5", None) == c.vehicle("quad5")
    with pytest.raises(ContentError, match="no payload 'anvil'"):
        c.fitted("quad5", "anvil")

    root = _copy(tmp_path)
    f = root / "vehicles" / "quad5.toml"
    payload = (CONTENT / "payloads" / "charge.toml").read_text().split("[params]")[0]
    f.write_text(f.read_text() + "\n[payload]\n" + payload)
    with pytest.raises(ContentError, match="quad5: a payload is fitted by a mode"):
        load(root)
