"""The server's radio mapping (drone_duels.rc.mapping) against the cases the page's passes too
(tests/fixtures/mapping.json)."""

import json
from pathlib import Path
from typing import Any

import pytest

from drone_duels.content.schema import Controller
from drone_duels.rc import mapping

CASES: dict[str, Any] = json.loads(
    (Path(__file__).parent / "fixtures" / "mapping.json").read_text()
)
PROFILES = {k: Controller.model_validate(v) for k, v in CASES["profiles"].items()}


@pytest.mark.parametrize("case", CASES["profile_for"])
def test_profile_for(case: dict[str, Any]) -> None:
    found = mapping.profile_for(case["id"], list(PROFILES.values()))
    assert (found.id if found else None) == case["profile"]


@pytest.mark.parametrize("case", CASES["calibrate"])
def test_calibrate(case: dict[str, Any]) -> None:
    assert mapping.calibrate(case["raw"], case["cal"], case["centred"]) == pytest.approx(
        case["out"]
    )


@pytest.mark.parametrize("case", CASES["channels"], ids=lambda c: c["why"])
def test_to_channels(case: dict[str, Any]) -> None:
    profile = PROFILES[case["profile"]] if case["profile"] else None
    got = mapping.to_channels(case["axes"], case["buttons"], profile, case["calibration"])
    assert got == pytest.approx(case["out"])


@pytest.mark.parametrize("case", CASES["switch_pos"])
def test_switch_pos(case: dict[str, Any]) -> None:
    assert mapping.switch_pos(case["x"]) == case["pos"]


@pytest.mark.parametrize("case", CASES["to_firmware"], ids=lambda c: c["why"])
def test_to_firmware(case: dict[str, Any]) -> None:
    assert mapping.to_firmware(case["channels"], case["firmware"]) == case["us"]


def test_calibrations_from_prefs_skip_what_does_not_parse() -> None:
    good = {"channels": [{"axis": 0, "reverse": False, "cal": {"min": -1, "mid": 0, "max": 1}}]}
    prefs = {"controls": {"calibrations": {"radio": good, "broken": {"channels": "no"}}}}
    assert mapping.calibrations(prefs) == {"radio": good}
    assert mapping.calibrations({}) == {}
