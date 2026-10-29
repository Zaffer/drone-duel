import io
import json
import urllib.error
from pathlib import Path

import pytest

from drone_duels.world import elevation
from drone_duels.world.elevation import ElevationError, ground_elevation_m


def test_looks_up_once_then_remembers(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    urls: list[str] = []

    def urlopen(url: str, timeout: float) -> io.BytesIO:
        urls.append(url)
        return io.BytesIO(json.dumps({"elevation": [75.0]}).encode())

    monkeypatch.setattr(elevation.urllib.request, "urlopen", urlopen)
    cache = tmp_path / "elevation.json"
    assert ground_elevation_m(43.6275, -79.396111, cache) == 75.0
    assert ground_elevation_m(43.6275, -79.396111, cache) == 75.0
    assert len(urls) == 1
    assert "latitude=43.627500&longitude=-79.396111" in urls[0]
    assert json.loads(cache.read_text()) == {"43.627500,-79.396111": 75.0}


def test_offline_asks_for_a_number(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    def urlopen(url: str, timeout: float) -> io.BytesIO:
        raise urllib.error.URLError("no network")

    monkeypatch.setattr(elevation.urllib.request, "urlopen", urlopen)
    with pytest.raises(ElevationError, match="ground_alt_m"):
        ground_elevation_m(43.0, -79.0, tmp_path / "elevation.json")
