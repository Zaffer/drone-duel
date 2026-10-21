import socket
import subprocess

import pytest

from swarm.settings import load_settings
from swarm.sim import SimStack, ports_in_use

FIELD = load_settings().field


def test_moved_field_keeps_a_and_b_at_the_same_offsets_from_home() -> None:
    moved = FIELD.moved_to(43.998759, -79.601691, 443.0)
    assert (moved.home.origin_lat_deg, moved.home.origin_lon_deg) == (43.998759, -79.601691)
    assert moved.home_alt_m == 443.0
    for old, new in ((FIELD.point_a, moved.point_a), (FIELD.point_b, moved.point_b)):
        assert moved.home.to_ne(*new) == pytest.approx(FIELD.home.to_ne(*old), abs=1e-6)


def test_points_can_be_put_elsewhere() -> None:
    field = FIELD.with_points(None, (43.0, -79.0))
    assert (field.point_a, field.point_b) == (FIELD.point_a, (43.0, -79.0))


def test_ports_in_use_finds_a_listener() -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("0.0.0.0", 0))
        sock.listen()
        port = sock.getsockname()[1]
        assert ports_in_use([port]) == [port]
    assert ports_in_use([port]) == []


def test_a_destroyed_copter_stops_and_reset_restarts_every_copter(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    stack = SimStack(sim_field=FIELD, count=2)
    started: list[tuple[int, bool | None]] = []

    def start(index: int, wipe: bool | None = None) -> None:
        started.append((index, wipe))
        stack._copters[index] = subprocess.Popen(["sleep", "30"])  # pyright: ignore[reportPrivateUsage]

    monkeypatch.setattr(stack, "_start_copter", start)
    for i in range(2):
        start(i)
    first = stack._copters[1]  # pyright: ignore[reportPrivateUsage]
    try:
        assert stack.destroy(2)
        assert first.poll() is not None
        assert not stack.destroy(2)  # down already
        assert not stack.destroy(9)  # no such copter
        started.clear()
        stack.reset()
        assert started == [(0, False), (1, False)]  # parameters kept
        assert all(p.poll() is None for p in stack._copters.values())  # pyright: ignore[reportPrivateUsage]
    finally:
        stack.stop()
