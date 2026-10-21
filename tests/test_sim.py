import socket

import pytest

from swarm.settings import load_settings
from swarm.sim import ports_in_use

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
