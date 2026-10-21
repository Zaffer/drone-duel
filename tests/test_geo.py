import math

from swarm.geo import LocalFrame


def test_roundtrip_near_origin() -> None:
    frame = LocalFrame(-35.3632621, 149.1652374)
    lat, lon = frame.to_latlon(120.0, -45.0)
    n, e = frame.to_ne(lat, lon)
    assert math.isclose(n, 120.0, abs_tol=1e-6)
    assert math.isclose(e, -45.0, abs_tol=1e-6)


def test_origin_maps_to_zero() -> None:
    frame = LocalFrame(10.0, 20.0)
    assert frame.to_ne(10.0, 20.0) == (0.0, 0.0)


def test_north_is_positive_latitude() -> None:
    frame = LocalFrame(0.0, 0.0)
    n, e = frame.to_ne(0.001, 0.0)
    assert n > 0 and math.isclose(e, 0.0, abs_tol=1e-9)
