"""The geofence: its shape, how it matches a drone's download, and the upload protocol."""

from pymavlink.dialects.v20 import ardupilotmega as apm

from swarm.fence import VERTEX, Fence, FenceSender, FenceUploader, contains, self_intersects
from swarm.mission import RawItem

SQUARE = ((45.0, -75.0), (45.0, -74.99), (44.99, -74.99), (44.99, -75.0))
FENCE = apm.MAV_MISSION_TYPE_FENCE


def _vertex(seq: int, lat: float, lon: float, count: int = 4) -> RawItem:
    return RawItem(seq, apm.MAV_FRAME_GLOBAL, VERTEX, (float(count), 0, 0, 0), lat, lon, 0.0)


def test_inside_and_outside() -> None:
    assert contains(SQUARE, 44.995, -74.995)
    assert not contains(SQUARE, 45.001, -74.995)
    assert not contains(SQUARE, 44.995, -74.98)
    l_shape = (
        (0.0, 0.0),
        (0.0, 0.002),
        (0.001, 0.002),
        (0.001, 0.001),
        (0.002, 0.001),
        (0.002, 0.0),
    )
    assert contains(l_shape, 0.0015, 0.0005)
    assert not contains(l_shape, 0.0015, 0.0015)  # the notch


def test_crossing_edges_are_found() -> None:
    assert not self_intersects(SQUARE)
    bow_tie = (SQUARE[0], SQUARE[2], SQUARE[1], SQUARE[3])
    assert self_intersects(bow_tie)


def test_a_download_matches_only_the_same_polygon() -> None:
    fence = Fence(True, SQUARE, 100.0, "brake")
    items = [_vertex(i, lat, lon) for i, (lat, lon) in enumerate(SQUARE)]
    assert fence.matches(items)
    assert fence.matches([_vertex(0, 45.0 + 1e-7, -75.0), *items[1:]])  # 1e-7 degree rounding
    assert not fence.matches(items[:3])
    assert not fence.matches([_vertex(0, 45.001, -75.0), *items[1:]])
    assert fence.params() == {"FENCE_TYPE": 5.0, "FENCE_ACTION": 4.0, "FENCE_ALT_MAX": 100.0}


def test_upload_answers_each_request_then_ends_on_the_ack() -> None:
    sent: list[tuple[object, ...]] = []
    uploader = FenceUploader(
        FenceSender(
            count=lambda n: sent.append(("count", n)),
            item=lambda seq, lat, lon, n: sent.append(("item", seq, lat, lon, n)),
        )
    )
    uploader.start(SQUARE, 0.0)
    assert sent == [("count", 4)] and uploader.busy
    uploader.on_request(0, apm.MAV_MISSION_TYPE_MISSION, 0.1)  # another plan's transfer
    for seq in range(4):
        uploader.on_request(seq, FENCE, 0.1 * seq)
    assert sent[1:] == [("item", i, lat, lon, 4) for i, (lat, lon) in enumerate(SQUARE)]
    assert uploader.on_ack(apm.MAV_MISSION_ACCEPTED, FENCE)
    assert not uploader.busy


def test_upload_starts_again_when_the_drone_goes_quiet_then_gives_up() -> None:
    sent: list[int] = []
    uploader = FenceUploader(FenceSender(count=sent.append, item=lambda *_: None))
    uploader.start(SQUARE, 0.0)
    for t in (1.0, 2.0, 4.0, 6.0):
        uploader.tick(t)
    assert sent == [4, 4, 4] and not uploader.busy
    uploader.start(SQUARE, 10.0)
    assert not uploader.on_ack(apm.MAV_MISSION_DENIED, FENCE) and not uploader.busy
