"""The game's mission link (telemetry/mission.py) against a stand-in vehicle speaking MAVLink's
mission protocol, as Betaflight's does (telemetry/mavlink_mission.c): it asks for each item, says
what it made of the mission, and says what it is flying."""

import socket
import threading
from collections.abc import Iterator

import pytest
from pymavlink.dialects.v20 import ardupilotmega as apm

from drone_duels.telemetry.mission import ACCEPTED, DENIED, STATE_ACTIVE, MissionLink, result_name


class Vehicle:
    """System 1 on a TCP port: takes a mission (or denies it), and says it flies waypoint 2."""

    def __init__(self, deny: bool = False) -> None:
        self.deny = deny
        self.items: list[apm.MAVLink_mission_item_int_message] = []
        self.server = socket.create_server(("127.0.0.1", 0))
        self.port = self.server.getsockname()[1]
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self) -> None:
        conn, _ = self.server.accept()
        out = apm.MAVLink(conn.makefile("wb", buffering=0), srcSystem=1, srcComponent=1)
        parse = apm.MAVLink(None)
        count = 0
        while True:
            data = conn.recv(4096)
            if not data:
                return
            for msg in parse.parse_buffer(data) or []:
                if msg.get_type() == "MISSION_COUNT":
                    assert msg.get_srcSystem() == 254  # the game
                    if self.deny:
                        out.mission_ack_send(254, 190, apm.MAV_MISSION_DENIED, 0)
                        continue
                    count, self.items = msg.count, []  # pyright: ignore[reportAttributeAccessIssue]
                    out.mission_request_int_send(254, 190, 0, 0)
                elif msg.get_type() == "MISSION_ITEM_INT":
                    self.items.append(msg)  # pyright: ignore[reportArgumentType]
                    if len(self.items) < count:
                        out.mission_request_int_send(254, 190, len(self.items), 0)
                    else:
                        out.mission_ack_send(254, 190, apm.MAV_MISSION_ACCEPTED, 0)
                        out.mission_current_send(2, count, apm.MISSION_STATE_ACTIVE)


@pytest.fixture
def vehicle() -> Iterator[Vehicle]:
    v = Vehicle()
    yield v
    v.server.close()


def test_uploads_a_mission_of_waypoints(vehicle: Vehicle) -> None:
    link = MissionLink(vehicle.port)
    try:
        points = [(43.6, -79.4, 20.0), (43.6001, -79.4, 20.0), (43.6001, -79.3999, 25.5)]
        assert link.upload(points) == ACCEPTED
        assert [(i.seq, i.x, i.y, i.z) for i in vehicle.items] == [
            (0, 436000000, -794000000, 20.0),
            (1, 436001000, -794000000, 20.0),
            (2, 436001000, -793999000, 25.5),
        ]
        assert {i.command for i in vehicle.items} == {apm.MAV_CMD_NAV_WAYPOINT}
        assert {i.frame for i in vehicle.items} == {apm.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT}
        deadline = threading.Event()
        for _ in range(50):
            if link.current is not None:
                break
            deadline.wait(0.02)
        assert link.current == (2, STATE_ACTIVE)
    finally:
        link.close()


def test_says_what_the_vehicle_answered() -> None:
    v = Vehicle(deny=True)
    link = MissionLink(v.port)
    try:
        assert link.upload([(43.6, -79.4, 20.0), (43.6, -79.39, 20.0)]) == DENIED
        assert result_name(DENIED) == "denied" and result_name(None) == "no answer"
    finally:
        link.close()
        v.server.close()


def test_no_answer_is_none() -> None:
    server = socket.create_server(("127.0.0.1", 0))  # listens, says nothing
    link = MissionLink(server.getsockname()[1])
    try:
        assert link.upload([(43.6, -79.4, 20.0), (43.6, -79.39, 20.0)], timeout_s=0.3) is None
    finally:
        link.close()
        server.close()


def test_nothing_listening_raises() -> None:
    server = socket.create_server(("127.0.0.1", 0))
    port = server.getsockname()[1]
    server.close()
    with pytest.raises(OSError):
        MissionLink(port, connect_s=0.5)
