"""Reading plans off a vehicle (mission protocol) and turning them into what the viewer draws."""

import pytest
from pymavlink.dialects.v20 import ardupilotmega as apm

from swarm.mission import (
    PLAN_TYPES,
    REFRESH_S,
    RETRY_S,
    STEP_RETRIES,
    STEP_TIMEOUT_S,
    PlanReader,
    PlanSender,
    RawItem,
    build_plan,
)

HOME = (43.998759, -79.601691, 443.0)
REL = apm.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
MISSION, FENCE, RALLY = PLAN_TYPES


def item(
    seq: int,
    command: int,
    lat: float = 0.0,
    lon: float = 0.0,
    alt: float = 0.0,
    frame: int = REL,
    p1: float = 0.0,
) -> RawItem:
    return RawItem(seq, frame, command, (p1, 0.0, 0.0, 0.0), lat, lon, alt)


def home_item() -> RawItem:
    return item(0, apm.MAV_CMD_NAV_WAYPOINT, *HOME, frame=apm.MAV_FRAME_GLOBAL)


def plan(mission: list[RawItem], fence: list[RawItem] | None = None):  # noqa: ANN201
    return build_plan(1, 0.0, mission, fence or [], [], {"FENCE_RADIUS": 30.0}, True)


def test_takeoff_waypoint_land_resolve_to_where_they_happen() -> None:
    north = HOME[0] + 5e-5
    p = plan(
        [
            home_item(),
            item(1, apm.MAV_CMD_NAV_TAKEOFF, alt=4.0),
            item(2, apm.MAV_CMD_NAV_WAYPOINT, north, HOME[1], 4.0),
            item(3, apm.MAV_CMD_NAV_LAND),
        ]
    )
    assert p["home"] == list(HOME)
    takeoff, wp, land = p["items"]
    assert (takeoff["name"], takeoff["lat"], takeoff["lon"], takeoff["alt"]) == (
        "TAKEOFF",
        HOME[0],
        HOME[1],
        4.0,
    )
    assert (wp["seq"], wp["lat"], wp["alt"]) == (2, north, 4.0)
    assert (land["name"], land["lat"], land["alt"]) == ("LAND", north, 0.0)
    assert p["fence_params"] == {
        "enabled": True,
        "types": 0,
        "radius": 30.0,
        "alt_max": None,
        "action": None,
    }


def test_amsl_heights_become_heights_above_home_and_rtl_goes_home() -> None:
    p = plan(
        [
            home_item(),
            item(1, apm.MAV_CMD_NAV_WAYPOINT, HOME[0], HOME[1], 460.0, frame=apm.MAV_FRAME_GLOBAL),
            item(2, apm.MAV_CMD_NAV_WAYPOINT, HOME[0], HOME[1], 12.0, frame=11),
            item(3, apm.MAV_CMD_NAV_RETURN_TO_LAUNCH),
        ]
    )
    amsl, terrain, rtl = p["items"]
    assert amsl["alt"] == pytest.approx(17.0)
    assert (amsl["terrain"], terrain["terrain"], terrain["alt"]) == (False, True, 12.0)
    assert (rtl["lat"], rtl["lon"], rtl["alt"]) == (HOME[0], HOME[1], 0.0)


def test_non_nav_items_are_kept_but_not_on_the_path() -> None:
    p = plan(
        [
            home_item(),
            item(1, apm.MAV_CMD_DO_CHANGE_SPEED, p1=1.0),
            item(2, apm.MAV_CMD_DO_SET_ROI_LOCATION, HOME[0] + 1e-4, HOME[1], 0.0),
        ]
    )
    speed, roi = p["items"]
    assert (speed["nav"], speed["lat"], speed["name"]) == (False, None, "SPEED")
    assert (roi["nav"], roi["name"], roi["lat"]) == (False, "ROI", HOME[0] + 1e-4)


def test_fence_polygons_and_circles() -> None:
    inc = apm.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
    exc = apm.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
    fence = [
        item(0, inc, 1.0, 1.0, p1=3),
        item(1, inc, 1.0, 2.0, p1=3),
        item(2, inc, 2.0, 2.0, p1=3),
        item(3, exc, 5.0, 5.0, p1=3),
        item(4, exc, 5.0, 6.0, p1=3),
        item(5, exc, 6.0, 6.0, p1=3),
        item(6, apm.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION, 3.0, 3.0, p1=15.0),
        item(7, apm.MAV_CMD_NAV_FENCE_RETURN_POINT, 1.5, 1.5),
    ]
    p = plan([home_item()], fence)
    inclusion, exclusion, circle = p["fence"]
    assert (inclusion["inclusion"], inclusion["points"]) == (
        True,
        [[1.0, 1.0], [1.0, 2.0], [2.0, 2.0]],
    )
    assert (exclusion["inclusion"], len(exclusion["points"])) == (False, 3)
    assert (circle["kind"], circle["inclusion"], circle["radius"]) == ("circle", False, 15.0)
    assert p["fence_return"] == [1.5, 1.5]


class FakeVehicle:
    """Answers the mission protocol from stored plans, like ArduPilot does."""

    def __init__(self, plans: dict[int, list[RawItem]]) -> None:
        self.plans = plans
        self.sent: list[tuple[str, ...]] = []
        self.reader = PlanReader(
            PlanSender(
                request_list=lambda t: self.sent.append(("list", str(t))),
                request_item=lambda seq, t: self.sent.append(("item", str(seq), str(t))),
                ack=lambda t: self.sent.append(("ack", str(t))),
            )
        )
        self.uploading = False
        self.unsupported: set[int] = set()

    def run(self, now: float) -> None:
        """Tick, then answer every request until the reader goes quiet."""
        self.reader.tick(now)
        while self.sent:
            req = self.sent.pop(0)
            if req[0] == "ack":
                continue
            t = int(req[-1])
            if self.uploading:
                self.reader.on_ack(apm.MAV_MISSION_DENIED, t, now)
            elif t in self.unsupported:
                self.reader.on_ack(apm.MAV_MISSION_UNSUPPORTED, t, now)
            elif req[0] == "list":
                self.reader.on_count(len(self.plans.get(t, [])), t, now)
            else:
                self.reader.on_item(self.plans[t][int(req[1])], t, now)


def _vehicle() -> FakeVehicle:
    return FakeVehicle(
        {
            MISSION: [home_item(), item(1, apm.MAV_CMD_NAV_TAKEOFF, alt=4.0)],
            FENCE: [item(0, apm.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION, 1.0, 1.0, p1=50.0)],
            RALLY: [],
        }
    )


def test_reader_downloads_mission_fence_and_rally_in_one_cycle() -> None:
    fv = _vehicle()
    fv.run(0.0)
    assert fv.reader.plans == {
        MISSION: tuple(fv.plans[MISSION]),
        FENCE: tuple(fv.plans[FENCE]),
        RALLY: (),
    }
    assert fv.reader.version == 3
    fv.run(REFRESH_S / 2)
    assert not fv.sent, "nothing more until the refresh is due"


def test_reader_sees_a_new_mission_on_refresh_and_on_a_count_change() -> None:
    fv = _vehicle()
    fv.run(0.0)
    version = fv.reader.version
    fv.run(REFRESH_S + 0.1)
    assert fv.reader.version == version, "same plan, same version"
    fv.plans[MISSION] = [*fv.plans[MISSION], item(2, apm.MAV_CMD_NAV_LAND)]
    fv.reader.mission_total_seen(1, REFRESH_S + 1)
    fv.reader.mission_total_seen(2, REFRESH_S + 1)
    fv.run(REFRESH_S + 1)
    assert fv.reader.version == version + 1
    assert len(fv.reader.plans[MISSION]) == 3


def test_reader_backs_off_while_another_gcs_uploads() -> None:
    fv = _vehicle()
    fv.uploading = True
    fv.run(0.0)
    assert fv.reader.plans == {}
    fv.uploading = False
    fv.run(RETRY_S / 2)
    assert fv.reader.plans == {}
    fv.run(RETRY_S)
    assert MISSION in fv.reader.plans


def test_reader_skips_an_unsupported_type_and_a_silent_one() -> None:
    fv = _vehicle()
    fv.unsupported.add(FENCE)
    fv.run(0.0)
    assert set(fv.reader.plans) == {MISSION, RALLY}

    quiet = PlanReader(PlanSender(lambda t: None, lambda seq, t: None, lambda t: None))
    now = 0.0
    for _ in range(len(PLAN_TYPES) * STEP_RETRIES + 1):
        quiet.tick(now)
        now += STEP_TIMEOUT_S
    assert not quiet._busy  # pyright: ignore[reportPrivateUsage]
    assert quiet.plans == {}


def test_reader_ignores_out_of_order_and_foreign_replies() -> None:
    fv = _vehicle()
    fv.reader.tick(0.0)
    fv.reader.on_count(5, FENCE, 0.0)  # another type's count
    fv.reader.on_count(2, MISSION, 0.0)
    fv.reader.on_item(fv.plans[MISSION][1], MISSION, 0.0)  # seq 1 before seq 0
    fv.reader.on_item(fv.plans[MISSION][0], MISSION, 0.0)
    fv.reader.on_item(fv.plans[MISSION][1], MISSION, 0.0)
    assert fv.reader.plans[MISSION] == tuple(fv.plans[MISSION])
