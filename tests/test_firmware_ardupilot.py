"""ArduCopter SITL's adapter (firmware/ardupilot.py) and the game's MAVLink link
(telemetry/mavlink.py), without starting either."""

import struct
from typing import Any

from pymavlink.dialects.v20 import ardupilotmega as apm

from drone_duels import ports
from drone_duels.firmware.ardupilot import (
    IDLE_US,
    osd_message,
    rc_packet,
    router_conf,
    video_params,
)
from drone_duels.server.protocol import OsdMsg, PlanMsg, TelemetryMsg
from drone_duels.telemetry.mavlink import GAME_SYSID, VehicleLink
from drone_duels.video.osd import DRAW, WRITE, DisplayPort


def test_rc_packet_is_16_channels_padded_idle() -> None:
    data = rc_packet([1500, 1500, 1200, 1500, 2000, 1000])
    assert len(data) == 32
    us = struct.unpack("<16H", data)
    assert us[:6] == (1500, 1500, 1200, 1500, 2000, 1000)
    assert us[6:] == IDLE_US[6:]
    assert struct.unpack("<16H", rc_packet([0, 5000] + [1500] * 20))[:2] == (800, 2200)
    assert IDLE_US[2] == 1000 and IDLE_US[4] == 1000  # throttle and the arm switch low


def test_router_conf_for_a_slot() -> None:
    conf = router_conf(1)
    assert "TcpServerPort = 0" in conf  # its own TCP server would take SITL's 5760
    assert f"Port = {ports.slot(1).ardupilot_mavlink}" in conf
    assert f"Port = {ports.ROUTER_QGC}" in conf and f"Port = {ports.ROUTER_GAME}" in conf
    assert "Mode = Server" in conf and "Mode = Normal" in conf


def test_video_system_sets_the_osd_grid() -> None:
    assert "OSD1_TXT_RES 0" in video_params("analog")
    assert "OSD1_TXT_RES 1" in video_params("digital")


def test_osd_message() -> None:
    port = DisplayPort(analog=True)
    port.handle(bytes([WRITE, 1, 2, 0]) + b"LOIT")
    port.handle(bytes([DRAW]))
    assert port.screen is not None
    msg = OsdMsg.model_validate(osd_message(port.screen))
    assert (msg.cols, msg.rows, msg.font) == (30, 13, "0")
    assert len(msg.codes) == 30 * 13
    assert bytes(msg.codes[32:36]) == b"LOIT"


class Collect:
    def __init__(self) -> None:
        self.msgs: list[dict[str, Any]] = []

    def __call__(self, msg: dict[str, Any]) -> None:
        self.msgs.append(msg)

    def of(self, ch: str) -> list[dict[str, Any]]:
        return [m for m in self.msgs if m["ch"] == ch]


VEHICLE = apm.MAVLink(None, srcSystem=1, srcComponent=apm.MAV_COMP_ID_AUTOPILOT1)
QGC = apm.MAVLink(None, srcSystem=255, srcComponent=190)


def heard(msg: apm.MAVLink_message, mav: apm.MAVLink = VEHICLE) -> apm.MAVLink_message:
    """`msg` as it arrives: packed by `mav`, decoded with its header."""
    rx = apm.MAVLink(None)
    out = rx.decode(bytearray(msg.pack(mav)))
    assert out is not None
    return out


def test_link_state_from_messages() -> None:
    publish = Collect()
    link = VehicleLink("udpout:127.0.0.1:1", publish)
    hb = VEHICLE.heartbeat_encode(
        apm.MAV_TYPE_QUADROTOR, apm.MAV_AUTOPILOT_ARDUPILOTMEGA,
        apm.MAV_MODE_FLAG_SAFETY_ARMED | apm.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 5,
        apm.MAV_STATE_ACTIVE,
    )
    link.handle(heard(hb), 1.0)
    assert link.heard() and link.state.armed and link.state.mode == "LOITER"
    # QGroundControl's heartbeat, relayed by the router, is not the vehicle's.
    qgc_hb = QGC.heartbeat_encode(apm.MAV_TYPE_GCS, apm.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    link.handle(heard(qgc_hb, QGC), 1.0)
    assert link.state.mode == "LOITER"

    link.handle(heard(VEHICLE.gps_raw_int_encode(0, 3, 0, 0, 0, 0, 0, 0, 0, 12)), 1.0)
    flags = (apm.EKF_ATTITUDE | apm.EKF_VELOCITY_HORIZ | apm.EKF_VELOCITY_VERT
             | apm.EKF_POS_HORIZ_ABS | apm.EKF_POS_VERT_ABS)
    link.handle(heard(VEHICLE.ekf_status_report_encode(flags, 0, 0, 0, 0, 0)), 1.0)
    text = VEHICLE.statustext_encode(apm.MAV_SEVERITY_CRITICAL, b"PreArm: Throttle too high")
    link.handle(heard(text), 1.0)
    assert not link.ready()
    assert link.state.blocker_list(2.0) == ["Throttle too high"]
    assert publish.of("log") == [
        {"ch": "log", "level": "error", "source": "ardupilot",
         "text": "PreArm: Throttle too high"}
    ]
    healthy = apm.MAV_SYS_STATUS_PREARM_CHECK
    status = VEHICLE.sys_status_encode(healthy, healthy, healthy, 0, 24800, 600, 87,
                                       0, 0, 0, 0, 0, 0)
    link.handle(heard(status), 2.0)
    assert link.ready() and link.state.blocker_list(2.0) == []
    pos = VEHICLE.global_position_int_encode(0, 436309300, -793894500, 87000, 10000,
                                             300, -400, -150, 27000)
    link.handle(heard(pos), 2.0)
    link.handle(heard(VEHICLE.attitude_encode(0, 0.1, -0.2, -0.5, 0, 0, 0)), 2.0)
    link.handle(heard(VEHICLE.vfr_hud_encode(0.0, 5.0, 270, 21, 87.0, 1.5)), 2.0)
    tel = TelemetryMsg.model_validate(link.state.telemetry(2.0))
    assert tel.armed and tel.mode == "LOITER"
    assert (tel.lat, tel.lon) == (43.63093, -79.38945)
    assert (tel.alt_amsl, tel.alt_rel) == (87.0, 10.0)
    assert tel.pitch_deg < 0 < tel.roll_deg and 330 < tel.yaw_deg < 332
    assert (tel.throttle_pct, tel.climb, tel.groundspeed) == (21.0, 1.5, 5.0)
    assert (tel.battery_v, tel.battery_pct, tel.gps_sats) == (24.8, 87.0, 12)


def test_link_reads_the_plans() -> None:
    publish = Collect()
    link = VehicleLink("udpout:127.0.0.1:1", publish)
    sent: list[tuple[str, int, int]] = []
    link._plans.sender.request_list = lambda t: sent.append(("list", t, -1))  # pyright: ignore[reportPrivateUsage]
    link._plans.sender.request_item = lambda seq, t: sent.append(("item", t, seq))  # pyright: ignore[reportPrivateUsage]
    link._plans.sender.ack = lambda t: sent.append(("ack", t, -1))  # pyright: ignore[reportPrivateUsage]
    link._plans.tick(0.0)  # pyright: ignore[reportPrivateUsage]
    assert sent == [("list", apm.MAV_MISSION_TYPE_MISSION, -1)]

    def item(seq: int, cmd: int, kind: int) -> apm.MAVLink_message:
        return heard(VEHICLE.mission_item_int_encode(
            GAME_SYSID, 190, seq, apm.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, cmd, 0, 1,
            0, 0, 0, 0, 436309300 + seq, -793894500, 10.0 * seq, kind))

    mission = apm.MAV_MISSION_TYPE_MISSION
    link.handle(heard(VEHICLE.mission_count_encode(GAME_SYSID, 190, 2, mission)), 0.1)
    link.handle(item(0, apm.MAV_CMD_NAV_WAYPOINT, mission), 0.2)
    link.handle(item(1, apm.MAV_CMD_NAV_TAKEOFF, mission), 0.3)
    for kind in (apm.MAV_MISSION_TYPE_FENCE, apm.MAV_MISSION_TYPE_RALLY):
        link.handle(heard(VEHICLE.mission_count_encode(GAME_SYSID, 190, 0, kind)), 0.4)
    assert ("ack", mission, -1) in sent
    # Each type downloaded once.
    assert link._plans.version == 3  # pyright: ignore[reportPrivateUsage]
    from drone_duels.telemetry.plans import plan_msg

    plan = PlanMsg.model_validate(plan_msg(link._plans.plans))  # pyright: ignore[reportPrivateUsage]
    assert [i.command for i in plan.mission] == [apm.MAV_CMD_NAV_WAYPOINT,
                                                 apm.MAV_CMD_NAV_TAKEOFF]
    assert plan.mission[1].alt == 10.0 and plan.fence == [] and plan.rally == []
