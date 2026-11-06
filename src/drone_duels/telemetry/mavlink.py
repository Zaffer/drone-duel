"""The game's MAVLink link to one ArduPilot vehicle, through mavlink-router, as a ground station
sees it (DESIGN.md §2.2): system 254, the router's game endpoint.

`VehicleLink` runs on a thread of its own. It keeps the vehicle's state from what it hears
(HEARTBEAT, GLOBAL_POSITION_INT, ATTITUDE, VFR_HUD, SYS_STATUS, GPS_RAW_INT, EKF_STATUS_REPORT),
asks for those streams at TELEMETRY_HZ (again every STREAM_REASSERT_S: QGroundControl on the same
serial port may change them), and publishes:
  - `telemetry` (protocol.TelemetryMsg) at TELEMETRY_HZ;
  - `log` for each STATUSTEXT;
  - `plan` (protocol.PlanMsg) whenever the mission, fence or rally points download differently.
Its `ready` says whether the vehicle could arm: a 3D GPS fix, an EKF with an absolute position,
and its pre-arm checks passing (SYS_STATUS's MAV_SYS_STATUS_PREARM_CHECK health bit). What stops
it arming, ArduPilot's "PreArm: ..." texts, are the telemetry's `blockers`.
"""

import logging
import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any, cast

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as apm

from drone_duels.server.protocol import TelemetryMsg
from drone_duels.telemetry.plans import PlanReader, PlanSender, plan_msg, raw_item

log = logging.getLogger(__name__)

GAME_SYSID = 254
"""The game's MAVLink system id. QGroundControl is 255; the vehicle takes both as GCS
(MAV_GCS_SYSID 254, MAV_GCS_SYSID_HI 255)."""
GAME_COMPID = apm.MAV_COMP_ID_MISSIONPLANNER
TELEMETRY_HZ = 10.0
STREAM_REASSERT_S = 10.0
STREAMS_HZ: dict[int, float] = {
    apm.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: TELEMETRY_HZ,
    apm.MAVLINK_MSG_ID_ATTITUDE: TELEMETRY_HZ,
    apm.MAVLINK_MSG_ID_VFR_HUD: TELEMETRY_HZ,
    apm.MAVLINK_MSG_ID_SYS_STATUS: 2.0,
    apm.MAVLINK_MSG_ID_GPS_RAW_INT: 2.0,
    apm.MAVLINK_MSG_ID_EKF_STATUS_REPORT: 2.0,
    apm.MAVLINK_MSG_ID_MISSION_CURRENT: 1.0,
}
PREARM_PREFIX = "PreArm: "
BLOCKER_TTL_S = 35.0
"""ArduPilot repeats failing pre-arm checks every 30 s: one not heard again for longer is gone."""
EKF_NEEDED = (
    apm.EKF_ATTITUDE
    | apm.EKF_VELOCITY_HORIZ
    | apm.EKF_VELOCITY_VERT
    | apm.EKF_POS_HORIZ_ABS
    | apm.EKF_POS_VERT_ABS
)
"""What the EKF must have before the vehicle can fly a position mode."""


ACM_MODES = {name: number for number, name in mavutil.mode_mapping_acm.items()}
"""ArduCopter's flight modes by name: GUIDED 4, LAND 9..."""
POSITION_ONLY = 0x0DF8
"""SET_POSITION_TARGET's type mask: use the position, ignore velocity, acceleration and yaw."""


def mode_name(custom_mode: int) -> str:
    return mavutil.mode_mapping_acm.get(custom_mode, str(custom_mode))


def log_level(severity: int) -> str:
    if severity <= apm.MAV_SEVERITY_ERROR:
        return "error"
    if severity == apm.MAV_SEVERITY_WARNING:
        return "warn"
    return "info"


@dataclass
class VehicleState:
    heartbeat_s: float = 0.0
    armed: bool = False
    mode: str = "?"
    lat: float | None = None
    lon: float | None = None
    alt_amsl: float | None = None
    alt_rel: float | None = None
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    groundspeed: float = 0.0
    airspeed: float | None = None
    climb: float = 0.0
    throttle_pct: float = 0.0
    battery_v: float | None = None
    battery_pct: float | None = None
    gps_sats: int = 0
    gps_fix: int = 0
    ekf_flags: int = 0
    prearm_ok: bool = False
    sys_status_s: float = 0.0
    blockers: dict[str, float] = field(default_factory=dict)
    """Pre-arm failures heard, without the "PreArm: ", and when last."""

    @property
    def ekf_ok(self) -> bool:
        return (self.ekf_flags & EKF_NEEDED) == EKF_NEEDED and not (
            self.ekf_flags & apm.EKF_CONST_POS_MODE
        )

    @property
    def gps_ok(self) -> bool:
        return self.gps_fix >= apm.GPS_FIX_TYPE_3D_FIX

    def blocker_list(self, now: float) -> list[str]:
        if self.prearm_ok:
            return []
        return sorted(t for t, at in self.blockers.items() if now - at < BLOCKER_TTL_S)

    def telemetry(self, now: float) -> dict[str, Any]:
        return TelemetryMsg(
            armed=self.armed,
            mode=self.mode,
            lat=self.lat,
            lon=self.lon,
            alt_amsl=self.alt_amsl,
            alt_rel=self.alt_rel,
            roll_deg=self.roll_deg,
            pitch_deg=self.pitch_deg,
            yaw_deg=self.yaw_deg,
            groundspeed=self.groundspeed,
            airspeed=self.airspeed,
            climb=self.climb,
            throttle_pct=self.throttle_pct,
            battery_v=self.battery_v,
            battery_pct=self.battery_pct,
            gps_sats=self.gps_sats,
            blockers=self.blocker_list(now),
        ).model_dump()


class VehicleLink:
    """MAVLink to vehicle `sysid` at `address` (pymavlink's form), on a thread; what it hears
    goes to `publish` as server → page messages."""

    def __init__(
        self,
        address: str,
        publish: Callable[[dict[str, Any]], None],
        sysid: int = 1,
    ) -> None:
        self.address = address
        self.publish = publish
        self.sysid = sysid
        self.state = VehicleState()
        self._changed = threading.Condition()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._conn: Any = None
        self._send_lock = threading.Lock()
        self._plans = PlanReader(self._plan_sender())
        self._plan_version = 0
        self._acks: dict[int, tuple[int, float]] = {}
        """The last COMMAND_ACK to the game for each command: its result, and when heard."""

    # Lifecycle.

    def start(self) -> None:
        self._conn = mavutil.mavlink_connection(
            self.address,
            source_system=GAME_SYSID,
            source_component=GAME_COMPID,
            dialect="ardupilotmega",
        )
        self._thread = threading.Thread(target=self._run, name="mavlink", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(2)
            self._thread = None
        if self._conn is not None:
            try:
                self._conn.close()
            except OSError:
                pass
            self._conn = None

    # What a spawn waits on.

    def heard(self) -> bool:
        return self.state.heartbeat_s > 0

    def ready(self) -> bool:
        s = self.state
        return self.heard() and s.gps_ok and s.ekf_ok and s.prearm_ok

    def wait(self, done: Callable[[], bool], timeout: float) -> bool:
        """Wait up to `timeout` s for `done()`, checked whenever a message changes the state."""
        deadline = time.monotonic() + timeout
        with self._changed:
            while not done():
                left = deadline - time.monotonic()
                if left <= 0:
                    return False
                self._changed.wait(min(left, 0.2))
        return True

    # Commands, as a GCS sends them.

    def _send(self, fn: Callable[[Any], None]) -> None:
        conn = self._conn
        if conn is None:
            return
        with self._send_lock:
            try:
                fn(conn.mav)
            except OSError as exc:
                log.debug("send: %s", exc)

    def command(self, command: int, *params: float) -> None:
        p = list(params) + [0.0] * (7 - len(params))
        self._send(
            lambda mav: mav.command_long_send(
                self.sysid, apm.MAV_COMP_ID_AUTOPILOT1, command, 0, *p
            )
        )

    def command_ack(self, command: int, *params: float, timeout: float = 1.5) -> int | None:
        """Send a command and wait for its COMMAND_ACK: the result (MAV_RESULT), or None if none
        came within `timeout` s."""
        sent = time.monotonic()
        self.command(command, *params)
        heard: list[int] = []

        def acked() -> bool:
            got = self._acks.get(command)
            if got is not None and got[1] >= sent:
                heard.append(got[0])
                return True
            return False

        return heard[-1] if self.wait(acked, timeout) else None

    def set_mode(self, mode: str, timeout: float = 1.5) -> int | None:
        """Switch to ArduCopter flight mode `mode` (GUIDED, LAND...): MAV_CMD_DO_SET_MODE's
        result, or None if unanswered."""
        number = ACM_MODES[mode]
        return self.command_ack(
            apm.MAV_CMD_DO_SET_MODE, apm.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, number,
            timeout=timeout,
        )

    def go_to(self, lat: float, lon: float, alt_rel: float) -> None:
        """In GUIDED, fly to a point `alt_rel` m above home (SET_POSITION_TARGET_GLOBAL_INT,
        position only)."""
        self._send(
            lambda mav: mav.set_position_target_global_int_send(
                0, self.sysid, apm.MAV_COMP_ID_AUTOPILOT1,
                apm.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, POSITION_ONLY,
                round(lat * 1e7), round(lon * 1e7), alt_rel,
                0, 0, 0, 0, 0, 0, 0, 0,
            )
        )

    def set_param(self, name: str, value: float) -> None:
        self._send(
            lambda mav: mav.param_set_send(
                self.sysid, apm.MAV_COMP_ID_AUTOPILOT1, name.encode(), value,
                apm.MAV_PARAM_TYPE_REAL32,
            )
        )

    def run_prearm_checks(self) -> None:
        """Ask the vehicle to run its pre-arm checks now and say what fails."""
        self.command(apm.MAV_CMD_RUN_PREARM_CHECKS)

    def _request_streams(self) -> None:
        for msg_id, hz in STREAMS_HZ.items():
            self.command(apm.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz)

    def _plan_sender(self) -> PlanSender:
        comp = apm.MAV_COMP_ID_AUTOPILOT1
        return PlanSender(
            request_list=lambda t: self._send(
                lambda mav: mav.mission_request_list_send(self.sysid, comp, t)
            ),
            request_item=lambda seq, t: self._send(
                lambda mav: mav.mission_request_int_send(self.sysid, comp, seq, t)
            ),
            ack=lambda t: self._send(
                lambda mav: mav.mission_ack_send(self.sysid, comp, apm.MAV_MISSION_ACCEPTED, t)
            ),
        )

    # The thread.

    def _run(self) -> None:
        conn = self._conn
        last_beat = last_streams = last_telemetry = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            if now - last_beat >= 1.0:
                last_beat = now
                self._send(
                    lambda mav: mav.heartbeat_send(
                        apm.MAV_TYPE_GCS, apm.MAV_AUTOPILOT_INVALID, 0, 0, apm.MAV_STATE_ACTIVE
                    )
                )
            if self.heard():
                if now - last_streams >= STREAM_REASSERT_S:
                    last_streams = now
                    self._request_streams()
                if now - last_telemetry >= 1 / TELEMETRY_HZ:
                    last_telemetry = now
                    self.publish(self.state.telemetry(now))
                self._plans.tick(now)
                if self._plans.version != self._plan_version:
                    self._plan_version = self._plans.version
                    self.publish(plan_msg(self._plans.plans))
            try:
                msg = conn.recv_match(blocking=True, timeout=0.05)
            except OSError as exc:  # the router is not up yet, or went away
                log.debug("recv: %s", exc)
                self._stop.wait(0.2)
                continue
            if msg is None:
                continue
            try:
                self.handle(msg, time.monotonic())
            except Exception:  # a malformed message must not end the link
                log.exception("handling %s", msg.get_type())

    def handle(self, msg: apm.MAVLink_message, now: float) -> None:
        """One message heard: the state, and what goes to the page."""
        if msg.get_srcSystem() != self.sysid:
            return
        kind = msg.get_type()
        s = self.state
        if kind == "HEARTBEAT":
            if msg.get_srcComponent() != apm.MAV_COMP_ID_AUTOPILOT1:
                return
            hb = cast(apm.MAVLink_heartbeat_message, msg)
            first = s.heartbeat_s == 0
            s.heartbeat_s = now
            s.armed = bool(hb.base_mode & apm.MAV_MODE_FLAG_SAFETY_ARMED)
            s.mode = mode_name(int(hb.custom_mode))
            if first:
                self._request_streams()
        elif kind == "GLOBAL_POSITION_INT":
            pos = cast(apm.MAVLink_global_position_int_message, msg)
            s.lat, s.lon = pos.lat * 1e-7, pos.lon * 1e-7
            s.alt_amsl, s.alt_rel = pos.alt * 1e-3, pos.relative_alt * 1e-3
        elif kind == "ATTITUDE":
            att = cast(apm.MAVLink_attitude_message, msg)
            s.roll_deg = _deg(att.roll)
            s.pitch_deg = _deg(att.pitch)
            s.yaw_deg = _deg(att.yaw) % 360
        elif kind == "VFR_HUD":
            hud = cast(apm.MAVLink_vfr_hud_message, msg)
            s.groundspeed = float(hud.groundspeed)
            s.airspeed = float(hud.airspeed)
            s.climb = float(hud.climb)
            s.throttle_pct = float(hud.throttle)
        elif kind == "SYS_STATUS":
            st = cast(apm.MAVLink_sys_status_message, msg)
            s.sys_status_s = now
            s.prearm_ok = bool(st.onboard_control_sensors_health & apm.MAV_SYS_STATUS_PREARM_CHECK)
            if s.prearm_ok:
                s.blockers.clear()
            s.battery_v = st.voltage_battery / 1000 if st.voltage_battery != 65535 else None
            s.battery_pct = float(st.battery_remaining) if st.battery_remaining >= 0 else None
        elif kind == "GPS_RAW_INT":
            gps = cast(apm.MAVLink_gps_raw_int_message, msg)
            s.gps_fix = int(gps.fix_type)
            s.gps_sats = int(gps.satellites_visible) if gps.satellites_visible != 255 else 0
        elif kind == "EKF_STATUS_REPORT":
            s.ekf_flags = int(cast(apm.MAVLink_ekf_status_report_message, msg).flags)
        elif kind == "STATUSTEXT":
            st_text = cast(apm.MAVLink_statustext_message, msg)
            text = str(st_text.text).rstrip("\x00")
            if text.startswith(PREARM_PREFIX):
                s.blockers[text[len(PREARM_PREFIX) :]] = now
            self.publish(
                {"ch": "log", "level": log_level(int(st_text.severity)), "source": "ardupilot",
                 "text": text}
            )
        elif kind == "COMMAND_ACK":
            ack = cast(apm.MAVLink_command_ack_message, msg)
            if int(getattr(ack, "target_system", 0) or 0) not in (0, GAME_SYSID):
                return  # QGroundControl's
            self._acks[int(ack.command)] = (int(ack.result), now)
        elif kind == "MISSION_CURRENT":
            total = int(getattr(msg, "total", 0) or 0)
            self._plans.mission_total_seen(total, now)
        elif kind in ("MISSION_COUNT", "MISSION_ITEM_INT", "MISSION_ACK"):
            if int(getattr(msg, "target_system", 0)) != GAME_SYSID:
                return
            mission_type = int(getattr(msg, "mission_type", 0) or 0)
            if kind == "MISSION_COUNT":
                count = cast(apm.MAVLink_mission_count_message, msg).count
                self._plans.on_count(int(count), mission_type, now)
            elif kind == "MISSION_ACK":
                result = cast(apm.MAVLink_mission_ack_message, msg).type
                self._plans.on_ack(int(result), mission_type, now)
            else:
                item = raw_item(cast(apm.MAVLink_mission_item_int_message, msg))
                self._plans.on_item(item, mission_type, now)
        else:
            return
        with self._changed:
            self._changed.notify_all()


def _deg(rad: float) -> float:
    return math.degrees(rad)
