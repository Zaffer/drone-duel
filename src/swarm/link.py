"""MAVLink I/O for a set of vehicles reachable over one or more connections.

A connection may carry one vehicle (SITL TCP port, a companion serial link) or many
(a mavlink-router UDP endpoint on the field laptop). Vehicles are keyed by system id.
"""

import logging
import time
from collections.abc import Sequence
from typing import cast

import numpy as np
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as apm

from swarm.fence import FenceSender, FenceUploader
from swarm.follow import FOLLOW_PARAMS
from swarm.mission import PLAN_PARAMS, PlanReader, PlanSender, RawItem
from swarm.state import VehicleState, mode_name

log = logging.getLogger(__name__)

GROUND_SYSID = 254
"""The ground stack's system id (swarm pilot, the watcher). The drones list 254..255 as GCS
ids, so QGroundControl (255) and we both count."""
STREAM_REASSERT_S = 10.0
"""Re-send message interval requests this often. Another GCS on the same link may reset rates."""
STREAMS_HZ: dict[int, float] = {
    apm.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 10.0,
    apm.MAVLINK_MSG_ID_ATTITUDE: 10.0,
    apm.MAVLINK_MSG_ID_SYS_STATUS: 2.0,
    apm.MAVLINK_MSG_ID_GPS_RAW_INT: 2.0,
    apm.MAVLINK_MSG_ID_EKF_STATUS_REPORT: 2.0,
    apm.MAVLINK_MSG_ID_EXTENDED_SYS_STATE: 2.0,
    apm.MAVLINK_MSG_ID_RC_CHANNELS: 10.0,
    apm.MAVLINK_MSG_ID_MISSION_CURRENT: 1.0,
}
PARAM_REFRESH_S = 15.0
AVOID_PARAMS = ("AVD_ENABLE", "AVD_F_ACTION")
"""ArduPilot avoidance between the drones (sitl/swarm.parm), read back from each vehicle."""
THREAT_NAMES = {
    apm.MAV_COLLISION_THREAT_LEVEL_NONE: "clear",
    apm.MAV_COLLISION_THREAT_LEVEL_LOW: "warning",
    apm.MAV_COLLISION_THREAT_LEVEL_HIGH: "too close",
}
AVOID_ACTION_NAMES = {
    apm.MAV_COLLISION_ACTION_NONE: "no action",
    apm.MAV_COLLISION_ACTION_REPORT: "reported",
    apm.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND: "climbing or descending away",
    apm.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY: "moving away",
    apm.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR: "escaping",
    apm.MAV_COLLISION_ACTION_RTL: "returning home",
    apm.MAV_COLLISION_ACTION_HOVER: "holding",
}
COPTER_TYPES = frozenset(
    {apm.MAV_TYPE_QUADROTOR, apm.MAV_TYPE_HEXAROTOR, apm.MAV_TYPE_OCTOROTOR, apm.MAV_TYPE_TRICOPTER}
)
RESULT_NAMES: dict[int, str] = {
    apm.MAV_RESULT_ACCEPTED: "ACCEPTED",
    apm.MAV_RESULT_TEMPORARILY_REJECTED: "TEMPORARILY_REJECTED",
    apm.MAV_RESULT_DENIED: "DENIED",
    apm.MAV_RESULT_UNSUPPORTED: "UNSUPPORTED",
    apm.MAV_RESULT_FAILED: "FAILED",
    apm.MAV_RESULT_IN_PROGRESS: "IN_PROGRESS",
}


class MavlinkLink:
    def __init__(
        self,
        connections: Sequence[str],
        source_system: int = GROUND_SYSID,
        log_name: str = __name__,
    ) -> None:
        self.log = logging.getLogger(log_name)
        """Logger for vehicles appearing and their STATUSTEXT. Two links in one process (the
        watcher and `swarm pilot` in `swarm fly`) set one quiet so each line appears once."""
        self._conns: list[mavutil.mavfile] = [
            cast(
                mavutil.mavfile,
                mavutil.mavlink_connection(
                    c,
                    source_system=source_system,
                    source_component=apm.MAV_COMP_ID_ONBOARD_COMPUTER,
                    autoreconnect=True,
                ),
            )
            for c in connections
        ]
        self._route: dict[int, mavutil.mavfile] = {}
        self.vehicles: dict[int, VehicleState] = {}
        self.plans: dict[int, PlanReader] = {}
        """Mission, fence and rally download per vehicle (swarm.mission)."""
        self._fence_uploads: dict[int, FenceUploader] = {}
        """Fence upload per vehicle, started by `upload_fence` (swarm.fence)."""
        self._last_heartbeat_sent = 0.0
        self._last_stream_request = 0.0
        self._last_param_request = 0.0

    # -- receive -----------------------------------------------------------------

    def poll(self) -> None:
        """Drain every connection and update vehicle states. Non-blocking."""
        now = time.monotonic()
        for conn in self._conns:
            while True:
                msg = conn.recv_match(blocking=False)
                if msg is None:
                    break
                self._handle(conn, msg, now)
        if now - self._last_heartbeat_sent >= 1.0:
            self._last_heartbeat_sent = now
            for conn in self._conns:
                conn.mav.heartbeat_send(
                    apm.MAV_TYPE_ONBOARD_CONTROLLER,
                    apm.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    apm.MAV_STATE_ACTIVE,
                )
        if now - self._last_stream_request >= STREAM_REASSERT_S:
            self._last_stream_request = now
            for sysid in self.vehicles:
                self._request_streams(sysid)
        if now - self._last_param_request >= PARAM_REFRESH_S:
            self._last_param_request = now
            for sysid in self.vehicles:
                self._request_display_params(sysid)
        for reader in self.plans.values():
            reader.tick(now)
        for upload in self._fence_uploads.values():
            upload.tick(now)

    def _handle(self, conn: mavutil.mavfile, msg: apm.MAVLink_message, now: float) -> None:
        sysid = msg.get_srcSystem()
        kind = msg.get_type()
        if kind == "BAD_DATA" or sysid == 0:
            return
        vehicle = self.vehicles.get(sysid)
        if vehicle is None:
            # Only a copter heartbeat registers a new vehicle. Ground stations are ignored.
            if kind != "HEARTBEAT" or msg.type not in COPTER_TYPES:  # type: ignore[attr-defined]
                return
            vehicle = VehicleState(sysid=sysid)
            self.vehicles[sysid] = vehicle
            self._route[sysid] = conn
            self.log.info("vehicle %d appeared", sysid)
            self._request_streams(sysid)
            self._request_display_params(sysid)
            self.plans[sysid] = PlanReader(self._plan_sender(sysid))
            self._fence_uploads[sysid] = FenceUploader(self._fence_sender(sysid))

        if kind == "HEARTBEAT":
            if msg.get_srcComponent() != apm.MAV_COMP_ID_AUTOPILOT1:
                return
            mode = int(msg.custom_mode)  # type: ignore[attr-defined]
            if mode != vehicle.mode:
                vehicle.mode_changed_s = now
                self.log.info("[%d] mode %s", sysid, mode_name(mode))
            vehicle.mode = mode
            vehicle.armed = bool(msg.base_mode & apm.MAV_MODE_FLAG_SAFETY_ARMED)  # type: ignore[attr-defined]
            vehicle.last_heartbeat_s = now
        elif kind == "GLOBAL_POSITION_INT":
            _apply_position(vehicle, msg, now)
        elif kind == "ATTITUDE":
            vehicle.roll_rad = float(msg.roll)  # type: ignore[attr-defined]
            vehicle.pitch_rad = float(msg.pitch)  # type: ignore[attr-defined]
        elif kind == "RC_CHANNELS":
            count = min(int(msg.chancount), 18)  # type: ignore[attr-defined]
            vehicle.rc_channels_us = tuple(
                int(getattr(msg, f"chan{i}_raw")) for i in range(1, count + 1)
            )
        elif kind == "SYS_STATUS":
            health: int = msg.onboard_control_sensors_health  # type: ignore[attr-defined]
            vehicle.rc_ok = bool(health & apm.MAV_SYS_STATUS_SENSOR_RC_RECEIVER)
            enabled: int = msg.onboard_control_sensors_enabled  # type: ignore[attr-defined]
            vehicle.fence_enabled = bool(enabled & apm.MAV_SYS_STATUS_GEOFENCE)
            vehicle.prearm_ok = bool(health & apm.MAV_SYS_STATUS_PREARM_CHECK)
            remaining = int(msg.battery_remaining)  # type: ignore[attr-defined]
            vehicle.battery_pct = remaining if remaining >= 0 else None
            millivolts = int(msg.voltage_battery)  # type: ignore[attr-defined]
            vehicle.battery_v = millivolts / 1000 if millivolts != 65535 else None
        elif kind == "GPS_RAW_INT":
            vehicle.gps_fix_type = int(msg.fix_type)  # type: ignore[attr-defined]
        elif kind == "EKF_STATUS_REPORT":
            vehicle.ekf_flags = int(msg.flags)  # type: ignore[attr-defined]
        elif kind == "EXTENDED_SYS_STATE":
            vehicle.landed_state = int(msg.landed_state)  # type: ignore[attr-defined]
        elif kind == "COMMAND_ACK":
            result: int = msg.result  # type: ignore[attr-defined]
            if result != apm.MAV_RESULT_ACCEPTED:
                self.log.warning(
                    "[%d] command %d -> %s",
                    sysid,
                    msg.command,  # type: ignore[attr-defined]
                    RESULT_NAMES.get(result, str(result)),
                )
        elif kind == "MISSION_CURRENT":
            vehicle.mission_seq = int(msg.seq)  # type: ignore[attr-defined]
            vehicle.mission_state = int(getattr(msg, "mission_state", 0) or 0)
            self.plans[sysid].mission_total_seen(int(getattr(msg, "total", 0) or 0), now)
        elif kind == "PARAM_VALUE":
            name = _param_name(msg.param_id)  # type: ignore[attr-defined]
            vehicle.params[name] = float(msg.param_value)  # type: ignore[attr-defined]
        elif kind in ("MISSION_COUNT", "MISSION_ITEM_INT", "MISSION_ACK"):
            if msg.target_system == GROUND_SYSID:  # type: ignore[attr-defined]
                self._plan_message(sysid, kind, msg, now)
        elif kind in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
            if msg.target_system == GROUND_SYSID:  # type: ignore[attr-defined]
                self._fence_uploads[sysid].on_request(
                    int(msg.seq),  # type: ignore[attr-defined]
                    int(getattr(msg, "mission_type", 0) or 0),
                    now,
                )
        elif kind == "COLLISION":
            self._collision(vehicle, msg)
        elif kind == "STATUSTEXT":
            self.log.info("[%d] %s", sysid, msg.text)  # type: ignore[attr-defined]

    def _collision(self, vehicle: VehicleState, msg: apm.MAVLink_message) -> None:
        """ArduPilot's avoidance reports its most serious threat; log when it changes."""
        level = int(msg.threat_level)  # type: ignore[attr-defined]
        other = int(msg.id)  # type: ignore[attr-defined]
        if (level, other) == (vehicle.threat_level, vehicle.threat_from):
            return
        vehicle.threat_level, vehicle.threat_from = level, other
        action = int(msg.action)  # type: ignore[attr-defined]
        across = float(msg.horizontal_minimum_delta)  # type: ignore[attr-defined]
        updown = float(msg.altitude_minimum_delta)  # type: ignore[attr-defined]
        log = self.log.warning if level == apm.MAV_COLLISION_THREAT_LEVEL_HIGH else self.log.info
        log(
            "[%d] %s %d: %s (closest %.1f m across, %.1f m up or down)",
            vehicle.sysid,
            THREAT_NAMES.get(level, str(level)),
            other,
            AVOID_ACTION_NAMES.get(action, str(action)),
            across,
            abs(updown),
        )

    def _request_streams(self, sysid: int) -> None:
        for msg_id, hz in STREAMS_HZ.items():
            self._command(sysid, apm.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz)

    # -- plans (read only) -------------------------------------------------------

    def _request_display_params(self, sysid: int) -> None:
        for name in PLAN_PARAMS + FOLLOW_PARAMS + AVOID_PARAMS:
            self.request_param(sysid, name)

    def _plan_sender(self, sysid: int) -> PlanSender:
        comp = apm.MAV_COMP_ID_AUTOPILOT1
        return PlanSender(
            request_list=lambda t: self._mav(sysid).mav.mission_request_list_send(sysid, comp, t),
            request_item=lambda seq, t: self._mav(sysid).mav.mission_request_int_send(
                sysid, comp, seq, t
            ),
            ack=lambda t: self._mav(sysid).mav.mission_ack_send(
                sysid, comp, apm.MAV_MISSION_ACCEPTED, t
            ),
        )

    def _plan_message(self, sysid: int, kind: str, msg: apm.MAVLink_message, now: float) -> None:
        reader = self.plans[sysid]
        mission_type = int(getattr(msg, "mission_type", 0) or 0)
        if kind == "MISSION_COUNT":
            reader.on_count(int(msg.count), mission_type, now)  # type: ignore[attr-defined]
        elif kind == "MISSION_ACK":
            result = int(msg.type)  # type: ignore[attr-defined]
            if self._fence_uploads[sysid].on_ack(result, mission_type):
                self.log.info("[%d] geofence uploaded", sysid)
                reader.refresh(now)  # read it back
            else:
                reader.on_ack(result, mission_type, now)
        else:
            reader.on_item(
                RawItem(
                    seq=int(msg.seq),  # type: ignore[attr-defined]
                    frame=int(msg.frame),  # type: ignore[attr-defined]
                    command=int(msg.command),  # type: ignore[attr-defined]
                    params=(
                        float(msg.param1),  # type: ignore[attr-defined]
                        float(msg.param2),  # type: ignore[attr-defined]
                        float(msg.param3),  # type: ignore[attr-defined]
                        float(msg.param4),  # type: ignore[attr-defined]
                    ),
                    lat_deg=msg.x * 1e-7,  # type: ignore[attr-defined]
                    lon_deg=msg.y * 1e-7,  # type: ignore[attr-defined]
                    alt_m=float(msg.z),  # type: ignore[attr-defined]
                ),
                mission_type,
                now,
            )

    # -- fence (swarm.fence) -----------------------------------------------------

    def upload_fence(self, sysid: int, corners: tuple[tuple[float, float], ...]) -> bool:
        """Start uploading the fence polygon. False while the link is still MAVLink 1: there a
        MISSION_COUNT loses its mission type and would replace the drone's mission instead.
        pymavlink switches to MAVLink 2 on the first MAVLink 2 packet it hears."""
        if self._mav(sysid).WIRE_PROTOCOL_VERSION != "2.0":
            self.log.warning("[%d] not uploading the geofence: the link is not MAVLink 2", sysid)
            return False
        self._fence_uploads[sysid].start(corners, time.monotonic())
        return True

    def fence_uploading(self, sysid: int) -> bool:
        upload = self._fence_uploads.get(sysid)
        return upload is not None and upload.busy

    def fence_on_vehicle(self, sysid: int) -> tuple[RawItem, ...] | None:
        """The fence items last downloaded from the vehicle; None before the first download."""
        reader = self.plans.get(sysid)
        return None if reader is None else reader.plans.get(apm.MAV_MISSION_TYPE_FENCE)

    def _fence_sender(self, sysid: int) -> FenceSender:
        comp, fence = apm.MAV_COMP_ID_AUTOPILOT1, apm.MAV_MISSION_TYPE_FENCE
        return FenceSender(
            count=lambda n: self._mav(sysid).mav.mission_count_send(sysid, comp, n, fence),
            item=lambda seq, lat, lon, n: self._mav(sysid).mav.mission_item_int_send(
                sysid,
                comp,
                seq,
                apm.MAV_FRAME_GLOBAL,
                apm.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0,
                0,
                float(n),
                0.0,
                0.0,
                0.0,
                round(lat * 1e7),
                round(lon * 1e7),
                0.0,
                fence,
            ),
        )

    # -- send --------------------------------------------------------------------

    def _mav(self, sysid: int) -> mavutil.mavfile:
        return self._route[sysid]

    def _command(self, sysid: int, command: int, *params: float) -> None:
        p = list(params) + [0.0] * (7 - len(params))
        self._mav(sysid).mav.command_long_send(sysid, apm.MAV_COMP_ID_AUTOPILOT1, command, 0, *p)

    def set_mode(self, sysid: int, mode: int) -> None:
        self._command(sysid, apm.MAV_CMD_DO_SET_MODE, apm.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode)

    def arm(self, sysid: int) -> None:
        self._command(sysid, apm.MAV_CMD_COMPONENT_ARM_DISARM, 1)

    def takeoff(self, sysid: int, altitude_m: float) -> None:
        self._command(sysid, apm.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude_m)

    def request_param(self, sysid: int, name: str) -> None:
        self._mav(sysid).mav.param_request_read_send(
            sysid, apm.MAV_COMP_ID_AUTOPILOT1, name.encode(), -1
        )

    def set_param(self, sysid: int, name: str, value: float) -> None:
        self._mav(sysid).mav.param_set_send(
            sysid, apm.MAV_COMP_ID_AUTOPILOT1, name.encode(), value, apm.MAV_PARAM_TYPE_REAL32
        )


def _param_name(raw: str | bytes) -> str:
    text = raw.decode(errors="replace") if isinstance(raw, bytes) else raw
    return text.rstrip("\x00")


def _apply_position(state: VehicleState, msg: apm.MAVLink_message, now: float) -> None:
    state.lat_deg = msg.lat * 1e-7  # type: ignore[attr-defined]
    state.lon_deg = msg.lon * 1e-7  # type: ignore[attr-defined]
    state.rel_alt_m = msg.relative_alt * 1e-3  # type: ignore[attr-defined]
    state.velocity_ned = np.array([msg.vx, msg.vy, msg.vz], dtype=float) * 1e-2  # type: ignore[attr-defined]
    if msg.hdg != 65535:  # type: ignore[attr-defined]
        state.heading_rad = float(np.radians(msg.hdg * 1e-2))  # type: ignore[attr-defined]
    state.last_position_s = now
