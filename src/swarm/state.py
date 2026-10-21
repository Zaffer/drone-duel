"""Per-vehicle state as reported over MAVLink."""

from dataclasses import dataclass, field

import numpy as np
from pymavlink.dialects.v20 import ardupilotmega as apm

PILOT_MODES: frozenset[int] = frozenset(
    {
        apm.COPTER_MODE_ACRO,
        apm.COPTER_MODE_STABILIZE,
        apm.COPTER_MODE_ALT_HOLD,
        apm.COPTER_MODE_LOITER,
        apm.COPTER_MODE_POSHOLD,
        apm.COPTER_MODE_SPORT,
        apm.COPTER_MODE_DRIFT,
    }
)
"""Modes in which a human on a transmitter is flying the vehicle."""

AIRBORNE_STATES = frozenset(
    {apm.MAV_LANDED_STATE_IN_AIR, apm.MAV_LANDED_STATE_TAKEOFF, apm.MAV_LANDED_STATE_LANDING}
)
"""EXTENDED_SYS_STATE landed states of a copter off the ground."""

MODE_NAMES: dict[int, str] = {
    apm.COPTER_MODE_ACRO: "ACRO",
    apm.COPTER_MODE_STABILIZE: "STABILIZE",
    apm.COPTER_MODE_SPORT: "SPORT",
    apm.COPTER_MODE_DRIFT: "DRIFT",
    apm.COPTER_MODE_ALT_HOLD: "ALT_HOLD",
    apm.COPTER_MODE_AUTO: "AUTO",
    apm.COPTER_MODE_GUIDED: "GUIDED",
    apm.COPTER_MODE_LOITER: "LOITER",
    apm.COPTER_MODE_RTL: "RTL",
    apm.COPTER_MODE_LAND: "LAND",
    apm.COPTER_MODE_POSHOLD: "POSHOLD",
    apm.COPTER_MODE_BRAKE: "BRAKE",
    apm.COPTER_MODE_FOLLOW: "FOLLOW",
    apm.COPTER_MODE_AVOID_ADSB: "AVOID_ADSB",
}


def mode_name(mode: int | None) -> str:
    if mode is None:
        return "?"
    return MODE_NAMES.get(mode, str(mode))


@dataclass
class VehicleState:
    sysid: int
    mode: int | None = None
    armed: bool = False
    rc_ok: bool = False
    prearm_ok: bool = False
    gps_fix_type: int = 0
    ekf_flags: int = 0
    landed_state: int = apm.MAV_LANDED_STATE_UNDEFINED
    lat_deg: float = 0.0
    lon_deg: float = 0.0
    rel_alt_m: float = 0.0
    velocity_ned: np.ndarray = field(default_factory=lambda: np.zeros(3))
    heading_rad: float = 0.0
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    battery_pct: int | None = None
    """Remaining battery as reported in SYS_STATUS, or None when the vehicle does not know."""
    battery_v: float | None = None
    """Battery voltage as reported in SYS_STATUS, or None when the vehicle does not know."""
    last_heartbeat_s: float = 0.0
    last_position_s: float = 0.0
    mode_changed_s: float = 0.0
    rc_channels_us: tuple[int, ...] = ()
    """Raw RC input per channel, channel 1 first, as reported in RC_CHANNELS."""
    mission_seq: int | None = None
    """Mission item being flown or next up, from MISSION_CURRENT. Home is item 0."""
    mission_state: int = 0
    """MISSION_STATE from MISSION_CURRENT: 1 no mission, 2 not started, 3 active, 4 paused,
    5 complete. 0 when not reported."""
    fence_enabled: bool = False
    params: dict[str, float] = field(default_factory=dict)
    """Parameter values the vehicle has reported (PARAM_VALUE), by name."""
    threat_level: int = apm.MAV_COLLISION_THREAT_LEVEL_NONE
    """The most serious collision threat ArduPilot's avoidance reports (COLLISION)."""
    threat_from: int | None = None
    """System id of that threat."""

    @property
    def has_position(self) -> bool:
        return self.last_position_s > 0.0

    @property
    def in_pilot_mode(self) -> bool:
        return self.mode in PILOT_MODES

    @property
    def in_air(self) -> bool:
        """Off the ground: flying, or climbing out or descending in a takeoff or landing.
        ArduCopter reports TAKEOFF and LANDING instead of IN_AIR while those run."""
        return self.landed_state in AIRBORNE_STATES

    @property
    def position_ok(self) -> bool:
        """True when the EKF can navigate. Mirrors Copter::position_ok for a disarmed vehicle."""
        has_abs = bool(self.ekf_flags & (apm.EKF_POS_HORIZ_ABS | apm.EKF_PRED_POS_HORIZ_ABS))
        return has_abs and self.gps_fix_type >= apm.GPS_FIX_TYPE_3D_FIX and self.has_position
