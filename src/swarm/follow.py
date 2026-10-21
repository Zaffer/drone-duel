"""ArduPilot FOLLOW mode: the autopilot keeps the formation.

Each follower runs Copter's FOLLOW mode (mode 23) on `FOLL_SYSID`, the leader. It tracks the
leader's GLOBAL_POSITION_INT, which mavlink-router relays to every drone, and holds a fixed
north, east, down offset from it (`FOLL_OFS_TYPE 0`), whatever way the leader faces: its slot
of the swarm formation (swarm.formation). swarm.launch sets these parameters and switches the
followers to FOLLOW; from then on no ground software is in the loop. FOLLOW is a GUIDED
sub-class in ArduPilot, so `FS_OPTIONS` bit 2 keeps a follower in FOLLOW through a radio
failsafe.
"""

from typing import TYPE_CHECKING

import numpy as np

from swarm.formation import rotate_body_to_ned

if TYPE_CHECKING:
    from swarm.state import VehicleState

OFFSET_TYPE_NED = 0
"""FOLL_OFS_TYPE: offsets are north, east, down, whatever way the leader faces."""
OFFSET_TYPE_RELATIVE = 1
"""FOLL_OFS_TYPE: offsets are forward, right, down in the leader's heading frame."""
YAW_SAME_AS_LEADER = 2
"""FOLL_YAW_BEHAVE: point where the leader points, so cameras look the same way. A follower
turns on the spot with the leader."""
ALT_TYPE_RELATIVE = 1
"""FOLL_ALT_TYPE: heights above home. Every drone's home is its own takeoff spot."""
FOLLOW_PARAMS = ("FOLL_SYSID", "FOLL_OFS_TYPE", "FOLL_OFS_X", "FOLL_OFS_Y", "FOLL_OFS_Z")
"""Read from each vehicle so the viewer can draw the slot ArduPilot is flying to."""

PARAM_TOLERANCE = 1e-3


def follow_params(leader: int, slot: np.ndarray, dist_max_m: float) -> dict[str, float]:
    """FOLL_* values for one follower. `slot` is (north, east, down) from the leader."""
    return {
        "FOLL_ENABLE": 1.0,
        "FOLL_SYSID": float(leader),
        "FOLL_OFS_TYPE": float(OFFSET_TYPE_NED),
        "FOLL_OFS_X": float(slot[0]),
        "FOLL_OFS_Y": float(slot[1]),
        "FOLL_OFS_Z": float(slot[2]),
        "FOLL_YAW_BEHAVE": float(YAW_SAME_AS_LEADER),
        "FOLL_ALT_TYPE": float(ALT_TYPE_RELATIVE),
        "FOLL_DIST_MAX": dist_max_m,
    }


def follow_slot_ned(
    follower: "VehicleState", leader: "VehicleState", leader_ned: np.ndarray
) -> np.ndarray | None:
    """Where ArduPilot FOLLOW puts `follower`, in the frame of `leader_ned`, from the FOLL_*
    parameters the follower reported. None until they are known."""
    p = follower.params
    if any(name not in p for name in FOLLOW_PARAMS) or int(p["FOLL_SYSID"]) != leader.sysid:
        return None
    offset = np.array([p["FOLL_OFS_X"], p["FOLL_OFS_Y"], p["FOLL_OFS_Z"]])
    if int(p["FOLL_OFS_TYPE"]) == OFFSET_TYPE_RELATIVE:
        offset = rotate_body_to_ned(offset, leader.heading_rad)
    return leader_ned + offset


def param_mismatch(v: "VehicleState", wanted: dict[str, float]) -> list[str]:
    """Names in `wanted` the vehicle has not reported at the wanted value, FOLL_ENABLE first:
    the other FOLL_* parameters exist only once it is set."""
    if abs(v.params.get("FOLL_ENABLE", 0.0) - 1.0) > PARAM_TOLERANCE:
        return ["FOLL_ENABLE"]
    return [
        name
        for name, value in wanted.items()
        if name not in v.params or abs(v.params[name] - value) > PARAM_TOLERANCE
    ]
