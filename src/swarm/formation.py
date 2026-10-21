"""The swarm formation: where each follower flies relative to the leader.

Offsets are (north, east, down) in metres from the leader, as ArduPilot FOLLOW takes them with
FOLL_OFS_TYPE 0 (swarm.follow). They ignore the leader's heading: the leader yaws in place and
the swarm stays where it is.
"""

import numpy as np

FORMATION_RADIUS_M = 5.0
"""Every follower flies this far from the leader, horizontally."""
FORMATION_ALT_STEP_M = 1.5
"""Heights step by this much, follower to follower, the first this much above the leader."""


def rotate_body_to_ned(offset_body: np.ndarray, heading_rad: float) -> np.ndarray:
    c, s = np.cos(heading_rad), np.sin(heading_rad)
    fwd, right, down = offset_body
    return np.array([c * fwd - s * right, s * fwd + c * right, down])


def formation_slots(
    count: int, radius: float = FORMATION_RADIUS_M, alt_step: float = FORMATION_ALT_STEP_M
) -> list[np.ndarray]:
    """Slot offsets (north, east, down) for `count` followers: the swarm formation.

    A ring `radius` round the leader, evenly spaced clockwise from north: with four followers a
    square, north, east, south and west of it. Fixed to north, not the leader's nose, so a yaw
    turns the leader on the spot and the followers stay put; one 90 degrees on from the next,
    the pilot's 120 degree FPV camera has one or two in frame whichever way it points. Heights
    climb follower by follower in `alt_step`s from the leader, the lowest: with four, 1.5, 3,
    4.5 and 6 m above it. No follower is ever below the leader, so none is taken into the
    ground when the leader flies low."""
    slots: list[np.ndarray] = []
    for k in range(count):
        bearing = 2 * np.pi * k / count
        slots.append(
            np.array([radius * np.cos(bearing), radius * np.sin(bearing), -alt_step * (k + 1)])
        )
    return slots
