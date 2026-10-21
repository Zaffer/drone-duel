"""ArduPilot FOLLOW parameters and the slot the viewer draws for a FOLLOW vehicle."""

import math

import numpy as np
from pymavlink.dialects.v20 import ardupilotmega as apm

from swarm.follow import follow_params, follow_slot_ned, param_mismatch
from swarm.formation import formation_slots
from swarm.state import VehicleState


def test_follow_params_use_the_formation_north_up() -> None:
    slot = formation_slots(4)[1]
    p = follow_params(1, slot, 100.0)
    assert p["FOLL_SYSID"] == 1
    assert p["FOLL_OFS_TYPE"] == 0
    np.testing.assert_allclose(
        [p["FOLL_OFS_X"], p["FOLL_OFS_Y"], p["FOLL_OFS_Z"]], [0, 5, -3], atol=1e-9
    )


def test_foll_enable_goes_first_then_only_what_differs() -> None:
    wanted = follow_params(1, np.array([-8.0, 8.0, 0.0]), 100.0)
    v = VehicleState(sysid=2)
    assert param_mismatch(v, wanted) == ["FOLL_ENABLE"]
    v.params = dict(wanted)
    assert param_mismatch(v, wanted) == []
    v.params["FOLL_OFS_Y"] = -8.0
    assert param_mismatch(v, wanted) == ["FOLL_OFS_Y"]


def test_follow_slot_ignores_the_leader_heading() -> None:
    leader = VehicleState(sysid=1, heading_rad=math.pi / 2)  # facing east
    follower = VehicleState(sysid=2, mode=apm.COPTER_MODE_FOLLOW)
    follower.params = follow_params(1, np.array([5.0, 0.0, 3.0]), 100.0)
    slot = follow_slot_ned(follower, leader, np.array([10.0, 20.0, -15.0]))
    assert slot is not None
    np.testing.assert_allclose(slot, [15.0, 20.0, -12.0], atol=1e-9)


def test_follow_slot_rotates_with_the_leader_heading_for_relative_offsets() -> None:
    leader = VehicleState(sysid=1, heading_rad=math.pi / 2)  # facing east
    follower = VehicleState(sysid=2, mode=apm.COPTER_MODE_FOLLOW)
    follower.params = follow_params(1, np.array([-8.0, 8.0, 0.0]), 100.0)
    follower.params["FOLL_OFS_TYPE"] = 1
    slot = follow_slot_ned(follower, leader, np.array([10.0, 20.0, -15.0]))
    assert slot is not None
    # 8 m behind an east-facing leader is west, 8 m right of it is south.
    np.testing.assert_allclose(slot, [2.0, 12.0, -15.0], atol=1e-9)


def test_follow_slot_unknown_until_params_name_this_leader() -> None:
    leader = VehicleState(sysid=1)
    follower = VehicleState(sysid=2, mode=apm.COPTER_MODE_FOLLOW)
    assert follow_slot_ned(follower, leader, np.zeros(3)) is None
    follower.params = follow_params(3, np.array([-8.0, 8.0, 0.0]), 100.0)
    assert follow_slot_ned(follower, leader, np.zeros(3)) is None
