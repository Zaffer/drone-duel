"""MavlinkLink's reading of vehicle messages: COLLISION reports from ArduPilot's avoidance,
the battery from SYS_STATUS."""

import logging
from types import SimpleNamespace

import pytest
from pymavlink.dialects.v20 import ardupilotmega as apm

from swarm.link import MavlinkLink
from swarm.state import VehicleState


def _collision(level: int, other: int, action: int) -> SimpleNamespace:
    return SimpleNamespace(
        threat_level=level,
        id=other,
        action=action,
        horizontal_minimum_delta=3.2,
        altitude_minimum_delta=-1.5,
    )


def test_collision_threats_are_kept_and_logged_once_per_change(
    caplog: pytest.LogCaptureFixture,
) -> None:
    link = MavlinkLink([])
    v = VehicleState(sysid=2)
    caplog.set_level(logging.INFO, logger="swarm.link")
    escape = apm.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR
    high = _collision(apm.MAV_COLLISION_THREAT_LEVEL_HIGH, 1, escape)
    link._collision(v, high)  # type: ignore[arg-type]
    link._collision(v, high)  # type: ignore[arg-type]
    clear = _collision(apm.MAV_COLLISION_THREAT_LEVEL_NONE, 1, apm.MAV_COLLISION_ACTION_NONE)
    link._collision(v, clear)  # type: ignore[arg-type]
    assert [r.getMessage() for r in caplog.records] == [
        "[2] too close 1: escaping (closest 3.2 m across, 1.5 m up or down)",
        "[2] clear 1: no action (closest 3.2 m across, 1.5 m up or down)",
    ]
    assert (v.threat_level, v.threat_from) == (apm.MAV_COLLISION_THREAT_LEVEL_NONE, 1)


def _sys_status(sysid: int, millivolts: int, remaining: int) -> apm.MAVLink_message:
    """A SYS_STATUS from `sysid`, packed and decoded as it arrives off the wire."""
    msg = apm.MAVLink_sys_status_message(0, 0, 0, 0, millivolts, -1, remaining, 0, 0, 0, 0, 0, 0)
    buf = msg.pack(apm.MAVLink(None, srcSystem=sysid, srcComponent=1))
    return apm.MAVLink(None).decode(bytearray(buf))


def test_battery_voltage_and_remaining_from_sys_status() -> None:
    link = MavlinkLink([])
    v = VehicleState(sysid=2)
    link.vehicles[2] = v
    link._handle(None, _sys_status(2, 24_850, 63), 0.0)  # type: ignore[arg-type]
    assert (v.battery_v, v.battery_pct) == (pytest.approx(24.85), 63)
    link._handle(None, _sys_status(2, 65535, -1), 0.0)  # type: ignore[arg-type]
    assert (v.battery_v, v.battery_pct) == (None, None)
