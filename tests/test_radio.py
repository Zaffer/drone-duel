"""Radio axis to RC pulse mapping and the SITL RC packet."""

import struct

from pymavlink.dialects.v20 import ardupilotmega as apm

from swarm.radio import (
    IDLE_PWM,
    LOST_THROTTLE_PWM,
    PWM_MIN,
    RC_FS_TIMEOUT_S,
    Axis,
    RcLink,
    rc_packet,
    sitl_rc_port,
    switch_mode,
    to_pwm,
    unit,
)


def test_axis_ends_and_centre_map_to_1000_1500_2000() -> None:
    assert (to_pwm(0, 0, 2047), to_pwm(2047, 0, 2047)) == (1000, 2000)
    assert abs(to_pwm(1023, 0, 2047) - 1500) <= 1


def test_reversed_axis_and_clamping() -> None:
    assert to_pwm(2047, 0, 2047, reversed_=True) == 1000
    assert to_pwm(-50, 0, 2047) == 1000 and to_pwm(5000, 0, 2047) == 2000


def test_packet_is_little_endian_uint16_and_ports_follow_sitl_instances() -> None:
    assert rc_packet([1000, 2000]) == struct.pack("<2H", 1000, 2000)
    assert [sitl_rc_port(s) for s in (1, 2, 5)] == [5501, 5511, 5541]


def test_mode_switch_low_loiter_middle_stabilize_high_acro() -> None:
    assert switch_mode(1000) == apm.COPTER_MODE_LOITER
    assert switch_mode(1500) == apm.COPTER_MODE_STABILIZE
    assert switch_mode(2000) == apm.COPTER_MODE_ACRO


class RecordingLink(RcLink):
    def __init__(self, fleet: list[int], sysid: int | None) -> None:
        self.now = 0.0
        super().__init__(fleet, sysid, clock=lambda: self.now)
        self.sent: dict[int, list[int]] = {}

    def _send_to(self, sysid: int, pwm: list[int]) -> None:
        self.sent[sysid] = pwm


def _throttles(link: RecordingLink, sticks: list[int]) -> dict[int, int]:
    link.sent.clear()
    link.send(sticks)
    return {s: pwm[2] for s, pwm in link.sent.items()}


STICKS = [1500, 1500, 1600, 1500, 1000, 1000, 1000, 1000]


def test_every_other_copter_gets_its_own_idle_controller() -> None:
    link = RecordingLink([1, 2, 3], 1)
    assert _throttles(link, STICKS) == {1: 1600, 2: PWM_MIN, 3: PWM_MIN}
    link.retarget(2)
    assert _throttles(link, STICKS) == {1: PWM_MIN, 2: 1600, 3: PWM_MIN}
    link.retarget(None)
    assert link.sent[1] == list(IDLE_PWM) and set(_throttles(link, STICKS).values()) == {PWM_MIN}


def test_controller_switched_off_holds_then_fails_safe_after_the_rc_timeout() -> None:
    link = RecordingLink([1, 2], 1)
    link.power(2, on=False)
    assert _throttles(link, STICKS) == {1: 1600}  # SITL holds copter 2's last input
    link.now = RC_FS_TIMEOUT_S
    assert _throttles(link, STICKS) == {1: 1600, 2: LOST_THROTTLE_PWM}
    link.power(2, on=True)
    assert _throttles(link, STICKS) == {1: 1600, 2: PWM_MIN}
    assert LOST_THROTTLE_PWM < 975  # below ArduPilot's FS_THR_VALUE default


def test_the_pilots_own_controller_can_fail_too() -> None:
    link = RecordingLink([1, 2], 1)
    link.power(1, on=False)
    link.now = RC_FS_TIMEOUT_S
    assert _throttles(link, STICKS) == {1: LOST_THROTTLE_PWM, 2: PWM_MIN}


def test_unit_puts_an_axis_on_minus_one_to_one() -> None:
    assert [unit(Axis(0, 2047, v)) for v in (0, 2047, -5, 9999)] == [-1.0, 1.0, -1.0, 1.0]
    assert abs(unit(Axis(0, 2047, 1023))) < 0.001
    assert unit(Axis(5, 5, 5)) == 0.0
