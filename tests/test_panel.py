"""Switch panel decoding and switch rules, against the launcher test's fake link."""

from pathlib import Path

from pymavlink.dialects.v20 import ardupilotmega as apm
from test_launch import Clock, FakeLink, _copter, _fly_leader

from swarm.launch import Launcher, Phase, Stage
from swarm.panel import (
    ALT,
    AVIONICS,
    BAT,
    BEACON,
    COWL,
    FUEL_PUMP,
    GEAR_DOWN,
    GEAR_UP,
    MAG_BOTH,
    MAG_L,
    MAG_OFF,
    MAG_R,
    MAG_START,
    NAV,
    PANEL,
    STROBE,
    Led,
    PanelControl,
    PanelState,
    find_panel,
    led_report,
)


def _state(*contacts: int) -> PanelState:
    return PanelState(sum(1 << c for c in contacts))


def _setup() -> tuple[FakeLink, Launcher, Clock, PanelControl]:
    leader = _copter(1, 0.0, apm.COPTER_MODE_LOITER)
    link = FakeLink([leader, _copter(2, 10.0), _copter(3, 15.0)])
    clock = Clock()
    launcher = Launcher(link, 1, [2, 3], clock=clock)  # type: ignore[arg-type]
    launcher.step()
    return link, launcher, clock, PanelControl(launcher, clock)


def _go_live(control: PanelControl, *contacts: int) -> None:
    control.update(_state(GEAR_DOWN, MAG_OFF))
    control.update(_state(GEAR_UP, *contacts))


def _in_the_air(link: FakeLink, *sysids: int) -> None:
    for sysid in sysids:
        v = link.vehicles[sysid]
        v.armed, v.landed_state, v.rel_alt_m = True, apm.MAV_LANDED_STATE_IN_AIR, 12.0


def test_reports_decode_as_seen_on_the_panel() -> None:
    # Captured: gear DOWN, rotary OFF; then BAT and ALT on, rotary START, gear UP.
    idle = PanelState.from_report(bytes.fromhex("002008"))
    assert idle.gear_down is True and idle.phase is Phase.LAND and not idle.on(BAT)
    live = PanelState.from_report(bytes.fromhex("030006"))
    assert live.gear_down is False and live.phase is Phase.HOLD
    assert live.on(BAT) and live.on(ALT) and not live.on(AVIONICS)
    assert live.describe() == "gear UP, rotary START, on: BAT, ALT"


def test_led_byte_is_green_bits_0_2_red_bits_3_5() -> None:
    assert led_report(Led.GREEN, Led.OFF, Led.OFF) == 0b000001
    assert led_report(Led.OFF, Led.RED, Led.OFF) == 0b010000
    assert led_report(Led.OFF, Led.OFF, Led.AMBER) == 0b100100


def test_finds_the_panel_by_usb_id(tmp_path: Path) -> None:
    for name, hid_id in (("hidraw0", "00000483:00005750"), ("hidraw1", "000006A3:00000D67")):
        device = tmp_path / name / "device"
        device.mkdir(parents=True)
        (device / "uevent").write_text(f"DRIVER=hid-generic\nHID_ID=0003:{hid_id}\n")
    assert find_panel(tmp_path) == Path("/dev/hidraw1")


def test_gear_up_at_start_stays_locked_until_cycled() -> None:
    link, launcher, clock, control = _setup()
    control.update(_state(GEAR_UP, AVIONICS, MAG_START))
    assert not control.live and launcher.followers["a"].stage is Stage.IDLE
    control.update(_state(GEAR_DOWN, AVIONICS, MAG_START))
    assert not link.calls  # found DOWN: nothing to land
    assert control.leds() == led_report(Led.RED, Led.OFF, Led.OFF)
    control.update(_state(GEAR_UP, AVIONICS, MAG_START))
    clock.now += 1.0
    control.step()
    assert control.live and launcher.phase is Phase.HOLD
    assert launcher.followers["a"].stage is Stage.ARMING  # ON as it went live
    assert control.leds() & 0b001001 == 0b001001  # nose amber: live, nothing on the radio


def test_bat_does_nothing() -> None:
    _, launcher, _, control = _setup()
    _go_live(control, ALT)
    assert launcher.lead.stage is Stage.ARMING
    control.update(_state(GEAR_UP, ALT, BAT))
    control.update(_state(GEAR_UP, ALT))
    assert launcher.lead.stage is Stage.ARMING and launcher.live


def test_switch_off_lands_its_drone_only() -> None:
    link, launcher, _, control = _setup()
    _go_live(control, MAG_OFF, AVIONICS, FUEL_PUMP)
    launcher.step()  # GUIDED
    launcher.step()  # arm
    control.update(_state(GEAR_UP, MAG_OFF, FUEL_PUMP))
    assert launcher.followers["a"].stage is Stage.LANDING
    assert link.vehicles[2].mode == apm.COPTER_MODE_LAND
    assert launcher.followers["b"].stage is Stage.ARMING


def test_gear_down_lands_everything_and_locks() -> None:
    link, launcher, _, control = _setup()
    _fly_leader(link, 12.0)
    _go_live(control, MAG_START, AVIONICS)
    launcher.step()
    launcher.step()
    control.update(_state(GEAR_DOWN, MAG_START, AVIONICS))
    assert link.vehicles[1].mode == apm.COPTER_MODE_LAND
    assert link.vehicles[2].mode == apm.COPTER_MODE_LAND
    assert not control.live and not launcher.live
    assert control.leds() & 0b001000  # nose red


def test_switch_back_on_calls_off_its_drones_landing() -> None:
    link, launcher, clock, control = _setup()
    _fly_leader(link, 12.0)
    _go_live(control, MAG_BOTH, AVIONICS)
    clock.now += 1.0
    control.step()  # BOTH: follow
    _in_the_air(link, 2)
    launcher.followers["a"].stage = Stage.FOLLOWING
    control.update(_state(GEAR_UP, MAG_BOTH))
    assert link.vehicles[2].mode == apm.COPTER_MODE_LAND
    control.update(_state(GEAR_UP, MAG_BOTH, AVIONICS))
    assert link.vehicles[2].mode == apm.COPTER_MODE_FOLLOW


def test_selector_switch_hands_its_drone_the_radio_in_loiter() -> None:
    link, launcher, _, control = _setup()
    _go_live(control, NAV)
    control.step()
    assert launcher.radio_sysid == 2
    assert link.vehicles[2].mode == apm.COPTER_MODE_LOITER
    assert control.leds() & 0b001001 == 0b001001  # nose amber: mode switch not yet at LOITER
    launcher.mode_switch(apm.COPTER_MODE_LOITER)
    assert control.leds() & 0b001001 == 0b000001  # nose green
    control.update(_state(GEAR_UP, NAV, PANEL))  # PANEL does nothing
    control.step()
    assert launcher.radio_sysid == 2
    control.update(_state(GEAR_UP))
    control.step()
    assert launcher.radio_sysid is None  # the radio flies nothing


def test_two_selector_switches_select_nothing_until_all_are_off() -> None:
    _, launcher, _, control = _setup()
    _go_live(control, NAV)
    control.step()
    control.update(_state(GEAR_UP, NAV, STROBE))
    control.step()
    assert launcher.radio_sysid is None
    control.update(_state(GEAR_UP, STROBE))  # one left on is not enough
    control.step()
    assert launcher.radio_sysid is None
    control.update(_state(GEAR_UP))
    control.update(_state(GEAR_UP, STROBE))
    control.step()
    assert launcher.radio_sysid == 3


def test_leader_needs_its_selector_and_holds_without_it() -> None:
    link, launcher, _, control = _setup()
    _in_the_air(link, 1)
    _go_live(control)
    control.step()
    assert launcher.radio_sysid is None
    assert link.vehicles[1].mode == apm.COPTER_MODE_GUIDED  # holds where it is
    control.update(_state(GEAR_UP, BEACON))
    control.step()
    assert launcher.radio_sysid == 1
    assert link.vehicles[1].mode == apm.COPTER_MODE_LOITER


def test_auto_select_starts_on_the_leader_and_moves_on_when_it_disarms() -> None:
    link, launcher, _, control = _setup()
    _in_the_air(link, 1, 3)
    _go_live(control, COWL)
    control.step()
    assert launcher.manual == 1
    link.vehicles[1].armed = False
    control.step()
    assert launcher.manual == 3  # copter 2 is on the ground: skipped
    link.vehicles[3].armed = False
    control.step()
    assert launcher.manual == 3  # nothing else in the air: stays


def test_gear_down_takes_the_radio_off() -> None:
    _, launcher, _, control = _setup()
    _go_live(control, NAV)
    control.step()
    control.update(_state(GEAR_DOWN, NAV))
    control.step()
    assert launcher.manual is None


def test_rotary_phase_starts_once_it_has_rested() -> None:
    link, launcher, clock, control = _setup()
    _fly_leader(link, 12.0)
    _go_live(control, MAG_BOTH)
    clock.now += 1.0
    control.step()
    assert launcher.phase is Phase.FOLLOW
    for position in (MAG_L, MAG_R, MAG_OFF):  # turned past L and R to OFF
        control.update(_state(GEAR_UP, position))
        clock.now += 0.2
        control.step()
    assert launcher.phase is Phase.FOLLOW
    assert link.vehicles[1].mode == apm.COPTER_MODE_GUIDED  # holding: not landed, not home
    assert ("mode", 1, apm.COPTER_MODE_RTL) not in link.calls
    clock.now += 0.6
    control.step()
    assert launcher.phase is Phase.LAND
    assert link.vehicles[1].mode == apm.COPTER_MODE_LAND


def test_rotary_flies_start_both_l_then_off() -> None:
    _, launcher, clock, control = _setup()
    _go_live(control, MAG_START)
    phases = []
    for position in (MAG_START, MAG_BOTH, MAG_L, MAG_R, MAG_OFF):
        control.update(_state(GEAR_UP, position))
        clock.now += 1.0
        control.step()
        phases.append(launcher.phase)
    assert phases == [Phase.HOLD, Phase.FOLLOW, Phase.HOLD, Phase.RTL, Phase.LAND]


def test_drones_arm_at_off_and_wait_on_the_ground_for_start() -> None:
    link, launcher, clock, control = _setup()
    _go_live(control, MAG_OFF, ALT, AVIONICS)
    clock.now += 1.0
    control.step()
    assert launcher.phase is Phase.LAND
    for _ in range(3):
        launcher.step()
    assert link.vehicles[1].armed and link.vehicles[2].armed
    assert launcher.lead.stage is Stage.ARMING
    assert launcher.followers["a"].stage is Stage.ARMING
    assert ("takeoff", 2) not in [c[:2] for c in link.calls]


def test_turning_fast_past_l_to_r_skips_the_hold() -> None:
    link, launcher, clock, control = _setup()
    _fly_leader(link, 12.0)
    _go_live(control, MAG_BOTH)
    clock.now += 1.0
    control.step()
    control.update(_state(GEAR_UP, MAG_L))
    clock.now += 0.3
    control.step()
    control.update(_state(GEAR_UP, MAG_R))
    clock.now += 0.3
    control.step()
    assert launcher.phase is Phase.FOLLOW  # R has not rested yet; L never started
    clock.now += 0.5
    control.step()
    assert launcher.phase is Phase.RTL
    assert ("mode", 1, apm.COPTER_MODE_LAND) not in link.calls
    assert link.vehicles[1].mode == apm.COPTER_MODE_RTL
