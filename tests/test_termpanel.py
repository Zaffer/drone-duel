"""The switch panel in the terminal: its controls, keys, the hardware beside it, and the
pretend drones of `swarm panel-demo` flown with it."""

from pymavlink.dialects.v20 import ardupilotmega as apm

from swarm.console import Console
from swarm.launch import Launcher, Phase, Stage
from swarm.panel import (
    ALT,
    AVIONICS,
    BEACON,
    COWL,
    FUEL_PUMP,
    GEAR_DOWN,
    GEAR_UP,
    LANDING,
    MAG_BOTH,
    MAG_L,
    MAG_OFF,
    MAG_START,
    NAV,
    PITOT_HEAT,
    Led,
    PanelState,
    led_report,
    leds_from_report,
)
from swarm.paneldemo import PILOT_ALT_M, DemoLink, DemoRadio
from swarm.termpanel import CONTROLS, START_BITS, Control, TerminalPanel, draw, nearest, roles

UP, DOWN, RIGHT, LEFT, ENTER = "\x1b[A", "\x1b[B", "\x1b[C", "\x1b[D", "\r"


def _at(contact: int) -> Control:
    return next(c for c in CONTROLS if c.contact == contact)


def _label(label: str) -> Control:
    return next(c for c in CONTROLS if c.label == label)


class Hardware:
    def __init__(self, bits: int | None = None) -> None:
        self.bits = bits
        self.attached = True
        self.leds = 0
        self._sent: int | None = None

    def poll(self) -> PanelState | None:
        if self.bits is None or self.bits == self._sent:
            return None
        self._sent = self.bits
        return PanelState(self.bits)

    def set_leds(self, value: int) -> None:
        self.leds = value


def test_labels_are_two_letters_a_word() -> None:
    labels = {c.label for c in CONTROLS}
    assert {"BO", "AV", "GE UP", "GE DO", "FU", "DE", "PI", "OF", "R", "L"} <= labels
    assert len(CONTROLS) == 5 + 6 + 7 + 2  # rotary, top row, bottom row, gear


def test_arrow_keys_go_to_the_nearest_control_that_way() -> None:
    assert nearest(CONTROLS, _label("BA"), "down") == _label("PA")  # COWL sticks out left
    assert nearest(CONTROLS, _at(AVIONICS), "down") == _at(NAV)
    assert nearest(CONTROLS, _at(PITOT_HEAT), "down") == _label("LA")
    assert nearest(CONTROLS, _at(PITOT_HEAT), "right") == _at(GEAR_UP)
    assert nearest(CONTROLS, _at(GEAR_UP), "down") == _at(GEAR_DOWN)
    assert nearest(CONTROLS, _label("BA"), "left") == _label("L")
    assert nearest(CONTROLS, _label("L"), "left") == _label("R")
    assert nearest(CONTROLS, _label("L"), "right") == _label("BA")
    assert nearest(CONTROLS, _label("LA"), "right") == _at(GEAR_DOWN)
    assert nearest(CONTROLS, _at(GEAR_UP), "left") == _at(PITOT_HEAT)
    assert nearest(CONTROLS, _label("R"), "down") == _label("OF")
    assert nearest(CONTROLS, _at(GEAR_DOWN), "right") == _at(GEAR_DOWN)  # the edge


def test_arrow_keys_move_and_enter_flips() -> None:
    term = TerminalPanel()
    assert term.focus == _at(GEAR_DOWN)
    term.feed(UP)
    assert term.focus == _at(GEAR_UP)
    term.feed("\x1b[")  # an arrow key split across two reads
    term.feed("B ")
    assert term.focus == _at(GEAR_DOWN)
    state = term.poll()
    assert state is not None and state.gear_down is True
    assert term.poll() is None  # a change is reported once


def test_shortcut_keys_work_their_control() -> None:
    term = TerminalPanel()
    term.feed("dUs126A")
    state = term.poll()
    assert state is not None and state.gear_down is False and state.on(MAG_START)
    assert all(state.on(c) for c in (ALT, AVIONICS, BEACON, COWL))
    assert term.focus == _at(COWL)
    term.feed("2b0cxl")  # c and x do nothing
    state = term.poll()
    assert state is not None and not state.on(AVIONICS) and state.on(LANDING)
    assert state.phase is Phase.HOLD and state.on(MAG_L)
    term.feed("O")
    state = term.poll()
    assert state is not None and state.on(MAG_OFF)


def test_a_scripted_panel_takes_no_keys() -> None:
    term = TerminalPanel(interactive=False)
    term.poll()
    term.feed("d")
    assert term.poll() is None


def test_switches_flip_and_the_rotary_and_gear_take_one_position() -> None:
    term = TerminalPanel()
    first = term.poll()
    assert first is not None and first.bits == START_BITS and first.gear_down is None
    term.press(_at(AVIONICS))
    term.press(_at(MAG_BOTH))
    term.press(_at(GEAR_UP))
    state = term.poll()
    assert state is not None and state.on(AVIONICS) and state.on(MAG_BOTH)
    assert not state.on(MAG_OFF) and state.gear_down is False
    term.press(_at(AVIONICS))
    term.press(_at(GEAR_DOWN))
    state = term.poll()
    assert state is not None and not state.on(AVIONICS) and state.gear_down is True


def test_the_hardware_is_taken_as_found_then_each_side_moves_what_it_moves() -> None:
    found = 1 << GEAR_DOWN | 1 << MAG_START | 1 << ALT
    hardware = Hardware(found)
    term = TerminalPanel(hardware)
    state = term.poll()
    assert state is not None and state.bits == found
    term.press(_at(AVIONICS))  # the terminal flips one the hardware has off
    hardware.bits = found ^ (1 << FUEL_PUMP)  # the hardware flips another
    state = term.poll()
    assert state is not None and state.on(AVIONICS) and state.on(FUEL_PUMP) and state.on(ALT)
    hardware.bits = found ^ (1 << FUEL_PUMP) ^ (1 << MAG_START) | 1 << MAG_BOTH
    state = term.poll()
    assert state is not None and state.on(MAG_BOTH) and not state.on(MAG_START)
    assert "* differs" in draw(term.view()).lines(plain=True)[0]  # AVIONICS on the terminal only
    hardware.attached = False
    assert "no hardware" in draw(term.view()).lines(plain=True)[0]


def test_hardware_found_late_does_not_undo_the_terminal() -> None:
    hardware = Hardware()
    term = TerminalPanel(hardware)
    term.press(_at(GEAR_DOWN))
    term.press(_at(GEAR_UP))
    term.poll()
    hardware.bits = 1 << GEAR_DOWN | 1 << MAG_OFF  # plugged in with the lever down
    assert term.poll() is None
    assert term.bits >> GEAR_UP & 1


def test_leds_show_on_the_panel_and_go_to_the_hardware() -> None:
    hardware = Hardware()
    term = TerminalPanel(hardware)
    value = led_report(Led.AMBER, Led.GREEN, Led.RED)
    assert leds_from_report(value) == (Led.AMBER, Led.GREEN, Led.RED)
    term.set_leds(value)
    assert hardware.leds == value
    assert "LIVE · radio idle" in draw(term.view()).lines(plain=True)[0]


def test_every_line_is_the_panel_width() -> None:
    term = TerminalPanel(roles=roles(1, {"a": 2, "b": 3}))
    lines = draw(term.view()).lines(plain=True)
    assert len(lines) == 11 and {len(line) for line in lines} == {80}
    assert "#2" in lines[4] and "auto" in lines[9]


class Clock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


def test_the_demo_flies_the_mission_from_the_terminal() -> None:
    clock = Clock()
    link = DemoLink(3, (45.3198, -75.7586), clock)
    launcher = Launcher(link, 1, [2, 3], clock=clock, own_mode_switch=True)  # type: ignore[arg-type]
    term = TerminalPanel()
    console = Console(launcher, term, DemoRadio(link), clock)

    def run(seconds: float) -> None:
        end = clock.now + seconds
        while clock.now < end:
            clock.now += 0.1
            console.tick()

    def flip(*contacts: int) -> None:
        for contact in contacts:
            term.press(_at(contact))
            run(1.0)

    leader, follower = link.vehicles[1], link.vehicles[2]
    flip(GEAR_DOWN, GEAR_UP, MAG_START, ALT)
    assert leader.armed and not leader.in_air
    assert leds_from_report(term.leds or 0)[:2] == (Led.AMBER, Led.AMBER)
    flip(BEACON)  # the radio takes the leader up
    run(10.0)
    assert leader.in_air and leader.rel_alt_m == PILOT_ALT_M
    assert leds_from_report(term.leds or 0)[:2] == (Led.GREEN, Led.GREEN)
    flip(AVIONICS)
    run(15.0)
    assert launcher.followers["a"].stage is Stage.HOLDING and follower.in_air
    flip(MAG_BOTH)
    assert follower.mode == apm.COPTER_MODE_FOLLOW
    flip(BEACON, COWL)  # radio to auto select: still the leader
    assert launcher.manual == 1
    flip(MAG_OFF)
    run(15.0)
    assert not follower.armed and not follower.in_air
    assert leader.in_air  # the radio's copter is the pilot's to land
    flip(COWL)  # radio off: the leader goes back to the phase, and lands
    run(15.0)
    assert not leader.armed and not leader.in_air
