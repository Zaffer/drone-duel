"""The switch panel drawn at the foot of the terminal, worked with the arrow keys and Enter.

Every control of the Logitech panel (`swarm.panel`) is there where it sits on the panel: the
rotary on the left, the two rows of switches, the gear lever and its three LEDs on the right.
Each is labelled with the first two letters of its name on the panel: BOTH is BO, AVIONICS AV,
FUEL PUMP FU; the gear lever's two positions take two letters a word, GE UP and GE DO. Under
each switch is the drone it acts on.
The arrow keys move the cursor; Enter or space flips the switch under it, turns the rotary to
the position under it, or moves the gear lever there. Shortcut keys go straight to a control
(SHORTCUTS); the panel takes every key, so the launcher's own keys (a-d, l) are off with it.

It is one more switch panel to `PanelControl`, under the same rules. With the hardware panel
attached the two work together: a change on either moves the switch, and a * marks a control
whose hardware position differs. The hardware's positions are taken as they are when it is
first found, unless something was already moved in the terminal. The terminal starts with the
gear lever between its positions, so it goes live only after GE DO then GE UP.

The logs scroll above the panel as usual; the panel keeps the bottom rows of the terminal
(`PinnedScreen`, a scroll region).
"""

import logging
import os
import re
import signal
import sys
import time
from dataclasses import dataclass
from enum import Enum
from types import FrameType, TracebackType
from typing import TYPE_CHECKING, TextIO

from swarm.panel import (
    ALT,
    AVIONICS,
    BAT,
    BEACON,
    COWL,
    DE_ICE,
    FOLLOWER_SWITCHES,
    FUEL_PUMP,
    GEAR_DOWN,
    GEAR_UP,
    LANDING,
    LEADER,
    MAG_BOTH,
    MAG_L,
    MAG_OFF,
    MAG_R,
    MAG_START,
    NAV,
    PANEL,
    PITOT_HEAT,
    ROTARY,
    STROBE,
    SWITCH_NAMES,
    TAKEOVER_SWITCHES,
    TAXI,
    Led,
    PanelState,
    leds_from_report,
)

if TYPE_CHECKING:
    from swarm.console import PanelDevice

log = logging.getLogger(__name__)

WIDTH, HEIGHT = 80, 11
"""The panel's size on screen, border included."""
MIN_LOG_ROWS = 3
"""Rows the logs keep above the panel; a shorter terminal shows no panel."""
REDRAW_S = 2.0
"""Draw the panel again this often even when nothing changed, in case anything wrote over it."""


class Kind(Enum):
    SWITCH = "switch"
    ROTARY = "rotary"
    GEAR = "gear"


def short_name(name: str) -> str:
    """A gear lever position's name in two letters a word: GEAR DOWN is GE DO."""
    return " ".join(word[:2] for word in name.split())


@dataclass(frozen=True)
class Control:
    contact: int
    kind: Kind
    label: str
    row: int
    col: int
    """Where its label starts."""

    @property
    def centre(self) -> float:
        return self.col + (len(self.label) - 1) / 2


def _switch_row(contacts: tuple[int, ...], centres: tuple[int, ...], row: int) -> list[Control]:
    out = []
    for contact, centre in zip(contacts, centres, strict=True):
        label = SWITCH_NAMES[contact][:2]
        out.append(Control(contact, Kind.SWITCH, label, row, centre - (len(label) - 1) // 2))
    return out


ROTARY_LABELS = {contact: name[:2] for contact, (name, _) in ROTARY.items()}
ROTARY_ROLES = {
    MAG_OFF: "land",
    MAG_R: "home",
    MAG_L: "hold",
    MAG_BOTH: "follow",
    MAG_START: "launch",
}
"""What each rotary position flies (`swarm.panel` ROTARY)."""
KNOB = (4, 10)
ROTARY_AT = {MAG_OFF: (4, 4), MAG_R: (2, 7), MAG_L: (2, 13), MAG_BOTH: (4, 15), MAG_START: (6, 13)}
POINTER = {
    MAG_OFF: ((4, 8), "──"),
    MAG_R: ((3, 9), "╲"),
    MAG_L: ((3, 11), "╱"),
    MAG_BOTH: ((4, 11), "──"),
    MAG_START: ((5, 11), "╲"),
}
"""The knob's pointer towards each position: where it starts, and how it is drawn."""
SWITCH_CENTRES = (23, 29, 35, 41, 47, 53, 59)
"""The switch columns. COWL alone has the first; the top row starts over PANEL."""
TOP_ROW = _switch_row((BAT, ALT, AVIONICS, FUEL_PUMP, DE_ICE, PITOT_HEAT), SWITCH_CENTRES[1:], 1)
BOTTOM_ROW = _switch_row((COWL, PANEL, BEACON, NAV, STROBE, TAXI, LANDING), SWITCH_CENTRES, 6)
GEAR_COL = 72
"""The gear lever's column."""
CONTROLS: tuple[Control, ...] = (
    *(
        Control(c, Kind.ROTARY, ROTARY_LABELS[c], *ROTARY_AT[c])
        for c in (MAG_OFF, MAG_R, MAG_L, MAG_BOTH, MAG_START)
    ),
    *TOP_ROW,
    *BOTTOM_ROW,
    Control(GEAR_UP, Kind.GEAR, short_name("GEAR UP"), 1, GEAR_COL - 2),
    Control(GEAR_DOWN, Kind.GEAR, short_name("GEAR DOWN"), 6, GEAR_COL - 2),
)
ROTARY_MASK = sum(1 << c for c in ROTARY)
GEAR_MASK = 1 << GEAR_UP | 1 << GEAR_DOWN
GROUPS = (ROTARY_MASK, GEAR_MASK)
"""Contacts that move together: one position of the rotary, one of the lever."""
START_BITS = 1 << MAG_OFF
"""Rotary OFF, every switch off, the gear lever between its positions."""

KEYS = {
    "\x1b[A": "up",
    "\x1b[B": "down",
    "\x1b[C": "right",
    "\x1b[D": "left",
    "\x1bOA": "up",
    "\x1bOB": "down",
    "\x1bOC": "right",
    "\x1bOD": "left",
    "\r": "enter",
    "\n": "enter",
    " ": "enter",
}
ESCAPE = re.compile(r"\x1b(\[[0-9;]*[A-Za-z~]?|O[A-Za-z]?)?")
"""A whole or partial escape sequence (an arrow key is three characters, maybe split)."""
STEP_WEIGHT = {"left": 5.0, "right": 5.0, "up": 0.5, "down": 0.5}
"""Cost of a row against a column, moving that way: a move stays in its row or column."""

LED_STYLE = {
    Led.OFF: ("○", "2"),
    Led.GREEN: ("●", "1;32"),
    Led.RED: ("●", "1;31"),
    Led.AMBER: ("●", "1;33"),
}
NOSE_STATUS = {
    Led.RED: ("LOCKED", "1;31"),
    Led.AMBER: ("LIVE · radio idle", "1;33"),
    Led.GREEN: ("LIVE · radio flying", "1;32"),
    Led.OFF: ("dark", "2"),
}
"""What the nose LED says (`PanelControl.leds`)."""
SHORTCUTS = {
    **dict(zip("12345", (ALT, AVIONICS, FUEL_PUMP, DE_ICE, PITOT_HEAT), strict=True)),
    **dict(zip("67890", (BEACON, NAV, STROBE, TAXI, LANDING), strict=True)),
    "a": COWL,
    **dict(zip("orlbs", (MAG_OFF, MAG_R, MAG_L, MAG_BOTH, MAG_START), strict=True)),
    "u": GEAR_UP,
    "d": GEAR_DOWN,
}
"""A key straight to a control, either case: 1-5 the arming switches, 6-0 the radio
selectors, A auto select, O R L B S the rotary, U and D the gear lever."""
HELP = "←↑↓→ ⏎ · 1-5 arm · 6-0 radio · A auto · ORLBS rotary · U/D gear · ^C quit"
SCRIPTED_HELP = "the script flips the switches · ^C stops"


def roles(leader: int, followers: dict[str, int]) -> dict[int, str]:
    """What each switch acts on, under it: the drone it arms, or hands the radio."""
    out = {COWL: "auto"}
    for contact, key in (*FOLLOWER_SWITCHES.items(), *TAKEOVER_SWITCHES.items()):
        sysid = leader if key == LEADER else followers.get(key)
        if sysid is not None:
            out[contact] = f"#{sysid}"
    out[ALT] = f"#{leader}"
    return out


def nearest(controls: tuple[Control, ...], current: Control, direction: str) -> Control:
    """The control an arrow key moves to: the closest one within 45 degrees of that way, else
    the closest one that way at all; `current` at the edge. A row is about two columns tall."""

    def cost(c: Control) -> tuple[bool, float] | None:
        d_row, d_col = c.row - current.row, c.centre - current.centre
        sideways = direction in ("left", "right")
        across, along = (d_col, d_row) if sideways else (d_row, d_col)
        ahead = across if direction in ("right", "down") else -across
        if ahead <= 0:
            return None
        in_cone = 2 * abs(along) <= ahead if sideways else abs(along) <= 2 * ahead
        return not in_cone, ahead + STEP_WEIGHT[direction] * abs(along)

    options = [(k, c) for c in controls if (k := cost(c)) is not None]
    return min(options, key=lambda option: option[0])[1] if options else current


class Canvas:
    """Characters with an SGR style each, written out as lines of ANSI text."""

    def __init__(self, width: int, height: int) -> None:
        self.width = width
        self.cells = [[(" ", "")] * width for _ in range(height)]

    def put(self, row: int, col: int, text: str, style: str = "") -> None:
        for i, ch in enumerate(text):
            if 0 <= col + i < self.width:
                self.cells[row][col + i] = (ch, style)

    def lines(self, width: int | None = None, plain: bool = False) -> list[str]:
        out = []
        for row in self.cells:
            line, style = "", ""
            for ch, s in row[: width or self.width]:
                if s != style and not plain:
                    line += "\x1b[0m" + (f"\x1b[{s}m" if s else "")
                    style = s
                line += ch
            out.append(line + ("\x1b[0m" if style else ""))
        return out


@dataclass(frozen=True)
class View:
    """What the panel shows."""

    bits: int
    hardware: int | None
    """The hardware's contacts while it is attached, else None."""
    leds: int | None
    focus: Control | None
    roles: dict[int, str]
    help: str
    source: str
    """What `hardware` is: the hardware panel, or the script."""


def _on(bits: int, contact: int) -> bool:
    return bool(bits >> contact & 1)


def draw(view: View) -> Canvas:
    canvas = Canvas(WIDTH, HEIGHT)
    hardware = view.hardware

    def differs(contact: int) -> bool:
        return hardware is not None and _on(hardware, contact) != _on(view.bits, contact)

    for c in CONTROLS:
        focused = c == view.focus
        here = _on(view.bits, c.contact)
        if c.kind is Kind.SWITCH:
            centre = int(c.centre)
            canvas.put(c.row, c.col, c.label, "7" if focused else "1" if here else "")
            canvas.put(c.row + 1, centre, "▲" if here else "▼", "1;32" if here else "2")
            canvas.put(c.row + 2, centre - 1, "ON" if here else "off", "1;32" if here else "2")
            if differs(c.contact):
                canvas.put(c.row + 2, centre + 2, "*", "1;33")
            role = view.roles.get(c.contact, "")
            canvas.put(c.row + 3, centre - (len(role) - 1) // 2, role, "2;36")
        else:  # a rotary or gear position: lit where it is, * where the hardware is instead
            style = "1;32" if here else "2"
            canvas.put(c.row, c.col, c.label, f"7;{style}" if focused else style)
            if hardware is not None and differs(c.contact) and _on(hardware, c.contact):
                canvas.put(c.row, c.col + len(c.label), "*", "1;33")
    # The rotary's knob and its pointer, and what the position flies.
    canvas.put(*KNOB, "◉", "1")
    position = next((c for c in ROTARY if _on(view.bits, c)), None)
    if position is not None:
        (row, col), text = POINTER[position]
        canvas.put(row, col, text, "1;32")
        canvas.put(8, 3, f"{ROTARY_LABELS[position]} {ROTARY_ROLES[position]}", "36")
    # The gear lever between its labels: the knob at the top for UP, the bottom for DOWN.
    gear = GEAR_UP if _on(view.bits, GEAR_UP) else GEAR_DOWN if _on(view.bits, GEAR_DOWN) else None
    knob = {GEAR_UP: {2: "█"}, GEAR_DOWN: {5: "█"}, None: {3: "▄", 4: "▀"}}[gear]
    for row in range(2, 6):
        canvas.put(row, GEAR_COL, knob.get(row, "│"), "1" if row in knob else "2")
    # The LEDs left of the lever: nose on top, the leader left, the followers right.
    nose, left, right = leds_from_report(view.leds or 0)
    for (row, col), led in (
        ((2, GEAR_COL - 5), nose),
        ((4, GEAR_COL - 7), left),
        ((4, GEAR_COL - 3), right),
    ):
        canvas.put(row, col, *LED_STYLE[led])
    # The border, the panel's state in the top edge, the keys in the bottom one.
    canvas.put(0, 0, "╭" + "─" * (WIDTH - 2) + "╮", "2")
    canvas.put(HEIGHT - 1, 0, "╰" + "─" * (WIDTH - 2) + "╯", "2")
    for row in range(1, HEIGHT - 1):
        canvas.put(row, 0, "│", "2")
        canvas.put(row, WIDTH - 1, "│", "2")
    canvas.put(0, 2, " SWITCH PANEL ", "1")
    status, style = NOSE_STATUS[nose] if view.leds is not None else ("starting", "2")
    attached = f"{view.source} attached" if hardware is not None else f"no {view.source}"
    if any(differs(c.contact) for c in CONTROLS):
        attached += ", * differs"
    start = WIDTH - 2 - len(f" {status} · {attached} ")
    canvas.put(0, start, f" {status} ", style)
    canvas.put(0, start + len(status) + 2, f"· {attached} ", "2")
    canvas.put(HEIGHT - 1, 2, f" {view.help} ", "2")
    return canvas


class TerminalPanel:
    """The switch panel in the terminal: a PanelDevice, merged with `inner` (the hardware
    panel, or a scripted one) when given."""

    def __init__(
        self,
        inner: "PanelDevice | None" = None,
        roles: dict[int, str] | None = None,
        screen: "PinnedScreen | None" = None,
        interactive: bool = True,
        help: str = HELP,
        source: str = "hardware",
    ) -> None:
        self.inner = inner
        self.source = source
        """What `inner` is, as the panel names it."""
        self.roles = roles or {}
        self.screen = screen
        self.interactive = interactive
        """False: shown only, the keys left alone."""
        self.help = help
        self.bits = START_BITS
        self.leds: int | None = None
        self.focus = next(c for c in CONTROLS if c.contact == GEAR_DOWN)
        self._hardware: int | None = None
        """The hardware's contacts in its last report."""
        self._touched = False
        """Something was moved in the terminal."""
        self._sent: int | None = None
        self._pending = ""
        """The start of an escape sequence split across reads."""
        self._drawn: View | None = None
        self._drawn_s = 0.0

    def feed(self, text: str) -> None:
        """Act on the keys typed: the arrows and Enter, and the shortcuts (SHORTCUTS), which
        move the cursor to their control and work it. Any other key does nothing."""
        if not self.interactive:
            return
        text, self._pending = self._pending + text, ""
        i = 0
        while i < len(text):
            m = ESCAPE.match(text, i)
            if m is not None:
                seq = m.group(0)
                if m.end() == len(text) and seq in ("\x1b", "\x1b[", "\x1bO"):
                    self._pending = seq  # the rest comes with the next read
                    break
                i = m.end()
            else:
                seq = text[i]
                i += 1
            key = KEYS.get(seq)
            if key == "enter":
                self.press()
            elif key is not None:
                self.focus = nearest(CONTROLS, self.focus, key)
            elif seq.lower() in SHORTCUTS:
                self.focus = next(c for c in CONTROLS if c.contact == SHORTCUTS[seq.lower()])
                self.press()

    def press(self, control: Control | None = None) -> None:
        """Flip the switch, turn the rotary or move the lever: `control`, or the one focused."""
        c = control or self.focus
        old = self.bits
        if c.kind is Kind.SWITCH:
            self.bits ^= 1 << c.contact
        else:
            group = ROTARY_MASK if c.kind is Kind.ROTARY else GEAR_MASK
            self.bits = self.bits & ~group | 1 << c.contact
        if self.bits == old:
            return
        self._touched = True
        if c.kind is Kind.SWITCH:
            log.info(
                "terminal panel: %s %s", c.label, "ON" if self.bits >> c.contact & 1 else "off"
            )
        elif c.kind is Kind.ROTARY:
            log.info("terminal panel: rotary %s", c.label)
        else:
            log.info("terminal panel: gear %s", "UP" if c.contact == GEAR_UP else "DOWN")

    def poll(self) -> PanelState | None:
        """The positions if they changed since the last call, from the terminal or the
        hardware, else None. Draws the panel."""
        if self.inner is not None:
            state = self.inner.poll()
            if state is not None:
                self._from_hardware(state.bits)
        self.redraw()
        if self.bits == self._sent:
            return None
        self._sent = self.bits
        return PanelState(self.bits)

    def set_leds(self, value: int) -> None:
        self.leds = value
        if self.inner is not None:
            self.inner.set_leds(value)

    @property
    def hardware(self) -> int | None:
        """The hardware's contacts while it is attached."""
        if self.inner is None or not getattr(self.inner, "attached", True):
            return None
        return self._hardware

    def view(self) -> View:
        return View(
            self.bits,
            self.hardware,
            self.leds,
            self.focus if self.interactive else None,
            self.roles,
            self.help,
            self.source,
        )

    def redraw(self, now: float | None = None) -> None:
        if self.screen is None:
            return
        view = self.view()
        now = now if now is not None else time.monotonic()
        if view == self._drawn and not self.screen.resized and now - self._drawn_s < REDRAW_S:
            return
        self.screen.draw(draw(view))
        self._drawn, self._drawn_s = view, now

    def _from_hardware(self, bits: int) -> None:
        """Take the hardware's changes: all its positions when first found, unless the
        terminal has moved something, then only what moved on the hardware."""
        old, self._hardware = self._hardware, bits
        if old is None:
            if not self._touched:
                self.bits = bits
            return
        changed = old ^ bits
        for group in GROUPS:
            if changed & group:
                self.bits = self.bits & ~group | bits & group
        single = changed & ~(ROTARY_MASK | GEAR_MASK)
        self.bits = self.bits & ~single | bits & single


class PinnedScreen:
    """The bottom rows of the terminal kept for the panel; everything else scrolls above it."""

    def __init__(self, height: int = HEIGHT, stream: TextIO | None = None) -> None:
        self.height = height
        self._out = stream or sys.stdout
        self.resized = False
        self.active = False
        self._rows = self._cols = 0
        self._old_handler: signal.Handlers | object | None = None

    @staticmethod
    def usable() -> bool:
        """A terminal on stdin, stdout and stderr."""
        return all(os.isatty(fd) for fd in (0, 1, 2)) and os.environ.get("TERM") != "dumb"

    def __enter__(self) -> "PinnedScreen":
        self._old_handler = signal.signal(signal.SIGWINCH, self._on_resize)
        self._setup(first=True)
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        if self.active:
            top = self._rows - self.height
            self._write(f"\x1b7\x1b[r\x1b8\x1b[{top + 1};1H\x1b[J\x1b[?25h")
        if self._old_handler is not None:
            signal.signal(signal.SIGWINCH, self._old_handler)  # pyright: ignore[reportArgumentType]

    def draw(self, canvas: Canvas) -> None:
        if self.resized:
            self.resized = False
            self._setup(first=False)
        if not self.active:
            return
        top = self._rows - self.height
        out = "\x1b7"
        for i, line in enumerate(canvas.lines(min(self._cols, canvas.width))):
            out += f"\x1b[{top + 1 + i};1H\x1b[2K{line}"
        self._write(out + "\x1b8")

    def _setup(self, first: bool) -> None:
        size = os.get_terminal_size(self._fd())
        self._cols, self._rows = size.columns, size.lines
        was, self.active = self.active, self._rows >= self.height + MIN_LOG_ROWS
        if not self.active:
            if was:
                self._write("\x1b7\x1b[r\x1b8\x1b[?25h")
            log.warning(
                "terminal too small for the switch panel: %dx%d, needs %d rows",
                self._cols,
                self._rows,
                self.height + MIN_LOG_ROWS,
            )
            return
        top = self._rows - self.height
        if first:
            # Scroll what is on screen up out of the panel's rows, keeping the cursor on it.
            self._write("\n" * self.height + f"\x1b[{self.height}A\x1b7\x1b[1;{top}r\x1b8\x1b[?25l")
        else:
            # The terminal reflowed the screen: clear the panel's rows, logs go on above.
            self._write(f"\x1b[1;{top}r\x1b[{top + 1};1H\x1b[J\x1b[{top};1H\x1b[?25l")

    def _on_resize(self, signum: int, frame: FrameType | None) -> None:
        self.resized = True

    def _fd(self) -> int:
        return self._out.fileno()

    def _write(self, text: str) -> None:
        """One write, between log records: every logging handler is held meanwhile."""
        handlers = logging.getLogger().handlers
        for h in handlers:
            h.acquire()
        try:
            sys.stdout.flush()
            sys.stderr.flush()
            data = text.encode()
            while data:
                data = data[os.write(self._fd(), data) :]
        finally:
            for h in reversed(handlers):
                h.release()
