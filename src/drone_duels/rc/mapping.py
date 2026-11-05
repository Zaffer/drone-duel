"""From a radio's raw axes to the firmware's pulses (DESIGN.md §12), as the page's
input/mapping.ts does it: the server flies its own radio with these, the page its gamepads and
keyboard with those, and both pass tests/fixtures/mapping.json.

Channels are "natural", whatever the firmware: each -1..1 from channel 1, roll +1 right, pitch +1
stick forward (nose down), throttle +1 full, yaw +1 right; switches -1 low, +1 high. Each
firmware's own conventions (ArduPilot reads pitch forward as a low pulse) are applied on the way
out (`to_firmware`).
"""

import math
from collections.abc import Mapping, Sequence
from typing import Any, Literal, TypedDict

from pydantic import TypeAdapter, ValidationError

from drone_duels.content.schema import ButtonMap, Controller, FirmwareKind

CENTRED = frozenset({0, 1, 3})
"""Channels whose travel has a centre (the sticks other than the throttle)."""


class AxisCal(TypedDict):
    """The raw axis value at a channel's low end, centre and high end."""

    min: float
    mid: float
    max: float


class ChannelCal(TypedDict):
    axis: int
    reverse: bool
    cal: AxisCal


class Calibration(TypedDict):
    """Per channel (from channel 1): which axis, which way round, and its travel."""

    channels: list[ChannelCal]


_calibration = TypeAdapter(Calibration)
_IDENTITY = AxisCal(min=-1, mid=0, max=1)


def calibrations(prefs: Mapping[str, Any]) -> dict[str, Calibration]:
    """The calibrations in the player's prefs (controls.calibrations), by controller id; any
    that does not parse is left out."""
    controls = prefs.get("controls")
    saved = controls.get("calibrations") if isinstance(controls, dict) else None
    out: dict[str, Calibration] = {}
    for key, value in (saved or {}).items():
        try:
            out[str(key)] = _calibration.validate_python(value)
        except ValidationError:
            continue
    return out


def profile_for(controller_id: str, profiles: Sequence[Controller]) -> Controller | None:
    """The first profile any of whose `match` strings appears in the controller's id."""
    lower = controller_id.lower()
    return next((p for p in profiles if any(m.lower() in lower for m in p.match)), None)


def calibrate(raw: float, cal: AxisCal, centred: bool) -> float:
    """A raw axis value through its calibration, to -1..1: centred channels about their centre,
    the others straight from low to high."""
    if centred:
        if raw >= cal["mid"]:
            x = (raw - cal["mid"]) / ((cal["max"] - cal["mid"]) or 1)
        else:
            x = (raw - cal["mid"]) / ((cal["mid"] - cal["min"]) or 1)
    else:
        x = (raw - cal["min"]) / ((cal["max"] - cal["min"]) or 1) * 2 - 1
    return max(-1.0, min(1.0, x))


def to_channels(
    axes: Sequence[float],
    buttons: Sequence[float],
    profile: Controller | None,
    calibration: Calibration | None,
) -> list[float]:
    """The channels from raw axes and buttons: the player's calibration if there is one, else
    the profile's map, else axis n as channel n + 1."""

    def axis(i: int) -> float:
        return axes[i] if i < len(axes) else 0.0

    if calibration and calibration["channels"]:
        out: list[float] = []
        for i, c in enumerate(calibration["channels"]):
            x = calibrate(axis(c["axis"]), c["cal"], i in CENTRED)
            out.append(-x if c["reverse"] else x)
        return out
    if profile:
        out = []
        for i, src in enumerate(profile.channels):
            if isinstance(src, ButtonMap):
                pressed = (buttons[src.button] if src.button < len(buttons) else 0) > 0.5
                out.append(1.0 if pressed else -1.0)
                continue
            x = calibrate(axis(src.axis), _IDENTITY, i in CENTRED)
            out.append(-x if src.reverse else x)
        return out
    return [max(-1.0, min(1.0, a)) for a in axes[:16]]


def switch_pos(x: float | None) -> Literal[0, 1, 2]:
    """A switch channel's position: 0 low, 1 middle, 2 high."""
    if x is None or x < -0.33:
        return 0
    return 2 if x > 0.33 else 1


def to_us(x: float) -> int:
    """A channel's value (-1..1) as a pulse width, µs (rounding halves up, as the page does)."""
    return math.floor(1500 + 500 * max(-1.0, min(1.0, x)) + 0.5)


def to_firmware(channels: Sequence[float], firmware: FirmwareKind) -> list[int]:
    """The channels as the firmware's pulses, µs, at least eight: ArduPilot reads pitch forward
    as a low pulse, INAV as a high one (as Betaflight)."""
    us = [to_us(x) for x in channels[:16]]
    us += [1000] * (8 - len(us))
    if firmware == "ardupilot-copter" and len(channels) > 1:
        us[1] = to_us(-channels[1])
    return us
