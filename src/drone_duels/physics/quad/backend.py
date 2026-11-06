"""The quad behind its firmware's simulator protocol: physics.runner's Backend for kind "quad".

QuadModel (model.py) is the vehicle and its world. Under ArduCopter, JsonBridge
(bridges/ardupilot_json.py) steps it in lockstep with SITL on the slot's JSON port; under
Betaflight, BetaflightBridge (bridges/betaflight.py) steps it in real time and Betaflight
follows. Both firmwares take their RC over their own UDP ports (firmware/ardupilot.py,
firmware/betaflight.py), so `rc` here does nothing. The plant is built and jitted before the
bridge listens, so the first frame is not kept waiting.
"""

import logging
from typing import Any

import numpy as np

from drone_duels.bridges.ardupilot_json import JsonBridge
from drone_duels.bridges.betaflight import BetaflightBridge
from drone_duels.physics.quad.model import QuadModel
from drone_duels.physics.quad.plant import Plant, airframe
from drone_duels.ports import SlotPorts

log = logging.getLogger(__name__)


class QuadBackend:
    def __init__(
        self,
        airframe_name: str,
        slot_ports: SlotPorts,
        origin: tuple[float, float, float],
        heading_deg: float,
        wind: dict[str, Any],
        firmware: str = "ardupilot-copter",
    ) -> None:
        frame = airframe(airframe_name)
        plant = Plant(frame)
        plant.warm()
        self.model = QuadModel(frame, origin, heading_deg, wind, plant=plant)
        self.model.esc.command(np.zeros(4), 25.2)  # jit the ESC map too
        self.bridge: JsonBridge | BetaflightBridge
        if firmware == "betaflight":
            self.bridge = BetaflightBridge(
                self.model, slot_ports.betaflight_pwm_raw, slot_ports.betaflight_state, origin
            )
            link = f"Betaflight's motors on UDP {slot_ports.betaflight_pwm_raw}"
        else:
            self.bridge = JsonBridge(self.model, slot_ports.ardupilot_json)
            link = f"JSON on UDP {slot_ports.ardupilot_json}"
        log.info(
            "quad %s at %.6f, %.6f, %.1f m, heading %.0f; %s",
            airframe_name, *origin, heading_deg, link,
        )

    def start(self) -> None:
        self.bridge.start()

    def stop(self) -> None:
        self.bridge.stop()

    def connected(self) -> bool:
        return self.bridge.connected()

    def phase(self) -> str:
        return self.model.phase

    def pose(self) -> tuple[float, ...] | None:
        return self.model.pose()

    def rc(self, us: list[int] | None) -> None:
        pass  # the firmware reads its RC on its own UDP port

    def ground(self, lat: float, lon: float, alt: float) -> None:
        self.model.set_ground(alt)

    def wind(self, wind: dict[str, Any], gust_now: bool) -> None:
        self.model.set_wind(wind, gust_now)

    def reset(self) -> None:
        self.model.reset()

    def throw(self) -> None:
        pass

    def hit(
        self,
        part: str,
        motor: int | None,
        impulse_ned: tuple[float, float, float],
        at_frd: tuple[float, float, float],
    ) -> None:
        self.model.hit(part, motor, impulse_ned, at_frd)

    def damage(self) -> dict[str, Any] | None:
        return self.model.damage_summary()

    def dynamics(self) -> dict[str, Any] | None:
        return None
