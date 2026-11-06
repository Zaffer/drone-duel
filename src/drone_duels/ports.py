"""Every port the game uses, in one place (DESIGN.md §3). Nothing else hard-codes a port.

A vehicle slot is one vehicle at a time: its firmware, its physics and the links between them.
v1 flies slot 0 only; the swarm (ROADMAP) will use more. ArduPilot SITL fixes its own ports
from its instance number (`-I`), ten apart: the slot is the instance. INAV SITL takes a TCP base
port, so it gets a block of its own, clear of ArduPilot's 5760, where mavlink-router connects.
Betaflight SITL, patched to take its ports, gets blocks of its own too.
"""

from dataclasses import dataclass

HTTP = 8600
"""The game's page and API. The Google Maps key is restricted to this port."""

ROUTER_QGC = 14550
"""mavlink-router's QGroundControl endpoint (QGC is system 255), the same for every slot: the
router only sends there."""

SLOT_STRIDE = 10


@dataclass(frozen=True)
class SlotPorts:
    slot: int

    # ArduPilot SITL, instance = slot.
    @property
    def ardupilot_mavlink(self) -> int:
        """SERIAL0, TCP: MAVLink, which mavlink-router connects to."""
        return 5760 + SLOT_STRIDE * self.slot

    @property
    def ardupilot_rc(self) -> int:
        """UDP: RC input, 8 or 16 little-endian uint16 pulse widths a packet."""
        return 5501 + SLOT_STRIDE * self.slot

    @property
    def ardupilot_json(self) -> int:
        """UDP: the JSON physics backend; the physics listens, ArduPilot sends servos here."""
        return 9002 + SLOT_STRIDE * self.slot

    @property
    def router_game(self) -> int:
        """UDP: mavlink-router's endpoint for the game's telemetry (system 254); the router
        listens, so each slot has its own."""
        return 14570 + self.slot

    @property
    def ardupilot_osd(self) -> int:
        """UDP: SERIAL5's MSP DisplayPort OSD; the server listens."""
        return 14700 + self.slot

    # INAV SITL.
    @property
    def inav_tcp_base(self) -> int:
        """TCP: UART1 (CLI and MSP) here, UART2 (MSP) +1, UART3 (the DJI OSD) +2."""
        return 6760 + SLOT_STRIDE * self.slot

    @property
    def inav_msp(self) -> int:
        return self.inav_tcp_base + 1

    @property
    def inav_osd(self) -> int:
        return self.inav_tcp_base + 2

    @property
    def inav_xplane(self) -> int:
        """UDP: the physics plays X-Plane here; INAV connects to it."""
        return 49000 + SLOT_STRIDE * self.slot

    # Betaflight SITL (scripts/betaflight-sitl.patch makes its ports arguments; its own defaults,
    # 9001-9004 and TCP 5760, are ArduPilot's).
    @property
    def betaflight_sim_base(self) -> int:
        """UDP, Betaflight's --simport: raw motor PWM out here, normalised motors +1 (both to the
        physics), the physics' state in +2, RC in +3."""
        return 9601 + SLOT_STRIDE * self.slot

    @property
    def betaflight_pwm_raw(self) -> int:
        return self.betaflight_sim_base

    @property
    def betaflight_pwm(self) -> int:
        return self.betaflight_sim_base + 1

    @property
    def betaflight_state(self) -> int:
        return self.betaflight_sim_base + 2

    @property
    def betaflight_rc(self) -> int:
        return self.betaflight_sim_base + 3

    @property
    def betaflight_config_sim_base(self) -> int:
        """UDP: --simport while Betaflight types its settings in (--config), where nothing sends:
        state arriving from the physics as it does crashes it."""
        return self.betaflight_sim_base + 4

    @property
    def betaflight_tcp_base(self) -> int:
        """TCP, Betaflight's --tcpbaseport: UARTn on this + n. UART1 the CLI and MSP, UART2 MSP
        (the ground station), UART3 MSP DisplayPort (the OSD, as to a DJI air unit)."""
        return 7760 + SLOT_STRIDE * self.slot

    @property
    def betaflight_msp(self) -> int:
        return self.betaflight_tcp_base + 2

    @property
    def betaflight_osd(self) -> int:
        return self.betaflight_tcp_base + 3

    # Our own physics process.
    @property
    def physics_control(self) -> int:
        """UDP: the physics listens for control (wind, ground, rc, reset)."""
        return 15000 + SLOT_STRIDE * self.slot

    @property
    def physics_state(self) -> int:
        """UDP: the server listens for the physics' state (pose, dynamics, status)."""
        return 15001 + SLOT_STRIDE * self.slot


def slot(n: int) -> SlotPorts:
    return SlotPorts(n)
