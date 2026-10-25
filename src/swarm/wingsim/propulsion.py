"""The power train: pack, ESC, brushless motor and prop.

The ESC turns the throttle into a duty cycle on the pack's voltage; the motor is a DC machine
(back EMF and winding resistance, the ESC's and wiring's lumped in); the prop's thrust and power
coefficients fall with the advance ratio J = V / (n D), so thrust fades as the wing speeds up and
the motor unloads. Their shape is UIUC's measured GWS 5x3 (Brandt and Selig, volume 2): power
falls about as fast as thrust, so past zero thrust the air drives the prop (windmilling); and the
air coming from behind (a tail slide, J < 0) loads it up. The rotor's speed is integrated, so
throttle changes spool, and the airframe takes the motor's torque, not the prop's: punching the
throttle kicks it in roll. With the duty at nothing the windings are shorted, as an ESC with brake
on stop does, and the prop stops.

The prop also turns the air through it: flow across the disc loses its sideways momentum to the
induced flow (Selig 2010, AIAA 2010-7938: "momentum drag"), a side force at the prop that matters
when it hangs on its thrust; in forward flight a spinning prop at an angle to the flow makes a
normal force too (Ribner's, McCormick's), so a pusher's prop is a fin behind the centre of mass,
steadying it in pitch and yaw. And the spinning rotor's gyroscopic moment (aircraft.py).
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class PowerTrain:
    kv: float
    """Motor speed constant, rpm per volt."""
    resistance: float
    """Winding plus ESC and wiring, ohms."""
    no_load_current: float
    rotor_inertia: float
    """Motor bell and prop, kg m²."""
    prop_diameter: float
    ct0: float
    """Static thrust coefficient: thrust = CT rho n² D⁴."""
    cp0: float
    """Static power coefficient: power = CP rho n³ D⁵."""
    j_zero: float
    """The advance ratio of no thrust, about the pitch over the diameter."""
    cells: int
    cell_full: float
    cell_empty: float
    capacity_mah: float
    pack_resistance: float
    torque_sign: float
    """The motor's reaction on the airframe about the thrust axis: -1 for a prop turning
    clockwise seen from behind, which rolls the wing left."""
    prop_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    """The prop's hub from the centre of mass, body frame, m."""
    momentum_drag: float = 0.9
    """The share of the flow across the disc that the induced flow turns (Selig's k, 0.8-1)."""
    normal_force: float = 0.2
    """In forward flight, the prop's normal force per unit of crossflow, as a share of the air
    coming through its disc: N = k rho A V v_across."""

    @property
    def disc_area(self) -> float:
        return math.pi * self.prop_diameter**2 / 4

    def coefficients(self, j: float) -> tuple[float, float]:
        """CT and CP at advance ratio `j` (negative: the air comes from behind). UIUC's GWS 5x3,
        thrust and power over their static values against J over J of no thrust."""
        x = max(-0.5, min(1.6, j / self.j_zero))
        return (
            self.ct0 * (1 - 0.8 * x - 0.2 * x * x),
            self.cp0 * (1 - 0.18 * x - 0.52 * x * x),
        )

    @property
    def back_emf(self) -> float:
        """Volts per rad/s, and newton-metres per amp."""
        return 60 / (2 * math.pi * self.kv)


@dataclass
class PowerTrainState:
    omega: float = 0.0
    """Prop speed, rad/s."""
    used_mah: float = 0.0
    thrust: float = 0.0
    torque: float = 0.0
    """The motor's reaction on the airframe about the thrust axis (PowerTrain.torque_sign)."""
    pack_voltage: float = 0.0
    pack_current: float = 0.0

    def step(
        self, pt: PowerTrain, throttle: float, axial_speed: float, rho: float, dt: float
    ) -> None:
        """Advance the rotor by `dt` at `throttle` (0..1), the air coming at the prop at
        `axial_speed` m/s."""
        duty = max(0.0, min(1.0, throttle))
        charge = max(0.0, 1 - self.used_mah / pt.capacity_mah)
        open_circuit = pt.cells * (pt.cell_empty + (pt.cell_full - pt.cell_empty) * charge)
        ke = pt.back_emf
        omega = max(0.0, self.omega)
        # The pack sags under the motor's current, which depends on the pack's voltage: solve the
        # loop for the motor current once, linear in it.
        motor_current = (duty * open_circuit - ke * omega) / (
            pt.resistance + duty * duty * pt.pack_resistance
        )
        pack_current = max(0.0, duty * motor_current)
        self.pack_voltage = open_circuit - pack_current * pt.pack_resistance
        self.pack_current = pack_current

        n = omega / (2 * math.pi)
        d = pt.prop_diameter
        if n > 1.0:
            ct, cp = pt.coefficients(axial_speed / (n * d))
            thrust = ct * rho * n * n * d**4
            prop_torque = cp * rho * n * n * d**5 / (2 * math.pi)
        else:
            # Stopped, or nearly: the air turns it, if it flows hard enough; a stopped prop drags.
            thrust = -0.5 * rho * pt.disc_area * 0.06 * axial_speed * abs(axial_speed)
            prop_torque = 0.0
        motor_torque = ke * (motor_current - math.copysign(pt.no_load_current, motor_current))
        if omega <= 0 and motor_torque < 0:
            motor_torque = 0.0
        self.omega = max(0.0, omega + (motor_torque - prop_torque) / pt.rotor_inertia * dt)
        self.thrust = thrust
        self.torque = pt.torque_sign * motor_torque
        self.used_mah += pack_current * dt / 3.6


def momentum_drag(
    pt: PowerTrain, state: PowerTrainState, axial: float, across: tuple[float, float], rho: float
) -> tuple[float, float]:
    """The side force (body y, z; N) on the spinning prop from the air crossing its disc at
    `across` (m/s, body y and z), the air coming at it axially at `axial`: the induced flow, from
    momentum theory, takes the crossing air's momentum with it, and the blades meeting the flow
    at an angle push it aside."""
    if state.omega < 2 * math.pi:
        return 0.0, 0.0
    area = pt.disc_area
    v = max(0.0, axial)
    thrust = max(0.0, state.thrust)
    induced = -v / 2 + math.sqrt(v * v / 4 + thrust / (2 * rho * area))
    k = rho * area * (pt.momentum_drag * induced + pt.normal_force * v)
    return -k * across[0], -k * across[1]
