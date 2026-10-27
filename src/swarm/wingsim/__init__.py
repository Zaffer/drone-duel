"""A fixed-wing flight model for INAV SITL: the TBS Chupito, flown by INAV over X-Plane's protocol.

INAV SITL has no physics of its own; it flies whatever simulator answers it over UDP as X-Plane
would. This package is that simulator: a 6-DOF rigid body with full-envelope strip aerodynamics
over a vortex lattice, a motor, prop and battery, the wind, a hand launch and the ground,
calibrated against TBS's specification and a real Chupito's INAV blackbox log.

    airframe    the Chupito's numbers, each with where it came from
    strips      the aerodynamics: strips, their stall, the downwash, ground effect
    aero        reference geometry and air data
    weather     the wind: its shear, turbulence and gusts (MIL-F-8785C)
    propulsion  ESC, motor, prop and pack
    rigidbody   6-DOF state and its integration (NED world, forward-right-down body)
    aircraft    the whole wing: hand, throw, flight, ground
    dynamics    all of it at work, to look into as it flies
    xplane      INAV's side of the X-Plane UDP protocol
    inav        the INAV SITL process and its configuration
    server      the real-time loop between INAV and the aircraft

Self-contained on purpose: it imports nothing else from swarm, and nothing but the standard
library, so it can move to a package of its own.
"""
