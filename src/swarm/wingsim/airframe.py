"""The TBS Chupito: its numbers, and where each came from.

Sources
  TBS     Team BlackSheep's Chupito manual (rev 1.09) and INAV CLI suggestions: 800 mm span,
          650 g all up, a forward-swept wing, 5x3x3 prop on a 2200 KV motor, 6S, elevon throws,
          the centre of mass at the wing's leading edge beside the body, 140 km/h, stall 35 km/h.
  GLB     narenana's 3D model of it (Nanawing), measured: the planform (0.147 m² seen from above,
          the leading edge straight, the trailing edge swept forward), the elevons (57 to 275 mm
          out, 45 mm chord), the fin and the pod. The manual's top view agrees.
  UIUC    Brandt and Selig's measured GWS 5x3 prop (UIUC propeller database, volume 2): the shape
          of its thrust and power against advance ratio.
  LIT     The literature: Selig's full-envelope method (strips.py), Lindenburg's broadside drag,
          thin-aerofoil flap theory, Munk's body moment; Goman and Khrabrov's separation lag,
          its time constants from a pitching aerofoil's fit ("Lift coefficient estimation for a
          rapidly pitching airfoil", arXiv 2005.01870: 3.75 and 4.4 chord times) and Leishman
          and Beddoes' lags (1.5 and 0.85); static stall hysteresis at this wing's Reynolds
          numbers, 1 to 3 x 10^5, from laminar separation bubbles (Mueller; Selig's low
          Reynolds number aerofoil tests: loops of 1 to 4 deg).
  LOG     One pilot's INAV 8.0.1 blackbox log of a 6S Chupito (RCGroups, Nov 2025), 6.9 min, no
          pitot: full-elevon rolls at 600-715 deg/s from 22 to 46 m/s; full pitch command at about
          29 m/s pulling a CL of 0.66 (median) at 7 g; about 42 m/s level at full throttle on
          540 W. Its cruise, about 29 m/s at 45 % throttle on 127 W, is beyond a stock 5x3 prop's
          pitch speed at that throttle: a higher pitch prop, or its GPS-derived airspeed reading
          high. Not used.
  EST     Estimated, and to be measured.

Calibrated (tests/test_wingsim.py holds the model to these)
  the prop's static coefficients and no-thrust advance ratio, the motor's resistance
                LOG's 42 m/s level on 540 W at full throttle; TBS's cruise, 55 km/h at its 35 %
                cruise throttle (58 km/h); a static thrust near 1.2 kgf
  the elevons' effectiveness, the wing's twist under them (FLEX_PA)
                LOG's steady full-elevon rolls, 600 deg/s at 23 m/s and 700 at 45 m/s, fitted
                in the full nonlinear model: 1.05 times a plain flap's thin-aerofoil effectiveness.
                LOG's pulls then agree (tests), with no fitting of their own
  zero lift, cm0
                CL 0.09 at zero angle of attack (the X8's, a reflexed foam wing), trimming at
                28 m/s with the elevons neutral (LOG)
  the stall, the centre of mass
                TBS's 35 km/h, a stall the elevons hold (13 deg up): CL max 0.75 so trimmed
"""

import math

from swarm.wingsim.aero import Geometry
from swarm.wingsim.aircraft import Airframe, GroundContact, Launch
from swarm.wingsim.propulsion import PowerTrain
from swarm.wingsim.rigidbody import Inertia
from swarm.wingsim.strips import Planform, Section, Strip, Wing, lifting_line, wing_strips

REFERENCE = Geometry(span=0.80, area=0.147)  # TBS span; GLB area: aspect ratio 4.35, chord 0.184 m

PLANFORM = Planform(
    # GLB, x forward from the model's origin: (y, leading edge, trailing edge), m. Inboard of
    # 80 mm the pod covers the wing; its edges there run on straight.
    stations=(
        (0.0, -0.011, -0.241),
        (0.08, -0.001, -0.203),
        (0.12, 0.004, -0.184),
        (0.16, 0.012, -0.168),
        (0.20, 0.018, -0.149),
        (0.24, 0.025, -0.133),
        (0.28, 0.031, -0.118),
        (0.32, 0.035, -0.100),
        (0.36, 0.028, -0.083),
        (0.388, 0.003, -0.072),
        (0.40, -0.015, -0.055),
    ),
    cuts=(0.0, 0.057, 0.12, 0.2, 0.275, 0.34, 0.40),  # strip edges at the elevons' ends
)
CENTRE_OF_MASS = (-0.012, 0.0)
"""GLB x, and its height in the wing's plane (EST), m. Calibrated: TBS's stall speed, which puts
it 10 mm behind the wing's leading edge beside the body, where TBS's manual marks it (in a
picture): a static margin of 8.6 %."""
ELEVON = (0.057, 0.275, 0.045)  # GLB: inner and outer edge, chord, m

ELEVON_MAX = math.radians(25)
"""Each elevon's deflection at full servo travel (500 µs). TBS's throws, 15 mm roll at 80 % and
12 mm pitch at 60 % mix, both come to about 19 mm at full travel on a 45 mm elevon chord (GLB)."""
ELEVON_EFFECTIVENESS = 1.05
"""Times thin-aerofoil theory's plain flap: LOG's rolls (see the header)."""
FLEX_PA = 997.0
"""LOG's rolls: the foam wing's twist halves the elevons' effect at this dynamic pressure."""

SEPARATION_LAG = 3.75  # LIT: Goman and Khrabrov's τ1, in chord times
SEPARATION_DELAY = 2.0
"""Goman and Khrabrov's τ2, chord times (EST): between Leishman and Beddoes' pressure lag (0.85)
and the pitching aerofoil's fit (4.4), a two-dimensional section pitched fast. With the
hysteresis, 4.4 holds this wing in a 9 to 20 deg pitch-bucking cycle at the stall; 2.0, a bob of
a few degrees: the gentle stall TBS describes."""
SECTION = Section(
    lift_slope=0.9 * 2 * math.pi,  # LIT: a thick foam section's, 90 % of thin aerofoil's
    zero_lift=math.radians(-1.64),  # calibrated
    cm0=0.0121,  # calibrated: the reflex
    cd0=0.012,  # EST: the section's profile drag
    alpha_stall=math.radians(15),  # calibrated: TBS's 35 km/h, CL max 0.75 held by the elevons
    alpha_stall_neg=math.radians(14),  # EST: 12 deg below zero lift, short of upright's 17
    stall_sharpness=30.0,  # Beard and McLain's typical value
    cd90=2.2 * (1 - 0.41 * (1 - math.exp(-17 / REFERENCE.aspect_ratio))),  # LIT: Lindenburg
    reattach=math.radians(2),  # LIT, EST: low Reynolds number aerofoils' loops are 1 to 4 deg
    separation_lag=SEPARATION_LAG,
    separation_delay=SEPARATION_DELAY,
)
# No washout: none is known, and without it the stall starts inboard, where the vortex lattice
# loads the wing most, and spreads out to the tips: the gentle stall TBS's "easy to fly" implies.
WING_STRIPS = wing_strips(PLANFORM, SECTION, CENTRE_OF_MASS, ELEVON, ELEVON_EFFECTIVENESS)
FIN_HEIGHT, FIN_AREA = 0.10, 0.009  # GLB: on the pod's tail, from 60 to 160 mm up
FIN = Strip(
    quarter_chord=(-0.165 - CENTRE_OF_MASS[0], 0.0, CENTRE_OF_MASS[1] - 0.11),  # GLB
    forward=(1.0, 0.0, 0.0),
    up=(0.0, 1.0, 0.0),
    chord=FIN_AREA / FIN_HEIGHT,
    width=FIN_HEIGHT,
    section=Section(
        lift_slope=3.0,  # LIT: aspect ratio 2.5 with the pod as an end plate
        zero_lift=0.0,
        cm0=0.0,
        cd0=0.012,
        alpha_stall=math.radians(20),
        alpha_stall_neg=math.radians(20),
        stall_sharpness=30.0,
        cd90=1.2,
        induced_drag=1 / (math.pi * 0.8 * 2.5),
        separation_lag=SEPARATION_LAG,
        separation_delay=SEPARATION_DELAY,
    ),
)
POD_VOLUME = math.pi / 6 * 0.47 * 0.098 * 0.085  # GLB: 470 x 98 x 85 mm
CD0 = 0.028
"""The whole wing's zero-lift drag: LOG, through the power train."""

WING = Wing(
    reference=REFERENCE,
    strips=(*WING_STRIPS, FIN),
    oswald=0.8,
    # Everything the sections' profile drag leaves of CD0: the pod, the camera, the gaps.
    body_cd=CD0 - SECTION.cd0 * (sum(s.area for s in WING_STRIPS) + FIN_AREA) / REFERENCE.area,
    flex_pa=FLEX_PA,
    body_volume=POD_VOLUME,
    body_munk=0.88 * 0.55,  # LIT: k2 - k1 at fineness 5, times the share real bodies show
    loading=lifting_line(WING_STRIPS),
)

INERTIA = Inertia(
    mass=0.65,  # TBS all-up weight
    # EST, summed from its parts where TBS's manual and the GLB put them: about 180 g of foam
    # wing (span² / 12), the 6S 1300 pack (200 g) in the bay ahead of the wing, the motor and prop
    # (40 g) 24 cm behind the centre of mass, the camera in the nose, servos in the wings.
    ixx=0.010,
    iyy=0.0085,
    izz=0.018,
    ixz=0.0004,
)

POWER_TRAIN = PowerTrain(
    # The prop's coefficients and the resistance are fitted together (see the header).
    kv=2200,  # TBS
    resistance=0.165,  # fitted
    no_load_current=1.0,  # EST
    rotor_inertia=6.0e-6,  # EST: 5 in tri-blade and a 20xx bell
    prop_diameter=5 * 0.0254,  # TBS: 5x3x3
    ct0=0.118,  # fitted: 12.7 N static
    cp0=0.0651,  # fitted
    j_zero=0.661,  # fitted: about a 3.3 in pitch
    cells=6,  # TBS
    cell_full=4.2,
    cell_empty=3.5,
    capacity_mah=1300,  # TBS's 20 minute pack
    pack_resistance=0.06,  # EST: a 1300 mAh 6S under 30 A
    torque_sign=-1.0,  # EST: which way the motor turns is unknown
    prop_position=(-0.253, 0.0, -0.01),  # GLB: the hub, behind the centre of mass
)

LAUNCH = Launch(
    height=1.8,  # thrown from overhead (TBS)
    speed=12.0,  # m/s at release
    climb=math.radians(15),
    pitch=math.radians(20),
    duration=0.25,  # the arm's swing: about 5 g, enough for INAV's launch detection
)

GROUND = GroundContact(
    landing_descent=2.0,  # m/s: softer than this onto belly foam is a landing
    landing_bank=math.radians(25),
    landing_speed=20.0,
    friction=0.6,  # EPP on grass
)

CHUPITO = Airframe(
    wing=WING,
    inertia=INERTIA,
    power_train=POWER_TRAIN,
    elevon_max=ELEVON_MAX,
    launch=LAUNCH,
    ground=GROUND,
)
