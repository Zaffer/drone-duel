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
          pitot: full-elevon rolls at 600-715 deg/s from 22 to 46 m/s, the servo 585 µs off its
          centre; full pitch command at about 29 m/s, 203 µs of pitch on the servo, pulling 7.3 g
          (median) at a CL of 0.66; moderate pulls at 1.0-1.4 deg of elevon a g; the trim flat
          within 0.3 deg from 20 to 36 m/s; about 42 m/s level at full throttle on 540 W; its
          launch at 65 % throttle drawing 16 A slow and 14 A at 25 m/s. Its cruise, about 29 m/s
          at 45 % throttle on 127 W, is beyond a stock 5x3 prop's pitch speed at that throttle:
          a higher pitch prop, or its GPS-derived airspeed reading high. Not used.
  PACK    The pack flown: Tattu's R-Line V6.0 1300 mAh 6S 160C, a standard LiPo (22.2 V
          nominal, 4.2 V a cell full), 197-203 g by its listings. Tattu gives no resistance.
  EST     Estimated, and to be measured.

Calibrated (tests/test_physics_wing.py holds the model to these)
  the prop's static coefficients and no-thrust advance ratio, the motor's resistance
                LOG's 42 m/s level on 540 W at full throttle (TBS's listing: 160 km/h); TBS's
                cruise at its 35 % cruise throttle, 55 km/h in its manual, 70 km/h in its
                listing (the model: about 70). LOG's launch currents then agree (tests); its
                static thrust, about 1.65 kgf on 43 A, nothing measures
  the elevons' effectiveness, the wing's twist under them (FLEX_PA)
                LOG's steady full-elevon rolls, 650 deg/s at 23 m/s and 700 at 45 m/s, at the
                servo travel logged, fitted in the full nonlinear model: a plain flap's
                thin-aerofoil effectiveness, halved by the twist at 34 m/s
  the centre of mass
                LOG's pulls, through the elevons so fitted: 7.3 g on 203 µs at 29 m/s, and
                1.35 deg a g (1.0-1.4): a static margin of 3.1 %, 21 mm behind the leading edge
                beside the body. TBS's manual marks it at that edge, in a picture, so a little
                ahead; their builds' differ, and the log's pulls are measured
  zero lift, cm0
                CL 0.09 at zero angle of attack (the X8's, a reflexed foam wing), trimming at
                28 m/s with the elevons neutral (LOG)
  the stall     TBS's 35 km/h through the section's stall angle (12.5 deg): CL max 0.74, the
                elevons holding it with 5 deg up

Guesses (EST here and where they live), most felt first. Of how it stalls, nothing is known
beyond TBS's 35 km/h and that it is "stable at low speeds and forgiving in stalls" (TBS's
dealers); no pilot's account of a wing drop, tip stall or bobbing has been found, nor a log.
  inertia       summed from parts (INERTIA): how fast it starts and stops rolling and pitching,
                the pitch bounce on letting go. A swing test would measure it. LOG's pitch
                bounces (23 rad/s at 26 m/s, the model's 17) were flown with INAV's rate loop
                closed, so they don't measure the airframe's: with that loop around this model,
                they would.
  the trim against speed
                LOG's is flat within 0.3 deg from 20 to 36 m/s; the model's moves 0.7 deg from
                22 to 28 and 1.8 to 40. LOG's airspeed is GPS less a wind estimate, whose noise
                flattens a slope, so how flat the real one is is uncertain
  the stall's shape
                how sharply each strip stalls (30 /rad, Beard and McLain's generic value); that
                the stall starts at the root and spreads out (no washout or stall spread is
                known); the negative stall (14 deg); how far an elevon moves the stall
                (flap_stall) and works in separated flow (flap_separated, strips.py). Together:
                whether a held stall mushes, as now, or drops a wing.
  the stall's memory
                SEPARATION_DELAY (2.0 chord times, chosen in a 0.85 to 4.4 range, by the held
                stall's bobbing), the hysteresis (2 deg, in a 1 to 4 range), the lag (3.75, a
                two-dimensional section's), DYNAMIC_STALL_MAX (10 deg, strips.py): how high a
                quick pull peaks, how long a stall lingers once the stick is eased.
  the centre of mass's height
  torque roll   LOG's wing rolls right with power, so the motor is taken to turn the way that
                does; but LOG's needs 15 times the elevon the motor's torque explains, and the
                rest, its cause unknown, is left out (see POWER_TRAIN)
  the prop      a two-blade's measured shape for TBS's three-blade, carried past the measured
                range (windmilling, reverse flow); its crossflow forces (momentum_drag,
                normal_force) and stopped drag (propulsion.py); its rotor's inertia
  reversed flow trailing edge first: its lift slope and its stall at 8 deg (strips.py): tail
                slides
  the fin       its stall (20 deg) and broadside drag (1.2): sideslip far past the stall
  ground effect McCormick's whole-wing formula taken strip by strip (strips.py): how far it
                floats, how hard a low tip is pushed up
  the gusts     how often (10 s), how long (2 to 5 s), how far off the wind (30 deg), how weak
                (40 % of the strongest, weather.py); the turbulence's least airspeed (2 m/s).
                The turbulence itself is MIL-F-8785C's, made for full-size aircraft, felt
                through its own span filter (weather.py)
  the rest      the servo's speed (600 deg/s), the belly's height, what counts as flying into
                something, a landing's limits and the grass's friction (aircraft.py, GROUND);
                the throw (LAUNCH); the section's profile drag (the rest of CD0 goes to the
                body); the motor's no-load current
  the pack      its resistance and slow sag, LOG's pack's (another make, a high-voltage one);
                its discharge curve, a typical LiPo cell's (LIPO_CELL). A charger's per-cell
                resistance readings, and a logged flight on it, would measure them
And the sources' own limits: TBS's numbers are a manufacturer's; LOG had no pitot, so its
airspeeds are GPS speeds in an unknown wind, and its pilot's mix and tune were their own.
"""

import math

from drone_duels.physics.core.rigidbody import Inertia
from drone_duels.physics.wing.aero import Geometry
from drone_duels.physics.wing.aircraft import Airframe, GroundContact, Launch
from drone_duels.physics.wing.propulsion import PowerTrain
from drone_duels.physics.wing.strips import (
    Planform,
    Section,
    Strip,
    Wing,
    lifting_line,
    wing_strips,
)

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
CENTRE_OF_MASS = (-0.022, 0.0)
"""GLB x, and its height in the wing's plane (EST), m. Calibrated: LOG's pulls (see the header),
21 mm behind the wing's leading edge beside the body: a static margin of 3.1 %."""
ELEVON = (0.057, 0.275, 0.045)  # GLB: inner and outer edge, chord, m

ELEVON_MAX = math.radians(25)
"""Each elevon's deflection at full servo travel (500 µs). TBS's throws, 15 mm roll at 80 % and
12 mm pitch at 60 % mix, both come to about 19 mm at full travel on a 45 mm elevon chord (GLB)."""
ELEVON_EFFECTIVENESS = 1.0
"""Times thin-aerofoil theory's plain flap: LOG's rolls (see the header)."""
FLEX_PA = 700.0
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
    cm0=0.008,  # calibrated: the reflex, trimming at 28 m/s
    cd0=0.012,  # EST: the section's profile drag
    alpha_stall=math.radians(12.5),  # calibrated: TBS's 35 km/h, CL max 0.74
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

# Torque roll. LOG, wings level with the stick centred for 3 to 17 s: INAV's roll I-term holds
# +10 to +40 below 8 A and -100 to -165 at 20 to 27 A, climbing, level or descending alike, and
# neither servo near its end; both servos sit about 180 µs higher at full power: about 9 deg of
# roll trim per elevon at TBS's throws. Its sign says the wing rolls right with power, as a prop
# turning counterclockwise seen from behind rolls it (torque_sign). Its size is 15 times what the
# motor's reaction needs here (0.11 N m at 27 A, from its KV: about 0.5 deg). Something else in
# that build rolls it with power (a canted motor? a warp?); unknown, so not modelled.
LIPO_CELL = (
    (0.00, 3.27),
    (0.05, 3.61),
    (0.10, 3.69),
    (0.20, 3.73),
    (0.30, 3.77),
    (0.40, 3.80),
    (0.50, 3.84),
    (0.60, 3.87),
    (0.70, 3.95),
    (0.80, 4.02),
    (0.90, 4.11),
    (1.00, 4.20),
)
"""EST: a LiPo cell's resting voltage against the charge left, the common hobby chart's: it falls
quickly from full, flat through the middle, and steeply in the last 10 %."""

POWER_TRAIN = PowerTrain(
    # The prop's coefficients and the resistance are fitted together (see the header).
    kv=2200,  # TBS
    resistance=0.095,  # fitted, with the pack's slow sag beside it
    no_load_current=1.0,  # EST
    rotor_inertia=6.0e-6,  # EST: 5 in tri-blade and a 20xx bell
    prop_diameter=5 * 0.0254,  # TBS: 5x3x3
    ct0=0.118,  # fitted: 16 N static on a full pack
    cp0=0.0651,  # fitted
    j_zero=0.661,  # fitted: about a 3.3 in pitch
    cells=6,  # TBS, PACK
    cell_curve=LIPO_CELL,
    capacity_mah=1300,  # PACK; TBS's 20 minute pack
    pack_resistance=0.06,  # EST: in LOG's pack's 0.050-0.062
    polarization_resistance=0.07,  # LOG's pack's (EST for ours)
    polarization_time=7.0,  # LOG's pack's 5-10 s (EST for ours)
    torque_sign=1.0,  # LOG: it rolls right with power (see Torque roll, above)
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
