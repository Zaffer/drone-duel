"""The wing as strips, each flying in its own air: aerodynamics over the full envelope.

Selig's method for aerobatic model aircraft (Selig 2010, AIAA 2010-7635; 2014, J. Aircraft 51(6)),
as X-Plane does it too: the wing is cut spanwise into strips, and each strip sees the air at its
own place, the aircraft's motion plus its rotation (v + ω × r), taken at the strip's 3/4-chord
point, which brings the section's own pitch damping. Less the downwash the whole wing's lift
induces there: a vortex lattice of the planform (Weissinger's: horseshoe vortices on the quarter
chord, flow tangent at the 3/4 chord), solved once, gives each strip's induced angle as a linear
function of all the strips' angles. Selig's induced angle is from lifting-line tables, tapered to
nothing at 90°; this tapers the same way. It sets the spanwise loading, so the lift slope, the
roll damping, the induced drag, and where along the span the stall begins.

Only the flow across a strip counts (the independence principle): sideslip over a swept strip,
and the part of it along the span, change its angle of attack and dynamic pressure, not its
section data. Each strip finds its angle of
attack and stalls on its own, so these come out of the geometry rather than being added:
  - roll damping, and autorotation once the down-going wing stalls: spins, snaps, wing drop;
  - pitch and roll damping at any airspeed, hovering and tail sliding included;
  - the elevons losing authority as their strips separate, and in reversed flow;
  - lift, drag and side force along the true flow direction at any angle of attack or sideslip.

Each strip's section data cover ±180°: attached flow linear in angle of attack, the elevon as a
plain flap (thin-aerofoil effectiveness); past the stall a flat plate whose normal force is
Lindenburg's broadside drag corrected to the wing's aspect ratio, acting from near the quarter
chord at the stall to mid-chord broadside, so lift over drag stays 1 / tan α; and, trailing edge
first, a weak reversed aerofoil that stalls early. Sigmoids blend them (Beard and McLain's form).
The fin is a strip of its own. The pod's drag acts at the centre of mass, and as a body of
revolution it turns broadside to the flow (Munk's moment, destabilising in pitch and yaw).

The flow over each strip takes time to separate and to reattach (Goman and Khrabrov's model,
`Flow`): a quickly rising angle of attack outruns the stall and lifts more for a moment (dynamic
stall), and once separated, the flow comes back only below the stall angle (hysteresis, at these
low Reynolds numbers a laminar separation bubble's). Near the ground the downwash weakens
(McCormick's ground effect, strip by strip at its own height, so a banked wing's low tip feels
it most): more lift and less induced drag.
"""

import math
from dataclasses import dataclass, field

from swarm.wingsim.aero import AirData, Geometry, air_data
from swarm.wingsim.rigidbody import Vec, add, cross, dot, scale, sub

REVERSED_STALL = math.radians(8)
"""Trailing edge first, the section holds attached flow only this far either side of 180°
(EST: a sharp trailing edge as leading edge stalls early; no data for this section)."""
DYNAMIC_STALL_MAX = math.radians(10)
"""The most a rising angle of attack can outrun the stall by, rad: Goman and Khrabrov's delay is
linear in the rate, real aerofoils' is not for long (EST, after Leishman's dynamic stall data)."""


@dataclass(frozen=True)
class Section:
    """A strip's aerofoil data, over the full ±180°."""

    lift_slope: float
    """Per rad: the wing's own lift slope, so the downwash is accounted for (strip theory)."""
    zero_lift: float
    """Angle of attack of zero lift, rad."""
    cm0: float
    """Pitching moment about the quarter chord in attached flow."""
    cd0: float
    alpha_stall: float
    """Attached flow up to here, rad, nose up."""
    alpha_stall_neg: float
    """And down to minus this, rad."""
    stall_sharpness: float
    """The blend's steepness, 1/rad."""
    cd90: float
    """Broadside drag coefficient, corrected to the wing's aspect ratio (Lindenburg)."""
    flap_lift: float = 0.0
    """The elevon's angle-of-attack effectiveness τ: dα per dδ."""
    flap_moment: float = 0.0
    """d cm (quarter chord) per dδ, rad: trailing edge down noses the section down."""
    flap_separated: float = 0.3
    """The fraction of τ still at work once the flow is separated (EST)."""
    flap_stall: float = 0.35
    """The fraction of the flap's angle (τ δ) by which it brings the stall closer: a flap down
    raises the section's maximum lift, by most of what it adds below the stall (EST)."""
    reverse_slope: float = 1.5
    """Lift slope trailing edge first, per rad (EST: Selig's reversed sections are weak)."""
    induced_drag: float = 0.0
    """1 / (π e AR) for a surface outside the wing's vortex lattice (a fin), whose lift slope is
    then its own three-dimensional one."""
    reattach: float = 0.0
    """Once separated, the flow reattaches only this far below the stall angle, rad: static
    hysteresis. 0: none."""
    separation_lag: float = 0.0
    """Goman and Khrabrov's τ1, in the times the air takes to cross the chord: how slowly the flow
    settles to separated or attached. 0: at once."""
    separation_delay: float = 0.0
    """Goman and Khrabrov's τ2, likewise: how long the flow takes to notice a changing angle of
    attack, so how far a rising one outruns the stall."""


@dataclass(frozen=True)
class Strip:
    """A spanwise piece of a lifting surface, its directions in the body frame (FRD) from the
    centre of mass."""

    quarter_chord: Vec
    """Where its forces act, m."""
    forward: Vec
    """Unit: along its chord, leading edge forward."""
    up: Vec
    """Unit: its lift side, normal to the chord and the span."""
    chord: float
    width: float
    """Spanwise, m."""
    section: Section
    elevon: int = -1
    """Which elevon it carries, 0 left, 1 right; -1 none."""
    elevon_fraction: float = 0.0
    """How much of its width the elevon spans."""
    stall_offset: float = 0.0
    """Added to its section's stall angles, rad: where along the span the stall starts."""
    twist: float = 0.0
    """Its incidence over the wing's, rad: washout, nose down, is negative."""

    @property
    def area(self) -> float:
        return self.chord * self.width

    @property
    def three_quarter_chord(self) -> Vec:
        return sub(self.quarter_chord, scale(self.forward, 0.5 * self.chord))

    @property
    def pitch_axis(self) -> Vec:
        """Unit: about which its pitching moment noses it up (toward its up side)."""
        return cross(self.forward, self.up)


@dataclass(frozen=True)
class Wing:
    """The whole aircraft's aerodynamics: its strips, the pod's drag, and how far the elevons
    twist the foam wing."""

    reference: Geometry
    strips: tuple[Strip, ...]
    body_cd: float
    """The pod's and everything else's drag, on the reference area, along the flow."""
    flex_pa: float
    """Dynamic pressure, Pa, at which the wing's twist under the elevons' load halves their
    effect: foam wings lose control authority with speed. 0: rigid."""
    body_volume: float = 0.0
    """The pod's, m³, for its Munk moment."""
    body_munk: float = 0.0
    """The pod's k2 - k1, from its fineness ratio (Munk's apparent-mass factors)."""
    loading: tuple[tuple[float, ...], ...] = ()
    """The vortex lattice's answer for the first len(loading) strips: each strip's angle of
    attack once the downwash is taken off, per radian of every strip's (lifting_line)."""


@dataclass
class Flow:
    """What each strip's flow remembers from one step to the next: the share of it attached, and
    the angle of attack it last saw, for how fast that changes."""

    attached: list[float] = field(default_factory=list)
    alpha: list[float] = field(default_factory=list)

    def forget(self) -> None:
        """Settle afresh at the next step."""
        self.attached.clear()
        self.alpha.clear()


@dataclass(frozen=True, slots=True)
class StripLoad:
    """One strip's flow and load at a step: rad, Pa, N (body frame)."""

    alpha: float
    """Its own angle of attack, the twist included, before the downwash."""
    downwash: float
    """Taken off `alpha` by the rest of the wing's lift; ground effect included."""
    ground_effect: float
    """The share of the downwash the ground leaves (1: far from it)."""
    lead: float
    """How far the stall has fallen behind a changing angle of attack (dynamic stall)."""
    flap: float
    """Its elevon's effective deflection, over the share of it the elevon spans."""
    attached: float
    """The share of its flow attached."""
    cl: float
    cd: float
    cm: float
    dynamic_pressure: float
    force: Vec
    air: Vec = (0.0, 0.0, 0.0)
    """The air coming past it, m/s: its own motion's and the aircraft's rotation's (at its 3/4
    chord), the part across its span turned by the downwash. (Its section feels only that part.)"""


@dataclass
class Loads:
    """The aerodynamics piece by piece, as `forces_and_moments` found them: each strip's, the
    elevons after the wing's twist (rad), the pod's drag (N) and Munk moment (N m)."""

    strips: tuple[StripLoad, ...] = ()
    elevons: tuple[float, float] = (0.0, 0.0)
    pod_drag: Vec = (0.0, 0.0, 0.0)
    munk: Vec = (0.0, 0.0, 0.0)


def _sigmoid(x: float) -> float:
    if x > 40:
        return 1.0
    if x < -40:
        return 0.0
    return 1 / (1 + math.exp(-x))


def attachment(
    sec: Section, alpha: float, flap: float, stall_offset: float = 0.0, separated: bool = False
) -> float:
    """The share of the section's flow attached once settled at angle of attack `alpha` (-π..π)
    with the flap at `flap`; `separated`: coming from separated flow, which reattaches lower."""
    m = sec.stall_sharpness
    if abs(alpha) <= math.pi / 2:
        loading = alpha + sec.flap_stall * sec.flap_lift * flap
        lower = sec.reattach if separated else 0.0
        up = sec.alpha_stall + stall_offset - lower
        down = sec.alpha_stall_neg + stall_offset - lower
        return (1 - _sigmoid(m * (loading - up))) * (1 - _sigmoid(-m * (loading + down)))
    backwards = alpha - math.copysign(math.pi, alpha)
    return (1 - _sigmoid(m * (backwards - REVERSED_STALL))) * (
        1 - _sigmoid(-m * (backwards + REVERSED_STALL))
    )


def section_coefficients(
    sec: Section,
    alpha: float,
    flap: float,
    stall_offset: float = 0.0,
    attached: float | None = None,
) -> tuple[float, float, float, float]:
    """Lift, drag and quarter-chord moment coefficients at angle of attack `alpha` (-π..π) with
    the flap at `flap` (rad, trailing edge down +), and the attached-flow lift alone, for the
    induced drag. `attached`: the share of the flow attached (default: settled, `attachment`)."""
    if attached is None:
        attached = attachment(sec, alpha, flap, stall_offset)
    separated = 1 - attached
    sa, ca = math.sin(alpha), math.cos(alpha)
    if abs(alpha) <= math.pi / 2:
        effective = alpha + sec.flap_lift * flap
        cl_att = sec.lift_slope * (effective - sec.zero_lift)
        cm_att = sec.cm0 + sec.flap_moment * flap
        centre = 0.25 + 0.25 * abs(sa)
    else:
        backwards = alpha - math.copysign(math.pi, alpha)
        cl_att = sec.reverse_slope * backwards
        cm_att = -0.5 * cl_att  # its lift acts near the 3/4 chord, now the leading quarter
        centre = 0.75 - 0.25 * abs(sa)
    normal = sec.cd90 * math.sin(alpha + sec.flap_separated * sec.flap_lift * flap)
    cl = attached * cl_att + separated * normal * ca
    cd = sec.cd0 + separated * normal * sa
    cm = attached * cm_att - separated * normal * (centre - 0.25)
    return cl, cd, cm, attached * cl_att


def elevon_deflections(
    wing: Wing, elevons: tuple[float, float], qbar: float
) -> tuple[float, float]:
    """The elevons' effective deflections, rad, after the wing's twist under their load."""
    if wing.flex_pa <= 0:
        return elevons
    k = 1 / (1 + qbar / wing.flex_pa)
    return elevons[0] * k, elevons[1] * k


def _settle(
    strip: Strip, alpha: float, flap: float, speed: float, flow: Flow, i: int, dt: float
) -> tuple[float, float]:
    """Goman and Khrabrov: strip `i`'s attached share relaxes toward the settled one, at an angle
    of attack lagging the strip's own by its delay. Returns the share, and that lag, rad."""
    sec = strip.section
    chord_time = strip.chord / max(speed, 0.5)
    rate = ((alpha - flow.alpha[i] + math.pi) % (2 * math.pi) - math.pi) / dt
    ahead = max(
        -DYNAMIC_STALL_MAX, min(DYNAMIC_STALL_MAX, sec.separation_delay * chord_time * rate)
    )
    attached = flow.attached[i]
    target = attachment(sec, alpha - ahead, flap, strip.stall_offset, separated=attached < 0.5)
    lag = sec.separation_lag * chord_time
    if lag > 0:
        target += (attached - target) * math.exp(-dt / lag)
    flow.attached[i], flow.alpha[i] = target, alpha
    return target, ahead


def _ground_effect(height: float, span: float) -> float:
    """McCormick's: the share of the downwash left at `height` above the ground, m. His is a
    whole wing's induced drag; taking it strip by strip, each at its own height, is ours (EST)."""
    x = 16 * max(0.0, height) / span
    return x * x / (1 + x * x)


def forces_and_moments(
    wing: Wing,
    v_body: Vec,
    rates: Vec,
    elevons: tuple[float, float],
    rho: float,
    flow: Flow | None = None,
    dt: float = 0.0,
    ground: tuple[float, Vec] | None = None,
    loads: Loads | None = None,
) -> tuple[Vec, Vec, AirData]:
    """Body-frame aerodynamic force (N) and moment (N m) about the centre of mass, the air's
    velocity being `v_body` (the aircraft's through the air), its rotation `rates`; elevons in
    rad, trailing edge down positive. `flow`: the strips' memory, from the step `dt` s ago
    (default: none, the flow settled). `ground`: the centre of mass's height above flat ground,
    m, and the body-frame unit vector up from it (default: none near). `loads`: filled in with
    the pieces, to look into them."""
    ad = air_data(v_body, rho)
    deflection = elevon_deflections(wing, elevons, ad.dynamic_pressure)
    # Each strip's flow, angle of attack and flap.
    flows: list[tuple[float, float, float, float]] = []
    for s in wing.strips:
        v = add(v_body, cross(rates, s.three_quarter_chord))
        u = dot(v, s.forward)
        w = -dot(v, s.up)
        flap = deflection[s.elevon] * s.elevon_fraction if s.elevon >= 0 else 0.0
        alpha = math.atan2(w, u) + s.twist if u * u + w * w > 1e-12 else 0.0
        flows.append((u, w, (alpha + math.pi) % (2 * math.pi) - math.pi, flap))
    # The downwash: the vortex lattice, linear in the angles from zero lift, flap included;
    # tapered to nothing broadside on and beyond.
    loaded = [
        a + st.section.flap_lift * flap - st.section.zero_lift
        for (_, _, a, flap), st in zip(flows, wing.strips, strict=False)
    ]
    downwash = [0.0] * len(wing.strips)
    near = [1.0] * len(wing.strips)
    for i, row in enumerate(wing.loading):
        alpha = flows[i][2]
        taper = math.cos(alpha) ** 2 if abs(alpha) < math.pi / 2 else 0.0
        downwash[i] = taper * (loaded[i] - sum(k * a for k, a in zip(row, loaded, strict=False)))
        if ground is not None:
            height, up = ground
            near[i] = _ground_effect(
                height + dot(wing.strips[i].quarter_chord, up), wing.reference.span
            )
            downwash[i] *= near[i]
    # What each strip's section sees.
    seen = [
        (a - eps + math.pi) % (2 * math.pi) - math.pi
        for (_, _, a, _), eps in zip(flows, downwash, strict=False)
    ]
    memory = flow if dt > 0 else None
    if memory is not None and len(memory.attached) != len(wing.strips):
        memory.attached = [
            attachment(s.section, a, f, s.stall_offset)
            for s, a, (_, _, _, f) in zip(wing.strips, seen, flows, strict=False)
        ]
        memory.alpha = seen[:]
    fx = fy = fz = mx = my = mz = 0.0
    pieces: list[StripLoad] = []
    for i, (s, (u, w, a, flap), eps) in enumerate(zip(wing.strips, flows, downwash, strict=False)):
        speed2 = u * u + w * w
        speed = math.sqrt(speed2)
        if memory is None:
            attached, lead = attachment(s.section, seen[i], flap, s.stall_offset), 0.0
        else:
            attached, lead = _settle(s, seen[i], flap, speed, memory, i, dt)
        if speed2 < 1e-6:
            if loads is not None:
                still = StripLoad(a, eps, near[i], lead, flap, attached, 0, 0, 0, 0, (0, 0, 0))
                pieces.append(still)
            continue
        cl, cd, cm, cl_att = section_coefficients(
            s.section, seen[i], flap, s.stall_offset, attached
        )
        cd += cl_att * cl_att * s.section.induced_drag
        q_area = 0.5 * rho * speed2 * s.area
        # Lift across the strip's flow, toward its up side, drag along it; both turned back by the
        # downwash, which so brings the induced drag. (The flow's own angle: the twist is the
        # section's, not the flow's.)
        f, n = s.forward, s.up
        ce, se = math.cos(eps), math.sin(eps)
        lu, lw = (u * ce + w * se) / speed, (w * ce - u * se) / speed
        lift_dir = (lw * f[0] + lu * n[0], lw * f[1] + lu * n[1], lw * f[2] + lu * n[2])
        drag_dir = (lw * n[0] - lu * f[0], lw * n[1] - lu * f[1], lw * n[2] - lu * f[2])
        force = add(scale(lift_dir, q_area * cl), scale(drag_dir, q_area * cd))
        moment = add(cross(s.quarter_chord, force), scale(s.pitch_axis, q_area * s.chord * cm))
        if loads is not None:
            # Across the span, the air goes the drag's way; along it, as the strip meets it.
            v = add(v_body, cross(rates, s.three_quarter_chord))
            along = sub(scale(v, -1), add(scale(f, -u), scale(n, w)))
            air = add(scale(drag_dir, speed), along)
            load = StripLoad(
                a, eps, near[i], lead, flap, attached, cl, cd, cm, q_area / s.area, force, air
            )
            pieces.append(load)
        fx += force[0]
        fy += force[1]
        fz += force[2]
        mx += moment[0]
        my += moment[1]
        mz += moment[2]
    pod_drag = munk_moment = (0.0, 0.0, 0.0)
    if ad.airspeed > 1e-3:
        body_drag = ad.dynamic_pressure * wing.reference.area * wing.body_cd / ad.airspeed
        pod_drag = scale(v_body, -body_drag)
        fx += pod_drag[0]
        fy += pod_drag[1]
        fz += pod_drag[2]
        # Munk: q Vol (k2 - k1) sin 2α, nose further up; and yawing away from the sideslip.
        munk = 0.5 * rho * wing.body_volume * wing.body_munk
        u, v, w = v_body
        munk_moment = (0.0, munk * 2 * u * w, -munk * 2 * u * v)
        my += munk_moment[1]
        mz += munk_moment[2]
    if loads is not None:
        loads.strips = tuple(pieces)
        loads.elevons = deflection
        loads.pod_drag = pod_drag
        loads.munk = munk_moment
    return (fx, fy, fz), (mx, my, mz), ad


@dataclass(frozen=True)
class Planform:
    """One half of a wing, seen from above: `stations` are (y out from the centre line, leading
    edge x, trailing edge x), x forward from any datum, m; `cuts`, the strips' edges, y from the
    centre line to the tip. `dihedral` raises the tips, rad."""

    stations: tuple[tuple[float, float, float], ...]
    cuts: tuple[float, ...]
    dihedral: float = 0.0

    def edges(self, y: float) -> tuple[float, float]:
        """Leading and trailing edge x at `y`, linearly between stations."""
        st = self.stations
        for (y0, le0, te0), (y1, le1, te1) in zip(st, st[1:], strict=False):
            if y <= y1 or (y1, le1, te1) == st[-1]:
                k = (y - y0) / (y1 - y0)
                return le0 + k * (le1 - le0), te0 + k * (te1 - te0)
        raise ValueError(y)

    def quarter_chord(self, y: float) -> float:
        le, te = self.edges(y)
        return le - 0.25 * (le - te)


def flap_effectiveness(chord_ratio: float) -> tuple[float, float]:
    """Thin-aerofoil theory for a plain flap of `chord_ratio` (flap chord over chord): the angle
    of attack per radian of flap, and the quarter-chord moment coefficient per radian."""
    hinge = math.acos(2 * chord_ratio - 1)
    tau = 1 - (hinge - math.sin(hinge)) / math.pi
    cm = -0.5 * math.sin(hinge) * (1 - math.cos(hinge))
    return tau, cm


def wing_strips(
    planform: Planform,
    section: Section,
    centre_of_mass: tuple[float, float],
    elevon: tuple[float, float, float],
    flap_efficiency: float,
    stall_spread: float = 0.0,
    washout: float = 0.0,
) -> tuple[Strip, ...]:
    """Both halves' strips, left then right. `centre_of_mass`: (x, height above the wing's
    root), m; `elevon`: its inner and outer y and its chord, m; `flap_efficiency`: the share of
    thin-aerofoil theory a real hinged elevon delivers; `stall_spread`: the stall angle's offset
    at the tip, rad, and minus it at the root; `washout`: the tip's twist nose down from the root,
    rad, linear between."""
    cg_x, cg_h = centre_of_mass
    tip = planform.cuts[-1]
    tan_dihedral = math.tan(planform.dihedral)
    strips: list[Strip] = []
    for side in (-1, 1):
        for y0, y1 in zip(planform.cuts, planform.cuts[1:], strict=False):
            ym = (y0 + y1) / 2
            le, te = planform.edges(ym)
            chord = le - te
            dx = planform.quarter_chord(y1) - planform.quarter_chord(y0)
            dy = y1 - y0
            dz = -dy * tan_dihedral
            run = math.hypot(dx, dy)
            forward = (dy / run, -side * dx / run, 0.0)
            along = (dx, side * dy, dz)
            length = math.sqrt(dx * dx + dy * dy + dz * dz)
            along = (along[0] / length, along[1] / length, along[2] / length)
            up = cross(along, forward) if side > 0 else cross(forward, along)
            covered = max(0.0, min(y1, elevon[1]) - max(y0, elevon[0])) / dy
            tau, cm = flap_effectiveness(min(0.9, elevon[2] / chord)) if covered else (0.0, 0.0)
            strip_section = Section(
                **{
                    **section.__dict__,
                    "flap_lift": tau * flap_efficiency,
                    "flap_moment": cm * flap_efficiency,
                }
            )
            strips.append(
                Strip(
                    quarter_chord=(
                        planform.quarter_chord(ym) - cg_x,
                        side * ym,
                        cg_h - ym * tan_dihedral,
                    ),
                    forward=forward,
                    up=up,
                    chord=chord * dy / run,
                    width=length,
                    section=strip_section,
                    elevon=0 if side < 0 else 1,
                    elevon_fraction=covered,
                    stall_offset=stall_spread * (2 * ym / tip - 1),
                    twist=-washout * ym / tip,
                )
            )
    return tuple(strips)


def lifting_line(strips: tuple[Strip, ...]) -> tuple[tuple[float, ...], ...]:
    """Weissinger's vortex lattice for `strips`, a planar wing seen from above: a horseshoe
    vortex on each strip's quarter chord, its legs trailing downstream, the flow tangent half a
    chord straight behind each one's middle (the 3/4 chord). Returns the matrix turning the strips'
    angles of attack (from zero lift) into the angles their sections see once the downwash is
    off: lift over 2π."""
    n = len(strips)
    far = 1e3
    bound: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for s in strips:
        half = s.width / 2
        span = cross(s.up, s.forward)
        a = (s.quarter_chord[0] - span[0] * half, s.quarter_chord[1] - span[1] * half)
        b = (s.quarter_chord[0] + span[0] * half, s.quarter_chord[1] + span[1] * half)
        bound.append((a, b) if a[1] < b[1] else (b, a))

    def segment(p: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
        """Downward velocity at `p` from a unit vortex filament a to b, all in the wing's plane."""
        r1 = (p[0] - a[0], p[1] - a[1])
        r2 = (p[0] - b[0], p[1] - b[1])
        r0 = (b[0] - a[0], b[1] - a[1])
        crossz = r1[0] * r2[1] - r1[1] * r2[0]
        if abs(crossz) < 1e-12:
            return 0.0
        l1, l2 = math.hypot(*r1), math.hypot(*r2)
        along = (r0[0] * r1[0] + r0[1] * r1[1]) / l1 - (r0[0] * r2[0] + r0[1] * r2[1]) / l2
        return crossz / (crossz * crossz) * along / (4 * math.pi)

    # Streamwise chords; each control point half of one behind its bound vortex's middle.
    chord = [s.chord * s.width / abs(b[1] - a[1]) for s, (a, b) in zip(strips, bound, strict=False)]
    controls = [
        (s.quarter_chord[0] - c / 2, s.quarter_chord[1])
        for s, c in zip(strips, chord, strict=False)
    ]
    influence = [[0.0] * n for _ in range(n)]
    for j, (a, b) in enumerate(bound):
        a_far, b_far = (a[0] - far, a[1]), (b[0] - far, b[1])
        for i, p in enumerate(controls):
            influence[i][j] = segment(p, a_far, a) + segment(p, a, b) + segment(p, b, b_far)
    # Circulation for unit angles, per unit airspeed: influence · Γ = α.
    inverse = _invert(influence)
    # Lift per unit span ρVΓ, so the section's lift coefficient 2Γ / (V c); over 2π.
    return tuple(tuple(2 * k / (chord[i] * 2 * math.pi) for k in inverse[i]) for i in range(n))


def _invert(m: list[list[float]]) -> list[list[float]]:
    """Gauss-Jordan, with partial pivoting."""
    n = len(m)
    a = [row[:] + [1.0 if i == j else 0.0 for j in range(n)] for i, row in enumerate(m)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(a[r][col]))
        a[col], a[pivot] = a[pivot], a[col]
        k = a[col][col]
        a[col] = [x / k for x in a[col]]
        for r in range(n):
            if r != col and a[r][col]:
                f = a[r][col]
                a[r] = [x - f * y for x, y in zip(a[r], a[col], strict=False)]
    return [row[n:] for row in a]
