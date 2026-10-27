// The flight model at work (swarm.wingsim.dynamics, in each /wing frame under `swarm wingsim`).
// On the wing, five layers, each on its number key and any together, a legend of what is on
// beside them:
//   1 Forces          what pushes it: each strip's force from its quarter chord, coloured by how
//                     much of its flow is attached (green attached, red separated); the prop's,
//                     the pod's drag, gravity, the whole aerodynamic force and the net force.
//   2 Air             what it flies through: the airflow at it (α, β), the air coming past each
//                     strip, the wind; and traced where the wing has been in the air, the
//                     trailing vortices its lift leaves, and smoke off the fin and elevons.
//   3 Lift & balance  where the wing works: each strip's patch tinted by its flow; on the root
//                     chord the centre of mass, the centre of pressure and the neutral point.
//   4 Rotation        why it turns and how fast: the net moment about each axis (thick arcs)
//                     and the rotation rate (thin).
//   5 Path            where it has been: a trail, coloured by load factor, airspeed, attached
//                     flow or energy gain (Shift+5 changes which).
// Apart from them (D): every number the model works with, the strips across the span, and in a
// panel of its own, the last few seconds of the ones that move most.

const C = Cesium;

/** Arrow lengths at scale 1: m per N for each strip's force, and for the whole aircraft's. */
const STRIP_M_PER_N = 0.25;
const WHOLE_M_PER_N = 0.06;
const WIND_M_PER_MS = 0.15;
/** Each strip's airflow arrow at scale 1, m per m/s: it ends at the strip's quarter chord. */
const FLOW_M_PER_MS = 0.02;
const FLOW_PX = 6;
/** The airflow's arrow: this long, stopping this far ahead of the centre of mass, m. */
const AIRFLOW = { length: 0.8, gap: 0.5 };
/** The traces: each point left in the air where the wing shed it, carried off with the wind and
 * the downwash, for `seconds`; a jump further than `jump`, m, between frames is a reset. The
 * vortices are drawn most of `wakePx` wide at a strength of `strength`, m²/s. */
const TRACE = { seconds: 0.8, jump: 5, px: 2, wakePx: 9, strength: 0.25 };
/** The tinted patches: a hatch line every this far along the span, m, this far over the wing. */
const HATCH = { every: 0.012, above: 0.015, px: 4 };
/** The root chord's ruler, this far above the centre of mass, m; each mark rises from it this
 * far, its label this many pixels off its top (CG's below the ruler): they are millimetres apart. */
const RULER = { above: 0.07, marks: { cg: 0.02, cp: 0.04, np: 0.04 }, labels: { cg: 26, cp: -3, np: -21 } };
/** The rotation arcs: their sweep for a moment of `moment` N m or a rate of `rate` °/s is most of
 * `max`; about each axis (body x, y, z), where, how big, and where each starts. */
const ARC = { max: C.Math.toRadians(270), moment: 0.3, rate: 180, steps: 36, px: 8, ratePx: 4 };
const ARCS = [
  { e1: [0, 1, 0], e2: [0, 0, 1], centre: [0.42, 0, 0], radius: 0.16, start: -Math.PI / 2 },
  { e1: [0, 0, 1], e2: [1, 0, 0], centre: [-0.05, 0.56, 0], radius: 0.16, start: Math.PI / 2 },
  { e1: [1, 0, 0], e2: [0, 1, 0], centre: [0, 0, -0.22], radius: 0.22, start: 0 },
];
/** The trail: how long it lasts, s; a point every so often, ms; a jump this far, m, is a reset. */
const TRAIL = { seconds: 30, every: 50, jump: 30, px: 5 };
const ARROW_PX = 9;
const SCALE_STEP = 1.5;
/** Every arrow's pick id: the page leaves them out when it finds the ground, or the wing would fly
 * into its own arrows. */
export const ARROW_ID = "wing dynamics arrow";
const PANEL_MS = 100;
const HISTORY_S = 10;
/** The spanwise chart's ranges: the strips' angles of attack, deg, and their lift coefficients. */
const SPAN_ALPHA = [-15, 30];
const SPAN_CL = [-0.6, 1.4];

const COLORS = {
  ok: "#3ddc84",
  warn: "#ffb020",
  bad: "#ff5a5a",
  prop: "#7fe3ff",
  pod: "#c792ff",
  gravity: "#d0d6dc",
  aero: "#ffc53d",
  net: "#ff6bd5",
  airflow: "#9ad0ff",
  flow: "#5fa8ff",
  wind: "#b8f5c8",
  vortex: "#c9b6ff",
  smoke: "#f4f1ea",
  cg: "#ffffff",
  cp: "#ffc53d",
  np: "#5ce1e6",
  rate: "#e8edf2",
};
const ATTACHED = `linear-gradient(90deg, ${COLORS.bad}, ${COLORS.warn}, ${COLORS.ok})`;

/** The layers, by number key: what each shows, and its legend. */
const LAYERS = [
  {
    key: "forces",
    name: "Forces",
    legend: [
      [ATTACHED, "strips: separated → attached"],
      [COLORS.aero, "aero (strips + pod)"],
      [COLORS.net, "net, gravity in"],
      [COLORS.prop, "prop"],
      [COLORS.pod, "pod drag"],
      [COLORS.gravity, "gravity"],
    ],
  },
  {
    key: "air",
    name: "Air",
    legend: [
      [COLORS.airflow, "airflow (α, β)"],
      [COLORS.flow, "air past each strip"],
      [COLORS.wind, "wind"],
      [COLORS.vortex, `trailing vortices, last ${TRACE.seconds} s (width: strength)`],
      [COLORS.smoke, "smoke off the fin and the elevons"],
    ],
  },
  {
    key: "lift",
    name: "Lift & balance",
    legend: [
      [ATTACHED, "wing: separated → attached"],
      [COLORS.cg, "CG centre of mass"],
      [COLORS.cp, "CP centre of pressure"],
      [COLORS.np, "NP neutral point: behind CG, stable"],
    ],
  },
  {
    key: "rotation",
    name: "Rotation",
    legend: [
      [COLORS.net, "net moment (thick)"],
      [COLORS.rate, "rotation rate (thin)"],
    ],
  },
  { key: "path", name: "Path", legend: [] },
];

/** What the trail can be coloured by: the value from a frame's dynamics, and its colour stops. */
const TRAIL_MODES = [
  { name: "load factor", unit: "g", of: (d) => -d.load[2], stops: [[0, "#4c8dff"], [1, "#e8edf2"], [4, "#ffb020"], [8, "#ff5a5a"]] },
  { name: "airspeed", unit: "m/s", of: (d) => d.airspeed, stops: [[0, "#4c8dff"], [15, "#3ddc84"], [30, "#ffb020"], [45, "#ff5a5a"]] },
  { name: "least attached", unit: "%", of: (d) => leastAttached(d) * 100, stops: [[0, COLORS.bad], [50, COLORS.warn], [100, COLORS.ok]] },
  { name: "energy gain", unit: "m/s", of: (d) => d.energy_rate, stops: [[-10, "#ff5a5a"], [0, "#8b98a5"], [10, "#3ddc84"]] },
];

const leastAttached = (d) => (d.strips.length ? Math.min(...d.strips.map((s) => s.attached)) : NaN);

/** `stops`' colour at `v`, between them. */
function ramp(stops, v) {
  if (!Number.isFinite(v)) return C.Color.GRAY;
  const color = (k) => C.Color.fromCssColorString(stops[k][1]);
  if (v <= stops[0][0]) return color(0);
  for (let k = 1; k < stops.length; k++) {
    if (v <= stops[k][0]) {
      const t = (v - stops[k - 1][0]) / (stops[k][0] - stops[k - 1][0]);
      return C.Color.lerp(color(k - 1), color(k), t, new C.Color());
    }
  }
  return color(stops.length - 1);
}

/** Body (forward, right, down) to local east-north-up, for a wing with `axes` there; times `k`. */
function toLocal(axes, v, k = 1) {
  const { right, up, nose } = axes;
  return new C.Cartesian3(
    k * (v[0] * nose.x + v[1] * right.x - v[2] * up.x),
    k * (v[0] * nose.y + v[1] * right.y - v[2] * up.y),
    k * (v[0] * nose.z + v[1] * right.z - v[2] * up.z),
  );
}

/** `line` from `from` along `vector` (local), hidden when too short to see. */
function place(line, from, vector) {
  const m = C.Cartesian3.magnitude(vector);
  line.show = m > 0.005;
  if (line.show) line.positions = [from, C.Cartesian3.add(from, vector, new C.Cartesian3())];
}

const nullable = (x, k = 1) => (x === null || x === undefined ? NaN : x * k);

/** Green attached, through amber, to red separated. */
function attachedColor(share) {
  const a = C.Color.fromCssColorString(COLORS.bad);
  const m = C.Color.fromCssColorString(COLORS.warn);
  const b = C.Color.fromCssColorString(COLORS.ok);
  const s = Math.max(0, Math.min(1, share));
  return s < 0.5 ? C.Color.lerp(a, m, s * 2, new C.Color()) : C.Color.lerp(m, b, (s - 0.5) * 2, new C.Color());
}

const fmt = (x, digits) => {
  if (!Number.isFinite(x)) return "–";
  const text = x.toFixed(digits);
  return /^-0(\.0*)?$/.test(text) ? text.slice(1) : text;
};

/** Every number, by section: its column heads, and rows of a label, the values from the frame's
 * dynamics `d` and wing `w`, and their decimals. */
const SECTIONS = [
  {
    title: "AIR",
    cols: [],
    rows: [
      ["airspeed m/s", (d) => [d.airspeed], 1],
      ["α · β °", (d) => [d.alpha, d.beta], 1],
      ["q̄ Pa", (d) => [d.dynamic_pressure], 0],
      ["CL · CD · L/D", (d) => [d.coefficients[0], d.coefficients[1], d.coefficients[0] / d.coefficients[1]], [3, 3, 1]],
      ["CY", (d) => [d.coefficients[2]], 3],
      ["Cl · Cm · Cn", (d) => d.coefficients.slice(3), 4],
    ],
  },
  {
    title: "WIND m/s",
    cols: ["N", "E", "D"],
    rows: [
      ["at the wing", (d) => d.wind, 2],
      ["mean, sheared", (d) => d.wind_mean, 2],
      ["turbulence", (d) => d.turbulence, 2],
      ["gust", (d) => d.gust, 2],
      ["turb. pqr °/s", (d) => d.turbulence_rates, 1],
    ],
  },
  {
    title: "FORCES N",
    cols: ["fwd", "right", "down"],
    rows: [
      ["aero", (d) => d.aero, 2],
      ["· pod drag", (d) => d.pod_drag, 2],
      ["prop", (d) => d.prop, 2],
      ["gravity", (d) => d.gravity, 2],
      ["net", (d) => d.total, 2],
      ["accel. g", (d) => d.load, 2],
      ["load factor n", (d) => [-d.load[2]], 2],
    ],
  },
  {
    title: "MOMENTS N·m",
    cols: ["roll", "pitch", "yaw"],
    rows: [
      ["aero", (d) => d.aero_moment, 3],
      ["· Munk (pod)", (d) => d.munk, 3],
      ["prop + motor", (d) => d.prop_moment, 3],
      ["gyroscopic", (d) => d.gyroscopic, 4],
      ["net", (d) => d.total_moment, 3],
      ["rates °/s", (d) => d.rates, 0],
      ["ang. acc. °/s²", (d) => d.angular_acceleration, 0],
    ],
  },
  {
    title: "ELEVONS °",
    cols: ["left", "right"],
    rows: [
      ["INAV's servo", (d) => d.servos, 1],
      ["where it is", (d) => d.elevons, 1],
      ["after twist", (d) => d.effective, 1],
      ["twist leaves %", (d) => [d.flex * 100], 0],
    ],
  },
  {
    title: "POWER",
    cols: [],
    rows: [
      ["throttle %", (d, w) => [w.motor * 100], 0],
      ["rpm", (d) => [d.rpm], 0],
      ["J · CT · CP", (d) => [d.advance_ratio, d.ct, d.cp], 3],
      ["thrust N", (d) => [d.thrust], 2],
      ["torque mot · prop", (d) => [d.motor_torque, d.prop_torque], 3],
      ["motor A", (d) => [d.motor_amps], 1],
      ["pack V · A · W", (d) => [d.pack_volts, d.pack_amps, d.pack_volts * d.pack_amps], [2, 1, 0]],
      ["used mAh", (d) => [d.used_mah], 0],
    ],
  },
  {
    title: "ENERGY",
    cols: [],
    rows: [
      ["height m", (d) => [d.height], 1],
      ["energy height m", (d) => [d.energy_height], 1],
      ["energy gain m/s", (d) => [d.energy_rate], 2],
    ],
  },
  {
    title: "BALANCE",
    cols: [],
    rows: [
      ["CP · NP mm fwd", (d) => [nullable(d.centre_of_pressure, 1000), nullable(d.neutral_point, 1000)], 0],
      ["static margin %", (d) => [nullable(d.static_margin, 100)], 1],
    ],
  },
];

/** The strip table's columns: head, unit, value, decimals. */
const STRIP_COLS = [
  ["α", "°", (s) => s.alpha, 1],
  ["ε", "°", (s) => s.downwash, 1],
  ["α eff", "°", (s) => s.seen, 1],
  ["stall", "°", (s) => s.stall, 1],
  ["lead", "°", (s) => s.lead, 1],
  ["flap", "°", (s) => s.flap, 1],
  ["att", "%", (s) => s.attached * 100, 0],
  ["CL", "", (s) => s.cl, 2],
  ["CD", "", (s) => s.cd, 3],
  ["Cm", "", (s) => s.cm, 3],
  ["q", "Pa", (s) => s.q, 0],
  ["GE", "%", (s) => s.ground_effect * 100, 0],
];

/** The history's lanes: label, value from the frame, range, decimals, and a mark to hold it to. */
const LANES = [
  ["α ° · stall", (d) => d.alpha, [-10, 30], 1, (d) => (d.strips.length ? Math.min(...d.strips.map((s) => s.stall)) : NaN)],
  ["CL", (d) => d.coefficients[0], [-0.5, 1.5], 2],
  ["n g", (d) => -d.load[2], [-2, 8], 1],
  ["least attached %", (d) => (d.strips.length ? Math.min(...d.strips.map((s) => s.attached)) * 100 : NaN), [0, 100], 0],
  ["airspeed m/s", (d) => d.airspeed, [0, 50], 1],
];

const el = (tag, cls, text) => {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text !== undefined) e.textContent = text;
  return e;
};

/** The wing's strips, named as seen from behind: L6 (left tip) to L1, R1 to R6, then the fin. */
function nameStrips(strips) {
  const named = strips.map((s, i) => ({ s, i, fin: Math.abs(s.up[1]) > 0.5 }));
  const wing = named.filter((n) => !n.fin).sort((a, b) => a.s.at[1] - b.s.at[1]);
  const left = wing.filter((n) => n.s.at[1] < 0);
  left.forEach((n, k) => (n.name = `L${left.length - k}`));
  wing.filter((n) => n.s.at[1] >= 0).forEach((n, k) => (n.name = `R${k + 1}`));
  const fin = named.filter((n) => n.fin).map((n) => ({ ...n, name: "fin" }));
  return [...wing, ...fin];
}

export class DynamicsView {
  /** On `viewer`'s map, in the east-north-up frame `localToFixed` turns into the globe's; the
   * numbers in `root`, the history in `historyRoot`, the layers' legend in `legendRoot`. */
  constructor(viewer, localToFixed, root, historyRoot, legendRoot) {
    this.root = root;
    this.historyRoot = historyRoot;
    this.legendRoot = legendRoot;
    this.panel = false;
    this.layers = Object.fromEntries(LAYERS.map((l) => [l.key, false]));
    this.scale = 1;
    const add = (primitive) => viewer.scene.primitives.add(primitive);
    this.lines = Object.fromEntries(
      LAYERS.map((l) => [l.key, add(new C.PolylineCollection({ modelMatrix: localToFixed, show: false }))]),
    );
    this.labels = add(new C.LabelCollection({ modelMatrix: localToFixed, show: false }));
    const color = (key) => C.Color.fromCssColorString(COLORS[key]);
    this.forces = Object.fromEntries(["aero", "net", "prop", "pod", "gravity"].map((k) => [k, this._arrow("forces", color(k))]));
    this.stripArrows = [];
    this.air = { airflow: this._arrow("air", color("airflow")), wind: this._arrow("air", color("wind")) };
    this.flowArrows = [];
    /** Each trace by its source: its line, and its points {at, t, drift} in the air. */
    this.traces = new Map();
    this.hatches = null;
    this.ruler = this._line("lift", C.Color.WHITE.withAlpha(0.7), 2);
    this.marks = Object.fromEntries(
      ["cg", "cp", "np"].map((k) => [
        k,
        {
          line: this._line("lift", color(k), 3),
          label: this.labels.add({
            id: ARROW_ID,
            text: k.toUpperCase(),
            font: "12px ui-monospace, monospace",
            fillColor: color(k),
            showBackground: true,
            backgroundColor: new C.Color(0, 0, 0, 0.55),
            horizontalOrigin: C.HorizontalOrigin.CENTER,
            verticalOrigin: k === "cg" ? C.VerticalOrigin.TOP : C.VerticalOrigin.BOTTOM,
            pixelOffset: new C.Cartesian2(0, RULER.labels[k]),
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
            show: false,
          }),
        },
      ]),
    );
    this.arcs = ARCS.map(() => ({
      moment: this._arrow("rotation", color("net"), ARC.px),
      rate: this._arrow("rotation", color("rate"), ARC.ratePx),
    }));
    this.trailMode = 0;
    this._trail = [];
    this._trailAt = 0;
    this._at = 0;
    this._history = [];
    this._lastFrame = null;
    this._build();
    this._buildLegend();
  }

  _arrow(layer, color, width = ARROW_PX) {
    return this.lines[layer].add({
      positions: [C.Cartesian3.ZERO, C.Cartesian3.UNIT_X],
      id: ARROW_ID,
      width,
      material: C.Material.fromType(C.Material.PolylineArrowType, { color }),
      show: false,
    });
  }

  _line(layer, color, width) {
    return this.lines[layer].add({
      positions: [C.Cartesian3.ZERO, C.Cartesian3.UNIT_X],
      id: ARROW_ID,
      width,
      material: C.Material.fromType(C.Material.ColorType, { color }),
      show: false,
    });
  }

  /** The numbers' panels shown or not; returns whether they are. */
  togglePanel() {
    this.panel = !this.panel;
    this.root.hidden = !this.panel;
    this.historyRoot.hidden = !this.panel;
    this.legendRoot?.classList.toggle("beside-panel", this.panel);
    return this.panel;
  }

  /** Layer `n` (1-5) on the wing shown or not. */
  toggleLayer(n) {
    const layer = LAYERS[n - 1];
    if (!layer) return;
    this.layers[layer.key] = !this.layers[layer.key];
    this.lines[layer.key].show = this.layers[layer.key];
    if (layer.key === "lift") this.labels.show = this.layers.lift;
    this._buildLegend();
  }

  /** The trail coloured by the next of its modes. */
  cycleTrail() {
    this.trailMode = (this.trailMode + 1) % TRAIL_MODES.length;
    const mode = TRAIL_MODES[this.trailMode];
    for (const point of this._trail) if (point.line) point.line.material.uniforms.color = ramp(mode.stops, point.values[this.trailMode]);
    this._buildLegend();
  }

  /** The arrows bigger (`up`) or smaller. */
  resize(up) {
    this.scale *= up ? SCALE_STEP : 1 / SCALE_STEP;
    this._buildLegend();
  }

  _scaleText() {
    const strip = 1 / (STRIP_M_PER_N * this.scale);
    const whole = 1 / (WHOLE_M_PER_N * this.scale);
    return `1 m: ${fmt(strip, strip < 10 ? 1 : 0)} N a strip, ${fmt(whole, whole < 10 ? 1 : 0)} N whole · [ ] size`;
  }

  _build() {
    const r = this.root;
    const title = el("div", "dyn-title");
    title.append(el("span", "", "DYNAMICS · swarm.wingsim"), el("span", "", "D hides · 1-5 on the wing"));
    const grid = el("div", "dyn-grid");
    this._cells = SECTIONS.map((sec) => {
      const box = el("section");
      const head = el("h3");
      head.append(el("span", "", sec.title), ...[0, 1, 2].map((k) => el("span", "", sec.cols[k] ?? "")));
      box.append(head);
      const rows = sec.rows.map(([label]) => {
        const row = el("div", "dyn-row");
        const values = [0, 1, 2].map(() => el("b"));
        row.append(el("span", "", label), ...values);
        box.append(row);
        return values;
      });
      grid.append(box);
      return rows;
    });
    const spanHead = el("h3", "dyn-wide", "STRIPS across the span, from behind: α eff (bar) against the stall (white; dashed: delayed by the lead), α before the downwash (outline); CL");
    this._span = el("canvas", "dyn-span");
    const table = el("table", "dyn-strips");
    const thead = el("thead");
    const heads = el("tr");
    heads.append(el("th", "", ""), ...STRIP_COLS.map(([h]) => el("th", "", h)));
    const units = el("tr", "units");
    units.append(el("th", "", ""), ...STRIP_COLS.map(([, u]) => el("th", "", u)));
    thead.append(heads, units);
    this._tbody = el("tbody");
    table.append(thead, this._tbody);
    this._stripRows = [];
    /** The strips as named and ordered for the table; null until the first frame. */
    this._named = null;
    r.append(title, grid, spanHead, this._span, table);
    this._hist = el("canvas");
    this.historyRoot.append(el("div", "dyn-title", `DYNAMICS · LAST ${HISTORY_S} s`), this._hist);
  }

  /** The legend: the layers, which are on, and what those draw. */
  _buildLegend() {
    const r = this.legendRoot;
    if (!r) return;
    const on = LAYERS.filter((l) => this.layers[l.key]);
    r.hidden = !on.length;
    const swatch = (color, text) => {
      const s = el("span");
      const i = el("i");
      i.style.background = color;
      s.append(i, text);
      return s;
    };
    const keys = el("div", "dyn-keys");
    LAYERS.forEach((l, k) => keys.append(el("span", this.layers[l.key] ? "on" : "", `${k + 1} ${l.name}`)));
    const rows = [keys];
    for (const l of on) {
      const row = el("div", "dyn-legend");
      row.append(el("b", "", l.name));
      for (const [color, text] of l.legend) row.append(swatch(color, text));
      if (l.key === "forces") row.append(el("span", "", this._scaleText()));
      if (l.key === "path") {
        const mode = TRAIL_MODES[this.trailMode];
        const stops = mode.stops;
        const span = stops[stops.length - 1][0] - stops[0][0];
        const gradient = stops.map(([v, c]) => `${c} ${((v - stops[0][0]) / span) * 100}%`).join(", ");
        row.append(
          swatch(`linear-gradient(90deg, ${gradient})`, `${mode.name}, ${stops[0][0]} to ${stops[stops.length - 1][0]} ${mode.unit}`),
          el("span", "", `last ${TRAIL.seconds} s · Shift+5 colours by ${TRAIL_MODES[(this.trailMode + 1) % TRAIL_MODES.length].name}`),
        );
      }
      rows.push(row);
    }
    r.replaceChildren(...rows);
  }

  /** The `frame` from /wing, the wing drawn at local `p` (east-north-up) with body `axes` (its
   * right, up and nose there). */
  update(frame, p, axes, now) {
    const d = frame?.dynamics;
    if (!d) return;
    if (frame !== this._lastFrame) {
      this._lastFrame = frame;
      this._remember(d, now);
    }
    this._record(d, p, now);
    if (this.layers.forces) this._drawForces(d, p, axes);
    if (this.layers.air) this._drawAir(d, p, axes, now);
    if (this.layers.lift) this._drawLift(d, p, axes);
    if (this.layers.rotation) this._drawRotation(d, p, axes);
    if (!this.panel || now - this._at < PANEL_MS) return;
    this._at = now;
    this._fillNumbers(d, frame.wing);
    this._fillStrips(d);
    this._drawSpan(d);
    this._drawHistory(now);
  }

  // On the wing.

  _drawForces(d, p, axes) {
    const V = C.Cartesian3;
    while (this.stripArrows.length < d.strips.length) this.stripArrows.push(this._arrow("forces", C.Color.WHITE));
    this.stripArrows.forEach((line, i) => {
      const s = d.strips[i];
      if (!s) {
        line.show = false;
        return;
      }
      line.material.uniforms.color = attachedColor(s.attached);
      place(line, V.add(p, toLocal(axes, s.at), new V()), toLocal(axes, s.force, STRIP_M_PER_N * this.scale));
    });
    const k = WHOLE_M_PER_N * this.scale;
    const f = this.forces;
    place(f.aero, p, toLocal(axes, d.aero, k));
    place(f.net, p, toLocal(axes, d.total, k));
    place(f.pod, p, toLocal(axes, d.pod_drag, k));
    place(f.gravity, p, toLocal(axes, d.gravity, k));
    place(f.prop, V.add(p, toLocal(axes, d.prop_at), new V()), toLocal(axes, d.prop, k));
  }

  _drawAir(d, p, axes, now) {
    const V = C.Cartesian3;
    // The air comes at it from where it flies to, at α and β.
    const [a, b] = [d.alpha, d.beta].map(C.Math.toRadians);
    const toward = [Math.cos(a) * Math.cos(b), Math.sin(b), Math.sin(a) * Math.cos(b)];
    const flying = d.airspeed > 0.5;
    const reach = AIRFLOW.gap + AIRFLOW.length;
    if (flying) place(this.air.airflow, V.add(p, toLocal(axes, toward, reach), new V()), toLocal(axes, toward, -AIRFLOW.length));
    else this.air.airflow.show = false;
    // The wind, NED, from above the wing.
    const [n, e, dn] = d.wind;
    place(this.air.wind, new V(p.x, p.y, p.z + 0.6), new V(e * WIND_M_PER_MS, n * WIND_M_PER_MS, -dn * WIND_M_PER_MS));
    // The air coming past each strip: its own motion's and the rotation's, turned by the
    // downwash; arriving at its quarter chord.
    const flowColor = C.Color.fromCssColorString(COLORS.flow);
    while (this.flowArrows.length < d.strips.length) this.flowArrows.push(this._arrow("air", flowColor, FLOW_PX));
    this.flowArrows.forEach((line, i) => {
      const s = d.strips[i];
      if (!s) {
        line.show = false;
        return;
      }
      const flow = toLocal(axes, s.air, FLOW_M_PER_MS * this.scale);
      place(line, V.subtract(V.add(p, toLocal(axes, s.at), new V()), flow, new V()), flow);
    });
    this._trace(d, p, axes, now);
  }

  /** Where the wing has been in the air: each source's points stay where it shed them, carried
   * off with the wind and sinking with the wing's downwash. The trailing vortices (the vortex
   * lattice's, off the strips' ends) lie along them, not straight back as the lattice takes
   * them; the smoke shows how the fin's top and the elevons' trailing edges have moved. */
  _trace(d, p, axes, now) {
    const V = C.Cartesian3;
    const [n, e, dn] = d.wind;
    const downwash = d.strips.length ? d.strips.reduce((sum, s) => sum + s.downwash, 0) / d.strips.length : 0;
    const sink = toLocal(axes, [0, 0, 1], d.airspeed * Math.sin(C.Math.toRadians(downwash)));
    const drift = V.add(new V(e, n, -dn), sink, new V());
    const vortex = C.Color.fromCssColorString(COLORS.vortex);
    const smoke = C.Color.fromCssColorString(COLORS.smoke).withAlpha(0.8);
    const sources = d.wake.map((w, i) => {
      const strength = Math.tanh(Math.abs(w[1]) / TRACE.strength);
      return { key: `wake ${i}`, at: w[0], width: 1 + (TRACE.wakePx - 1) * strength, color: vortex.withAlpha(0.3 + 0.7 * strength) };
    });
    const fin = d.strips.find((s) => Math.abs(s.up[1]) >= 0.5);
    if (fin) {
      // The fin's top trailing corner: up is -z.
      const top = [fin.corners[2], fin.corners[3]].reduce((a, b) => (b[2] < a[2] ? b : a));
      sources.push({ key: "fin", at: top, width: TRACE.px, color: smoke });
    }
    for (const side of [0, 1]) {
      // Each elevon's trailing edge, midway along it.
      const on = d.strips.filter((s) => s.elevon === side).sort((a, b) => a.at[1] - b.at[1]);
      const mid = on[Math.floor(on.length / 2)];
      if (!mid) continue;
      const [, , te1, te0] = mid.corners;
      sources.push({ key: `elevon ${side}`, at: te0.map((x, j) => (x + te1[j]) / 2), width: TRACE.px, color: smoke });
    }
    const live = new Set();
    for (const src of sources) {
      live.add(src.key);
      let trace = this.traces.get(src.key);
      if (!trace) {
        trace = { line: this._line("air", src.color, src.width), points: [] };
        this.traces.set(src.key, trace);
      }
      const at = V.add(p, toLocal(axes, src.at), new V());
      const points = trace.points;
      const last = points[points.length - 1];
      if (last && V.distance(last.at, at) > TRACE.jump) points.length = 0;
      points.push({ at, t: now, drift });
      while (points.length && now - points[0].t > TRACE.seconds * 1000) points.shift();
      trace.line.width = src.width;
      trace.line.material.uniforms.color = src.color;
      trace.line.show = points.length > 1;
      if (trace.line.show) {
        trace.line.positions = points.map((pt) => V.add(pt.at, V.multiplyByScalar(pt.drift, (now - pt.t) / 1000, new V()), new V()));
      }
    }
    for (const [key, trace] of this.traces) if (!live.has(key)) trace.line.show = false;
  }

  _drawLift(d, p, axes) {
    const V = C.Cartesian3;
    const at = (v) => V.add(p, toLocal(axes, v), new V());
    // Each strip's patch, hatched chordwise just over its lift side, tinted by its flow.
    if (this.hatches?.length !== d.strips.length) {
      for (const lines of this.hatches ?? []) for (const line of lines) this.lines.lift.remove(line);
      this.hatches = d.strips.map((s) => {
        const [a, b] = s.corners;
        const n = Math.max(2, Math.round(Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]) / HATCH.every));
        return Array.from({ length: n }, () => this._line("lift", C.Color.WHITE, HATCH.px));
      });
    }
    d.strips.forEach((s, i) => {
      const [le0, le1, te1, te0] = s.corners;
      const color = attachedColor(s.attached).withAlpha(0.85);
      const lines = this.hatches[i];
      lines.forEach((line, k) => {
        const t = (k + 0.5) / lines.length;
        // Along the span to this line, then just off the strip on its lift side.
        const across = (u, v) => u.map((x, j) => x + (v[j] - x) * t + s.up[j] * HATCH.above);
        line.material.uniforms.color = color;
        line.positions = [at(across(le0, le1)), at(across(te0, te1))];
        line.show = true;
      });
    });
    // The root chord, and on it the centre of mass, the centre of pressure, the neutral point.
    const root = d.strips.filter((s) => Math.abs(s.up[1]) < 0.5).flatMap((s) => s.corners).filter((c) => Math.abs(c[1]) < 0.002);
    if (!root.length) return;
    const xs = root.map((c) => c[0]);
    const z = -RULER.above;
    this.ruler.positions = [at([Math.max(...xs), 0, z]), at([Math.min(...xs), 0, z])];
    this.ruler.show = true;
    const margin = Number.isFinite(nullable(d.static_margin)) ? ` ${fmt(d.static_margin * 100, 1)}%` : "";
    const places = { cg: 0, cp: d.centre_of_pressure, np: d.neutral_point };
    for (const [key, mark] of Object.entries(this.marks)) {
      const x = places[key];
      const shown = x !== null && x !== undefined;
      mark.line.show = mark.label.show = shown;
      if (!shown) continue;
      const top = at([x, 0, z - RULER.marks[key]]);
      mark.line.positions = [at([x, 0, z]), top];
      mark.label.position = top;
      if (key === "np") mark.label.text = `NP${margin}`;
    }
  }

  _drawRotation(d, p, axes) {
    const V = C.Cartesian3;
    const moment = d.total_moment;
    ARCS.forEach((arc, axis) => {
      const sweep = (v, scale) => Math.sign(v) * ARC.max * Math.tanh(Math.abs(v) / scale);
      const draw = (line, angle, radius) => {
        line.show = Math.abs(angle) > C.Math.toRadians(3);
        if (!line.show) return;
        const positions = [];
        for (let k = 0; k <= ARC.steps; k++) {
          const t = arc.start + (angle * k) / ARC.steps;
          const v = arc.centre.map((c, j) => c + radius * (Math.cos(t) * arc.e1[j] + Math.sin(t) * arc.e2[j]));
          positions.push(V.add(p, toLocal(axes, v), new V()));
        }
        line.positions = positions;
      };
      draw(this.arcs[axis].moment, sweep(moment[axis], ARC.moment), arc.radius);
      draw(this.arcs[axis].rate, sweep(d.rates[axis], ARC.rate), arc.radius * 1.3);
    });
  }

  /** The trail's points, kept whether it shows or not: each a segment from the one before. */
  _record(d, p, now) {
    if (now - this._trailAt < TRAIL.every) return;
    this._trailAt = now;
    const trail = this._trail;
    const last = trail[trail.length - 1];
    if (last && C.Cartesian3.distance(last.p, p) > TRAIL.jump) this._clearTrail();
    else if (last && C.Cartesian3.distance(last.p, p) < 0.01) return;
    const values = TRAIL_MODES.map((m) => m.of(d));
    let line = null;
    const before = trail[trail.length - 1];
    if (before) {
      line = this.lines.path.add({
        positions: [before.p, C.Cartesian3.clone(p)],
        id: ARROW_ID,
        width: TRAIL.px,
        material: C.Material.fromType(C.Material.ColorType, { color: ramp(TRAIL_MODES[this.trailMode].stops, values[this.trailMode]) }),
      });
    }
    trail.push({ p: C.Cartesian3.clone(p), t: now, values, line });
    while (trail.length && now - trail[0].t > TRAIL.seconds * 1000) {
      const old = trail.shift();
      if (old.line) this.lines.path.remove(old.line);
    }
  }

  _clearTrail() {
    for (const point of this._trail) if (point.line) this.lines.path.remove(point.line);
    this._trail = [];
  }

  // Beside it.

  _fillNumbers(d, wing) {
    SECTIONS.forEach((sec, si) => {
      sec.rows.forEach(([, value, digits], ri) => {
        const values = value(d, wing);
        this._cells[si][ri].forEach((cell, k) => {
          const text = k < values.length ? fmt(values[k], Array.isArray(digits) ? digits[k] : digits) : "";
          if (cell.textContent !== text) cell.textContent = text;
        });
      });
    });
  }

  _fillStrips(d) {
    if (this._named?.length !== d.strips.length) {
      this._tbody.replaceChildren();
      this._named = nameStrips(d.strips);
      this._stripRows = this._named.map((n) => {
        const tr = el("tr");
        const name = el("td", "", n.name);
        const cells = STRIP_COLS.map(() => el("td"));
        tr.append(name, ...cells);
        this._tbody.append(tr);
        return { tr, name, cells };
      });
      if (!d.strips.length) {
        const tr = el("tr");
        const td = el("td", "", "No air on it: in the hand, or down.");
        td.colSpan = STRIP_COLS.length + 1;
        tr.append(td);
        this._tbody.append(tr);
      }
    }
    this._named.forEach((n, row) => {
      const s = d.strips[n.i];
      const r = this._stripRows[row];
      STRIP_COLS.forEach(([, , value, digits], k) => {
        const text = fmt(value(s), digits);
        if (r.cells[k].textContent !== text) r.cells[k].textContent = text;
      });
      r.name.style.color = attachedColor(s.attached).toCssColorString();
    });
  }

  _canvas(canvas, height) {
    const dpr = devicePixelRatio || 1;
    const width = canvas.clientWidth;
    if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) {
      canvas.width = Math.round(width * dpr);
      canvas.height = Math.round(height * dpr);
      canvas.style.height = `${height}px`;
    }
    const g = canvas.getContext("2d");
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, width, height);
    g.font = "10px ui-monospace, monospace";
    return { g, width };
  }

  /** The strips across the span: their angles of attack against their stall, and their lift. */
  _drawSpan(d) {
    const { g, width } = this._canvas(this._span, 142);
    const wing = (this._named ?? []).filter((n) => n.name !== "fin");
    if (!wing.length) return;
    const ys = wing.map((n) => n.s.at[1]);
    // Each strip's edges: halfway to its neighbours, as far again at the tips.
    const edges = ys.map((y, k) => {
      const lo = k > 0 ? (ys[k - 1] + y) / 2 : y - ((ys[1] ?? y + 0.05) - y) / 2;
      const hi = k < ys.length - 1 ? (ys[k + 1] + y) / 2 : y + (y - (ys[k - 1] ?? y - 0.05)) / 2;
      return [lo, hi];
    });
    const span = [edges[0][0], edges[edges.length - 1][1]];
    const pad = 34;
    const x = (y) => pad + ((y - span[0]) / (span[1] - span[0])) * (width - pad - 4);
    const plots = [
      { top: 4, height: 80, range: SPAN_ALPHA, label: "α °", ticks: [-10, 0, 10, 20] },
      { top: 92, height: 36, range: SPAN_CL, label: "CL", ticks: [0, 1] },
    ];
    for (const plot of plots) {
      const yOf = (v) => plot.top + plot.height * (1 - (v - plot.range[0]) / (plot.range[1] - plot.range[0]));
      plot.yOf = (v) => Math.max(plot.top, Math.min(plot.top + plot.height, yOf(v)));
      g.strokeStyle = "rgba(255,255,255,0.08)";
      g.fillStyle = "#8b98a5";
      for (const t of plot.ticks) {
        g.beginPath();
        g.moveTo(pad, plot.yOf(t));
        g.lineTo(width - 4, plot.yOf(t));
        g.stroke();
        g.fillText(String(t), pad - 6 - g.measureText(String(t)).width, plot.yOf(t) + 3);
      }
      g.fillStyle = "#e8edf2";
      g.fillText(plot.label, width - 4 - g.measureText(plot.label).width, plot.top + 9);
    }
    const [pa, pc] = plots;
    wing.forEach((n, k) => {
      const s = n.s;
      const [x0, x1] = [x(edges[k][0]) + 1, x(edges[k][1]) - 1];
      const color = attachedColor(s.attached).toCssColorString();
      g.fillStyle = color;
      g.fillRect(x0, Math.min(pa.yOf(0), pa.yOf(s.seen)), x1 - x0, Math.abs(pa.yOf(s.seen) - pa.yOf(0)));
      g.fillRect(x0, Math.min(pc.yOf(0), pc.yOf(s.cl)), x1 - x0, Math.abs(pc.yOf(s.cl) - pc.yOf(0)));
      g.strokeStyle = "rgba(232,237,242,0.55)";
      g.strokeRect(x0 + 0.5, Math.min(pa.yOf(0), pa.yOf(s.alpha)) + 0.5, x1 - x0 - 1, Math.abs(pa.yOf(s.alpha) - pa.yOf(0)));
      g.strokeStyle = "#ffffff";
      g.lineWidth = 2;
      g.beginPath();
      g.moveTo(x0, pa.yOf(s.stall));
      g.lineTo(x1, pa.yOf(s.stall));
      g.stroke();
      if (Math.abs(s.lead) > 0.05) {
        g.setLineDash([3, 3]);
        g.beginPath();
        g.moveTo(x0, pa.yOf(s.stall + s.lead));
        g.lineTo(x1, pa.yOf(s.stall + s.lead));
        g.stroke();
        g.setLineDash([]);
      }
      g.lineWidth = 1;
      g.fillStyle = "#8b98a5";
      g.fillText(n.name, (x0 + x1) / 2 - g.measureText(n.name).width / 2, pc.top + pc.height + 11);
    });
  }

  _remember(d, now) {
    const h = this._history;
    h.push({ t: now, v: LANES.map(([, value]) => value(d)), m: LANES.map(([, , , , mark]) => mark?.(d) ?? NaN) });
    while (h.length && now - h[0].t > HISTORY_S * 1000) h.shift();
  }

  _drawHistory(now) {
    const laneH = 40;
    const { g, width } = this._canvas(this._hist, LANES.length * laneH);
    const h = this._history;
    const pad = 4;
    const x = (t) => pad + (1 - (now - t) / (HISTORY_S * 1000)) * (width - 2 * pad);
    LANES.forEach(([label, , range, digits], k) => {
      const top = k * laneH;
      const y = (v) => top + 3 + (laneH - 6) * (1 - (Math.max(range[0], Math.min(range[1], v)) - range[0]) / (range[1] - range[0]));
      g.strokeStyle = "rgba(255,255,255,0.08)";
      g.strokeRect(pad + 0.5, top + 0.5, width - 2 * pad - 1, laneH - 1);
      if (range[0] < 0 && range[1] > 0) {
        g.beginPath();
        g.moveTo(pad, y(0));
        g.lineTo(width - pad, y(0));
        g.stroke();
      }
      const trace = (of) => {
        g.beginPath();
        let down = false;
        for (const s of h) {
          const v = of(s);
          if (!Number.isFinite(v)) {
            down = false;
            continue;
          }
          if (down) g.lineTo(x(s.t), y(v));
          else g.moveTo(x(s.t), y(v));
          down = true;
        }
        g.stroke();
      };
      g.strokeStyle = COLORS.bad;
      g.setLineDash([3, 3]);
      trace((s) => s.m[k]);
      g.setLineDash([]);
      g.strokeStyle = COLORS.prop;
      trace((s) => s.v[k]);
      const last = h[h.length - 1]?.v[k];
      g.fillStyle = "#e8edf2";
      g.fillText(`${label} ${fmt(last, digits)}`, pad + 4, top + 12);
      g.fillStyle = "#8b98a5";
      g.fillText(`${range[1]}`, width - pad - 24, top + 12);
      g.fillText(`${range[0]}`, width - pad - 24, top + laneH - 4);
    });
  }
}
