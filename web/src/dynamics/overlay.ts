// The Chupito's flight model at work (DESIGN.md §1: the dynamics overlay), from the server's
// `dynamics` channel (drone_duels.physics.wing.dynamics). On the wing, five layers, any together,
// toggled from the legend's chips or Shift+1..5:
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
//                     flow or energy gain (the legend's button changes which).
// Beside it: every number the model works with, the strips across the span, and in a panel of
// its own the last few seconds of the ones that move most. [ and ] size the arrows.
//
// It draws in an east-north-up frame anchored where it first saw the wing (moved on when the
// wing goes far), the wing at the true pose's place with its body axes from the pose's attitude.

import * as C from "cesium";
import type { Pose } from "../net/server.ts";
import { bodyAxesEnu, toFixed } from "../world/geo.ts";
import type { World } from "../world/world.ts";
import "./dynamics.css";
import {
  asDynamics,
  bodyToEnu,
  type Dynamics,
  fmt,
  leastAttached,
  nameStrips,
  rampAt,
  type StripDynamics,
  type V3,
} from "./model.ts";

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
 * far, its label this many pixels off its top (CG's below the ruler). */
const RULER = {
  above: 0.07,
  marks: { cg: 0.02, cp: 0.04, np: 0.04 },
  labels: { cg: 26, cp: -3, np: -21 },
} as const;
/** The rotation arcs: their sweep for a moment of `moment` N m or a rate of `rate` °/s is most of
 * `max`; about each axis (body x, y, z), where, how big, and where each starts. */
const ARC = { max: C.Math.toRadians(270), moment: 0.3, rate: 180, steps: 36, px: 8, ratePx: 4 };
const ARCS: { e1: V3; e2: V3; centre: V3; radius: number; start: number }[] = [
  { e1: [0, 1, 0], e2: [0, 0, 1], centre: [0.42, 0, 0], radius: 0.16, start: -Math.PI / 2 },
  { e1: [0, 0, 1], e2: [1, 0, 0], centre: [-0.05, 0.56, 0], radius: 0.16, start: Math.PI / 2 },
  { e1: [1, 0, 0], e2: [0, 1, 0], centre: [0, 0, -0.22], radius: 0.22, start: 0 },
];
/** The trail: how long it lasts, s; a point every so often, ms; a jump this far, m, is a reset. */
const TRAIL = { seconds: 30, every: 50, jump: 30, px: 5 };
const ARROW_PX = 9;
const SCALE_STEP = 1.5;
/** Every primitive's pick id: ground sampling leaves them out (`primitives`), or the wing would
 * fly into its own arrows. */
export const DYNAMICS_ID = "wing dynamics arrow";
const PANEL_MS = 100;
const HISTORY_S = 10;
/** The local frame moves on when the wing is this far from its anchor, m. */
const REANCHOR_M = 3000;
/** The spanwise chart's ranges: the strips' angles of attack, deg, and their lift coefficients. */
const SPAN_ALPHA: [number, number] = [-15, 30];
const SPAN_CL: [number, number] = [-0.6, 1.4];

/** The overlay's data colours (each force and flow its own), with Dusk's for attached flow. */
const COLORS = {
  ok: "#8be07a", // --leaf
  warn: "#ffc845", // --sun
  bad: "#ff4f6e", // --rose
  prop: "#7fe3ff",
  pod: "#c792ff",
  gravity: "#d0d6dc",
  aero: "#ffb020",
  net: "#ff6bd5",
  airflow: "#9ad0ff",
  flow: "#6ea8ff", // --sky
  wind: "#b8f5c8",
  vortex: "#c9b6ff",
  smoke: "#f4f1ea",
  cg: "#ffffff",
  cp: "#ffb020",
  np: "#22c7b8", // --tide
  rate: "#eef0f7", // --text
} as const;
type ColorKey = keyof typeof COLORS;
const ATTACHED = `linear-gradient(90deg, ${COLORS.bad}, ${COLORS.warn}, ${COLORS.ok})`;
const cesium = (key: ColorKey): C.Color => C.Color.fromCssColorString(COLORS[key]);

type LayerKey = "forces" | "air" | "lift" | "rotation" | "path";

/** The layers, in key order: what each shows, and its legend. */
const LAYERS: { key: LayerKey; name: string; legend: [string, string][] }[] = [
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

/** What the trail can be coloured by: the value from the dynamics, and its colour stops. */
const TRAIL_MODES: {
  name: string;
  unit: string;
  of: (d: Dynamics) => number;
  stops: [number, string][];
}[] = [
  {
    name: "load factor",
    unit: "g",
    of: (d) => -d.load[2],
    stops: [
      [0, "#6ea8ff"],
      [1, "#eef0f7"],
      [4, "#ffc845"],
      [8, "#ff4f6e"],
    ],
  },
  {
    name: "airspeed",
    unit: "m/s",
    of: (d) => d.airspeed,
    stops: [
      [0, "#6ea8ff"],
      [15, "#8be07a"],
      [30, "#ffc845"],
      [45, "#ff4f6e"],
    ],
  },
  {
    name: "least attached",
    unit: "%",
    of: (d) => leastAttached(d) * 100,
    stops: [
      [0, COLORS.bad],
      [50, COLORS.warn],
      [100, COLORS.ok],
    ],
  },
  {
    name: "energy gain",
    unit: "m/s",
    of: (d) => d.energy_rate,
    stops: [
      [-10, "#ff4f6e"],
      [0, "#8e93ad"],
      [10, "#8be07a"],
    ],
  },
];

/** `stops`' colour at `v`, between them; grey for no value. */
function ramp(stops: [number, string][], v: number): C.Color {
  if (!Number.isFinite(v)) return C.Color.GRAY;
  const { a, b, t } = rampAt(stops, v);
  return C.Color.lerp(
    C.Color.fromCssColorString(stops[a]![1]),
    C.Color.fromCssColorString(stops[b]![1]),
    t,
    new C.Color(),
  );
}

/** Green attached, through amber, to red separated. */
function attachedColor(share: number): C.Color {
  const [a, m, b] = [cesium("bad"), cesium("warn"), cesium("ok")];
  const s = Math.max(0, Math.min(1, share));
  return s < 0.5
    ? C.Color.lerp(a, m, s * 2, new C.Color())
    : C.Color.lerp(m, b, (s - 0.5) * 2, new C.Color());
}

const nullable = (x: number | null | undefined, k = 1): number =>
  x === null || x === undefined ? Number.NaN : x * k;

type Row = [string, (d: Dynamics) => readonly number[], number | number[]];

/** Every number, by section: its column heads, and rows of a label, the values, decimals. */
const SECTIONS: { title: string; cols: string[]; rows: Row[] }[] = [
  {
    title: "AIR",
    cols: [],
    rows: [
      ["airspeed m/s", (d) => [d.airspeed], 1],
      ["α · β °", (d) => [d.alpha, d.beta], 1],
      ["q̄ Pa", (d) => [d.dynamic_pressure], 0],
      [
        "CL · CD · L/D",
        (d) => [d.coefficients[0]!, d.coefficients[1]!, d.coefficients[0]! / d.coefficients[1]!],
        [3, 3, 1],
      ],
      ["CY", (d) => [d.coefficients[2]!], 3],
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
      ["throttle %", (d) => [nullable(d.motor, 100)], 0],
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
      [
        "CP · NP mm fwd",
        (d) => [nullable(d.centre_of_pressure, 1000), nullable(d.neutral_point, 1000)],
        0,
      ],
      ["static margin %", (d) => [nullable(d.static_margin, 100)], 1],
    ],
  },
];

/** The strip table's columns: head, unit, value, decimals. */
const STRIP_COLS: [string, string, (s: StripDynamics) => number, number][] = [
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

/** The history's lanes: label, value, range, decimals, and a mark to hold it to. */
const LANES: [
  string,
  (d: Dynamics) => number,
  [number, number],
  number,
  ((d: Dynamics) => number)?,
][] = [
  [
    "α ° · stall",
    (d) => d.alpha,
    [-10, 30],
    1,
    (d) => (d.strips.length ? Math.min(...d.strips.map((s) => s.stall)) : Number.NaN),
  ],
  ["CL", (d) => d.coefficients[0]!, [-0.5, 1.5], 2],
  ["n g", (d) => -d.load[2], [-2, 8], 1],
  ["least attached %", (d) => leastAttached(d) * 100, [0, 100], 0],
  ["airspeed m/s", (d) => d.airspeed, [0, 50], 1],
];

function el<K extends keyof HTMLElementTagNameMap>(
  tag: K,
  cls?: string,
  text?: string,
): HTMLElementTagNameMap[K] {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text !== undefined) e.textContent = text;
  return e;
}

const V = C.Cartesian3;
const cart = (v: V3): C.Cartesian3 => new V(v[0], v[1], v[2]);

/** The wing's place and body axes in the local frame, this frame. */
interface Frame {
  p: C.Cartesian3;
  axes: { fwd: V3; right: V3; up: V3 };
}

/** A body vector in the local frame, times `k`. */
const toLocal = (f: Frame, v: readonly number[], k = 1): C.Cartesian3 =>
  cart(bodyToEnu(f.axes, v, k));
/** A body point (from the centre of mass) in the local frame. */
const at = (f: Frame, v: readonly number[]): C.Cartesian3 => V.add(f.p, toLocal(f, v), new V());

const finite = (c: C.Cartesian3): boolean =>
  Number.isFinite(c.x) && Number.isFinite(c.y) && Number.isFinite(c.z);

/** `line` through `pts`, or hidden when they are not all finite or are fewer than two apart:
 * Cesium throws on such a polyline, and a throw in its render loop stops it for good. */
function setLine(line: C.Polyline, pts: C.Cartesian3[]): void {
  let distinct = false;
  for (const p of pts) {
    if (!finite(p)) {
      line.show = false;
      return;
    }
    if (!distinct && pts[0] && V.distanceSquared(p, pts[0]) > 1e-10) distinct = true;
  }
  line.show = distinct;
  if (distinct) line.positions = pts;
}

/** `line` from `from` along `vector`, hidden when too short to see. */
function place(line: C.Polyline, from: C.Cartesian3, vector: C.Cartesian3): void {
  if (!(V.magnitude(vector) > 0.005)) {
    line.show = false;
    return;
  }
  setLine(line, [from, V.add(from, vector, new V())]);
}

type Collections = Record<LayerKey, C.PolylineCollection>;

interface TracePoint {
  at: C.Cartesian3;
  t: number;
  drift: C.Cartesian3;
}

export class DynamicsOverlay {
  private on = false;
  private readonly layers: Record<LayerKey, boolean> = {
    forces: true,
    air: false,
    lift: false,
    rotation: false,
    path: false,
  };
  private numbersShown = true;
  private scale = 1;
  private trailMode = 0;
  /** Local east-north-up at the anchor to the earth's frame, and back. */
  // Somewhere on the ground until the wing is seen: Cesium places (and so needs) every point.
  private readonly localToFixed = C.Transforms.eastNorthUpToFixedFrame(V.fromDegrees(0, 0));
  private readonly fixedToLocal = C.Matrix4.inverseTransformation(
    this.localToFixed,
    new C.Matrix4(),
  );
  private anchored = false;
  private readonly lines: Collections;
  private readonly labels: C.LabelCollection;
  private readonly forces: Record<"aero" | "net" | "prop" | "pod" | "gravity", C.Polyline>;
  private readonly stripArrows: C.Polyline[] = [];
  private readonly air: { airflow: C.Polyline; wind: C.Polyline };
  private readonly flowArrows: C.Polyline[] = [];
  private readonly traces = new Map<string, { line: C.Polyline; points: TracePoint[] }>();
  private hatches: C.Polyline[][] | null = null;
  private readonly ruler: C.Polyline;
  private readonly marks: Record<"cg" | "cp" | "np", { line: C.Polyline; label: C.Label }>;
  private readonly arcs: { moment: C.Polyline; rate: C.Polyline }[];
  private trail: { p: C.Cartesian3; t: number; values: number[]; line: C.Polyline | null }[] = [];
  private trailAt = 0;
  private panelAt = 0;
  private history: { t: number; v: number[]; m: number[] }[] = [];
  private lastData: Record<string, unknown> | null = null;
  private named: ReturnType<typeof nameStrips> | null = null;
  // The panels.
  private readonly numbersEl: HTMLDivElement;
  private readonly historyEl: HTMLDivElement;
  private readonly legendEl: HTMLDivElement;
  private cells: HTMLElement[][][] = [];
  private tbody!: HTMLTableSectionElement;
  private stripRows: { name: HTMLElement; cells: HTMLElement[] }[] = [];
  private span!: HTMLCanvasElement;
  private hist!: HTMLCanvasElement;
  private readonly onKey = (e: KeyboardEvent): void => this.key(e);

  constructor(
    private readonly world: World,
    host: HTMLElement,
  ) {
    const add = <T extends object>(p: T): T => world.scene.primitives.add(p) as T;
    const collection = (): C.PolylineCollection =>
      add(new C.PolylineCollection({ modelMatrix: this.localToFixed, show: false }));
    this.lines = {
      forces: collection(),
      air: collection(),
      lift: collection(),
      rotation: collection(),
      path: collection(),
    };
    this.labels = add(new C.LabelCollection({ modelMatrix: this.localToFixed, show: false }));
    this.forces = {
      aero: this.arrow("forces", cesium("aero")),
      net: this.arrow("forces", cesium("net")),
      prop: this.arrow("forces", cesium("prop")),
      pod: this.arrow("forces", cesium("pod")),
      gravity: this.arrow("forces", cesium("gravity")),
    };
    this.air = {
      airflow: this.arrow("air", cesium("airflow")),
      wind: this.arrow("air", cesium("wind")),
    };
    this.ruler = this.line("lift", C.Color.WHITE.withAlpha(0.7), 2);
    const mark = (k: "cg" | "cp" | "np") => ({
      line: this.line("lift", cesium(k), 3),
      label: this.labels.add({
        position: V.ZERO,
        id: DYNAMICS_ID,
        text: k.toUpperCase(),
        font: "12px 'JetBrains Mono', ui-monospace, monospace",
        fillColor: cesium(k),
        showBackground: true,
        backgroundColor: new C.Color(0, 0, 0, 0.55),
        horizontalOrigin: C.HorizontalOrigin.CENTER,
        verticalOrigin: k === "cg" ? C.VerticalOrigin.TOP : C.VerticalOrigin.BOTTOM,
        pixelOffset: new C.Cartesian2(0, RULER.labels[k]),
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
        show: false,
      }),
    });
    this.marks = { cg: mark("cg"), cp: mark("cp"), np: mark("np") };
    this.arcs = ARCS.map(() => ({
      moment: this.arrow("rotation", cesium("net"), ARC.px),
      rate: this.arrow("rotation", cesium("rate"), ARC.ratePx),
    }));

    this.numbersEl = el("div", "dyn-panel dyn-numbers");
    this.historyEl = el("div", "dyn-panel dyn-history");
    this.legendEl = el("div", "dyn-panel dyn-legend-panel beside");
    for (const panel of [this.numbersEl, this.historyEl, this.legendEl]) {
      panel.hidden = true;
      host.append(panel);
    }
    this.buildNumbers();
    this.buildLegend();
    window.addEventListener("keydown", this.onKey, { capture: true });
  }

  /** Its primitives, which ground sampling must look through. */
  get primitives(): object[] {
    return [...Object.values(this.lines), this.labels];
  }

  setOn(on: boolean): void {
    if (on === this.on) return;
    this.on = on;
    this.legendEl.hidden = !on;
    this.numbersEl.hidden = this.historyEl.hidden = !(on && this.numbersShown);
    this.showLayers();
    if (!on) this.lastData = null;
  }

  /** Each animation frame while on: the latest dynamics (null when none has come), at the wing's
   * true pose now. */
  update(data: Record<string, unknown> | null, pose: Pose | null): void {
    if (!this.on) return;
    const d = asDynamics(data);
    if (!d || !pose) return;
    const placed = [pose.lat, pose.lon, pose.alt, ...pose.q].every(Number.isFinite);
    if (!placed) return;
    const now = performance.now();
    const f = this.frameOf(pose);
    if (data !== this.lastData) {
      this.lastData = data;
      this.remember(d, now);
    }
    this.record(d, f.p, now);
    if (this.layers.forces) this.drawForces(d, f);
    if (this.layers.air) this.drawAir(d, f, now);
    if (this.layers.lift) this.drawLift(d, f);
    if (this.layers.rotation) this.drawRotation(d, f);
    if (!this.numbersShown || now - this.panelAt < PANEL_MS) return;
    this.panelAt = now;
    this.fillNumbers(d);
    this.fillStrips(d);
    this.drawSpan();
    this.drawHistory(now);
  }

  dispose(): void {
    window.removeEventListener("keydown", this.onKey, { capture: true });
    for (const p of this.primitives) this.world.scene.primitives.remove(p);
    for (const panel of [this.numbersEl, this.historyEl, this.legendEl]) panel.remove();
  }

  // -- controls --------------------------------------------------------------------------------

  private key(e: KeyboardEvent): void {
    if (!this.on || e.repeat || e.ctrlKey || e.metaKey || e.altKey) return;
    const target = e.target as HTMLElement | null;
    if (target?.closest("input, select, textarea")) return;
    const digit = /^Digit([1-5])$/.exec(e.code);
    if (digit && e.shiftKey) {
      this.toggleLayer(Number(digit[1]) - 1);
    } else if (e.code === "BracketLeft" || e.code === "BracketRight") {
      this.resize(e.code === "BracketRight");
    } else {
      return;
    }
    // Ours: Shift+digit is not the view keys' digit.
    e.preventDefault();
    e.stopPropagation();
  }

  /** Layer `i` (0-4) on the wing shown or not. */
  toggleLayer(i: number): void {
    const layer = LAYERS[i];
    if (!layer) return;
    this.layers[layer.key] = !this.layers[layer.key];
    this.showLayers();
    this.buildLegend();
  }

  private showLayers(): void {
    for (const l of LAYERS) this.lines[l.key].show = this.on && this.layers[l.key];
    this.labels.show = this.on && this.layers.lift;
  }

  /** The trail coloured by the next of its modes. */
  private cycleTrail(): void {
    this.trailMode = (this.trailMode + 1) % TRAIL_MODES.length;
    const mode = TRAIL_MODES[this.trailMode]!;
    for (const point of this.trail) {
      if (point.line)
        point.line.material.uniforms.color = ramp(mode.stops, point.values[this.trailMode]!);
    }
    this.buildLegend();
  }

  /** The arrows bigger (`up`) or smaller. */
  private resize(up: boolean): void {
    this.scale *= up ? SCALE_STEP : 1 / SCALE_STEP;
    this.buildLegend();
  }

  private scaleText(): string {
    const strip = 1 / (STRIP_M_PER_N * this.scale);
    const whole = 1 / (WHOLE_M_PER_N * this.scale);
    return `1 m: ${fmt(strip, strip < 10 ? 1 : 0)} N a strip, ${fmt(whole, whole < 10 ? 1 : 0)} N whole`;
  }

  // -- primitives ----------------------------------------------------------------------------

  private arrow(layer: LayerKey, color: C.Color, width = ARROW_PX): C.Polyline {
    return this.lines[layer].add({
      positions: [V.ZERO, V.UNIT_X],
      id: DYNAMICS_ID,
      width,
      material: C.Material.fromType(C.Material.PolylineArrowType, { color }),
      show: false,
    });
  }

  private line(layer: LayerKey, color: C.Color, width: number): C.Polyline {
    return this.lines[layer].add({
      positions: [V.ZERO, V.UNIT_X],
      id: DYNAMICS_ID,
      width,
      material: C.Material.fromType(C.Material.ColorType, { color }),
      show: false,
    });
  }

  /** The wing's place in the local frame, anchoring the frame there the first time or when it
   * has gone far (everything traced in the old one goes). */
  private frameOf(pose: Pose): Frame {
    const fixed = toFixed(pose.lat, pose.lon, pose.alt);
    let p = C.Matrix4.multiplyByPoint(this.fixedToLocal, fixed, new V());
    if (!this.anchored || V.magnitude(p) > REANCHOR_M) {
      C.Transforms.eastNorthUpToFixedFrame(fixed, C.Ellipsoid.WGS84, this.localToFixed);
      C.Matrix4.inverseTransformation(this.localToFixed, this.fixedToLocal);
      for (const c of [...Object.values(this.lines), this.labels]) {
        c.modelMatrix = this.localToFixed;
      }
      this.anchored = true;
      this.clearTrail();
      for (const t of this.traces.values()) t.points.length = 0;
      p = new V(0, 0, 0);
    }
    return { p, axes: bodyAxesEnu(pose.q) };
  }

  // -- on the wing ---------------------------------------------------------------------------

  private drawForces(d: Dynamics, f: Frame): void {
    while (this.stripArrows.length < d.strips.length) {
      this.stripArrows.push(this.arrow("forces", C.Color.WHITE));
    }
    this.stripArrows.forEach((line, i) => {
      const s = d.strips[i];
      if (!s) {
        line.show = false;
        return;
      }
      line.material.uniforms.color = attachedColor(s.attached);
      place(line, at(f, s.at), toLocal(f, s.force, STRIP_M_PER_N * this.scale));
    });
    const k = WHOLE_M_PER_N * this.scale;
    const fs = this.forces;
    place(fs.aero, f.p, toLocal(f, d.aero, k));
    place(fs.net, f.p, toLocal(f, d.total, k));
    place(fs.pod, f.p, toLocal(f, d.pod_drag, k));
    place(fs.gravity, f.p, toLocal(f, d.gravity, k));
    place(fs.prop, at(f, d.prop_at), toLocal(f, d.prop, k));
  }

  private drawAir(d: Dynamics, f: Frame, now: number): void {
    // The air comes at it from where it flies to, at α and β.
    const [a, b] = [C.Math.toRadians(d.alpha), C.Math.toRadians(d.beta)];
    const toward: V3 = [Math.cos(a) * Math.cos(b), Math.sin(b), Math.sin(a) * Math.cos(b)];
    const reach = AIRFLOW.gap + AIRFLOW.length;
    if (d.airspeed > 0.5) {
      place(
        this.air.airflow,
        V.add(f.p, toLocal(f, toward, reach), new V()),
        toLocal(f, toward, -AIRFLOW.length),
      );
    } else {
      this.air.airflow.show = false;
    }
    // The wind, NED, from above the wing.
    const [n, e, dn] = d.wind;
    place(
      this.air.wind,
      new V(f.p.x, f.p.y, f.p.z + 0.6),
      new V(e * WIND_M_PER_MS, n * WIND_M_PER_MS, -dn * WIND_M_PER_MS),
    );
    // The air coming past each strip: its own motion's and the rotation's, turned by the
    // downwash; arriving at its quarter chord.
    const flowColor = cesium("flow");
    while (this.flowArrows.length < d.strips.length) {
      this.flowArrows.push(this.arrow("air", flowColor, FLOW_PX));
    }
    this.flowArrows.forEach((line, i) => {
      const s = d.strips[i];
      if (!s) {
        line.show = false;
        return;
      }
      const flow = toLocal(f, s.air, FLOW_M_PER_MS * this.scale);
      place(line, V.subtract(at(f, s.at), flow, new V()), flow);
    });
    this.trace(d, f, now);
  }

  /** Where the wing has been in the air: each source's points stay where it shed them, carried
   * off with the wind and sinking with the wing's downwash. The trailing vortices (the vortex
   * lattice's, off the strips' ends) lie along them, not straight back as the lattice takes
   * them; the smoke shows how the fin's top and the elevons' trailing edges have moved. */
  private trace(d: Dynamics, f: Frame, now: number): void {
    const [n, e, dn] = d.wind;
    const downwash = d.strips.length
      ? d.strips.reduce((sum, s) => sum + s.downwash, 0) / d.strips.length
      : 0;
    const sink = toLocal(f, [0, 0, 1], d.airspeed * Math.sin(C.Math.toRadians(downwash)));
    const drift = V.add(new V(e, n, -dn), sink, new V());
    const vortex = cesium("vortex");
    const smoke = cesium("smoke").withAlpha(0.8);
    const sources: { key: string; at: readonly number[]; width: number; color: C.Color }[] =
      d.wake.map(([where, gamma], i) => {
        const strength = Math.tanh(Math.abs(gamma) / TRACE.strength);
        return {
          key: `wake ${i}`,
          at: where,
          width: 1 + (TRACE.wakePx - 1) * strength,
          color: vortex.withAlpha(0.3 + 0.7 * strength),
        };
      });
    const fin = d.strips.find((s) => Math.abs(s.up[1]) >= 0.5);
    if (fin) {
      // The fin's top trailing corner: up is -z.
      const top = fin.corners[2][2] < fin.corners[3][2] ? fin.corners[2] : fin.corners[3];
      sources.push({ key: "fin", at: top, width: TRACE.px, color: smoke });
    }
    for (const side of [0, 1]) {
      // Each elevon's trailing edge, midway along it.
      const on = d.strips.filter((s) => s.elevon === side).sort((a, b) => a.at[1] - b.at[1]);
      const mid = on[Math.floor(on.length / 2)];
      if (!mid) continue;
      const [, , te1, te0] = mid.corners;
      sources.push({
        key: `elevon ${side}`,
        at: te0.map((x, j) => (x + te1[j]!) / 2),
        width: TRACE.px,
        color: smoke,
      });
    }
    const live = new Set<string>();
    for (const src of sources) {
      live.add(src.key);
      let trace = this.traces.get(src.key);
      if (!trace) {
        trace = { line: this.line("air", src.color, src.width), points: [] };
        this.traces.set(src.key, trace);
      }
      const here = at(f, src.at);
      const points = trace.points;
      const last = points[points.length - 1];
      if (last && V.distance(last.at, here) > TRACE.jump) points.length = 0;
      points.push({ at: here, t: now, drift });
      while (points.length && now - points[0]!.t > TRACE.seconds * 1000) points.shift();
      trace.line.width = src.width;
      trace.line.material.uniforms.color = src.color;
      setLine(
        trace.line,
        points.map((pt) =>
          V.add(pt.at, V.multiplyByScalar(pt.drift, (now - pt.t) / 1000, new V()), new V()),
        ),
      );
    }
    for (const [key, trace] of this.traces) if (!live.has(key)) trace.line.show = false;
  }

  private drawLift(d: Dynamics, f: Frame): void {
    // Each strip's patch, hatched chordwise just over its lift side, tinted by its flow.
    if (this.hatches?.length !== d.strips.length) {
      for (const lines of this.hatches ?? [])
        for (const line of lines) this.lines.lift.remove(line);
      this.hatches = d.strips.map((s) => {
        const [a, b] = s.corners;
        const n = Math.max(
          2,
          Math.round(Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]) / HATCH.every),
        );
        return Array.from({ length: n }, () => this.line("lift", C.Color.WHITE, HATCH.px));
      });
    }
    const hatches = this.hatches;
    d.strips.forEach((s, i) => {
      const [le0, le1, te1, te0] = s.corners;
      const color = attachedColor(s.attached).withAlpha(0.85);
      const lines = hatches[i]!;
      lines.forEach((line, k) => {
        const t = (k + 0.5) / lines.length;
        // Along the span to this line, then just off the strip on its lift side.
        const across = (u: V3, v: V3): number[] =>
          u.map((x, j) => x + (v[j]! - x) * t + s.up[j]! * HATCH.above);
        line.material.uniforms.color = color;
        setLine(line, [at(f, across(le0, le1)), at(f, across(te0, te1))]);
      });
    });
    // The root chord, and on it the centre of mass, the centre of pressure, the neutral point.
    const root = d.strips
      .filter((s) => Math.abs(s.up[1]) < 0.5)
      .flatMap((s) => s.corners)
      .filter((c) => Math.abs(c[1]) < 0.002);
    if (!root.length) return;
    const xs = root.map((c) => c[0]);
    const z = -RULER.above;
    setLine(this.ruler, [at(f, [Math.max(...xs), 0, z]), at(f, [Math.min(...xs), 0, z])]);
    const margin = Number.isFinite(nullable(d.static_margin))
      ? ` ${fmt(nullable(d.static_margin, 100), 1)}%`
      : "";
    const places = { cg: 0, cp: d.centre_of_pressure, np: d.neutral_point };
    for (const key of ["cg", "cp", "np"] as const) {
      const mark = this.marks[key];
      const x = places[key];
      const top = x === null || x === undefined ? null : at(f, [x, 0, z - RULER.marks[key]]);
      if (x === null || x === undefined || !top || !finite(top)) {
        mark.line.show = mark.label.show = false;
        continue;
      }
      setLine(mark.line, [at(f, [x, 0, z]), top]);
      mark.label.show = true;
      mark.label.position = top;
      if (key === "np") mark.label.text = `NP${margin}`;
    }
  }

  private drawRotation(d: Dynamics, f: Frame): void {
    const moment = d.total_moment;
    ARCS.forEach((arc, axis) => {
      const sweep = (v: number, scale: number): number =>
        Math.sign(v) * ARC.max * Math.tanh(Math.abs(v) / scale);
      const draw = (line: C.Polyline, angle: number, radius: number): void => {
        if (!(Math.abs(angle) > C.Math.toRadians(3))) {
          line.show = false;
          return;
        }
        const positions: C.Cartesian3[] = [];
        for (let k = 0; k <= ARC.steps; k++) {
          const t = arc.start + (angle * k) / ARC.steps;
          const v = arc.centre.map(
            (c, j) => c + radius * (Math.cos(t) * arc.e1[j]! + Math.sin(t) * arc.e2[j]!),
          );
          positions.push(at(f, v));
        }
        setLine(line, positions);
      };
      const arcs = this.arcs[axis]!;
      draw(arcs.moment, sweep(moment[axis]!, ARC.moment), arc.radius);
      draw(arcs.rate, sweep(d.rates[axis]!, ARC.rate), arc.radius * 1.3);
    });
  }

  /** The trail's points, kept whether it shows or not: each a segment from the one before. */
  private record(d: Dynamics, p: C.Cartesian3, now: number): void {
    if (now - this.trailAt < TRAIL.every || !finite(p)) return;
    this.trailAt = now;
    const trail = this.trail;
    const last = trail[trail.length - 1];
    if (last && V.distance(last.p, p) > TRAIL.jump) this.clearTrail();
    else if (last && V.distance(last.p, p) < 0.01) return;
    const values = TRAIL_MODES.map((m) => m.of(d));
    const before = this.trail[this.trail.length - 1];
    const line = before
      ? this.lines.path.add({
          positions: [before.p, V.clone(p)],
          id: DYNAMICS_ID,
          width: TRAIL.px,
          material: C.Material.fromType(C.Material.ColorType, {
            color: ramp(TRAIL_MODES[this.trailMode]!.stops, values[this.trailMode]!),
          }),
        })
      : null;
    this.trail.push({ p: V.clone(p), t: now, values, line });
    while (this.trail.length && now - this.trail[0]!.t > TRAIL.seconds * 1000) {
      const old = this.trail.shift();
      if (old?.line) this.lines.path.remove(old.line);
    }
  }

  private clearTrail(): void {
    for (const point of this.trail) if (point.line) this.lines.path.remove(point.line);
    this.trail = [];
  }

  // -- beside it -------------------------------------------------------------------------------

  private buildNumbers(): void {
    const title = el("div", "dyn-title");
    title.append(
      el("span", "", "DYNAMICS · wingsim"),
      el("span", "", "Shift+1-5 layers · [ ] size"),
    );
    const grid = el("div", "dyn-grid");
    this.cells = SECTIONS.map((sec) => {
      const box = el("section");
      const head = el("h3");
      head.append(
        el("span", "", sec.title),
        ...[0, 1, 2].map((k) => el("span", "", sec.cols[k] ?? "")),
      );
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
    const spanHead = el(
      "h3",
      "dyn-wide",
      "STRIPS across the span, from behind: α eff (bar) against the stall (white; dashed: delayed by the lead), α before the downwash (outline); CL",
    );
    this.span = el("canvas", "dyn-span");
    const table = el("table", "dyn-strips");
    const thead = el("thead");
    const heads = el("tr");
    heads.append(el("th", "", ""), ...STRIP_COLS.map(([h]) => el("th", "", h)));
    const units = el("tr", "units");
    units.append(el("th", "", ""), ...STRIP_COLS.map(([, u]) => el("th", "", u)));
    thead.append(heads, units);
    this.tbody = el("tbody");
    table.append(thead, this.tbody);
    this.numbersEl.append(title, grid, spanHead, this.span, table);
    this.hist = el("canvas");
    this.historyEl.append(el("div", "dyn-title", `DYNAMICS · LAST ${HISTORY_S} s`), this.hist);
  }

  /** The legend: a chip per layer (and the numbers), and what the layers on draw. */
  private buildLegend(): void {
    const swatch = (color: string, text: string): HTMLElement => {
      const s = el("span", "dyn-swatch");
      const i = el("i");
      i.style.background = color;
      s.append(i, text);
      return s;
    };
    const chip = (
      text: string,
      on: boolean,
      title: string,
      onClick: () => void,
    ): HTMLButtonElement => {
      const b = el("button", on ? "dyn-chip on" : "dyn-chip", text);
      b.type = "button";
      b.title = title;
      b.setAttribute("aria-pressed", String(on));
      b.addEventListener("click", onClick);
      return b;
    };
    const keys = el("div", "dyn-chips");
    LAYERS.forEach((l, k) => {
      keys.append(
        chip(`${k + 1} ${l.name}`, this.layers[l.key], `Shift+${k + 1}`, () => this.toggleLayer(k)),
      );
    });
    keys.append(
      chip("Numbers", this.numbersShown, "the model's numbers and history", () => {
        this.numbersShown = !this.numbersShown;
        this.numbersEl.hidden = this.historyEl.hidden = !(this.on && this.numbersShown);
        this.legendEl.classList.toggle("beside", this.numbersShown);
        this.buildLegend();
      }),
    );
    const rows: HTMLElement[] = [keys];
    for (const l of LAYERS.filter((x) => this.layers[x.key])) {
      const row = el("div", "dyn-legend");
      row.append(el("b", "", l.name));
      for (const [color, text] of l.legend) row.append(swatch(color, text));
      if (l.key === "forces") {
        const smaller = el("button", "dyn-mini", "−");
        smaller.title = "smaller arrows ([)";
        smaller.addEventListener("click", () => this.resize(false));
        const bigger = el("button", "dyn-mini", "+");
        bigger.title = "bigger arrows (])";
        bigger.addEventListener("click", () => this.resize(true));
        row.append(el("span", "", this.scaleText()), smaller, bigger);
      }
      if (l.key === "path") {
        const mode = TRAIL_MODES[this.trailMode]!;
        const stops = mode.stops;
        const lo = stops[0]![0];
        const hi = stops[stops.length - 1]![0];
        const gradient = stops.map(([v, c]) => `${c} ${((v - lo) / (hi - lo)) * 100}%`).join(", ");
        const next = TRAIL_MODES[(this.trailMode + 1) % TRAIL_MODES.length]!;
        const cycle = el("button", "dyn-mini wide", `colour by ${next.name}`);
        cycle.addEventListener("click", () => this.cycleTrail());
        row.append(
          swatch(
            `linear-gradient(90deg, ${gradient})`,
            `${mode.name}, ${lo} to ${hi} ${mode.unit}`,
          ),
          el("span", "", `last ${TRAIL.seconds} s`),
          cycle,
        );
      }
      rows.push(row);
    }
    this.legendEl.replaceChildren(...rows);
  }

  private fillNumbers(d: Dynamics): void {
    SECTIONS.forEach((sec, si) => {
      sec.rows.forEach(([, value, digits], ri) => {
        const values = value(d);
        this.cells[si]![ri]!.forEach((cell, k) => {
          const text =
            k < values.length ? fmt(values[k], Array.isArray(digits) ? digits[k]! : digits) : "";
          if (cell.textContent !== text) cell.textContent = text;
        });
      });
    });
  }

  private fillStrips(d: Dynamics): void {
    if (this.named?.length !== d.strips.length) {
      this.tbody.replaceChildren();
      this.named = nameStrips(d.strips);
      this.stripRows = this.named.map((n) => {
        const tr = el("tr");
        const name = el("td", "", n.name);
        const cells = STRIP_COLS.map(() => el("td"));
        tr.append(name, ...cells);
        this.tbody.append(tr);
        return { name, cells };
      });
      if (!d.strips.length) {
        const tr = el("tr");
        const td = el("td", "", "No air on it: in the hand, or down.");
        td.colSpan = STRIP_COLS.length + 1;
        tr.append(td);
        this.tbody.append(tr);
      }
    }
    this.named.forEach((n, row) => {
      const s = d.strips[n.i];
      const r = this.stripRows[row];
      if (!s || !r) return;
      n.s = s;
      STRIP_COLS.forEach(([, , value, digits], k) => {
        const text = fmt(value(s), digits);
        const cell = r.cells[k]!;
        if (cell.textContent !== text) cell.textContent = text;
      });
      r.name.style.color = attachedColor(s.attached).toCssColorString();
    });
  }

  private canvas(
    canvas: HTMLCanvasElement,
    height: number,
  ): { g: CanvasRenderingContext2D; width: number } {
    const dpr = window.devicePixelRatio || 1;
    const width = canvas.clientWidth;
    if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) {
      canvas.width = Math.round(width * dpr);
      canvas.height = Math.round(height * dpr);
      canvas.style.height = `${height}px`;
    }
    const g = canvas.getContext("2d")!;
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, width, height);
    g.font = "10px 'JetBrains Mono', ui-monospace, monospace";
    return { g, width };
  }

  /** The strips across the span: their angles of attack against their stall, and their lift. */
  private drawSpan(): void {
    const { g, width } = this.canvas(this.span, 142);
    const wing = (this.named ?? []).filter((n) => n.name !== "fin");
    if (!wing.length) return;
    const ys = wing.map((n) => n.s.at[1]);
    // Each strip's edges: halfway to its neighbours, as far again at the tips.
    const edges = ys.map((y, k) => {
      const lo = k > 0 ? (ys[k - 1]! + y) / 2 : y - ((ys[1] ?? y + 0.05) - y) / 2;
      const hi = k < ys.length - 1 ? (ys[k + 1]! + y) / 2 : y + (y - (ys[k - 1] ?? y - 0.05)) / 2;
      return [lo, hi] as const;
    });
    const span = [edges[0]![0], edges[edges.length - 1]![1]] as const;
    const pad = 34;
    const x = (y: number): number =>
      pad + ((y - span[0]) / (span[1] - span[0])) * (width - pad - 4);
    const plots = [
      { top: 4, height: 80, range: SPAN_ALPHA, label: "α °", ticks: [-10, 0, 10, 20] },
      { top: 92, height: 36, range: SPAN_CL, label: "CL", ticks: [0, 1] },
    ].map((plot) => {
      const raw = (v: number): number =>
        plot.top + plot.height * (1 - (v - plot.range[0]) / (plot.range[1] - plot.range[0]));
      return {
        ...plot,
        yOf: (v: number) => Math.max(plot.top, Math.min(plot.top + plot.height, raw(v))),
      };
    });
    for (const plot of plots) {
      g.strokeStyle = "rgba(238,240,247,0.08)";
      g.fillStyle = "#8e93ad";
      for (const t of plot.ticks) {
        g.beginPath();
        g.moveTo(pad, plot.yOf(t));
        g.lineTo(width - 4, plot.yOf(t));
        g.stroke();
        g.fillText(String(t), pad - 6 - g.measureText(String(t)).width, plot.yOf(t) + 3);
      }
      g.fillStyle = "#eef0f7";
      g.fillText(plot.label, width - 4 - g.measureText(plot.label).width, plot.top + 9);
    }
    const [pa, pc] = plots as [(typeof plots)[0], (typeof plots)[0]];
    wing.forEach((n, k) => {
      const s = n.s;
      const [x0, x1] = [x(edges[k]![0]) + 1, x(edges[k]![1]) - 1];
      g.fillStyle = attachedColor(s.attached).toCssColorString();
      g.fillRect(
        x0,
        Math.min(pa.yOf(0), pa.yOf(s.seen)),
        x1 - x0,
        Math.abs(pa.yOf(s.seen) - pa.yOf(0)),
      );
      g.fillRect(
        x0,
        Math.min(pc.yOf(0), pc.yOf(s.cl)),
        x1 - x0,
        Math.abs(pc.yOf(s.cl) - pc.yOf(0)),
      );
      g.strokeStyle = "rgba(238,240,247,0.55)";
      g.strokeRect(
        x0 + 0.5,
        Math.min(pa.yOf(0), pa.yOf(s.alpha)) + 0.5,
        x1 - x0 - 1,
        Math.abs(pa.yOf(s.alpha) - pa.yOf(0)),
      );
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
      g.fillStyle = "#8e93ad";
      g.fillText(n.name, (x0 + x1) / 2 - g.measureText(n.name).width / 2, pc.top + pc.height + 11);
    });
  }

  private remember(d: Dynamics, now: number): void {
    const h = this.history;
    h.push({
      t: now,
      v: LANES.map(([, value]) => value(d)),
      m: LANES.map(([, , , , mark]) => mark?.(d) ?? Number.NaN),
    });
    while (h.length && now - h[0]!.t > HISTORY_S * 1000) h.shift();
  }

  private drawHistory(now: number): void {
    const laneH = 40;
    const { g, width } = this.canvas(this.hist, LANES.length * laneH);
    const h = this.history;
    const pad = 4;
    const x = (t: number): number => pad + (1 - (now - t) / (HISTORY_S * 1000)) * (width - 2 * pad);
    LANES.forEach(([label, , range, digits], k) => {
      const top = k * laneH;
      const y = (v: number): number =>
        top +
        3 +
        (laneH - 6) *
          (1 - (Math.max(range[0], Math.min(range[1], v)) - range[0]) / (range[1] - range[0]));
      g.strokeStyle = "rgba(238,240,247,0.08)";
      g.strokeRect(pad + 0.5, top + 0.5, width - 2 * pad - 1, laneH - 1);
      if (range[0] < 0 && range[1] > 0) {
        g.beginPath();
        g.moveTo(pad, y(0));
        g.lineTo(width - pad, y(0));
        g.stroke();
      }
      const trace = (of: (s: (typeof h)[0]) => number): void => {
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
      trace((s) => s.m[k]!);
      g.setLineDash([]);
      g.strokeStyle = COLORS.flow;
      trace((s) => s.v[k]!);
      const last = h[h.length - 1]?.v[k];
      g.fillStyle = "#eef0f7";
      g.fillText(`${label} ${fmt(last, digits)}`, pad + 4, top + 12);
      g.fillStyle = "#8e93ad";
      g.fillText(`${range[1]}`, width - pad - 24, top + 12);
      g.fillText(`${range[0]}`, width - pad - 24, top + laneH - 4);
    });
  }
}
