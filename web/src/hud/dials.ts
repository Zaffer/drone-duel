// The aviation dials (DESIGN.md §11, bottom left in flight): six small dials drawn as SkyFlow
// draws its instrument strip (thin outlines, one needle or line, one number under each), with
// the pilot's sticks on meters round the edge as QGroundControl draws them:
//
//                                              yaw          roll
//   throttle ( altitude ) ( climb ) ( speed ) ( compass ) ( attitude ) pitch
//
// The altimeter is an aircraft's counter-drum-pointer: the needle turns once every 100 m (or
// 1000 ft) over a 0-9 scale, and a drum counter under it rolls the height.
//
// Every value is what a ground station sees (the `telemetry` signal, DESIGN.md §2.2): the
// autopilot's own estimates, never the physics' truth. The sticks are the radio's channels as
// the game sends them (the `channels` signal). Colours are Dusk's, after glass-cockpit practice:
// light scales, green live values, a yellow aircraft symbol, sky-blue pilot input (hud.css).

import { effect } from "alien-signals";
import { loop } from "../engine/loop.ts";
import type { TelemetryMsg } from "../net/protocol.ts";
import { vehicle } from "../state/app.ts";
import { channels } from "../state/input.ts";
import { prefs } from "../state/prefs.ts";
import { telemetry } from "../state/session.ts";
import {
  altNeedleDeg,
  CLIMB_T0,
  CY,
  climbNeedleDeg,
  climbText,
  drumPositions,
  headingOf,
  headingText,
  horizonDy,
  R,
  type Scales,
  SPEED_T0,
  SWEEP,
  scalesFor,
  speedNeedleDeg,
  stickMeters,
} from "./dialmath.ts";
import "./hud.css";

const SVG_NS = "http://www.w3.org/2000/svg";
/** Centre to centre: the dials sit side by side, with no meter between them. */
const DIAL_STEP = 2 * R + 5;
const HEIGHT = 75;
const METER_GAP = 7;
const METER_W = 4;
/** The altimeter's drum counter: each digit's width and line height. */
const DRUM_W = 7.5;
const DRUM_LINE = 13;

const f = (x: number): string => x.toFixed(1);

let nextId = 0;

function el<K extends keyof SVGElementTagNameMap>(
  name: K,
  attrs: Record<string, string | number> = {},
  parent: Element | null = null,
): SVGElementTagNameMap[K] {
  const node = document.createElementNS(SVG_NS, name);
  for (const [k, v] of Object.entries(attrs)) node.setAttribute(k, String(v));
  parent?.append(node);
  return node;
}

/** A horizontal meter above a dial centred at `cx`; returns its marker. */
function meterAbove(svg: SVGElement, cx: number): SVGRectElement {
  const g = el("g", { class: "meter" }, svg);
  el("rect", { class: "track", x: cx - R, y: 4, width: 2 * R, height: METER_W, rx: 2 }, g);
  el("line", { class: "zero", x1: cx, y1: 2, x2: cx, y2: 4 + METER_W + 2 }, g);
  return el("rect", { class: "mark", x: -1.5, y: 2, width: 3, height: METER_W + 4, rx: 1 }, g);
}

/** A vertical meter with its left edge at `x`; returns its marker and fill. */
function meterSide(svg: SVGElement, x: number, zeroLine = true) {
  const g = el("g", { class: "meter" }, svg);
  el("rect", { class: "track", x, y: CY - R, width: METER_W, height: 2 * R, rx: 2 }, g);
  if (zeroLine) el("line", { class: "zero", x1: x - 2, y1: CY, x2: x + METER_W + 2, y2: CY }, g);
  const fill = el("rect", { class: "fill", x, y: CY + R, width: METER_W, height: 0, rx: 2 }, g);
  const mark = el(
    "rect",
    { class: "mark", x: x - 2, y: -1.5, width: METER_W + 4, height: 3, rx: 1 },
    g,
  );
  return { mark, fill };
}

function ticks(g: SVGElement, cx: number, angles: number[], marked: number[] = []): void {
  for (const a of angles) {
    const c = Math.cos(a);
    const s = Math.sin(a);
    el(
      "line",
      {
        class: marked.includes(a) ? "tick ref" : "tick",
        x1: cx + c * (R - 5),
        y1: CY + s * (R - 5),
        x2: cx + c * R,
        y2: CY + s * R,
      },
      g,
    );
  }
}

/** A 270° scale from angle `t0`, clockwise, with five ticks (`marked` the reference ones) and
 * the dial's name in the gap at (`dx`, `dy`) from its centre; returns its needle. */
function scale(
  svg: SVGElement,
  cx: number,
  t0: number,
  marked: number[],
  name: string,
  dx: number,
  dy: number,
): SVGLineElement {
  const end = t0 + SWEEP;
  el(
    "path",
    {
      class: "rim",
      d:
        `M${f(cx + R * Math.cos(t0))} ${f(CY + R * Math.sin(t0))} ` +
        `A${R} ${R} 0 1 1 ${f(cx + R * Math.cos(end))} ${f(CY + R * Math.sin(end))}`,
    },
    svg,
  );
  const steps = [0, 0.25, 0.5, 0.75, 1].map((t) => t0 + t * SWEEP);
  ticks(
    svg,
    cx,
    steps,
    marked.map((i) => steps[i]!),
  );
  el("text", { class: "label", x: cx + dx, y: CY + dy, "text-anchor": "middle" }, svg).textContent =
    name;
  const needle = el("line", { class: "needle", x1: 0, y1: 0, x2: 0.85 * R, y2: 0 }, svg);
  el("circle", { class: "hub", cx, cy: CY, r: 1.8 }, svg);
  return needle;
}

function readout(svg: SVGElement, cx: number, title: string): SVGTextElement {
  el("title", {}, el("circle", { class: "hit", cx, cy: CY, r: R }, svg)).textContent = title;
  return el("text", { class: "value", x: cx, y: CY + R + 14, "text-anchor": "middle" }, svg);
}

/** The drum counter's window under a dial centred at `cx`: a column per digit, each a strip of
 * 0-9 and 0 again that rolls up through the window. Returns the columns, most significant
 * first. */
function drum(svg: SVGElement, cx: number, id: string, digits: number): SVGGElement[] {
  const width = digits * DRUM_W + 4;
  const x = cx - width / 2;
  const y = CY + R + 2;
  const clip = el("clipPath", { id }, el("defs", {}, svg));
  el("rect", { x, y, width, height: DRUM_LINE }, clip);
  el("rect", { class: "drum", x, y, width, height: DRUM_LINE, rx: 1.5 }, svg);
  const pane = el("g", { "clip-path": `url(#${id})` }, svg);
  const columns: SVGGElement[] = [];
  for (let i = 0; i < digits; i++) {
    const strip = el("g", {}, pane);
    const dx = x + 2 + (i + 0.5) * DRUM_W;
    for (let d = 0; d <= 10; d++) {
      const dy = y + DRUM_LINE - 2.8 + d * DRUM_LINE;
      el("text", { class: "digit", x: dx, y: dy, "text-anchor": "middle" }, strip).textContent =
        String(d % 10);
    }
    columns.push(strip);
  }
  return columns;
}

// What moves is written only when it changes. The sticks redraw the dials every frame, and an
// SVG takes every write, the same value or not, as a change: restyled and painted again (with
// the minimap, a quarter of a frame's time on a phone-speed thread, measured 2026-11-22). The
// telemetry comes ten times a second; between, nothing is written.
const put = (node: Element, name: string, value: string): void => {
  if (node.getAttribute(name) !== value) node.setAttribute(name, value);
};
const say = (node: Element, text: string): void => {
  if (node.textContent !== text) node.textContent = text;
};

const rotate = (node: Element, cx: number, cy: number, angle: number): void =>
  put(node, "transform", `translate(${f(cx)} ${f(cy)}) rotate(${f(angle)})`);

/** The dials as one SVG, for `scales`. */
class Dials {
  readonly svg: SVGSVGElement;
  private readonly cx: { alt: number; vs: number; gs: number; hdg: number; att: number };
  private readonly altNeedle: SVGLineElement;
  private readonly drum: SVGGElement[];
  private readonly climbNeedle: SVGLineElement;
  private readonly vsText: SVGTextElement;
  private readonly speedNeedle: SVGLineElement;
  private readonly gsText: SVGTextElement;
  private readonly needle: SVGLineElement;
  private readonly hdgText: SVGTextElement;
  private readonly horizon: SVGLineElement;
  private readonly bank: SVGLineElement;
  private readonly attText: SVGTextElement;
  private readonly throttleMeter: { mark: SVGRectElement; fill: SVGRectElement };
  private readonly pitchMeter: { mark: SVGRectElement; fill: SVGRectElement };
  private readonly rollMeter: SVGRectElement;
  private readonly yawMeter: SVGRectElement;

  constructor(readonly scales: Scales) {
    const id = `dials-${nextId++}`;
    const width = 2 * (4 + METER_W + METER_GAP + R) + 4 * DIAL_STEP;
    const svg = el("svg", {
      class: "dials",
      viewBox: `0 0 ${width} ${HEIGHT}`,
      "aria-hidden": "true",
    });
    this.svg = svg;
    el(
      "rect",
      { class: "panel", x: 0.5, y: 0.5, width: width - 1, height: HEIGHT - 1, rx: 6 },
      svg,
    );

    // Throttle, then the altimeter: a needle over 0-9 tenths of a turn, the drum under it.
    this.throttleMeter = meterSide(svg, 4, false);
    let cx = 4 + METER_W + METER_GAP + R;
    const alt = cx;
    el("circle", { class: "rim", cx, cy: CY, r: R }, svg);
    const tens = [...Array(10).keys()].map((i) => (i / 10) * 2 * Math.PI - Math.PI / 2);
    ticks(svg, cx, tens, [tens[0]!]);
    for (const [i, a] of tens.entries()) {
      const x = f(cx + (R - 9.5) * Math.cos(a));
      const y = f(CY + (R - 9.5) * Math.sin(a) + 2.3);
      el("text", { class: "number", x, y, "text-anchor": "middle" }, svg).textContent = String(i);
    }
    this.altNeedle = el(
      "line",
      { class: "needle", x1: 0, y1: 0.25 * R, x2: 0, y2: -0.82 * R },
      svg,
    );
    el("circle", { class: "hub", cx, cy: CY, r: 1.8 }, svg);
    el("title", {}, el("circle", { class: "hit", cx, cy: CY, r: R }, svg)).textContent =
      `altitude above home, ${scales.alt.unit}: the needle turns once every ${scales.alt.turn} ${scales.alt.unit}`;
    this.drum = drum(svg, cx, `${id}-drum`, scales.alt.digits);

    // Climb as an aircraft's vertical speed indicator: 9 o'clock level, up climbing.
    cx += DIAL_STEP;
    const vs = cx;
    this.climbNeedle = scale(svg, cx, CLIMB_T0, [2], "VS", R - 6, 3);
    this.vsText = readout(
      svg,
      cx,
      `vertical speed, ${scales.climb.unit} (±${scales.climb.full} full scale)`,
    );

    // Ground speed: 0 to full scale clockwise from 7:30.
    cx += DIAL_STEP;
    const gs = cx;
    this.speedNeedle = scale(svg, cx, SPEED_T0, [0, 4], "GS", 0, R - 2);
    this.gsText = readout(
      svg,
      cx,
      `ground speed, ${scales.speed.unit} (${scales.speed.full} full scale)`,
    );

    // Compass, north up: the needle points along the heading.
    cx += DIAL_STEP;
    const hdg = cx;
    el("circle", { class: "rim", cx, cy: CY, r: R }, svg);
    ticks(svg, cx, [-Math.PI / 2, 0, Math.PI / 2, Math.PI], [-Math.PI / 2]);
    el(
      "text",
      { class: "north", x: cx, y: CY - R + 11, "text-anchor": "middle" },
      svg,
    ).textContent = "N";
    this.needle = el("line", { class: "needle", x1: 0, y1: 0.35 * R, x2: 0, y2: -0.8 * R }, svg);
    el("circle", { class: "hub", cx, cy: CY, r: 1.8 }, svg);
    this.yawMeter = meterAbove(svg, cx);
    this.hdgText = readout(svg, cx, "heading, degrees true");

    // Attitude: a horizon line rolled with the aircraft and moved by pitch, a bank pointer,
    // and the fixed aircraft symbol.
    cx += DIAL_STEP;
    const att = cx;
    const clip = el("clipPath", { id: `${id}-att` }, el("defs", {}, svg));
    el("circle", { cx, cy: CY, r: R - 1 }, clip);
    el("circle", { class: "rim", cx, cy: CY, r: R }, svg);
    const inside = el("g", { "clip-path": `url(#${id}-att)` }, svg);
    this.horizon = el("line", { class: "horizon", x1: -R, y1: 0, x2: R, y2: 0 }, inside);
    this.bank = el("line", { class: "bank", x1: 0, y1: -(R - 7), x2: 0, y2: -(R - 1) }, svg);
    el(
      "path",
      {
        class: "wings",
        d: `M${cx - 9} ${CY} h6 v2.5 M${cx + 9} ${CY} h-6 v2.5 M${cx} ${CY} h0.01`,
      },
      svg,
    );
    this.rollMeter = meterAbove(svg, cx);
    this.pitchMeter = meterSide(svg, cx + R + METER_GAP);
    this.attText = readout(svg, cx, "roll and pitch, degrees");
    this.cx = { alt, vs, gs, hdg, att };
    this.telemetry(null);
    this.sticks(null);
  }

  /** The dials from what a ground station sees; at rest without it. */
  telemetry(t: TelemetryMsg | null): void {
    const s = this.scales;
    this.svg.classList.toggle("no-data", t === null);
    const alt = (t?.alt_rel ?? 0) * s.alt.perM;
    rotate(this.altNeedle, this.cx.alt, CY, altNeedleDeg(alt, s.alt.turn));
    for (const [i, pos] of drumPositions(Math.max(alt, 0), s.alt.digits).entries()) {
      const column = this.drum[i];
      if (column) put(column, "transform", `translate(0 ${f(-pos * DRUM_LINE)})`);
    }

    const roll = t?.roll_deg ?? 0;
    const pitch = t?.pitch_deg ?? 0;
    const { att } = this.cx;
    put(
      this.horizon,
      "transform",
      `translate(${att} ${f(CY + horizonDy(pitch))}) rotate(${f(-roll)})`,
    );
    rotate(this.bank, att, CY, -roll);
    say(this.attText, t ? `R${Math.round(roll)} P${Math.round(pitch)}` : "–");

    rotate(this.needle, this.cx.hdg, CY, headingOf(t?.yaw_deg ?? 0));
    say(this.hdgText, t ? headingText(t.yaw_deg) : "–");

    const climb = (t?.climb ?? 0) * s.climb.perMs;
    rotate(this.climbNeedle, this.cx.vs, CY, climbNeedleDeg(climb, s.climb.full));
    say(this.vsText, t ? climbText(climb, s.climb.decimals) : "–");

    const speed = (t?.groundspeed ?? 0) * s.speed.perMs;
    rotate(this.speedNeedle, this.cx.gs, CY, speedNeedleDeg(speed, s.speed.full));
    say(this.gsText, t ? speed.toFixed(s.speed.perMs === 1 ? 1 : 0) : "–");
  }

  /** The stick meters; hidden without sticks. */
  sticks(m: ReturnType<typeof stickMeters>): void {
    this.svg.classList.toggle("no-sticks", m === null);
    if (!m) return;
    const across = (mark: SVGRectElement, cx0: number, v: number): void =>
      put(mark, "x", f(cx0 + v * (R - 2) - 1.5));
    const up = (mark: SVGRectElement, v: number): void => put(mark, "y", f(CY - v * (R - 2) - 1.5));
    across(this.rollMeter, this.cx.att, m.roll);
    up(this.pitchMeter.mark, m.pitch);
    across(this.yawMeter, this.cx.hdg, m.yaw);
    up(this.throttleMeter.mark, 2 * m.throttle - 1);
    put(this.throttleMeter.fill, "y", f(CY + R - 2 * R * m.throttle));
    put(this.throttleMeter.fill, "height", f(2 * R * m.throttle));
  }
}

/** The dials in `el`, kept to the telemetry and the sticks (drawn once a frame at most, in the
 * frame loop's place phase); rebuilt when the units or the vehicle change. Returns what takes
 * them down. */
export function mountDials(el: HTMLElement): () => void {
  let dials: Dials | null = null;
  let key = "";
  let pending = false;
  const stopLoop = new AbortController();

  const draw = (): void => {
    pending = false;
    const v = vehicle();
    const units = prefs().hud.units;
    const kind = v?.kind ?? "multirotor";
    const k = `${units}/${kind}`;
    if (!dials || k !== key) {
      key = k;
      dials?.svg.remove();
      dials = new Dials(scalesFor(units, kind));
      el.append(dials.svg);
    }
    dials.telemetry(telemetry());
    // The channels are natural (stick forward +1) whatever the firmware (input/mapping.ts).
    dials.sticks(stickMeters(channels(), false));
  };

  draw();
  loop.each("dials", "place", () => pending && draw(), stopLoop.signal);
  const stop = effect(() => {
    // Read what the dials show, so a change of any of them draws the next frame.
    telemetry();
    channels();
    prefs();
    vehicle();
    pending = true;
  });
  return () => {
    stop();
    stopLoop.abort();
    dials?.svg.remove();
    dials = null;
  };
}
