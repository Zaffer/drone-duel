// Five small dials for a fleet card, drawn as SkyFlow draws its instrument strip (thin
// outlines, one needle or line, one number under each), with the pilot's stick input on
// meters round the edge as QGroundControl draws them:
//
//                                              yaw          roll
//   throttle ( altitude ) ( climb ) ( speed ) ( compass ) ( attitude ) pitch
//
// The altimeter is an aircraft's counter-drum-pointer: the needle turns once every
// ALT_TURN_M over a 0-9 scale, and a drum counter under it rolls the metres.
//
// Every value is the autopilot's own estimate, as MAVLink reports it (swarm.link): altitude
// above home from its EKF, mostly the barometer; climb and ground speed from the EKF's
// velocity, GPS and accelerometers fused; heading from gyros and compass; attitude from the
// gyros and accelerometers. None of it is raw GPS.
//
// Colours follow glass-cockpit practice on black: white scales, green live data, a yellow
// aircraft symbol, cyan for pilot input (the .card-instruments rules in style.css).
//
// Sticks come from RC_CHANNELS. On a drone the autopilot flies they are whatever its
// receiver last saw, so the meters are dimmed there.

const SVG_NS = "http://www.w3.org/2000/svg";
const R = 21;
/** Centre to centre: the dials sit side by side, with no meter between them. */
const DIAL_STEP = 2 * R + 5;
const CY = 35;
const HEIGHT = 75;
const METER_GAP = 7;
const METER_W = 4;
/** Pitch at which the horizon line reaches the dial's edge (times 0.8). */
const PITCH_FULL_DEG = 45;
const CLIMB_FULL_MS = 5;
/** Ground speed at the end of the speed dial's scale. */
const SPEED_FULL_MS = 20;
/** Metres for one turn of the altimeter's needle. */
const ALT_TURN_M = 100;
/** The altimeter's drum counter: digits shown, their width and line height. */
const DRUM_DIGITS = 3;
const DRUM_W = 7.5;
const DRUM_LINE = 13;
/** Both scales are 270° arcs. Climb: zero at 9 o'clock, gap on the right. Speed: zero at
 * 7:30, clockwise, gap at the bottom. */
const SWEEP = 1.5 * Math.PI;
const CLIMB_T0 = 0.25 * Math.PI;
const SPEED_T0 = 0.75 * Math.PI;

const clamp = (x, lo, hi) => Math.min(Math.max(x, lo), hi);
const deg = (rad) => (rad * 180) / Math.PI;
/** A centred stick's pulse width as -1 (1000 µs) to 1 (2000 µs). */
const centred = (pwm) => clamp((pwm - 1500) / 500, -1, 1);
const f = (x) => x.toFixed(1);

let nextId = 0;

function el(name, attrs = {}, parent = null) {
  const node = document.createElementNS(SVG_NS, name);
  for (const [k, v] of Object.entries(attrs)) node.setAttribute(k, String(v));
  parent?.append(node);
  return node;
}

/** A horizontal meter above a dial centred at `cx`; returns its marker. */
function meterAbove(svg, cx) {
  const g = el("g", { class: "meter" }, svg);
  el("rect", { class: "track", x: cx - R, y: 4, width: 2 * R, height: METER_W, rx: 2 }, g);
  el("line", { class: "zero", x1: cx, y1: 2, x2: cx, y2: 4 + METER_W + 2 }, g);
  return el("rect", { class: "mark", x: -1.5, y: 2, width: 3, height: METER_W + 4, rx: 1 }, g);
}

/** A vertical meter with its left edge at `x`; returns its marker and fill. */
function meterSide(svg, x, zeroLine = true) {
  const g = el("g", { class: "meter" }, svg);
  el("rect", { class: "track", x, y: CY - R, width: METER_W, height: 2 * R, rx: 2 }, g);
  if (zeroLine) el("line", { class: "zero", x1: x - 2, y1: CY, x2: x + METER_W + 2, y2: CY }, g);
  const fill = el("rect", { class: "fill", x, y: CY + R, width: METER_W, height: 0, rx: 2 }, g);
  const mark = el("rect", { class: "mark", x: x - 2, y: -1.5, width: METER_W + 4, height: 3, rx: 1 }, g);
  return { mark, fill };
}

/** A 270° scale from angle `t0`, clockwise, with five ticks (`marked` are the reference
 * ones) and the dial's name written in the gap at (`dx`, `dy`) from its centre. */
function scale(svg, cx, t0, marked, name, dx, dy) {
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
  ticks(svg, cx, steps, marked.map((i) => steps[i]));
  el("text", { class: "label", x: cx + dx, y: CY + dy, "text-anchor": "middle" }, svg).textContent = name;
  const needle = el("line", { class: "needle", x1: 0, y1: 0, x2: 0.85 * R, y2: 0 }, svg);
  el("circle", { class: "hub", cx, cy: CY, r: 1.8 }, svg);
  return needle;
}

function readout(svg, cx, title) {
  el("title", {}, el("circle", { class: "hit", cx, cy: CY, r: R }, svg)).textContent = title;
  return el("text", { class: "value", x: cx, y: CY + R + 14, "text-anchor": "middle" }, svg);
}

function ticks(g, cx, angles, marked = []) {
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

/** The drum counter's window under a dial centred at `cx`: a column per digit, each a strip
 * of 0-9 and 0 again that rolls up through the window. Returns the columns, most
 * significant first. */
function drum(svg, cx, id) {
  const width = DRUM_DIGITS * DRUM_W + 4;
  const x = cx - width / 2;
  const y = CY + R + 2;
  const clip = el("clipPath", { id }, el("defs", {}, svg));
  el("rect", { x, y, width, height: DRUM_LINE }, clip);
  el("rect", { class: "drum", x, y, width, height: DRUM_LINE, rx: 1.5 }, svg);
  const pane = el("g", { "clip-path": `url(#${id})` }, svg);
  const columns = [];
  for (let i = 0; i < DRUM_DIGITS; i++) {
    const strip = el("g", {}, pane);
    const dx = x + 2 + (i + 0.5) * DRUM_W;
    for (let d = 0; d <= 10; d++) {
      const dy = y + DRUM_LINE - 2.8 + d * DRUM_LINE;
      el("text", { class: "digit", x: dx, y: dy, "text-anchor": "middle" }, strip).textContent = String(d % 10);
    }
    columns.push(strip);
  }
  return columns;
}

/** Each drum column's position for `metres`, 0 to 10 digits up its strip: the units turn
 * with the altitude, and each higher digit turns over in the last metre before it changes,
 * as an odometer's do. */
export function drumPositions(metres) {
  const m = clamp(metres, 0, 10 ** DRUM_DIGITS - 1e-6);
  const out = [];
  for (let i = DRUM_DIGITS - 1; i >= 0; i--) {
    const place = 10 ** i;
    const below = m % place;
    const digit = Math.floor(m / place) % 10;
    // Turning over: everything below this digit reads 9s and the units are rolling to 0.
    out.push(i === 0 ? m % 10 : digit + Math.max(0, below - (place - 1)));
  }
  return out;
}

export class Instruments {
  constructor() {
    const id = `inst-${nextId++}`;
    const width = 2 * (4 + METER_W + METER_GAP + R) + 4 * DIAL_STEP;
    this.svg = el("svg", {
      class: "instruments",
      viewBox: `0 0 ${width} ${HEIGHT}`,
      width,
      height: HEIGHT,
      "aria-hidden": "true",
    });
    const svg = this.svg;
    el("rect", { class: "panel", x: 0, y: 0, width, height: HEIGHT, rx: 6 }, svg);

    // Throttle, then the altimeter: a needle over 0-9 tens of metres, the drum under it.
    this.throttleMeter = meterSide(svg, 4, false);
    let cx = 4 + METER_W + METER_GAP + R;
    this.alt = { cx };
    el("circle", { class: "rim", cx, cy: CY, r: R }, svg);
    const tens = [...Array(10).keys()].map((i) => (i / 10) * 2 * Math.PI - Math.PI / 2);
    ticks(svg, cx, tens, [tens[0]]);
    for (const [i, a] of tens.entries()) {
      const x = f(cx + (R - 9.5) * Math.cos(a));
      const y = f(CY + (R - 9.5) * Math.sin(a) + 2.3);
      el("text", { class: "number", x, y, "text-anchor": "middle" }, svg).textContent = String(i);
    }
    this.altNeedle = el("line", { class: "needle", x1: 0, y1: 0.25 * R, x2: 0, y2: -0.82 * R }, svg);
    el("circle", { class: "hub", cx, cy: CY, r: 1.8 }, svg);
    el("title", {}, el("circle", { class: "hit", cx, cy: CY, r: R }, svg)).textContent =
      `altitude above home, m: the needle turns once every ${ALT_TURN_M} m`;
    this.drum = drum(svg, cx, `${id}-drum`);

    // Climb as an aircraft's vertical speed indicator: needle at 9 o'clock for level, up for
    // climbing, down for descending.
    cx += DIAL_STEP;
    this.vs = { cx };
    this.climbNeedle = scale(svg, cx, CLIMB_T0, [2], "VS", R - 6, 3);
    this.vsText = readout(svg, cx, `vertical speed, m/s (±${CLIMB_FULL_MS} full scale)`);

    // Ground speed: 0 to SPEED_FULL_MS clockwise from 7:30.
    cx += DIAL_STEP;
    this.gs = { cx };
    this.speedNeedle = scale(svg, cx, SPEED_T0, [0, 4], "GS", 0, R - 2);
    this.gsText = readout(svg, cx, `ground speed, m/s (${SPEED_FULL_MS} full scale)`);

    // Compass, north up: the needle points along the heading.
    cx += DIAL_STEP;
    this.hdg = { cx };
    el("circle", { class: "rim", cx, cy: CY, r: R }, svg);
    ticks(svg, cx, [-Math.PI / 2, 0, Math.PI / 2, Math.PI], [-Math.PI / 2]);
    el("text", { class: "north", x: cx, y: CY - R + 11, "text-anchor": "middle" }, svg).textContent = "N";
    this.needle = el("line", { class: "needle", x1: 0, y1: 0.35 * R, x2: 0, y2: -0.8 * R }, svg);
    el("circle", { class: "hub", cx, cy: CY, r: 1.8 }, svg);
    this.yawMeter = meterAbove(svg, cx);
    this.hdgText = readout(svg, cx, "heading, degrees true");

    // Attitude: a horizon line rolled with the drone and moved by pitch, a bank pointer,
    // and the fixed aircraft symbol.
    cx += DIAL_STEP;
    this.att = { cx };
    const clip = el("clipPath", { id: `${id}-att` }, el("defs", {}, svg));
    el("circle", { cx, cy: CY, r: R - 1 }, clip);
    el("circle", { class: "rim", cx, cy: CY, r: R }, svg);
    const inside = el("g", { "clip-path": `url(#${id}-att)` }, svg);
    this.horizon = el("line", { class: "horizon", x1: -R, y1: 0, x2: R, y2: 0 }, inside);
    this.bank = el("line", { class: "bank", x1: 0, y1: -(R - 7), x2: 0, y2: -(R - 1) }, svg);
    this.wings = el(
      "path",
      { class: "wings", d: `M${cx - 9} ${CY} h6 v2.5 M${cx + 9} ${CY} h-6 v2.5 M${cx} ${CY} h0.01` },
      svg,
    );
    this.rollMeter = meterAbove(svg, cx);
    this.pitchMeter = meterSide(svg, cx + R + METER_GAP);
    this.attText = readout(svg, cx, "roll and pitch, degrees");
  }

  /** Draw a frame vehicle. */
  update(v) {
    this.svg.classList.toggle("pilot", Boolean(v.pilot));

    const alt = Math.max(v.alt, 0);
    const turn = ((alt % ALT_TURN_M) / ALT_TURN_M) * 360;
    this.altNeedle.setAttribute("transform", `translate(${this.alt.cx} ${CY}) rotate(${f(turn)})`);
    for (const [i, pos] of drumPositions(alt).entries()) {
      this.drum[i].setAttribute("transform", `translate(0 ${f(-pos * DRUM_LINE)})`);
    }

    const roll = deg(v.roll);
    const pitch = deg(v.pitch);
    const dy = clamp(pitch / PITCH_FULL_DEG, -1, 1) * R * 0.8;
    const { cx } = this.att;
    this.horizon.setAttribute("transform", `translate(${cx} ${f(CY + dy)}) rotate(${f(-roll)})`);
    this.bank.setAttribute("transform", `translate(${cx} ${CY}) rotate(${f(-roll)})`);
    this.attText.textContent = `R${Math.round(roll)} P${Math.round(pitch)}`;

    const heading = (deg(v.heading) + 360) % 360;
    this.needle.setAttribute("transform", `translate(${this.hdg.cx} ${CY}) rotate(${f(heading)})`);
    this.hdgText.textContent = `${String(Math.round(heading) % 360).padStart(3, "0")}°`;

    const climb = -v.vd;
    const a = CLIMB_T0 + (0.5 + 0.5 * clamp(climb / CLIMB_FULL_MS, -1, 1)) * SWEEP;
    this.climbNeedle.setAttribute("transform", `translate(${this.vs.cx} ${CY}) rotate(${f(deg(a))})`);
    this.vsText.textContent = `${climb >= 0 ? "+" : "−"}${Math.abs(climb).toFixed(1)}`;

    const speed = Math.hypot(v.vn, v.ve);
    const b = SPEED_T0 + clamp(speed / SPEED_FULL_MS, 0, 1) * SWEEP;
    this.speedNeedle.setAttribute("transform", `translate(${this.gs.cx} ${CY}) rotate(${f(deg(b))})`);
    this.gsText.textContent = speed.toFixed(1);

    const [rollUs, pitchUs, throttleUs, yawUs] = v.sticks ?? [];
    const known = yawUs !== undefined;
    this.svg.classList.toggle("no-sticks", !known);
    if (!known) return;
    const across = (mark, cx0, s) => mark.setAttribute("x", f(cx0 + s * (R - 2) - 1.5));
    const up = (mark, s) => mark.setAttribute("y", f(CY - s * (R - 2) - 1.5));
    across(this.rollMeter, this.att.cx, centred(rollUs));
    up(this.pitchMeter.mark, -centred(pitchUs)); // stick forward is a low pulse, drawn up
    across(this.yawMeter, this.hdg.cx, centred(yawUs));
    const throttle = clamp((throttleUs - 1000) / 1000, 0, 1);
    up(this.throttleMeter.mark, 2 * throttle - 1);
    this.throttleMeter.fill.setAttribute("y", f(CY + R - 2 * R * throttle));
    this.throttleMeter.fill.setAttribute("height", f(2 * R * throttle));
  }
}
