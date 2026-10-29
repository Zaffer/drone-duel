// The radio calibration wizard (DESIGN.md §12): records each axis's travel and which stick and
// switch is which channel, which way round, for any radio. The result is saved in the prefs under
// the controller's id, so the server keeps it.
//
//   1. sticks centred, throttle down, switches low: the rest positions
//   2. every stick round its travel, every switch flicked: each axis's ends
//   3. roll right, pitch forward, throttle up, yaw right: the sticks' axes and directions
//   4. the arm switch up, the mode switch to its last position: the switches' axes
// Every other axis follows as CH7, CH8, ... in order.

import { radio } from "../input/radio.ts";
import type { Calibration } from "../state/prefs.ts";
import { prefs, updatePrefs } from "../state/prefs.ts";
import { h } from "./h.ts";

interface Step {
  ask: string;
  kind: "rest" | "travel" | "move";
  channel?: number;
}

const STEPS: Step[] = [
  {
    ask: "Centre the sticks, throttle all the way down, every switch low. Then Next.",
    kind: "rest",
  },
  {
    ask: "Move both sticks round their full travel and flick every switch both ways. Then Next.",
    kind: "travel",
  },
  { ask: "Hold the roll stick fully right.", kind: "move", channel: 0 },
  { ask: "Hold the pitch stick fully forward.", kind: "move", channel: 1 },
  { ask: "Hold the throttle fully up.", kind: "move", channel: 2 },
  { ask: "Hold the yaw stick fully right.", kind: "move", channel: 3 },
  { ask: "Flip the arm switch up (CH5).", kind: "move", channel: 4 },
  { ask: "Move the mode switch to its last position (CH6).", kind: "move", channel: 5 },
];

/** Moved this far from rest: the axis being asked for. */
const PICKED = 0.5;
/** Held there this long. */
const HOLD_MS = 400;

export function calibrationWizard(onDone: () => void): HTMLElement {
  const raw = radio.raw;
  const box = h("div", { class: "wizard" });
  if (!raw) {
    box.append(
      h(
        "div",
        { class: "ask" },
        "No controller found: plug the radio in, set it to USB joystick mode and move a stick.",
      ),
    );
    return box;
  }
  const id = raw.id;
  let step = 0;
  let rest: number[] = [];
  const min: number[] = [];
  const max: number[] = [];
  const picked = new Map<number, { axis: number; reverse: boolean }>();
  let since = 0;
  let candidate: { axis: number; reverse: boolean } | null = null;
  const ask = h("div", { class: "ask" });
  const note = h("div", { class: "muted" });
  const next = h("button", { class: "primary", onclick: () => advance() }, "Next");
  const cancel = h("button", { onclick: () => finish(false) }, "Cancel");
  box.append(ask, note, h("div", { class: "actions", style: "margin-top:10px" }, cancel, next));

  const axes = () => (radio.raw?.id === id ? [...radio.raw.axes] : null);

  const show = () => {
    const s = STEPS[step]!;
    ask.textContent = `${step + 1}/${STEPS.length}. ${s.ask}`;
    next.hidden = s.kind === "move";
    note.textContent = s.kind === "move" ? "Waiting for it to move…" : "";
  };

  const advance = () => {
    const a = axes();
    if (!a) return;
    const s = STEPS[step]!;
    if (s.kind === "rest") {
      rest = a;
      a.forEach((v, i) => {
        min[i] = v;
        max[i] = v;
      });
    }
    step++;
    if (step >= STEPS.length) finish(true);
    else show();
  };

  const timer = setInterval(() => {
    const a = axes();
    const s = STEPS[step];
    if (!a || !s) return;
    a.forEach((v, i) => {
      min[i] = Math.min(min[i] ?? v, v);
      max[i] = Math.max(max[i] ?? v, v);
    });
    if (s.kind !== "move") return;
    const taken = new Set([...picked.values()].map((p) => p.axis));
    let best = -1;
    let delta = 0;
    a.forEach((v, i) => {
      const d = v - (rest[i] ?? 0);
      if (!taken.has(i) && Math.abs(d) > Math.abs(delta)) {
        best = i;
        delta = d;
      }
    });
    if (best >= 0 && Math.abs(delta) > PICKED) {
      const c = { axis: best, reverse: delta < 0 };
      if (candidate?.axis !== c.axis) {
        candidate = c;
        since = performance.now();
      } else if (performance.now() - since > HOLD_MS) {
        picked.set(s.channel!, c);
        note.textContent = `Axis ${c.axis}${c.reverse ? ", reversed" : ""}`;
        candidate = null;
        setTimeout(advance, 250);
        step = Math.min(step, STEPS.length - 1);
      }
    } else {
      candidate = null;
    }
  }, 33);

  const finish = (save: boolean) => {
    clearInterval(timer);
    if (save) {
      const taken = new Set([...picked.values()].map((p) => p.axis));
      const rest8 = rest.map((_, i) => i).filter((i) => !taken.has(i));
      const order = [0, 1, 2, 3, 4, 5].map((ch) => picked.get(ch)!);
      for (const axis of rest8) order.push({ axis, reverse: false });
      const cal: Calibration = {
        channels: order.map((p) => ({
          axis: p.axis,
          reverse: p.reverse,
          cal: { min: min[p.axis] ?? -1, mid: rest[p.axis] ?? 0, max: max[p.axis] ?? 1 },
        })),
      };
      // The throttle rests at its low end, not its middle: centre it on its travel.
      const thr = cal.channels[2];
      if (thr) thr.cal.mid = (thr.cal.min + thr.cal.max) / 2;
      updatePrefs((p) => ({
        ...p,
        controls: { ...p.controls, calibrations: { ...p.controls.calibrations, [id]: cal } },
      }));
    }
    onDone();
  };

  show();
  return box;
}

/** Forget the calibration of controller `id` (back to its profile). */
export function forgetCalibration(id: string): void {
  const calibrations = { ...prefs().controls.calibrations };
  delete calibrations[id];
  updatePrefs((p) => ({ ...p, controls: { ...p.controls, calibrations } }));
}
