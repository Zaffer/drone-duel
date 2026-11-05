// The performance panel (P; DESIGN.md §3.1): the frame rate, the frame time, how far past the
// newest pose the vehicle is drawn (carried on along its velocity), and where the sticks come
// from. `perf` records every frame whether the panel shows or not, so a test can read it
// (window.duels.perf).

import { inputSource } from "../state/input.ts";
import "./hud.css";

/** Frames kept, s. */
const KEEP_MS = 2000;
const SOURCES = {
  server: "the radio, from the server",
  radio: "the radio, from the page",
  gamepad: "a gamepad, from the page",
  keyboard: "the keyboard, from the page",
  none: "none",
} as const;

export interface PerfStats {
  fps: number;
  /** Time between frames, ms. */
  frameP50: number;
  frameP95: number;
  /** How far past the newest pose it was drawn, ms (0 between poses). */
  aheadP50: number;
  aheadP95: number;
  sticks: string;
}

const quantile = (xs: number[], q: number): number =>
  xs.length ? [...xs].sort((a, b) => a - b)[Math.floor(q * (xs.length - 1))]! : 0;

class Perf {
  private frames: { at: number; ahead: number }[] = [];

  /** A frame at `now` (performance.now()), drawn `ahead` s past the newest pose. */
  frame(now: number, ahead: number): void {
    this.frames.push({ at: now, ahead: Math.max(0, ahead) * 1000 });
    while (this.frames.length && now - this.frames[0]!.at > KEEP_MS) this.frames.shift();
  }

  stats(): PerfStats {
    const f = this.frames;
    const gaps = f.slice(1).map((x, i) => x.at - f[i]!.at);
    const span = f.length > 1 ? f[f.length - 1]!.at - f[0]!.at : 0;
    const ahead = f.map((x) => x.ahead);
    return {
      fps: span ? ((f.length - 1) * 1000) / span : 0,
      frameP50: quantile(gaps, 0.5),
      frameP95: quantile(gaps, 0.95),
      aheadP50: quantile(ahead, 0.5),
      aheadP95: quantile(ahead, 0.95),
      sticks: SOURCES[inputSource()],
    };
  }
}

export const perf = new Perf();

/** The panel in `el`, shown or hidden with `toggle()`; updated four times a second. */
export function mountPerf(el: HTMLElement): { toggle(): void; dispose(): void } {
  el.classList.add("perf");
  el.hidden = true;
  const timer = setInterval(() => {
    if (el.hidden) return;
    const s = perf.stats();
    el.textContent = [
      `${s.fps.toFixed(0)} fps`,
      `frame ${s.frameP50.toFixed(0)} ms, 95% ${s.frameP95.toFixed(0)} ms`,
      `pose carried on ${s.aheadP50.toFixed(0)} ms, 95% ${s.aheadP95.toFixed(0)} ms`,
      `sticks: ${s.sticks}`,
    ].join("\n");
  }, 250);
  return {
    toggle: () => {
      el.hidden = !el.hidden;
    },
    dispose: () => clearInterval(timer),
  };
}
