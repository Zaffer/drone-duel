// The page's one frame loop (DESIGN.md §3.2). Each animation frame runs every system in its
// phase, in the order they joined it, then draws:
//
//   input   read the controls (input/actions.ts)
//   sim     what moves by its own rules: fixed-rate systems step here, `hz` times a second of
//           real time whatever the frame rate, and a system that steps to the clock itself
//           (the rounds) runs here too
//   place   put the camera, the models and the HUD where the sim has them
//   draw    Cesium renders
//
// Every system is timed, by name, and the performance panel (P) shows the dearest. A fixed-rate
// system catches up at most MAX_CATCHUP_S a frame: past that the time is dropped, and counted,
// rather than spent, so one slow frame can never make the next one slower (2026-11-06: rounds
// that looked along all of a slow frame's flight took seconds a frame).
//
// Pausing stops input (input/actions.ts), not time: the firmware on the server cannot stop, so
// nothing in the world does.

export type Phase = "input" | "sim" | "place" | "draw";
const PHASES: Phase[] = ["input", "sim", "place", "draw"];

/** A frame's real time is taken as at most this, s (a tab in the background, a debugger). */
const MAX_DT_S = 0.1;
/** A fixed-rate system's catching up, at most, a frame, s. */
const MAX_CATCHUP_S = 0.1;
/** How long the timings are kept, ms. */
const KEEP_MS = 2000;

export interface SystemStats {
  name: string;
  phase: Phase | "cost";
  /** Its time a frame, ms: median, 95th percentile, most. */
  p50: number;
  p95: number;
  max: number;
  /** Fixed-rate systems: time dropped catching up in the last KEEP_MS, s. */
  dropped: number;
}

interface System {
  name: string;
  run(now: number, dt: number): void;
}

/** A name's times, one a frame it ran. */
class Times {
  at: number[] = [];
  ms: number[] = [];
  dropped: { at: number; s: number }[] = [];
  /** Costs metered within a frame add up before they are recorded. */
  pending = 0;
  constructor(readonly phase: Phase | "cost") {}
  add(now: number, ms: number): void {
    this.at.push(now);
    this.ms.push(ms);
    while (this.at.length && now - this.at[0]! > KEEP_MS) {
      this.at.shift();
      this.ms.shift();
    }
  }
}

const quantile = (xs: number[], q: number): number =>
  xs.length ? [...xs].sort((a, b) => a - b)[Math.floor(q * (xs.length - 1))]! : 0;

export class Loop {
  private readonly systems = new Map<Phase, Set<System>>(PHASES.map((p) => [p, new Set()]));
  private readonly times = new Map<string, Times>();
  private drawFn: ((now: number) => void) | null = null;
  private pending = 0;
  private last: number | null = null;
  private now = 0;
  /** Each frame's start (performance.now()), for frame rate and frame time. */
  readonly frames: number[] = [];

  /** Run `fn(now, dt)` every frame in `phase` until `signal` aborts. `now` is the frame's
   * performance.now(), `dt` its real time since the last frame, s (at most MAX_DT_S). */
  each(
    name: string,
    phase: Phase,
    fn: (now: number, dt: number) => void,
    signal?: AbortSignal,
  ): void {
    this.join(phase, { name, run: fn }, signal);
  }

  /** Run `fn(h)` `hz` times a second of real time, h = 1/hz s, in the sim phase. */
  fixed(name: string, hz: number, fn: (h: number) => void, signal?: AbortSignal): void {
    const h = 1 / hz;
    const most = Math.ceil(MAX_CATCHUP_S / h);
    let acc = 0;
    this.join(
      "sim",
      {
        name,
        run: (now, dt) => {
          acc += dt;
          let n = 0;
          while (acc >= h && n < most) {
            fn(h);
            acc -= h;
            n++;
          }
          if (acc >= h) {
            this.timesOf(name, "sim").dropped.push({ at: now, s: acc });
            acc = 0;
          }
        },
      },
      signal,
    );
  }

  /** What draws each frame, after every system (the world's render; null: nothing). */
  draw(fn: ((now: number) => void) | null): void {
    this.drawFn = fn;
    this.run();
  }

  /** Add `ms` to `name`'s time this frame: a cost paid inside some system, named on its own
   * (a pick render, a tile built). */
  meter(name: string, ms: number): void {
    this.timesOf(name, "cost").pending += ms;
  }

  /** `fn()`, metered as `name`. */
  timed<T>(name: string, fn: () => T): T {
    const t0 = performance.now();
    try {
      return fn();
    } finally {
      this.meter(name, performance.now() - t0);
    }
  }

  /** Every name's times over the last KEEP_MS, dearest (95th percentile) first. */
  stats(): SystemStats[] {
    const out: SystemStats[] = [];
    for (const [name, t] of this.times) {
      while (t.dropped.length && this.now - t.dropped[0]!.at > KEEP_MS) t.dropped.shift();
      out.push({
        name,
        phase: t.phase,
        p50: quantile(t.ms, 0.5),
        p95: quantile(t.ms, 0.95),
        max: t.ms.length ? Math.max(...t.ms) : 0,
        dropped: t.dropped.reduce((a, d) => a + d.s, 0),
      });
    }
    return out.sort((a, b) => b.p95 - a.p95);
  }

  private join(phase: Phase, sys: System, signal?: AbortSignal): void {
    if (signal?.aborted) return;
    const set = this.systems.get(phase)!;
    set.add(sys);
    this.timesOf(sys.name, phase);
    signal?.addEventListener("abort", () => set.delete(sys), { once: true });
    this.run();
  }

  private timesOf(name: string, phase: Phase | "cost"): Times {
    let t = this.times.get(name);
    if (!t) {
      t = new Times(phase);
      this.times.set(name, t);
    }
    return t;
  }

  private run(): void {
    if (this.pending || typeof requestAnimationFrame === "undefined") return;
    this.pending = requestAnimationFrame((t) => this.tick(t));
  }

  private tick(now: number): void {
    this.pending = requestAnimationFrame((t) => this.tick(t));
    this.frame(now);
  }

  /** One frame at `now` (the animation frame's; a test calls it directly). */
  frame(now: number): void {
    const dt = this.last === null ? 0 : Math.min(MAX_DT_S, (now - this.last) / 1000);
    this.last = now;
    this.now = now;
    this.frames.push(now);
    while (this.frames.length && now - this.frames[0]! > KEEP_MS) this.frames.shift();
    for (const phase of PHASES) {
      for (const sys of [...this.systems.get(phase)!]) {
        const t0 = performance.now();
        try {
          sys.run(now, dt);
        } catch (err) {
          reportError(err);
        }
        this.timesOf(sys.name, phase).add(now, performance.now() - t0);
      }
      if (phase === "draw" && this.drawFn) {
        const t0 = performance.now();
        this.drawFn(now);
        this.timesOf("cesium render", "draw").add(now, performance.now() - t0);
      }
    }
    for (const t of this.times.values()) {
      if (t.phase === "cost") {
        t.add(now, t.pending);
        t.pending = 0;
      }
    }
  }
}

/** The page's loop. */
export const loop = new Loop();
