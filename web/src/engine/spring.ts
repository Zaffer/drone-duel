// A damped spring, stepped by its exact solution (DESIGN.md §3.2): any step, a millisecond or a
// tenth of a second, lands where the spring would really be, so it can never ring up and run
// away (stepped by Euler a frame at a time, a 4 Hz spring grew without end under ~20 fps and
// turned the shooter's view over: 2026-11-06). Every spring in the page is one of these.
//
// x'' = -ω²(x - target) - 2ζω x', ω = 2π hz. A kick is a change of velocity, between steps.

export class Spring {
  x = 0;
  v = 0;
  target = 0;
  constructor(
    public hz: number,
    public zeta: number,
  ) {}

  /** On by `dt` s, the target held. */
  step(dt: number): void {
    if (dt <= 0) return;
    const w = 2 * Math.PI * this.hz;
    const z = this.zeta;
    const x0 = this.x - this.target;
    const v0 = this.v;
    let x: number;
    let v: number;
    if (Math.abs(z - 1) < 1e-6) {
      const e = Math.exp(-w * dt);
      const b = v0 + w * x0;
      x = e * (x0 + b * dt);
      v = e * (v0 - w * b * dt);
    } else if (z < 1) {
      const wd = w * Math.sqrt(1 - z * z);
      const e = Math.exp(-z * w * dt);
      const c = Math.cos(wd * dt);
      const s = Math.sin(wd * dt);
      x = e * (x0 * c + ((v0 + z * w * x0) / wd) * s);
      v = e * (v0 * c - ((w * w * x0 + z * w * v0) / wd) * s);
    } else {
      const q = w * Math.sqrt(z * z - 1);
      const r1 = -z * w + q;
      const r2 = -z * w - q;
      const a = (v0 - r2 * x0) / (r1 - r2);
      const b = x0 - a;
      const e1 = Math.exp(r1 * dt);
      const e2 = Math.exp(r2 * dt);
      x = a * e1 + b * e2;
      v = a * r1 * e1 + b * r2 * e2;
    }
    this.x = x + this.target;
    this.v = v;
  }
}

/** The fraction of the way an exponential approach with time constant `tau` s goes in `dt` s:
 * `x += (want - x) * approach(dt, tau)`, exact at any step. */
export const approach = (dt: number, tau: number): number => 1 - Math.exp(-dt / tau);
