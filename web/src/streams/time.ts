// Time as Observables (DESIGN.md §9): animation frames and intervals, each running while
// subscribed and stopping with the subscription's signal. Frames come from the page's one loop
// (engine/loop.ts), named and timed there; systems that care about their phase join the loop
// directly.

import { loop, type Phase } from "../engine/loop.ts";

/** Every animation frame's timestamp (performance.now()), as the system `name` in `phase`. */
export function frames(name: string, phase: Phase = "place"): Observable<number> {
  return new Observable<number>((sub) => {
    const stop = new AbortController();
    loop.each(name, phase, (now) => sub.next(now), stop.signal);
    sub.addTeardown(() => stop.abort());
  });
}

/** performance.now() every `ms`. */
export function every(ms: number): Observable<number> {
  return new Observable<number>((sub) => {
    const id = setInterval(() => sub.next(performance.now()), ms);
    sub.addTeardown(() => clearInterval(id));
  });
}

/** One value after `ms`. */
export function after(ms: number): Observable<number> {
  return new Observable<number>((sub) => {
    const id = setTimeout(() => {
      sub.next(performance.now());
      sub.complete();
    }, ms);
    sub.addTeardown(() => clearTimeout(id));
  });
}
