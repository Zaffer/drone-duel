// Time as Observables (DESIGN.md §9): animation frames and intervals, each running while
// subscribed and stopping with the subscription's signal.
//
// Every frame is one requestAnimationFrame (DESIGN.md §3.1): each subscriber of `frames()` in
// the order they subscribed (the views place the camera and the vehicle), then the draw that
// `drawEachFrame` set (the world's), so a frame shows what it placed, not the frame before's.

const subscribers = new Set<(t: number) => void>();
let draw: ((t: number) => void) | null = null;
let pending = 0;

function tick(t: number): void {
  pending = requestAnimationFrame(tick);
  for (const f of [...subscribers]) {
    try {
      f(t);
    } catch (err) {
      reportError(err);
    }
  }
  draw?.(t);
}

const run = (): void => {
  if (!pending) pending = requestAnimationFrame(tick);
};

/** Every animation frame's timestamp (performance.now()). */
export function frames(): Observable<number> {
  return new Observable<number>((sub) => {
    const f = (t: number) => sub.next(t);
    subscribers.add(f);
    run();
    sub.addTeardown(() => subscribers.delete(f));
  });
}

/** What draws each frame, after every `frames()` subscriber (null: nothing). */
export function drawEachFrame(fn: ((t: number) => void) | null): void {
  draw = fn;
  run();
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
