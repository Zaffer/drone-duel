// Time as Observables (DESIGN.md §9): animation frames and intervals, each running while
// subscribed and stopping with the subscription's signal.

/** Every animation frame's timestamp (performance.now()). */
export function frames(): Observable<number> {
  return new Observable<number>((sub) => {
    let id = requestAnimationFrame(function tick(t) {
      sub.next(t);
      id = requestAnimationFrame(tick);
    });
    sub.addTeardown(() => cancelAnimationFrame(id));
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
