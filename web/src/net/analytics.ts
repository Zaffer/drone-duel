// Counting who plays the web version (DESIGN.md §18): GoatCounter, cookieless, whose script the
// web build puts in the page (scripts/build.ts) and only there, so the game served by `duels`
// counts nothing. Its page view is the script's own; what the page adds are events.

interface GoatCounter {
  count(vars: { path: string; title?: string; event?: boolean }): void;
}

/** Counts an event (in GoatCounter's list by its path): nothing when the script is not in the
 * page, has not loaded yet, or is blocked. */
export function countEvent(path: string, title?: string): void {
  const gc = (window as unknown as { goatcounter?: GoatCounter }).goatcounter;
  try {
    gc?.count?.({ path, title: title ?? path, event: true });
  } catch {
    // Analytics never stops a flight.
  }
}
