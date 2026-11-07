// What a soldier on foot gets over, and how (DESIGN.md §7.1, Shoot Down). By the rise of what
// is in front over the feet (EST, a fit soldier in some 15 kg of kit):
// - a step, up to STEP_M (knee high): walked onto, no hands;
// - a vault, up to VAULT_M (waist high): hands on it and over, by walking on into it or a jump;
// - a climb, up to CLIMB_M: a jump at it, a grab of the top, a pull up and over;
// - higher, a wall. Unless it is thin with ground they could stand on within PUSH_M beyond it:
//   leaning into it they push through. The tiles' ground is heights from above, so a hedge, a
//   bush, a tree's crown and a glitch in the mesh are all solid blobs to it; this is the way out
//   of them. (A thin wall too high to climb gives way the same: the tiles can't tell it from a
//   hedge.)
// Down, anything: a drop over a step is a fall, and a long one a heavy landing.
//
// The ground ahead comes as heights at distances along the way they go; `plan` decides from them.

export const STEP_M = 0.45;
export const VAULT_M = 1.1;
export const CLIMB_M = 2.3;
export const PUSH_M = 4;
/** How far ahead an edge can be to be reached for, m (from the body's middle). */
export const REACH_M = 1.0;
/** The looks ahead: from here, this far apart, m. */
export const LOOK_FROM_M = 0.35;
export const LOOK_EVERY_M = 0.3;

/** How long it takes, s (EST): a vault, from a walk or a run; a climb; pushing through, m/s. */
export function vaultS(rise: number, running: boolean): number {
  return (0.45 + 0.35 * rise) * (running ? 0.7 : 1);
}
export function climbS(rise: number): number {
  return 0.9 + 0.5 * rise;
}
export const PUSH_MS = 1.0;

export type OverKind = "vault" | "climb" | "push";

export interface Over {
  kind: OverKind;
  /** How far along the way they end up, m. */
  dist: number;
  /** The feet's height at the end (on the top, or the far side's ground for a push), m. */
  feet: number;
  /** The ground where they end up (lower than `feet` over a fence: they drop off it), m. */
  ground: number;
  /** What they got over, above the feet they started from, m. */
  rise: number;
}

/** The distances the looks ahead are at, out to `far` m. */
export function lookDistances(far: number): number[] {
  const out: number[] = [];
  for (let d = LOOK_FROM_M; d <= far + 1e-9; d += LOOK_EVERY_M) out.push(d);
  return out;
}

/**
 * What to do about the ground ahead: the heights `hs` (undefined where not known) at `ds` m
 * along the way, the feet at `feet`. `allow` is what this attempt may do: walking on into it
 * vaults and pushes, a jump vaults and climbs. Null: nothing in the way that it can get over.
 */
export function plan(
  feet: number,
  ds: number[],
  hs: (number | undefined)[],
  allow: ReadonlySet<OverKind>,
): Over | null {
  const i0 = hs.findIndex((h) => h !== undefined && h > feet + STEP_M);
  if (i0 < 0 || ds[i0]! > REACH_M) return null;
  // The top: the highest of the edge and just beyond it (the tiles' walls lean, so the first
  // look over a step can be partway up one).
  let iTop = i0;
  for (let i = i0 + 1; i <= i0 + 2 && i < hs.length; i++) {
    const h = hs[i];
    if (h !== undefined && h > hs[iTop]! && h <= feet + CLIMB_M) iTop = i;
  }
  const top = hs[iTop]!;
  const rise = top - feet;
  const kind: OverKind | null = rise <= VAULT_M ? "vault" : rise <= CLIMB_M ? "climb" : null;
  if (kind && allow.has(kind)) {
    // Onto the top, a body's width past it; the ground there (lower: over a fence, down off it).
    const beyond = hs[iTop + 1];
    return {
      kind,
      dist: ds[iTop]! + LOOK_EVERY_M,
      feet: top,
      ground: beyond !== undefined && beyond < top - STEP_M ? beyond : top,
      rise,
    };
  }
  if (kind === "vault" || !allow.has("push")) return null;
  // Too high to climb (or a climb not asked for): through, if there is ground beyond it, near.
  for (let i = i0 + 1; i < hs.length; i++) {
    const h = hs[i];
    if (ds[i]! > PUSH_M) break;
    if (h !== undefined && h <= feet + STEP_M) {
      // Not where it first dips (the lean of a blob's edge): a look on, if that is ground too.
      const next = hs[i + 1];
      const settle = next !== undefined && next <= feet + STEP_M ? i + 1 : i;
      return { kind: "push", dist: ds[settle]!, feet: hs[settle]!, ground: hs[settle]!, rise };
    }
  }
  return null;
}

/** What the ground right ahead would mean, for the HUD: a vault, a climb, too high, or nothing. */
export function ahead(rise: number): "vault" | "climb" | "wall" | null {
  return rise <= STEP_M ? null : rise <= VAULT_M ? "vault" : rise <= CLIMB_M ? "climb" : "wall";
}
