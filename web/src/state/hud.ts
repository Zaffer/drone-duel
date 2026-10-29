// What a mode shows on the flight HUD: a timer in the top bar, a big banner (countdown, GO,
// results), and a panel (the scoreboard, lap times).

import { signal } from "alien-signals";

export const hudTimer = signal<string | null>(null);
export const hudBanner = signal<{
  big: string;
  small?: string;
  tone?: "sun" | "leaf" | "rose";
} | null>(null);
export const hudPanel = signal<HTMLElement | null>(null);

export function clearHud(): void {
  hudTimer(null);
  hudBanner(null);
  hudPanel(null);
}
