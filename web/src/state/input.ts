// The pilot's controls as the UI reads them (DESIGN.md §12): the radio's channels, each -1..1
// from channel 1, as last sampled (written at the radio's rate by input/radio.ts), and which
// controller they come from. The 250 Hz stream itself is input/radio.ts's `radio.samples()`.

import { signal } from "alien-signals";

/** -1..1 per channel, from channel 1 (AETR, then AUX). Empty with no controller. */
export const channels = signal<number[]>([]);
/** The controller in use, by name; null with none. */
export const controllerName = signal<string | null>(null);
/** Where the sticks come from. */
export const inputSource = signal<"radio" | "gamepad" | "keyboard" | "server" | "none">("none");

/** A channel's value (-1..1) as a pulse width, µs. */
export const toUs = (x: number): number => Math.round(1500 + 500 * Math.max(-1, Math.min(1, x)));
