// The app's own state (DESIGN.md §9, §11): which screen, mode, location and vehicle, the view,
// and the in-flight toggles. Plain signals; the UI reads them in effects.

import { computed, signal } from "alien-signals";
import type { ConfigMsg, ContentMsg, Course, Location, Vehicle } from "../net/protocol.ts";
import type { ViewId } from "./prefs.ts";

export type Screen = "loading" | "menu" | "lobby" | "flying";
export type ModeId = "freefly" | "duel" | "race" | "shoot";

export const content = signal<ContentMsg | null>(null);
export const config = signal<ConfigMsg | null>(null);

export const screen = signal<Screen>("loading");
export const mode = signal<ModeId | null>(null);
export const locationId = signal<string | null>(null);
export const vehicleId = signal<string | null>(null);
export const view = signal<ViewId>("fpv");
export const paused = signal(false);
export const dynamicsOn = signal(false);
/** The current wind, as set in the Wind tab (or the location's). */
export const wind = signal<Location["wind"] | null>(null);

export const location = computed<Location | null>(() => {
  const id = locationId();
  return id ? (content()?.locations[id] ?? null) : null;
});
export const vehicle = computed<Vehicle | null>(() => {
  const id = vehicleId();
  return id ? (content()?.vehicles[id] ?? null) : null;
});
/** The race course at the location (the first, if it has several). */
export const course = computed<Course | null>(() => {
  const id = locationId();
  return Object.values(content()?.courses ?? {}).find((c) => c.location === id) ?? null;
});

/** A toast for the player: what happened, briefly. */
export interface Toast {
  id: number;
  text: string;
  tone: "info" | "good" | "warn" | "bad";
}
export const toasts = signal<Toast[]>([]);
let toastId = 0;
export function toast(text: string, tone: Toast["tone"] = "info", ms = 3500): void {
  const t = { id: ++toastId, text, tone };
  toasts([...toasts(), t]);
  setTimeout(() => toasts(toasts().filter((x) => x.id !== t.id)), ms);
}
