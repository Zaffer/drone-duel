// The player's preferences (DESIGN.md §11, §12): a signal, loaded from the server and saved back
// to it a moment after each change, so they live in ~/.config/drone-duels/prefs.json.

import { effect, signal } from "alien-signals";
import { api } from "../net/api.ts";
import type { Location } from "../net/protocol.ts";

export type ViewId = "fpv" | "chase" | "far" | "spotter" | "map" | "orbit";
export type VideoSystem = "analog" | "digital";
export type AnalogChain = "auto" | "full" | "light";

/** A channel's calibration: the raw axis value at its low end, centre and high end. */
export interface AxisCal {
  min: number;
  mid: number;
  max: number;
}

export interface Calibration {
  /** Per channel (from channel 1): which axis, which way round, and its travel. */
  channels: { axis: number; reverse: boolean; cal: AxisCal }[];
}

export interface Prefs {
  player: { name: string };
  video: {
    system: VideoSystem;
    /** The analog picture's maker: the whole signal chain, the light one for a phone's GPU, or
     * by the device (light where it is flown by touch). */
    analog: AnalogChain;
    vtxMw: number;
    osd: boolean;
    tiltDeg: number | null;
  };
  view: { initial: ViewId; chaseDistance: number; fovDeg: number };
  controls: {
    /** Gamepad id → its calibration from the wizard. */
    calibrations: Record<string, Calibration>;
    /** Read the radio in the server (Linux evdev) rather than through Chrome. */
    serverRadio: boolean;
  };
  graphics: {
    quality: "low" | "medium" | "high";
    maxFps: number;
    /** The sun's hour of local solar time at the place flown (0–24), or null for the real time
     * (World.setTimeOfDay). */
    timeOfDay: number | null;
  };
  audio: { on: boolean; master: number; motors: number; effects: number };
  hud: { minimap: boolean; dials: boolean; units: "metric" | "imperial" };
  /** Shoot Down's rifle and how the shooter aims it. */
  shoot: {
    /** Mouse: milliradians a count, at the hip (aiming scales it by the zoom). */
    mouseMrad: number;
    /** Gamepad: the right stick's fastest turn, degrees a second. */
    padDegS: number;
    /** On the sight, the view narrows to this share of the field of view (1: none). */
    adsZoom: number;
    /** The right button or trigger: hold to aim, or press to aim and press again. */
    adsHold: boolean;
    sight: "holo" | "irons";
    /** Tracers: as the magazine is loaded (the weapon's file), every round, or none. */
    tracers: "loaded" | "every" | "none";
    /** The holo's brightness, 0..1. */
    holoBright: number;
    /** The wind meter on the HUD. */
    windMeter: boolean;
  };
  /** The places the player typed in as GPS points (world/places.ts), newest first. */
  places: Location[];
}

export const DEFAULT_PREFS: Prefs = {
  player: { name: "Pilot" },
  video: { system: "analog", analog: "auto", vtxMw: 25, osd: true, tiltDeg: null },
  view: { initial: "fpv", chaseDistance: 4, fovDeg: 75 },
  controls: { calibrations: {}, serverRadio: false },
  graphics: { quality: "high", maxFps: 0, timeOfDay: 12 },
  audio: { on: true, master: 0.8, motors: 0.6, effects: 0.8 },
  hud: { minimap: true, dials: true, units: "metric" },
  shoot: {
    mouseMrad: 1.2,
    padDegS: 160,
    adsZoom: 0.8,
    adsHold: true,
    sight: "holo",
    tracers: "loaded",
    holoBright: 0.8,
    windMeter: true,
  },
  places: [],
};

function merge<T>(base: T, over: unknown): T {
  if (typeof base !== "object" || base === null || Array.isArray(base)) {
    // A saved null is a choice (the real time of day), not a missing value.
    return (over === undefined ? base : over) as T;
  }
  if (typeof over !== "object" || over === null) return base;
  const out: Record<string, unknown> = { ...(base as Record<string, unknown>) };
  for (const [k, v] of Object.entries(over)) {
    const b = (base as Record<string, unknown>)[k];
    // Free-form maps (calibrations) take whatever was saved.
    out[k] =
      b === undefined || (typeof b === "object" && b !== null && Object.keys(b).length === 0)
        ? v
        : merge(b, v);
  }
  return out as T;
}

export const prefs = signal<Prefs>(DEFAULT_PREFS);

/** Change part of the prefs: `update(p => ({...p, hud: {...p.hud, minimap: false}}))`. */
export function updatePrefs(change: (p: Prefs) => Prefs): void {
  prefs(change(prefs()));
}

export async function loadPrefs(): Promise<void> {
  try {
    prefs(merge(DEFAULT_PREFS, await api.prefs()));
  } catch (err) {
    console.warn("prefs: using the defaults", err);
  }
  let first = true;
  let timer: ReturnType<typeof setTimeout> | undefined;
  effect(() => {
    const p = prefs();
    if (first) {
      first = false;
      return;
    }
    clearTimeout(timer);
    timer = setTimeout(() => void api.savePrefs(p as unknown as Record<string, unknown>), 400);
  });
}

export { merge as mergePrefs };
