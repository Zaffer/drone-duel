// The game's hotkeys, as tables of actions (input/actions.ts): the main menu's, flight's, on foot
// (Shoot Down's, beside the shooter's own: shoot/controls.ts), and the dynamics overlay's. The
// code that acts on them mounts these; the Inputs tab and the HUD's key strip are drawn from
// them. Tables that name a count (the modes, the views, the layers) are made when asked, so
// their owners load first.

import type { Bindings } from "../input/actions.ts";

/** Digit keys 1..n. */
const digits = (n: number): string[] => Array.from({ length: n }, (_, i) => `Digit${i + 1}`);

/** The main menu's; `modes` names the scenarios in their order. */
export const menuBindings = (modes: string[]) =>
  ({
    mode: {
      does: `Pick a mode: ${modes.map((m, i) => `${i + 1} ${m}`).join(", ")}`,
      codes: digits(modes.length),
      label: modes.map((_, i) => String(i + 1)),
    },
    options: { does: "Options, on the Controls tab", codes: ["KeyO"] },
    back: { does: "Back out of the chosen mode", codes: ["Escape"] },
  }) satisfies Bindings<string>;

/** Flying or driving, in the HUD's order; `views` names the views in their order. */
export const flightBindings = (views: string[]) =>
  ({
    menu: { does: "Pause menu (Esc again resumes)", codes: ["Escape"], strip: "menu" },
    nextView: { does: "Next view", codes: ["KeyV"], shift: false, strip: "view" },
    lastView: { does: "Previous view", codes: ["KeyV"], shift: true },
    view: {
      does: `Jump to a view: ${views.map((v, i) => `${i + 1} ${v}`).join(", ")}`,
      codes: digits(views.length),
      shift: false,
      label: [`1–${views.length}`],
    },
    reset: {
      does: "Reset to the start (the truck: back on its wheels)",
      codes: ["KeyR"],
      strip: "reset",
    },
    vehicle: {
      does: "Switch vehicle (the pause menu's Vehicle tab)",
      codes: ["KeyB"],
      strip: "vehicle",
    },
    throw: { does: "Throw a hand-launched wing", codes: ["KeyT"], strip: "throw" },
    dynamics: {
      does: "The dynamics overlay, on or off (aircraft that have it)",
      codes: ["KeyD"],
      strip: "dynamics",
    },
    minimap: { does: "The minimap, on or off", codes: ["KeyN"], strip: "minimap" },
    help: { does: "The key strip, shown or hidden", codes: ["KeyH"], strip: "help" },
    gust: { does: "A gust now, as the Wind tab sets it", codes: ["KeyG"] },
    perf: {
      does: "The performance panel: frame rate, frame time, the dearest systems, how far the pose is carried on, where the sticks come from",
      codes: ["KeyP"],
    },
  }) satisfies Bindings<string>;

/** On foot (Shoot Down): the flight's keys that still mean something. */
export const footBindings = () =>
  ({
    gust: { does: "A gust now, as the Wind tab sets it", codes: ["KeyG"] },
    help: { does: "The key strip, shown or hidden", codes: ["KeyH"] },
    perf: { does: "The performance panel", codes: ["KeyP"] },
    menu: { does: "Pause menu", codes: ["Escape"], strip: "menu" },
  }) satisfies Bindings<string>;

/** The dynamics overlay's, while it is on; `layers` names its layers in their order. */
export const overlayBindings = (layers: string[]) =>
  ({
    layer: {
      does: `A layer on or off: ${layers.map((l, i) => `${i + 1} ${l}`).join(", ")}`,
      codes: digits(layers.length),
      shift: true,
      label: [`Shift+1–${layers.length}`],
    },
    bigger: { does: "Arrows 1.5× bigger", codes: ["BracketRight"] },
    smaller: { does: "Arrows 1.5× smaller", codes: ["BracketLeft"] },
  }) satisfies Bindings<string>;
