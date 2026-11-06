// The Inputs tab (Options): every key, and how to fly or drive each vehicle on a radio, a
// gamepad or the keyboard. It says what the code that reads them does: ui/app.ts (the menu's
// keys), ui/hud.ts (flight's), dynamics/overlay.ts, input/mapping.ts and input/radio.ts (the
// aircraft's sticks) and vehicles/truck/controls.ts (the truck's). Change one, change this.

import { LAYERS } from "../dynamics/overlay.ts";
import { MODES } from "../modes/registry.ts";
import type { Vehicle } from "../net/protocol.ts";
import { content } from "../state/app.ts";
import { VIEW_NAMES, VIEW_ORDER } from "../views/rigs.ts";
import { h } from "./h.ts";

/** A hotkey: the keys that do it (any one), what it does, and its word on the HUD's key strip. */
export interface Hotkey {
  keys: string[];
  does: string;
  strip?: string;
}

export const menuKeys = (): Hotkey[] => [
  {
    keys: MODES.map((_, i) => String(i + 1)),
    does: `Pick a mode: ${MODES.map((m, i) => `${i + 1} ${m.name}`).join(", ")}`,
  },
  { keys: ["O"], does: "Options, on the Controls tab" },
  { keys: ["Esc"], does: "Back out of the chosen mode" },
];

/** In flight, in the HUD's order; those with `strip` are on its key strip too. */
export const flightKeys = (): Hotkey[] => [
  { keys: ["Esc"], does: "Pause menu (Esc again resumes)", strip: "menu" },
  { keys: ["V", "Shift+V"], does: "Next / previous view", strip: "view" },
  {
    keys: [`1–${VIEW_ORDER.length}`],
    does: `Jump to a view: ${VIEW_ORDER.map((v, i) => `${i + 1} ${VIEW_NAMES[v]}`).join(", ")}`,
  },
  { keys: ["R"], does: "Reset to the start (the truck: back on its wheels)", strip: "reset" },
  { keys: ["B"], does: "Switch vehicle (the pause menu's Vehicle tab)", strip: "vehicle" },
  { keys: ["T"], does: "Throw a hand-launched wing", strip: "throw" },
  {
    keys: ["D"],
    does: "The dynamics overlay, on or off (aircraft that have it)",
    strip: "dynamics",
  },
  { keys: ["N"], does: "The minimap, on or off", strip: "minimap" },
  { keys: ["H"], does: "The key strip, shown or hidden", strip: "help" },
  { keys: ["G"], does: "A gust now, as the Wind tab sets it" },
  {
    keys: ["P"],
    does: "The performance panel: frame rate, frame time, how far the pose is carried on, where the sticks come from",
  },
];

/** Shoot Down's keys (shoot/shooter.ts). */
export const shooterKeys = (): Hotkey[] => [
  { keys: ["Click"], does: "Take the mouse (Esc gives it back)" },
  { keys: ["Mouse", "Right stick"], does: "Look and aim" },
  {
    keys: ["Right button", "LT"],
    does: "On the sight (held, or a toggle: the Rifle tab)",
    strip: "aim",
  },
  { keys: ["Left button", "RT"], does: "Fire: one press a shot", strip: "fire" },
  { keys: ["R", "X"], does: "Reload", strip: "reload" },
  { keys: ["W A S D", "Left stick"], does: "Walk" },
  { keys: ["Shift", "L3"], does: "With forward: sprint (the rifle comes down)", strip: "sprint" },
  { keys: ["Space", "A"], does: "Jump (standing)", strip: "jump" },
  { keys: ["Shift", "RB"], does: "On the sight: hold your breath", strip: "breath" },
  { keys: ["C"], does: "Kneel or stand", strip: "kneel" },
  { keys: ["Z"], does: "Prone or stand", strip: "prone" },
  { keys: ["B (pad)"], does: "Standing, kneeling, prone in turn" },
  { keys: ["F", "LB"], does: "The rangefinder: the range to what the sight is on", strip: "range" },
  { keys: ["I", "Y"], does: "Holo or iron sights", strip: "sight" },
  { keys: ["G"], does: "A gust now, as the Wind tab sets it" },
  { keys: ["H"], does: "The key strip, shown or hidden" },
  { keys: ["Esc"], does: "Pause menu", strip: "menu" },
];

const overlayKeys = (): Hotkey[] => [
  {
    keys: [`Shift+1–${LAYERS.length}`],
    does: `A layer on or off: ${LAYERS.map((l, i) => `${i + 1} ${l.name}`).join(", ")}`,
  },
  { keys: ["]", "["], does: "Arrows 1.5× bigger / smaller" },
];

/** What each flight mode name does, for the mode switch's row. */
const MODE_NOTES: Record<string, string> = {
  LOITER: "holds its place and height; the throttle's centre holds height",
  STABILIZE: "self-levelling; the throttle is all yours",
  ACRO: "rates: the sticks set how fast it turns, nothing levels it",
  ANGLE: "self-levelling, bank and pitch limited",
  MANUAL: "the sticks move the control surfaces, no help at all",
};

const kbd = (k: string) => h("span", { class: "kbd" }, k);

/** Keys in a cell: each a key cap, alternatives apart. */
const keyCell = (keys: string[]) =>
  h("td", { class: "keys" }, ...keys.flatMap((k, i) => (i ? [" ", kbd(k)] : [kbd(k)])));

export function hotkeyTable(keys: Hotkey[]): HTMLElement {
  return h(
    "table",
    { class: "keys-table" },
    h(
      "tbody",
      null,
      keys.map((k) => h("tr", null, keyCell(k.keys), h("td", null, k.does))),
    ),
  );
}

type Row = [job: string, radio: string, gamepad: string, keyboard: string[]];

function controlTable(rows: Row[]): HTMLElement {
  return h(
    "table",
    { class: "keys-table controls-table" },
    h(
      "thead",
      null,
      h(
        "tr",
        null,
        ["", "Radio", "Gamepad", "Keyboard"].map((t) => h("th", null, t)),
      ),
    ),
    h(
      "tbody",
      null,
      rows.map(([job, radio, pad, keys]) =>
        h(
          "tr",
          null,
          h("td", null, job),
          h("td", { class: "num" }, radio),
          h("td", null, pad),
          keys.length ? keyCell(keys) : h("td", { class: "muted" }, "—"),
        ),
      ),
    ),
  );
}

const POSITIONS = ["low", "middle", "high"];

/** An aircraft: how to get it up, then its sticks and switches on each kind of controller. */
function aircraft(v: Vehicle): HTMLElement[] {
  const aux = v.aux;
  const hand = v.launch === "hand";
  const steps = hand
    ? "Arm with the throttle down, raise the throttle, then throw it: T, or flick the launch switch up. Bank to turn, pull to climb."
    : "On the ground, arm with the throttle down, then raise the throttle to lift off.";
  const rows: Row[] = [
    ["Roll", "CH1", "Right stick ←→", ["←", "→"]],
    ["Pitch", "CH2", "Right stick ↑↓", ["↑", "↓"]],
    ["Throttle", "CH3", "Left stick ↑↓ (centre is half)", ["W", "S"]],
    ["Yaw", "CH4", "Left stick ←→", ["Q", "E"]],
  ];
  if (aux) rows.push(["Arm", `CH${aux.arm} high`, "A (toggles)", ["Enter"]]);
  if (aux?.mode && aux.modes.length) {
    rows.push(["Flight mode", `CH${aux.mode}`, "X (steps through)", ["M"]]);
  }
  if (hand) rows.push(["Throw", aux?.launch ? `CH${aux.launch} flick up` : "—", "—", ["T"]]);
  const modes =
    aux?.mode && aux.modes.length
      ? h(
          "ul",
          { class: "how" },
          aux.modes.map((m, i) =>
            h(
              "li",
              null,
              h("span", { class: "num" }, m),
              ` (${POSITIONS[i] ?? ""}): ${MODE_NOTES[m] ?? ""}`,
            ),
          ),
        )
      : null;
  return [
    h("h5", null, v.name),
    h("p", { class: "muted" }, steps),
    controlTable(rows),
    ...(modes
      ? [h("p", { class: "muted" }, "The mode switch, low to high (M and X step through):"), modes]
      : []),
  ];
}

/** A ground vehicle: the truck's controls (vehicles/truck/controls.ts). */
function ground(v: Vehicle): HTMLElement[] {
  return [
    h("h5", null, v.name),
    h(
      "p",
      { class: "muted" },
      "On a radio the throttle stick drives in the gear the mode switch picks; it does nothing until it has been at the bottom once after each start or reset.",
    ),
    controlTable([
      ["Throttle", "CH3, its whole travel", "Right trigger", ["W", "↑"]],
      ["Brake, then reverse", "CH6 low (reverse gear)", "Left trigger", ["S", "↓"]],
      ["Gear", "CH6: high forward, middle neutral, low reverse", "—", []],
      ["Steer", "CH4", "Left stick ←→", ["A", "D", "←", "→"]],
      ["Handbrake", "CH5 high", "A", ["Space"]],
      ["Back on its wheels", "CH7, flicked either way", "Y", ["R"]],
    ]),
  ];
}

export function inputsTab(): HTMLElement {
  const vehicles = Object.values(content()?.vehicles ?? {});
  const withOverlay = vehicles.filter((v) => v.dynamics_overlay).map((v) => v.name);
  return h(
    "div",
    { class: "inputs" },
    h("h4", null, "Inputs"),
    h(
      "p",
      { class: "muted" },
      "A radio's channels are as the Controls tab shows them (a mode 2 radio: throttle and yaw on the left stick). A gamepad flies as a mode 2 radio. The keyboard flies when there is no controller, or when Keyboard flying is on.",
    ),
    ...vehicles.flatMap((v) => (v.kind === "ground" ? ground(v) : aircraft(v))),
    h("h4", { class: "section" }, "Hotkeys"),
    h("h5", null, "In flight"),
    hotkeyTable(flightKeys()),
    h(
      "h5",
      null,
      `The dynamics overlay${withOverlay.length ? ` (${withOverlay.join(", ")})` : ""}, while D has it on`,
    ),
    hotkeyTable(overlayKeys()),
    h("h5", null, "In the main menu"),
    hotkeyTable(menuKeys()),
    h("h5", null, "In this window"),
    hotkeyTable([{ keys: ["Esc"], does: "Close it (resume)" }]),
  );
}
