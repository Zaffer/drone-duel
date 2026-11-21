// The Inputs tab (Options): every key, and how to fly or drive each vehicle on a radio, a
// gamepad, the keyboard or a touch screen's sticks (a touch screen shows that column, not the
// keyboard's: ui.css). The hotkeys are drawn from the very tables the code mounts
// (ui/keys.ts, shoot/controls.ts: input/actions.ts). The sticks' rows say what input/mapping.ts
// and input/radio.ts (the aircraft), vehicles/truck/controls.ts (the truck) and ui/sticks.ts
// and flight.ts (the touch screen) do: change one, change this.

import { LAYERS } from "../dynamics/overlay.ts";
import { hotkeys } from "../input/actions.ts";
import { MODES } from "../modes/registry.ts";
import type { Vehicle } from "../net/protocol.ts";
import { shooterAxes, shooterBindings } from "../shoot/controls.ts";
import { content } from "../state/app.ts";
import { VIEW_NAMES, VIEW_ORDER } from "../views/rigs.ts";
import { h } from "./h.ts";
import { flightBindings, footBindings, menuBindings, overlayBindings } from "./keys.ts";

/** A hotkey: the keys that do it (any one), what it does, and its word on the HUD's key strip. */
export interface Hotkey {
  keys: string[];
  does: string;
  strip?: string;
}

// Each list is the table the code mounts (ui/keys.ts, shoot/controls.ts), as shown.

export const menuKeys = (): Hotkey[] => hotkeys(menuBindings(MODES.map((m) => m.name)));

/** In flight, in the HUD's order; those with `strip` are on its key strip too. */
export const flightKeys = (): Hotkey[] =>
  hotkeys(flightBindings(VIEW_ORDER.map((v) => VIEW_NAMES[v])));

/** Shoot Down's: the shooter's, then the few of flight's that still mean something. */
export const shooterKeys = (): Hotkey[] => [
  ...shooterAxes,
  ...hotkeys(shooterBindings),
  ...hotkeys(footBindings()),
];

const overlayKeys = (): Hotkey[] => hotkeys(overlayBindings(LAYERS.map((l) => l.name)));

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

type Row = [job: string, radio: string, gamepad: string, keyboard: string[], touch: string];

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
        h("th"),
        h("th", null, "Radio"),
        h("th", null, "Gamepad"),
        h("th", { class: "for-keys" }, "Keyboard"),
        h("th", { class: "for-touch" }, "Touch screen"),
      ),
    ),
    h(
      "tbody",
      null,
      rows.map(([job, radio, pad, keys, glass]) => {
        const keyboard = keys.length ? keyCell(keys) : h("td", { class: "muted" }, "—");
        keyboard.classList.add("for-keys");
        return h(
          "tr",
          null,
          h("td", null, job),
          h("td", { class: "num" }, radio),
          h("td", null, pad),
          keyboard,
          h("td", { class: "for-touch" }, glass),
        );
      }),
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
    ["Roll", "CH1", "Right stick ←→", ["←", "→"], "Right stick ←→"],
    ["Pitch", "CH2", "Right stick ↑↓", ["↑", "↓"], "Right stick ↑↓"],
    ["Throttle", "CH3", "Left stick ↑↓ (centre is half)", ["W", "S"], "Left stick ↑↓ (it stays)"],
    ["Yaw", "CH4", "Left stick ←→", ["Q", "E"], "Left stick ←→"],
  ];
  if (aux) rows.push(["Arm", `CH${aux.arm} high`, "A (toggles)", ["Enter"], "Arm (toggles)"]);
  if (aux?.mode && aux.modes.length) {
    rows.push([
      "Flight mode",
      `CH${aux.mode}`,
      "X (steps through)",
      ["M"],
      "The mode's button (steps through)",
    ]);
  }
  if (hand) rows.push(["Throw", aux?.launch ? `CH${aux.launch} flick up` : "—", "—", ["T"], "—"]);
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
      ["Throttle", "CH3, its whole travel", "Right trigger", ["W", "↑"], "Left stick ↑"],
      ["Brake, then reverse", "CH6 low (reverse gear)", "Left trigger", ["S", "↓"], "Left stick ↓"],
      ["Gear", "CH6: high forward, middle neutral, low reverse", "—", [], "—"],
      ["Steer", "CH4", "Left stick ←→", ["A", "D", "←", "→"], "Right stick ←→"],
      ["Handbrake", "CH5 high", "A", ["Space"], "Brake (held)"],
      ["Back on its wheels", "CH7, flicked either way", "Y", ["R"], "Reset, on the top bar"],
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
      "A radio's channels are as the Controls tab shows them (a mode 2 radio: throttle and yaw on the left stick). A gamepad flies as a mode 2 radio. With no controller the keyboard flies, or on a touch screen two sticks on the glass, each moved by a thumb anywhere on its side; Keyboard flying on, the keyboard flies whatever else there is.",
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
