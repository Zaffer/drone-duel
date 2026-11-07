// The shooter's controls (shoot/shooter.ts), keyboard and mouse or a standard gamepad, as a
// table of actions (input/actions.ts). Walking and looking are axes as well: W A S D or the left
// stick, the mouse or the right stick.

import type { Bindings } from "../input/actions.ts";

export const shooterBindings = {
  aim: {
    does: "On the sight (held, or a toggle: the Rifle tab)",
    codes: ["Mouse2", "Pad6"],
    strip: "aim",
  },
  fire: { does: "Fire: one press a shot", codes: ["Mouse0", "Pad7"], strip: "fire" },
  reload: { does: "Reload", codes: ["KeyR", "Pad2"], strip: "reload" },
  forward: { does: "Walk forward", codes: ["KeyW"], unlisted: true },
  back: { does: "Walk back", codes: ["KeyS"], unlisted: true },
  left: { does: "Step left", codes: ["KeyA"], unlisted: true },
  right: { does: "Step right", codes: ["KeyD"], unlisted: true },
  sprint: {
    does: "With forward: sprint (the rifle comes down)",
    codes: ["ShiftLeft", "ShiftRight", "Pad10"],
    strip: "sprint",
  },
  jump: {
    does: "Jump; at a wall, vault or climb it",
    codes: ["Space", "Pad0"],
    strip: "jump · climb",
  },
  breath: {
    does: "On the sight: hold your breath",
    codes: ["ShiftLeft", "ShiftRight", "Pad5"],
    strip: "breath",
  },
  kneel: { does: "Kneel or stand", codes: ["KeyC"], strip: "kneel" },
  prone: { does: "Prone or stand", codes: ["KeyZ"], strip: "prone" },
  stance: { does: "Standing, kneeling, prone in turn", codes: ["Pad1"] },
  range: {
    does: "The rangefinder: the range to what the sight is on",
    codes: ["KeyF", "Pad4"],
    strip: "range",
  },
  sight: { does: "Holo or iron sights", codes: ["KeyI", "Pad3"], strip: "sight" },
} satisfies Bindings<string>;

export type ShooterAction = keyof typeof shooterBindings;

/** The axes, for the Inputs tab: not actions, read each frame. */
export const shooterAxes = [
  { keys: ["Click"], does: "Take the mouse (Esc gives it back)" },
  { keys: ["Mouse", "Right stick"], does: "Look and aim" },
  {
    keys: ["W A S D", "Left stick"],
    does: "Walk; on into a low wall vaults it, leaning into a bush pushes through",
  },
];
