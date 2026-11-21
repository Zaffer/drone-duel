// A touch screen's sticks (DESIGN.md §12): two thumbs on the glass as a mode 2 radio, for a
// phone or a tablet with no controller. This is their state and what a thumb does to it; they
// are drawn, and the thumbs heard, in ui/sticks.ts. input/radio.ts turns it into the pilot's
// channels (input/mapping.ts's `touchChannels`); the truck reads it as it stands (flight.ts).
//
// A stick moves with the thumb's travel from where the thumb came down, not to where it is: a
// thumb landing anywhere on its side of the screen takes the stick as it stood, so nothing jumps
// (a throttle least of all). Let go, a sprung axis goes back to its centre; an aircraft's
// throttle stays where it was left, as a radio's does. The truck's is sprung: forward of centre
// drives, back of it brakes and then reverses.

import { touchScreen } from "../state/input.ts";

/** A stick, each axis -1..1: x +1 right, y +1 up (away from the pilot). */
export interface Stick {
  x: number;
  y: number;
}

const clamp = (v: number): number => Math.max(-1, Math.min(1, v));

/** `from` moved by a thumb's travel, px (the screen's y runs down), on a stick whose full throw
 * from its centre is `throwPx`. */
export function dragged(from: Stick, dx: number, dy: number, throwPx: number): Stick {
  const t = Math.max(throwPx, 1);
  return { x: clamp(from.x + dx / t), y: clamp(from.y - dy / t) };
}

export class TouchSticks {
  /** Throttle (y) and yaw (x). */
  left: Stick = { x: 0, y: -1 };
  /** Pitch (y) and roll (x). */
  right: Stick = { x: 0, y: 0 };
  /** The arm switch. */
  armed = false;
  /** The flight mode switch: 0 low, 1 middle, 2 high. */
  modePos = 0;
  /** The truck's handbrake, while its button is held. */
  handbrake = false;
  /** Whether the left stick's up and down is sprung (the truck's) or stays (a throttle). */
  sprung = false;

  /** As a radio just switched on: sticks let go, the throttle down, disarmed, the mode switch
   * at `modePos` (input/mapping.ts's `startMode`). */
  reset(sprung: boolean, modePos = 0): void {
    this.sprung = sprung;
    this.left = { x: 0, y: sprung ? 0 : -1 };
    this.right = { x: 0, y: 0 };
    this.armed = false;
    this.modePos = modePos;
    this.handbrake = false;
  }

  /** Called when the sticks or switches are moved from here, not by a thumb (ui/sticks.ts
   * draws them again). */
  onChange: (() => void) | null = null;

  /** Disarmed, the throttle down: before the vehicle is put back at the start, so it does not
   * leap off the pad on a throttle left where it was. */
  rest(): void {
    this.armed = false;
    if (!this.sprung) this.left = { x: this.left.x, y: -1 };
    this.onChange?.();
  }

  /** A thumb moved on a stick: where it stood when the thumb came down, and the travel since. */
  drag(side: "left" | "right", from: Stick, dx: number, dy: number, throwPx: number): void {
    this[side] = dragged(from, dx, dy, throwPx);
  }

  /** The thumb is off the stick. */
  release(side: "left" | "right"): void {
    if (side === "right") this.right = { x: 0, y: 0 };
    else this.left = { x: 0, y: this.sprung ? 0 : this.left.y };
  }
}

/** The page's one pair. */
export const touch = new TouchSticks();

/** Keep `touchScreen` (state/input.ts) to what the player last used: it starts as the device
 * says (a phone's pointer is coarse), a touch turns it on, a key pressed turns it off (a
 * laptop with a touch screen flies by whichever was used last). */
export function watchTouch(): void {
  touchScreen(window.matchMedia?.("(pointer: coarse)").matches ?? false);
  window.addEventListener(
    "pointerdown",
    (e) => {
      if (e.pointerType === "touch" && !touchScreen()) touchScreen(true);
    },
    { capture: true },
  );
  window.addEventListener("keydown", (e) => {
    // A phone's own keyboard, typing a name, is not flying by the keyboard.
    const typing = (e.target as HTMLElement | null)?.closest?.("input, select, textarea");
    if (!typing && touchScreen()) touchScreen(false);
  });
}
