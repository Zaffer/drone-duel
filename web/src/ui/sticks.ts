// The touch screen's controls in flight (DESIGN.md §11, §12): two sticks along the bottom, each
// the whole of its side of the screen to a thumb, and between them the radio's switches as
// buttons: Arm and the flight mode for an aircraft, the handbrake for the truck. Shown while the
// sticks on the glass are what flies (`inputSource` "touch": no controller, a touch screen); a
// radio or a gamepad plugged into the phone takes over and they go. Their state and what a thumb
// does to it are input/touch.ts.

import { effect } from "alien-signals";
import { startMode } from "../input/mapping.ts";
import { type Stick, touch } from "../input/touch.ts";
import { vehicle } from "../state/app.ts";
import { inputSource } from "../state/input.ts";
import { h } from "./h.ts";

/** The knob's travel from the centre, as a share of the stick's width (ui.css: the knob is 38%
 * of it, and stops at the rim). */
const THROW = 0.31;

type Side = "left" | "right";

export function mountSticks(host: HTMLElement): () => void {
  const abort = new AbortController();
  const { signal } = abort;

  const stick = (side: Side) => {
    const knob = h("div", { class: "knob" });
    const ring = h("div", { class: "stick" }, knob);
    const zone = h("div", { class: `stick-zone ${side}` }, ring);
    const throwPx = () => ring.clientWidth * THROW;
    const draw = () => {
      const t = throwPx();
      const s = touch[side];
      knob.style.transform = `translate(${s.x * t}px, ${-s.y * t}px)`;
    };
    /** The thumb on this stick: its pointer, where it came down, and the stick then. */
    let held: { id: number; x: number; y: number; from: Stick } | null = null;
    zone.addEventListener(
      "pointerdown",
      (e) => {
        if (held) return;
        e.preventDefault();
        zone.setPointerCapture(e.pointerId);
        held = { id: e.pointerId, x: e.clientX, y: e.clientY, from: { ...touch[side] } };
        ring.classList.add("held");
      },
      { signal },
    );
    zone.addEventListener(
      "pointermove",
      (e) => {
        if (held?.id !== e.pointerId) return;
        touch.drag(side, held.from, e.clientX - held.x, e.clientY - held.y, throwPx());
        draw();
      },
      { signal },
    );
    const up = (e: PointerEvent) => {
      if (held?.id !== e.pointerId) return;
      held = null;
      ring.classList.remove("held");
      touch.release(side);
      draw();
    };
    zone.addEventListener("pointerup", up, { signal });
    zone.addEventListener("pointercancel", up, { signal });
    // A long press is not a menu here.
    zone.addEventListener("contextmenu", (e) => e.preventDefault(), { signal });
    return { zone, draw };
  };

  const left = stick("left");
  const right = stick("right");

  const arm = h("button", { class: "tbtn arm" }, "Arm");
  const mode = h("button", { class: "tbtn mode" });
  const brake = h("button", { class: "tbtn brake" }, "Brake");
  const drawSwitches = () => {
    arm.classList.toggle("on", touch.armed);
    arm.textContent = touch.armed ? "Disarm" : "Arm";
    mode.textContent = vehicle()?.aux?.modes[touch.modePos] ?? `Mode ${touch.modePos + 1}`;
    brake.classList.toggle("on", touch.handbrake);
  };
  arm.addEventListener(
    "click",
    () => {
      touch.armed = !touch.armed;
      drawSwitches();
    },
    { signal },
  );
  mode.addEventListener(
    "click",
    () => {
      touch.modePos = (touch.modePos + 1) % 3;
      drawSwitches();
    },
    { signal },
  );
  const hold = (on: boolean) => (e: PointerEvent) => {
    if (on) brake.setPointerCapture(e.pointerId);
    touch.handbrake = on;
    drawSwitches();
  };
  brake.addEventListener("pointerdown", hold(true), { signal });
  brake.addEventListener("pointerup", hold(false), { signal });
  brake.addEventListener("pointercancel", hold(false), { signal });
  brake.addEventListener("contextmenu", (e) => e.preventDefault(), { signal });

  const buttons = h("div", { class: "touch-buttons" }, arm, mode, brake);
  const root = h("div", { class: "touch-controls" }, left.zone, buttons, right.zone);
  host.append(root);

  const disposers = [
    // Each vehicle starts as a radio just switched on: disarmed, the throttle down (the truck's
    // at its centre).
    effect(() => {
      const v = vehicle();
      const ground = v?.kind === "ground";
      touch.reset(ground, startMode(v?.aux?.modes ?? []));
      arm.hidden = ground || !v?.aux;
      mode.hidden = ground || !v?.aux?.mode || v.aux.modes.length < 2;
      brake.hidden = !ground;
      drawSwitches();
      left.draw();
      right.draw();
    }),
    effect(() => {
      const on = inputSource() === "touch";
      root.hidden = !on;
      // The HUD makes room (ui.css): the bottom corners are the thumbs'.
      document.documentElement.classList.toggle("touch-sticks", on);
      if (on) {
        left.draw();
        right.draw();
      }
    }),
  ];
  const redraw = () => {
    left.draw();
    right.draw();
    drawSwitches();
  };
  window.addEventListener("resize", redraw, { signal });
  touch.onChange = redraw;

  return () => {
    abort.abort();
    touch.onChange = null;
    for (const d of disposers) d();
    document.documentElement.classList.remove("touch-sticks");
    touch.reset(false);
    root.remove();
  };
}
