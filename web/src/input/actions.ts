// The controls (DESIGN.md §12): one place that hears the keyboard, the mouse and a standard
// gamepad, and turns them into the game's actions. Each part of the game declares its actions
// as a table (what each does, the keys and buttons that do it, its word on the HUD's key strip)
// and mounts it while it is in use; the Inputs tab is drawn from the same tables, so it says what
// the code does. (The aircraft's sticks are not actions: they are radio channels, input/radio.ts.)
//
// Codes: a KeyboardEvent.code ("KeyR", "Digit1", "Escape", "ShiftLeft"); "Mouse0" (left),
// "Mouse2" (right); "Pad0".."Pad16", a standard gamepad's buttons (0 A, 1 B, 2 X, 3 Y, 4 LB,
// 5 RB, 6 LT, 7 RT, 10 L3). Mouse buttons count only while the pointer is locked to the page
// (Shoot Down's aim); the pad is read once a frame in the frame loop's input phase.
//
// While paused (the pause menu), nothing happens but the actions marked `paused`; what is held
// reads as let go.

import { loop } from "../engine/loop.ts";
import { paused } from "../state/app.ts";

export type Code = string;

export interface Binding {
  /** What it does, for the Inputs tab. */
  does: string;
  /** The codes that do it, any one. */
  codes: Code[];
  /** Its word on the HUD's key strip, if it is there. */
  strip?: string;
  /** Shift held with it (true) or not (false); either, if unset. */
  shift?: boolean;
  /** It happens while paused too. */
  paused?: boolean;
  /** How the Inputs tab shows its codes, if not one by one (a row of digits as "1–5"). */
  label?: string[];
  /** Not a row of its own on the Inputs tab (an axis's key, listed with the axis). */
  unlisted?: boolean;
}

export type Bindings<A extends string> = Record<A, Binding>;

const PAD_DEADZONE = 0.12;

const PAD_NAMES = ["A", "B", "X", "Y", "LB", "RB", "LT", "RT", "Back", "Start", "L3", "R3"];
const KEY_NAMES: Record<string, string> = {
  Escape: "Esc",
  Space: "Space",
  ShiftLeft: "Shift",
  ShiftRight: "Shift",
  Enter: "Enter",
  BracketLeft: "[",
  BracketRight: "]",
  ArrowUp: "↑",
  ArrowDown: "↓",
  ArrowLeft: "←",
  ArrowRight: "→",
  Mouse0: "Left button",
  Mouse2: "Right button",
};

/** How a code is shown: "R", "1", "Esc", "Left button", "RT". */
export function codeLabel(code: Code): string {
  if (KEY_NAMES[code]) return KEY_NAMES[code];
  const pad = /^Pad(\d+)$/.exec(code);
  if (pad) return PAD_NAMES[Number(pad[1])] ?? `Button ${pad[1]}`;
  const key = /^(?:Key|Digit)(.)$/.exec(code);
  return key ? key[1]! : code;
}

/** A table's actions as the Inputs tab and the key strip show them: keys, then pad buttons. */
export function hotkeys<A extends string>(
  b: Bindings<A>,
): { keys: string[]; does: string; strip?: string }[] {
  return (Object.values(b) as Binding[])
    .filter((x) => !x.unlisted)
    .map((x) => {
      const shown = x.label ?? [
        ...new Set(x.codes.map((c) => (x.shift ? `Shift+${codeLabel(c)}` : codeLabel(c)))),
      ];
      return { keys: shown, does: x.does, ...(x.strip ? { strip: x.strip } : {}) };
    });
}

interface Layer {
  bindings: Record<string, Binding>;
  handlers: Map<string, ((code: Code) => void)[]>;
}

/** The page's one listener. */
class Hub {
  private readonly held = new Set<Code>();
  private readonly layers: Layer[] = [];
  private padWas: boolean[] = [];
  private mouseX = 0;
  private mouseY = 0;
  private started = false;
  /** The canvas the mouse is locked to, when it is. */
  lockedTo: HTMLElement | null = null;

  start(): void {
    if (this.started) return;
    this.started = true;
    window.addEventListener("keydown", (e) => {
      if (isTyping(e)) return;
      this.held.add(e.code);
      if (!e.repeat) this.press(e.code, e.shiftKey, e);
    });
    window.addEventListener("keyup", (e) => this.held.delete(e.code));
    window.addEventListener("blur", () => this.held.clear());
    // Pointer events, not mouse events: Cesium cancels pointerdown on its canvas, and a cancelled
    // pointerdown takes the mouse events with it. A second button pressed while one is held comes
    // as a pointermove with new `buttons`, so presses and releases are read from that mask.
    let mask = 0;
    const buttons = (e: PointerEvent) => {
      const was = mask;
      mask = e.buttons;
      if (!this.lockedTo || document.pointerLockElement !== this.lockedTo) return;
      for (let i = 0; i < 5; i++) {
        const bit = 1 << [0, 2, 1, 3, 4][i]!;
        const code = `Mouse${i}`;
        if (mask & bit && !(was & bit)) {
          this.held.add(code);
          this.press(code, e.shiftKey);
        } else if (!(mask & bit) && was & bit) this.held.delete(code);
      }
    };
    window.addEventListener(
      "pointerdown",
      (e) => {
        if (e.pointerType !== "mouse") return;
        const canvas = this.lockedTo;
        if (canvas && document.pointerLockElement === canvas) {
          // Locked, the press is ours alone: Cesium's own handler would try to capture a locked
          // pointer, which Chrome refuses (an error every press).
          e.stopPropagation();
          buttons(e);
          return;
        }
        mask = e.buttons;
        if (canvas && e.target === canvas && !paused()) {
          // The first click takes the mouse; it does nothing else. Asked once Cesium has had the
          // press, so its pointer capture comes before the lock and not after it.
          setTimeout(() => {
            if (this.lockedTo !== canvas) return;
            void (canvas.requestPointerLock as (o?: object) => Promise<void>)
              .call(canvas, { unadjustedMovement: true })
              ?.catch?.(() => canvas.requestPointerLock());
          });
        }
      },
      { capture: true },
    );
    window.addEventListener("contextmenu", (e) => {
      if (this.lockedTo && e.target === this.lockedTo) e.preventDefault();
    });
    window.addEventListener("pointerup", buttons);
    window.addEventListener("pointermove", (e) => {
      if (e.buttons !== mask) buttons(e);
      if (this.lockedTo && document.pointerLockElement === this.lockedTo && !paused()) {
        this.mouseX += e.movementX;
        this.mouseY += e.movementY;
      }
    });
    document.addEventListener("pointerlockchange", () => {
      if (document.pointerLockElement) return;
      mask = 0;
      for (const c of [...this.held]) if (c.startsWith("Mouse")) this.held.delete(c);
    });
    loop.each("controls", "input", () => this.readPad());
  }

  mount(layer: Layer): () => void {
    this.start();
    this.layers.push(layer);
    return () => {
      const i = this.layers.indexOf(layer);
      if (i >= 0) this.layers.splice(i, 1);
    };
  }

  /** Whether `code` is held (not while paused). */
  isHeld(code: Code): boolean {
    return !paused() && this.held.has(code);
  }

  /** Every code held now (the truck's keys). */
  get codes(): ReadonlySet<Code> {
    return paused() ? new Set() : this.held;
  }

  /** The mouse's movement since last asked, px (locked only). */
  takeMouse(): [number, number] {
    const out: [number, number] = [this.mouseX, this.mouseY];
    this.mouseX = this.mouseY = 0;
    return out;
  }

  /** A standard gamepad's axis, -1..1 past a dead zone (0 with none, or paused). */
  stick(axis: number): number {
    const pad = standardPad();
    const x = pad?.axes[axis] ?? 0;
    if (paused() || Math.abs(x) < PAD_DEADZONE) return 0;
    return (Math.sign(x) * (Math.abs(x) - PAD_DEADZONE)) / (1 - PAD_DEADZONE);
  }

  private readPad(): void {
    const pad = standardPad();
    if (!pad) {
      for (const c of [...this.held]) if (c.startsWith("Pad")) this.held.delete(c);
      this.padWas = [];
      return;
    }
    const now = pad.buttons.map((b) => b.value > 0.5);
    now.forEach((down, i) => {
      const code = `Pad${i}`;
      if (down) this.held.add(code);
      else this.held.delete(code);
      if (down && !this.padWas[i]) this.press(code, false);
    });
    this.padWas = now;
  }

  private press(code: Code, shift: boolean, e?: KeyboardEvent): void {
    // The newest layer first: a mode's controls before the flight's.
    for (const layer of [...this.layers].reverse()) {
      for (const [action, b] of Object.entries(layer.bindings)) {
        if (!b.codes.includes(code)) continue;
        if (b.shift !== undefined && b.shift !== shift && !code.startsWith("Shift")) continue;
        if (paused() && !b.paused) continue;
        const handlers = layer.handlers.get(action);
        if (!handlers?.length) continue;
        if (code === "Space" || code.startsWith("Key") || code.startsWith("Digit")) {
          // Not a press of whatever button last had the focus, nor a scroll.
          e?.preventDefault();
        }
        for (const h of handlers) h(code);
      }
    }
  }
}

function isTyping(e: KeyboardEvent): boolean {
  const t = e.target as HTMLElement | null;
  return !!t?.closest?.("input, select, textarea");
}

function standardPad(): Gamepad | null {
  if (typeof navigator === "undefined" || !navigator.getGamepads) return null;
  return navigator.getGamepads().find((p): p is Gamepad => !!p && p.mapping === "standard") ?? null;
}

const hub = new Hub();

/** A table of actions, mounted while in use: `on` what happens on a press, `held` whether one is
 * held. */
export class Controls<A extends string> {
  private readonly layer: Layer;
  private readonly unmount: () => void;

  constructor(readonly bindings: Bindings<A>) {
    this.layer = { bindings, handlers: new Map() };
    this.unmount = hub.mount(this.layer);
  }

  /** On a press of `action`: `fn(code)`, the code that did it. */
  on(action: A, fn: (code: Code) => void): this {
    const list = this.layer.handlers.get(action) ?? [];
    list.push(fn);
    this.layer.handlers.set(action, list);
    return this;
  }

  /** Whether any of `action`'s codes is held (never while paused). */
  held(action: A): boolean {
    return this.bindings[action].codes.some((c) => hub.isHeld(c));
  }

  /** Keys as an axis: +1 for `pos` held, -1 for `neg`. */
  axis(neg: A, pos: A): number {
    return (this.held(pos) ? 1 : 0) - (this.held(neg) ? 1 : 0);
  }

  dispose(): void {
    this.unmount();
  }
}

/** The whole page's input, below the tables: the pad's sticks, the locked mouse, the keys held. */
export const input = {
  stick: (axis: number) => hub.stick(axis),
  takeMouse: () => hub.takeMouse(),
  get codes(): ReadonlySet<Code> {
    return hub.codes;
  },
  /** Lock the mouse to `canvas` on a click there (the first click takes it, and does nothing
   * else), and read its buttons while locked; null: no more. */
  lockMouseTo(canvas: HTMLElement | null): void {
    hub.start();
    if (!canvas && hub.lockedTo && document.pointerLockElement === hub.lockedTo) {
      document.exitPointerLock();
    }
    hub.lockedTo = canvas;
  },
  /** Whether the mouse is locked to the page now. */
  get mouseLocked(): boolean {
    return !!hub.lockedTo && document.pointerLockElement === hub.lockedTo;
  },
  start: () => hub.start(),
};
