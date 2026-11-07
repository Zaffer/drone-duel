// The pilot's controller (DESIGN.md §12): a drone radio through Chrome's Gamepad API, or the same
// radio read by the server through Linux (when Chrome cannot see all its axes), or a standard
// gamepad, or the keyboard. Sampled at RATE_HZ; `samples()` is that stream, and the `channels`
// signal (state/input.ts) its latest value for the UI.
//
// The source with the most axes wins; on a tie, the server's: Chrome can list an axis and never
// move it (the LiteRadio 2's CH5 and CH8), which Linux reads fine. A radio is any pad without the
// standard mapping that has four axes or more.
//
// The server's radio flies from the server (rc/pilot.py), never through this page's thread, which
// is busy drawing (DESIGN.md §3.1, §12): this says whether it flies (`radio_source`), and takes
// its channels as the server maps them, for the UI and the truck. Flight sends `rc` only for the
// page's own controllers.

import { link } from "../net/server.ts";
import { content } from "../state/app.ts";
import { channels, controllerName, inputSource } from "../state/input.ts";
import { prefs } from "../state/prefs.ts";
import { Controls, input } from "./actions.ts";
import {
  gamepadChannels,
  type KeyState,
  keyboardChannels,
  profileFor,
  toChannels,
} from "./mapping.ts";

export const RATE_HZ = 250;

export interface RawController {
  id: string;
  axes: readonly number[];
  buttons: readonly number[];
  standard: boolean;
  source: "radio" | "gamepad" | "server";
}

/** The keyboard's switches when it flies (the Inputs tab lists them with the sticks). */
const KEYBOARD_SWITCHES = {
  arm: { does: "Arm or disarm (flying by the keyboard)", codes: ["Enter"] },
  mode: { does: "The next flight mode (flying by the keyboard)", codes: ["KeyM"] },
};

class Radio extends EventTarget {
  /** The server's reading of the radio, newest, with its channels as the server maps them. */
  private server: { name: string | null; axes: number[]; channels: number[]; at: number } | null =
    null;
  /** Whether the server's radio flies (as last told to the server). */
  private serverFlies = false;
  private keyState: KeyState = { throttle: -1, armed: false, modePos: 0 };
  private padArm = { armed: false, modePos: 0, a: false, x: false };
  private timer: ReturnType<typeof setInterval> | undefined;
  private last = performance.now();
  /** Fly by the keyboard even with a controller plugged in (Options → Controls). */
  keyboard = false;
  /** The player is on foot (Shoot Down): the server's radio must not fly the vehicle, which is
   * the target, flying itself (an arm switch moved would disarm it). */
  onFoot = false;
  /** The latest raw controller, for the calibration wizard. */
  raw: RawController | null = null;

  start(): void {
    const tell = () => link.send({ ch: "radio_source", on: true, fly: this.serverFlies });
    tell();
    link.when("open").subscribe(tell);
    link.on("radio").subscribe((m) => {
      this.server = { name: m.name, axes: m.axes, channels: m.channels, at: performance.now() };
    });
    // Flying by the keyboard: its switches (the sticks are the keys held, input/mapping.ts).
    new Controls(KEYBOARD_SWITCHES)
      .on("arm", () => {
        if (inputSource() === "keyboard") this.keyState.armed = !this.keyState.armed;
      })
      .on("mode", () => {
        if (inputSource() === "keyboard") this.keyState.modePos = (this.keyState.modePos + 1) % 3;
      });
    this.timer = setInterval(() => this.sample(), 1000 / RATE_HZ);
  }

  stop(): void {
    clearInterval(this.timer);
  }

  /** The keys held now (the truck reads them too). */
  get held(): ReadonlySet<string> {
    return input.codes;
  }

  /** Every sample: the channels, -1..1 from channel 1. */
  samples(): Observable<number[]> {
    return this.when<CustomEvent<number[]>>("sample").map((e) => e.detail);
  }

  /** The best controller there is now. */
  pick(): RawController | null {
    const candidates: RawController[] = [];
    const s = this.server;
    if (s?.name && performance.now() - s.at < 500) {
      candidates.push({ id: s.name, axes: s.axes, buttons: [], standard: false, source: "server" });
    }
    for (const pad of navigator.getGamepads()) {
      if (!pad || !pad.connected) continue;
      const standard = pad.mapping === "standard";
      if (!standard && pad.axes.length < 4) continue;
      candidates.push({
        id: pad.id,
        axes: pad.axes,
        buttons: pad.buttons.map((b) => b.value),
        standard,
        source: standard ? "gamepad" : "radio",
      });
    }
    const radios = candidates.filter((c) => !c.standard);
    if (radios.length > 0) {
      return radios.reduce((a, b) => (b.axes.length > a.axes.length ? b : a));
    }
    return candidates[0] ?? null;
  }

  private sample(): void {
    const now = performance.now();
    const dt = (now - this.last) / 1000;
    this.last = now;
    const raw = this.pick();
    this.raw = raw;
    const serverFlies = !this.keyboard && !this.onFoot && raw?.source === "server";
    if (serverFlies !== this.serverFlies) {
      this.serverFlies = serverFlies;
      link.send({ ch: "radio_source", on: true, fly: serverFlies });
    }
    let out: number[];
    if (this.keyboard || !raw) {
      // The keyboard: chosen in Controls, or the only controller there is.
      out = keyboardChannels(input.codes, this.keyState, dt);
      if (inputSource() !== "keyboard") inputSource("keyboard");
      if (controllerName() !== "Keyboard") controllerName("Keyboard");
    } else if (raw.source === "server" && this.server?.channels.length) {
      out = this.server.channels;
      if (controllerName() !== this.serverName()) controllerName(this.serverName());
      if (inputSource() !== "server") inputSource("server");
    } else if (!raw.standard) {
      const profiles = Object.values(content()?.controllers ?? {});
      const profile = profileFor(raw.id, profiles);
      out = toChannels(raw.axes, raw.buttons, profile, prefs().controls.calibrations[raw.id]);
      if (controllerName() !== (profile?.name ?? raw.id)) controllerName(profile?.name ?? raw.id);
      if (inputSource() !== raw.source) inputSource(raw.source);
    } else {
      const pad = navigator.getGamepads().find((p) => p?.id === raw.id && p.mapping === "standard");
      if (!pad) return;
      const a = pad.buttons[0]?.pressed ?? false;
      const x = pad.buttons[2]?.pressed ?? false;
      if (a && !this.padArm.a) this.padArm.armed = !this.padArm.armed;
      if (x && !this.padArm.x) this.padArm.modePos = (this.padArm.modePos + 1) % 3;
      this.padArm.a = a;
      this.padArm.x = x;
      out = gamepadChannels(pad, this.padArm.armed, this.padArm.modePos);
      if (controllerName() !== raw.id) controllerName(raw.id);
      if (inputSource() !== "gamepad") inputSource("gamepad");
    }
    this.latest = out;
    this.dispatchEvent(new CustomEvent("sample", { detail: out }));
  }

  /** The server's radio by its profile's name, else its own. */
  private serverName(): string | null {
    const name = this.server?.name ?? null;
    if (!name) return null;
    const profile = profileFor(name, Object.values(content()?.controllers ?? {}));
    return profile?.name ?? name;
  }

  /** The newest sample (the UI copies it into `channels` once a frame). */
  latest: number[] = [];

  /** Copy the newest sample into the `channels` signal: once a frame is plenty for the UI. */
  publish(): void {
    if (this.latest.length) channels(this.latest);
  }
}

export const radio = new Radio();
