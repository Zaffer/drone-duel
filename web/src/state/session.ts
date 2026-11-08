// What the server says about the session (DESIGN.md §8, §9): its status, the telemetry a ground
// station would see, the firmware's OSD, the vehicle's plan, and the log. Each is a signal fed
// from its channel; the true pose is not here (it is a stream: link.poses()).

import { computed, signal } from "alien-signals";
import type { Damage, LogMsg, OsdMsg, PlanMsg, StatusMsg, TelemetryMsg } from "../net/protocol.ts";
import { link } from "../net/server.ts";
import { toast } from "./app.ts";

const IDLE: StatusMsg = {
  ch: "status",
  state: "idle",
  location: null,
  vehicle: null,
  pad: 0,
  step: "",
  video: "analog",
  origin: null,
  heading_deg: 0,
  payload: null,
};

export const status = signal<StatusMsg>(IDLE);
export const telemetry = signal<TelemetryMsg | null>(null);
export const osd = signal<OsdMsg | null>(null);
export const plan = signal<PlanMsg | null>(null);
export const log = signal<LogMsg[]>([]);
/** What the physics says the vehicle is doing: "ground", "hand", "flying", "crashed"... */
export const phase = signal<string>("");
/** What hits have taken from it (Shoot Down), or null: whole. */
export const damage = signal<Damage | null>(null);
export const ready = computed(() => status().state === "ready");

const LOG_KEPT = 50;

/** A new session asked for: until the server says otherwise it is starting, and the last one's
 * "ready", telemetry, OSD and plan are not this one's (a race would count down while the new
 * firmware boots). */
export function sessionStarting(location: string | null, vehicle: string, pad: number): void {
  status({ ...status(), state: "starting", step: "", location, vehicle, pad, origin: null });
  telemetry(null);
  osd(null);
  plan(null);
  damage(null);
}

/** Feed the signals from the server link. Once, at start. */
export function wireSession(): void {
  link.on("status").subscribe((s) => {
    status(s);
    if (s.state !== "ready") {
      telemetry(null);
      osd(null);
      plan(null);
    }
  });
  link.on("telemetry").subscribe((t) => telemetry(t));
  link.on("osd").subscribe((o) => osd(o));
  link.on("plan").subscribe((p) => plan(p));
  link.on("physics").subscribe((p) => {
    phase(p.phase);
    damage(p.damage ?? null);
  });
  link.on("log").subscribe((l) => {
    log([...log().slice(-(LOG_KEPT - 1)), l]);
    if (l.source === "server" && l.level === "error") toast(l.text, "bad", 10000);
  });
}
