// What the server says about the session (DESIGN.md §8, §9): its status, the telemetry a ground
// station would see, the firmware's OSD, the vehicle's plan, and the log. Each is a signal fed
// from its channel; the true pose is not here (it is a stream: link.poses()).

import { computed, signal } from "alien-signals";
import type { LogMsg, OsdMsg, PlanMsg, StatusMsg, TelemetryMsg } from "../net/protocol.ts";
import { link } from "../net/server.ts";

const IDLE: StatusMsg = {
  ch: "status",
  state: "idle",
  scenario: null,
  vehicle: null,
  pad: 0,
  step: "",
  video: "analog",
  origin: null,
  heading_deg: 0,
};

export const status = signal<StatusMsg>(IDLE);
export const telemetry = signal<TelemetryMsg | null>(null);
export const osd = signal<OsdMsg | null>(null);
export const plan = signal<PlanMsg | null>(null);
export const log = signal<LogMsg[]>([]);
/** What the physics says the vehicle is doing: "ground", "hand", "flying", "crashed"... */
export const phase = signal<string>("");
export const ready = computed(() => status().state === "ready");

const LOG_KEPT = 50;

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
  link.on("physics").subscribe((p) => phase(p.phase));
  link.on("log").subscribe((l) => log([...log().slice(-(LOG_KEPT - 1)), l]));
}
