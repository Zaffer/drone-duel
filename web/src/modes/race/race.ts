// Race (DESIGN.md §7): through the course's gates against the clock, alone (a time trial) or
// head to head, live: each player flies their own sim and sees the other as a ghost.
//
// The lobby: Solo, or Host (a room code to read out) and Join. Then both fly to the grid, the
// host on pad 1, the guest on pad 2. Once both vehicles are ready the host sets the start a few
// seconds ahead, on its clock; each page counts down to it on its own clock less the offset the
// pings measured. Gates count from GO. The first to finish wins; both see both times.
//
// Messages (peer.ts): lobby {vehicle, ready}  start {} (host: to the grid)  grid {} (on it, ready)
// go {at}  pose PeerPose  gate {next, t}
// done {time, laps}  again {}

import { effect, signal } from "alien-signals";
import { sounds } from "../../audio/sounds.ts";
import type { Flight, ModeRuntime } from "../../flight.ts";
import { cleanRoomCode, newRoomCode, type Role, Room } from "../../net/peer.ts";
import type { Course } from "../../net/protocol.ts";
import type { Pose } from "../../net/server.ts";
import { content, course as currentCourse, toast } from "../../state/app.ts";
import { hudBanner, hudPanel, hudTimer } from "../../state/hud.ts";
import { prefs, updatePrefs } from "../../state/prefs.ts";
import { status } from "../../state/session.ts";
import { h, render } from "../../ui/h.ts";
import { mapOffset } from "../../world/geo.ts";
import { Ghost, type PeerPose } from "../ghost.ts";
import type { ModeContext, ModeDef } from "../types.ts";
import { CourseView } from "./course.ts";
import { fmtTime, Progress, passes } from "./gates.ts";

const TYPES = ["lobby", "start", "grid", "go", "pose", "gate", "done", "again"];
const COUNTDOWN_S = 5;
const POSE_HZ = 30;
const BEST_KEY = "drone-duels-best";

function best(courseId: string): number | null {
  try {
    const v = JSON.parse(localStorage.getItem(BEST_KEY) ?? "{}")[courseId];
    return typeof v === "number" ? v : null;
  } catch {
    return null;
  }
}

function saveBest(courseId: string, t: number): boolean {
  const old = best(courseId);
  if (old !== null && old <= t) return false;
  try {
    const all = JSON.parse(localStorage.getItem(BEST_KEY) ?? "{}");
    all[courseId] = t;
    localStorage.setItem(BEST_KEY, JSON.stringify(all));
  } catch {}
  return true;
}

export const race: ModeDef = {
  id: "race",
  name: "Race",
  desc: "Through the gates against the clock, alone or head to head.",
  vehicles: (all) => all.filter((v) => v.kind !== "ground"),
  places: (all) =>
    all.filter((loc) => Object.values(content()?.courses ?? {}).some((c) => c.location === loc.id)),
  lobby(host, ctx) {
    return raceLobby(host, ctx);
  },
};

function raceLobby(host: HTMLElement, ctx: ModeContext): () => void {
  const mine = ctx.vehicle;
  let room: Room | null = null;
  const peerState = signal<{ vehicle: string; ready: boolean } | null>(null);
  const disposers: (() => void)[] = [];
  const nameInput = h("input", { value: prefs().player.name }) as HTMLInputElement;
  nameInput.addEventListener("change", () =>
    updatePrefs((p) => ({ ...p, player: { name: nameInput.value.trim() || "Pilot" } })),
  );
  const codeInput = h("input", {
    placeholder: "room code",
    style: "width: 130px; text-transform: lowercase",
  }) as HTMLInputElement;
  const body = h("div");

  const solo = () => ctx.fly(mine(), 0, (f) => new RaceRuntime(f, null, mine(), null));

  const enter = async (role: Role, code: string) => {
    room = await Room.join("race", code, role, nameInput.value.trim() || "Pilot", TYPES);
    const r = room;
    const lobbyOnly = new AbortController();
    disposers.push(() => lobbyOnly.abort());
    const signal = lobbyOnly.signal;
    r.on<{ vehicle: string; ready: boolean }>("lobby").subscribe((m) => peerState(m), { signal });
    // The host starts: both go to the grid.
    r.on("start").subscribe(
      () => {
        lobbyOnly.abort();
        const opponent = peerState()?.vehicle ?? mine();
        room = null; // the runtime owns it now
        ctx.fly(mine(), 1, (f) => new RaceRuntime(f, r, mine(), opponent));
      },
      { signal },
    );
    let ready = false;
    const sendLobby = () => r.send("lobby", { vehicle: mine(), ready });
    disposers.push(
      effect(() => {
        const peer = r.peer();
        const ps = peerState();
        const vehicle = mine();
        const err = r.error();
        if (peer) sendLobby();
        const go = h(
          "button",
          {
            class: "primary",
            disabled: role === "host" ? !(peer && ps?.ready) : !peer,
            onclick: () => {
              if (role === "host") {
                r.send("start", {});
                lobbyOnly.abort();
                const opponent = ps?.vehicle ?? vehicle;
                room = null;
                ctx.fly(vehicle, 0, (f) => new RaceRuntime(f, r, vehicle, opponent));
              } else {
                ready = !ready;
                sendLobby();
                go.textContent = ready ? "Not ready" : "Ready";
              }
            },
          },
          role === "host" ? "Start the race" : ready ? "Not ready" : "Ready",
        );
        render(
          body,
          h(
            "div",
            { class: "lobby" },
            h(
              "div",
              { class: "player p1" },
              h(
                "div",
                { class: "who" },
                `${role === "host" ? nameInput.value : (peer?.name ?? "…")}`,
              ),
              h(
                "div",
                { class: "muted" },
                role === "host"
                  ? content()?.vehicles[vehicle]?.name
                  : ps
                    ? content()?.vehicles[ps.vehicle]?.name
                    : "waiting",
              ),
            ),
            h(
              "div",
              { class: "player p2" },
              h(
                "div",
                { class: "who" },
                `${role === "guest" ? nameInput.value : (peer?.name ?? "Waiting for a player…")}`,
              ),
              h(
                "div",
                { class: "muted" },
                role === "guest"
                  ? content()?.vehicles[vehicle]?.name
                  : ps
                    ? `${content()?.vehicles[ps.vehicle]?.name}${ps.ready ? " · ready" : ""}`
                    : "",
              ),
            ),
          ),
          h(
            "div",
            { class: "actions", style: "margin-top: 12px" },
            h("span", null, "Room ", h("span", { class: "room-code" }, code)),
            h("span", { class: "spacer" }),
            ...(err ? [h("span", { style: "color: var(--rose)" }, err)] : []),
            go,
          ),
        );
      }),
    );
  };

  render(
    host,
    h("label", { class: "field" }, "Your name", nameInput),
    h(
      "div",
      { class: "actions" },
      h("button", { class: "primary", onclick: solo }, "Solo time trial"),
      h("button", { onclick: () => void enter("host", newRoomCode()) }, "Host a race"),
    ),
    h(
      "div",
      { class: "actions" },
      codeInput,
      h("button", { onclick: () => void enter("guest", cleanRoomCode(codeInput.value)) }, "Join"),
    ),
    body,
  );
  return () => {
    for (const d of disposers) d();
    void room?.leave();
  };
}

type Phase = "grid" | "countdown" | "racing" | "finished";

class RaceRuntime implements ModeRuntime {
  private view: CourseView;
  private course: Course;
  private progress: Progress;
  private phase: Phase = "grid";
  private startAt = 0;
  private lastPose: Pose | null = null;
  private ghost: Ghost | null = null;
  private peerNext = 0;
  private peerDone: number | null = null;
  private myTime: number | null = null;
  private lastSent = 0;
  private lastBeep = 99;
  private abort = new AbortController();
  private disposers: (() => void)[] = [];
  private placed = false;
  private iAmReady = false;
  private peerReady = false;

  constructor(
    private readonly flight: Flight,
    private readonly room: Room | null,
    vehicle: string,
    opponent: string | null,
  ) {
    this.course = currentCourse()!;
    this.view = new CourseView(flight.world, this.course);
    this.progress = new Progress(this.course.gates.length, this.course.laps);
    void vehicle;
    const signal = this.abort.signal;
    if (room) {
      const spec = content()!.vehicles[opponent ?? vehicle]!;
      this.ghost = new Ghost(flight.world, spec, "#22C7B8", room.peer()?.name ?? "Opponent");
      room.on<PeerPose>("pose").subscribe((p) => this.ghost?.push(p), { signal });
      room.on<{ next: number }>("gate").subscribe((m) => (this.peerNext = m.next), { signal });
      room.on<{ time: number }>("done").subscribe(
        (m) => {
          this.peerDone = m.time;
          if (this.myTime === null)
            toast(`${room.peer()?.name ?? "Opponent"} finished: ${fmtTime(m.time)}`, "warn");
          this.results();
        },
        { signal },
      );
      room
        .on<{ at: number }>("go")
        .subscribe((m) => this.countdown(m.at - room.clockOffset), { signal });
      room.on("grid").subscribe(() => (this.peerReady = true), { signal });
      room.on("again").subscribe(() => this.again(false), { signal });
    }
    this.disposers.push(
      effect(() => {
        // On the grid and ready: say so; the host starts when both are.
        if (status().state === "ready" && !this.iAmReady && this.phase === "grid") {
          this.iAmReady = true;
          if (!room) this.countdown(Date.now() + COUNTDOWN_S * 1000);
          else room.send("grid", {});
        }
      }),
      effect(() => {
        if (flight.pad() && !this.placed) {
          this.placed = true;
          void this.view.place().then(() => this.draw());
        }
      }),
    );
    hudBanner({ big: "", small: room ? "To the grid: arm when ready" : "Time trial: get ready" });
  }

  private countdown(at: number): void {
    this.phase = "countdown";
    this.startAt = at;
    this.lastBeep = 99;
  }

  tick(now: number, pose: Pose | null): void {
    void now;
    const t = (Date.now() - this.startAt) / 1000;
    // The host starts once both are on the grid.
    if (this.room?.role === "host" && this.phase === "grid" && this.iAmReady && this.peerReady) {
      const at = Date.now() + COUNTDOWN_S * 1000;
      this.room.send("go", { at: at + this.room.clockOffset });
      this.countdown(at);
    }
    if (this.phase === "countdown") {
      const left = Math.ceil(-t);
      if (left !== this.lastBeep && left <= 3 && left >= 0) {
        this.lastBeep = left;
        sounds.countdown(left);
      }
      if (t >= 0) {
        this.phase = "racing";
        hudBanner({ big: "GO", tone: "leaf" });
        setTimeout(() => this.phase === "racing" && hudBanner(null), 900);
      } else hudBanner({ big: String(left), small: this.course.name });
    }
    if (this.phase === "racing" && pose) {
      hudTimer(fmtTime(t));
      const g = this.view.gates[this.progress.gate];
      if (g && this.lastPose && passes(g, this.lastPose, pose)) {
        const lapDone = this.progress.pass(t);
        this.room?.send("gate", { next: this.progress.next, t });
        if (this.progress.done) this.finish(t);
        else if (lapDone) {
          sounds.lap();
          toast(`Lap ${this.progress.laps.length}: ${fmtTime(this.progress.laps.at(-1)!)}`, "good");
        } else sounds.gate();
        this.draw();
      }
    }
    this.lastPose = pose;
    if (pose && this.room && now - this.lastSent > 1000 / POSE_HZ) {
      this.lastSent = now;
      this.room.sendFast("pose", {
        lat: pose.lat,
        lon: pose.lon,
        h: pose.alt + mapOffset(),
        q: pose.q,
        v: pose.v,
        // The sender's clock when it was there: the frame it was sampled for.
        t: performance.timeOrigin + now,
      });
    }
    this.ghost?.draw();
    this.panel();
  }

  /** The course as the race stands: the next gates marked, this lap's passed ones done. */
  private draw(): void {
    this.view.draw(this.progress.upcoming(), this.progress.next % this.progress.n);
  }

  private finish(t: number): void {
    this.phase = "finished";
    this.myTime = t;
    sounds.finish();
    this.room?.send("done", { time: t, laps: this.progress.laps });
    const record = saveBest(this.course.id, t);
    const won = this.peerDone === null && this.room !== null;
    hudBanner({
      big: fmtTime(t),
      small: record ? "Your best on this course!" : won ? "You win!" : "Finished",
      tone: won || record ? "leaf" : "sun",
    });
    hudTimer(fmtTime(t));
    this.results();
  }

  private results(): void {
    if (this.myTime === null) return;
    const panel = h("div", { class: "scoreboard panel" });
    const rows: [string, string, string][] = [[prefs().player.name, fmtTime(this.myTime), "p1"]];
    if (this.room)
      rows.push([
        this.room.peer()?.name ?? "Opponent",
        this.peerDone === null ? "racing…" : fmtTime(this.peerDone),
        "p2",
      ]);
    const b = best(this.course.id);
    panel.append(
      h("div", { style: "font-weight:600; margin-bottom:6px" }, "Results"),
      ...rows.map(([n, t, c]) =>
        h("div", { class: "row" }, h("span", { class: c }, n), h("span", { class: "num" }, t)),
      ),
      ...(b !== null
        ? [
            h(
              "div",
              { class: "row muted" },
              h("span", null, "Best here"),
              h("span", { class: "num" }, fmtTime(b)),
            ),
          ]
        : []),
      h(
        "div",
        { class: "actions", style: "margin-top:10px" },
        h("button", { class: "primary", onclick: () => this.again(true) }, "Race again"),
      ),
    );
    hudPanel(panel);
  }

  /** Back to the grid for another go (`tell`: ask the other page too). */
  private again(tell: boolean): void {
    if (tell) this.room?.send("again", {});
    this.phase = "grid";
    this.progress = new Progress(this.course.gates.length, this.course.laps);
    this.myTime = null;
    this.peerDone = null;
    this.peerReady = false;
    this.iAmReady = false;
    hudPanel(null);
    hudTimer(null);
    hudBanner({ big: "", small: "Back to the grid" });
    this.draw();
    this.flight.reset();
  }

  private panel(): void {
    if (this.myTime !== null || this.phase !== "racing") {
      if (this.phase !== "finished" && this.phase !== "racing") return;
      if (this.myTime !== null) return;
    }
    const n = this.progress.n;
    const rows = [
      h(
        "div",
        { class: "row" },
        h("span", null, "Lap"),
        h("span", { class: "num" }, `${this.progress.lap}/${this.course.laps}`),
      ),
      h(
        "div",
        { class: "row" },
        h("span", null, "Gate"),
        h("span", { class: "num" }, `${this.progress.gate + 1}/${n}`),
      ),
    ];
    if (this.room)
      rows.push(
        h(
          "div",
          { class: "row p2" },
          h("span", null, this.room.peer()?.name ?? "Opponent"),
          h(
            "span",
            { class: "num" },
            `lap ${Math.min(this.course.laps, Math.floor(this.peerNext / n) + 1)} gate ${(this.peerNext % n) + 1}`,
          ),
        ),
      );
    hudPanel(h("div", { class: "scoreboard panel" }, ...rows));
  }

  marks(): ReturnType<ModeRuntime["marks"]> {
    const out: ReturnType<ModeRuntime["marks"]> = this.view.gates.map((g, i) => ({
      lat: g.lat,
      lon: g.lon,
      kind: i === this.progress.gate ? ("next-gate" as const) : ("gate" as const),
      heading: Math.atan2(g.fwd[0], g.fwd[1]),
      label: String(i + 1),
    }));
    const p = this.ghost?.poseNow();
    if (p) out.push({ lat: p.lat, lon: p.lon, kind: "peer", label: this.room?.peer()?.name ?? "" });
    return out;
  }

  dispose(): void {
    this.abort.abort();
    for (const d of this.disposers) d();
    this.view.dispose();
    this.ghost?.dispose();
    void this.room?.leave();
  }
}
