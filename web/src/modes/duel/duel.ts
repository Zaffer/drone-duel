// Duel (DESIGN.md §7): a match of turns between two players, each on their own sim. In each turn
// one is the pilot, flying the quad or the Chupito (their pick) to strike the truck; the other is
// the driver, keeping the truck away on the map itself. A strike scores and spends the aircraft;
// so does a crash. The pilot has `lives` aircraft a turn, each back at the start. The turn ends
// when its time is up or the pilot has none left; then they swap. The host flies first.
//
// The host's page runs the match and sends it; the pilot's page judges strikes and crashes on
// what its pilot sees, and tells the other. The pages meet in a room over Trystero (peer.ts).
//
// Messages: match Match (host → guest)  aircraft {vehicle}  pose PeerPose+{kind}
// hit {lat, lon, h}  lost {}  out {}

import { effect, signal } from "alien-signals";
import { sounds } from "../../audio/sounds.ts";
import type { Flight, ModeRuntime } from "../../flight.ts";
import { crash, puff } from "../../fx/smoke.ts";
import { cleanRoomCode, newRoomCode, type Role, Room } from "../../net/peer.ts";
import type { Pose } from "../../net/server.ts";
import { content, toast, vehicleId } from "../../state/app.ts";
import { hudBanner, hudPanel, hudTimer } from "../../state/hud.ts";
import { prefs, updatePrefs } from "../../state/prefs.ts";
import { phase as physicsPhase, status } from "../../state/session.ts";
import { vehicleCard } from "../../ui/app.ts";
import { h, render } from "../../ui/h.ts";
import { mapOffset } from "../../world/geo.ts";
import { Ghost, type PeerPose } from "../ghost.ts";
import { fmtTime } from "../race/gates.ts";
import type { ModeContext, ModeDef } from "../types.ts";
import { type Match, pilotOf, strikes, winner } from "./judge.ts";

const TYPES = ["start", "match", "aircraft", "pose", "hit", "lost", "out"];
const POSE_HZ = 20;
const RESPAWN_MS = 2500;
const SWAP_S = 6;
const TRUCK = "truck";
/** The truck starts this far ahead of the pad. */
export const TRUCK_AHEAD_M = 70;

export const duel: ModeDef = {
  id: "duel",
  name: "Duel",
  desc: "Strike the truck from the air, or drive it out of reach. Then swap.",
  vehicles: (all) => all,
  lobby(host, ctx) {
    return duelLobby(host, ctx);
  },
};

function duelLobby(host: HTMLElement, ctx: ModeContext): () => void {
  const aircraft = ctx.vehicles.filter((v) => v.kind !== "ground");
  const mine = signal(aircraft.find((v) => v.id === vehicleId())?.id ?? aircraft[0]?.id ?? "quad5");
  const rules = { rounds: 2, minutes: 2, lives: 3 };
  let room: Room | null = null;
  const peerAircraft = signal<string | null>(null);
  const disposers: (() => void)[] = [];
  const nameInput = h("input", { value: prefs().player.name }) as HTMLInputElement;
  nameInput.addEventListener("change", () =>
    updatePrefs((p) => ({ ...p, player: { name: nameInput.value.trim() || "Pilot" } })),
  );
  const codeInput = h("input", {
    placeholder: "room code",
    style: "width: 130px",
  }) as HTMLInputElement;
  const cards = aircraft.map((v) => {
    const c = vehicleCard(v, () => {
      mine(v.id);
      room?.send("aircraft", { vehicle: v.id });
    });
    return c;
  });
  disposers.push(
    effect(() =>
      cards.forEach((c, i) => c.classList.toggle("selected", aircraft[i]!.id === mine())),
    ),
  );
  const body = h("div");

  const go = (r: Room, m: Match) => {
    const role = r.role;
    const pilot = pilotOf(m.turn) === role;
    room = null;
    ctx.fly(
      pilot ? mine() : TRUCK,
      0,
      (f) => new DuelRuntime(f, r, mine(), peerAircraft() ?? mine(), m),
    );
  };

  const enter = async (role: Role, code: string) => {
    const r = await Room.join("duel", code, role, nameInput.value.trim() || "Pilot", TYPES);
    room = r;
    const lobbyOnly = new AbortController();
    disposers.push(() => lobbyOnly.abort());
    const signal = lobbyOnly.signal;
    r.on<{ vehicle: string }>("aircraft").subscribe((m) => peerAircraft(m.vehicle), { signal });
    r.on<Match>("start").subscribe(
      (m) => {
        lobbyOnly.abort();
        go(r, { ...m, endsAt: m.endsAt - r.clockOffset });
      },
      { signal },
    );
    const num = (key: keyof typeof rules, min: number, max: number) => {
      const i = h("input", {
        type: "number",
        min,
        max,
        value: String(rules[key]),
        style: "width: 64px",
        disabled: role !== "host",
      }) as HTMLInputElement;
      i.addEventListener(
        "change",
        () => (rules[key] = Math.max(min, Math.min(max, Number(i.value) || min))),
      );
      return i;
    };
    const rulesRow = h(
      "div",
      { class: "actions" },
      h("label", { class: "field" }, "Rounds", num("rounds", 1, 5)),
      h("label", { class: "field" }, "Minutes a turn", num("minutes", 1, 10)),
      h("label", { class: "field" }, "Aircraft a turn", num("lives", 1, 9)),
    );
    disposers.push(
      effect(() => {
        const peer = r.peer();
        const err = r.error();
        if (peer) r.send("aircraft", { vehicle: mine() });
        const start = h(
          "button",
          {
            class: "primary",
            disabled: role !== "host" || !peer,
            onclick: () => {
              const m: Match = {
                phase: "live",
                turn: 0,
                turns: rules.rounds * 2,
                endsAt: Date.now() + rules.minutes * 60_000 + 30_000,
                scores: { host: 0, guest: 0 },
                lives: rules.lives,
                minutes: rules.minutes,
              };
              r.send("start", { ...m, endsAt: m.endsAt + r.clockOffset });
              lobbyOnly.abort();
              go(r, m);
            },
          },
          role === "host" ? "Start the duel" : "The host starts",
        );
        const pa = peerAircraft();
        render(
          body,
          h(
            "div",
            { class: "lobby" },
            h(
              "div",
              { class: "player p1" },
              h("div", { class: "who" }, role === "host" ? nameInput.value : (peer?.name ?? "…")),
              h("div", { class: "muted" }, "Host: flies first"),
            ),
            h(
              "div",
              { class: "player p2" },
              h(
                "div",
                { class: "who" },
                role === "guest" ? nameInput.value : (peer?.name ?? "Waiting for a player…"),
              ),
              h(
                "div",
                { class: "muted" },
                peer
                  ? `drives first${pa ? ` · flies the ${content()?.vehicles[pa]?.name}` : ""}`
                  : "",
              ),
            ),
          ),
          rulesRow,
          h(
            "div",
            { class: "actions", style: "margin-top: 12px" },
            h("span", null, "Room ", h("span", { class: "room-code" }, code)),
            h("span", { class: "spacer" }),
            ...(err ? [h("span", { style: "color: var(--rose)" }, err)] : []),
            start,
          ),
        );
      }),
    );
  };

  render(
    host,
    h(
      "h2",
      null,
      `${ctx.location.name.split(": ").at(-1)}: your aircraft for your turns as the pilot`,
    ),
    h("div", { class: "cards" }, cards),
    h(
      "div",
      { class: "actions" },
      h("label", { class: "field" }, "Your name", nameInput),
      h("span", { class: "spacer" }),
      h(
        "button",
        { class: "primary", onclick: () => void enter("host", newRoomCode()) },
        "Host a duel",
      ),
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

class DuelRuntime implements ModeRuntime {
  private match: Match;
  private ghost: Ghost | null = null;
  private ghostVehicle = "";
  private lastSent = 0;
  private lastMatchSent = 0;
  private livesLeft: number;
  private respawning = false;
  private abort = new AbortController();
  private disposers: (() => void)[] = [];
  private lastTurn = -1;

  constructor(
    private readonly flight: Flight,
    private readonly room: Room,
    private readonly myAircraft: string,
    private readonly peerAircraft: string,
    match: Match,
  ) {
    this.match = match;
    this.livesLeft = match.lives;
    flight.truckAhead = TRUCK_AHEAD_M;
    const signal = this.abort.signal;
    room
      .on<Match>("match")
      .subscribe((m) => this.apply({ ...m, endsAt: m.endsAt - room.clockOffset }), { signal });
    room.on<PeerPose & { kind: string }>("pose").subscribe((p) => this.peerPose(p), { signal });
    room
      .on<{ lat: number; lon: number; h: number }>("hit")
      .subscribe((m) => this.wasHit(m), { signal });
    room
      .on("lost")
      .subscribe(() => toast(`${room.peer()?.name ?? "The pilot"} crashed`, "good"), { signal });
    room.on("out").subscribe(() => this.isHost && this.endTurn(), { signal });
    this.disposers.push(
      effect(() => {
        if (physicsPhase() === "crashed" && this.amPilot && this.match.phase === "live")
          this.spent("crash");
      }),
    );
    this.apply(match);
  }

  private get isHost(): boolean {
    return this.room.role === "host";
  }

  private get amPilot(): boolean {
    return pilotOf(this.match.turn) === this.room.role;
  }

  /** A new state of the match (from the host, or the host's own). */
  private apply(m: Match): void {
    const newTurn = m.turn !== this.lastTurn;
    this.match = m;
    if (newTurn && m.phase === "live") {
      this.lastTurn = m.turn;
      this.livesLeft = m.lives;
      const want = this.amPilot ? this.myAircraft : TRUCK;
      if (vehicleId() !== want) this.flight.switchVehicle(want);
      else this.flight.reset();
      const other = this.room.peer()?.name ?? "the other player";
      hudBanner({
        big: this.amPilot ? "PILOT" : "DRIVER",
        small: this.amPilot ? `Strike ${other}'s truck` : `Keep away from ${other}`,
        tone: "sun",
      });
      setTimeout(() => hudBanner(null), 3000);
      this.ghostFor(this.amPilot ? TRUCK : this.peerAircraft);
    }
    if (m.phase === "swap") hudBanner({ big: "SWAP", small: "Next turn…", tone: "sun" });
    if (m.phase === "over") {
      const w = winner(m);
      const me = this.room.role;
      hudBanner({
        big: w === null ? "DRAW" : w === me ? "YOU WIN" : "YOU LOSE",
        small: `${m.scores[me]} to ${m.scores[me === "host" ? "guest" : "host"]}`,
        tone: w === me ? "leaf" : w === null ? "sun" : "rose",
      });
    }
  }

  private ghostFor(vehicle: string): void {
    if (this.ghostVehicle === vehicle) return;
    this.ghost?.dispose();
    this.ghostVehicle = vehicle;
    const spec = content()!.vehicles[vehicle]!;
    this.ghost = new Ghost(
      this.flight.world,
      spec,
      "#22C7B8",
      this.room.peer()?.name ?? "Opponent",
      () => this.room.clockOffset,
    );
  }

  private peerPose(p: PeerPose & { kind: string }): void {
    if ((p.kind === "truck") !== (this.ghostVehicle === TRUCK)) return; // from the last turn
    this.ghost?.push(p);
  }

  /** The aircraft is spent (a strike or a crash): it stops dead, and the next starts soon. */
  private spent(why: "strike" | "crash"): void {
    if (this.respawning) return;
    this.respawning = true;
    this.livesLeft--;
    if (why === "crash") {
      this.room.send("lost", {});
      const p = this.flight.pose();
      if (p) crash(this.flight.world, p.lat, p.lon, p.alt);
    }
    this.flight.destroy();
    setTimeout(() => {
      this.respawning = false;
      if (this.match.phase !== "live" || !this.amPilot) return;
      if (this.livesLeft > 0) this.flight.reset();
      else {
        toast("No aircraft left this turn", "warn");
        if (this.isHost) this.endTurn();
        else this.room.send("out", {});
      }
    }, RESPAWN_MS);
  }

  private wasHit(m: { lat: number; lon: number; h: number }): void {
    puff(this.flight.world, m.lat, m.lon, m.h - mapOffset());
    sounds.strike();
    toast("Hit!", "bad");
    if (this.isHost) {
      this.match.scores.guest++;
      this.sendMatch();
    }
  }

  private endTurn(): void {
    const m = this.match;
    if (m.phase !== "live") return;
    if (m.turn + 1 >= m.turns) {
      this.apply({ ...m, phase: "over" });
    } else {
      this.apply({ ...m, phase: "swap", endsAt: Date.now() + SWAP_S * 1000 });
    }
    this.sendMatch();
  }

  private sendMatch(): void {
    this.lastMatchSent = performance.now();
    this.room.send("match", { ...this.match, endsAt: this.match.endsAt + this.room.clockOffset });
  }

  tick(now: number, pose: Pose | null): void {
    const m = this.match;
    const left = (m.endsAt - Date.now()) / 1000;
    hudTimer(m.phase === "over" ? null : fmtTime(Math.max(0, left)));
    if (this.isHost) {
      if (m.phase === "live" && left <= 0) this.endTurn();
      else if (m.phase === "swap" && left <= 0) {
        this.apply({
          ...m,
          phase: "live",
          turn: m.turn + 1,
          endsAt: Date.now() + m.minutes * 60_000 + 30_000,
        });
        this.sendMatch();
      } else if (now - this.lastMatchSent > 1000) this.sendMatch();
    }
    const ready = status().state === "ready";
    if (pose && ready && now - this.lastSent > 1000 / POSE_HZ) {
      this.lastSent = now;
      this.room.send("pose", {
        kind: this.amPilot ? "air" : "truck",
        lat: pose.lat,
        lon: pose.lon,
        h: pose.alt + mapOffset(),
        q: pose.q,
        v: pose.v,
        t: Date.now(),
      });
    }
    const other = this.ghost?.draw() ?? null;
    // The pilot's page judges: a strike scores and spends the aircraft.
    if (this.amPilot && m.phase === "live" && pose && other && ready && !this.respawning) {
      const me = { lat: pose.lat, lon: pose.lon, h: pose.alt + mapOffset() };
      if (strikes(other, me)) {
        this.room.send("hit", me);
        puff(this.flight.world, pose.lat, pose.lon, pose.alt);
        sounds.strike();
        toast("Strike!", "good");
        if (this.isHost) {
          m.scores.host++;
          this.sendMatch();
        }
        this.spent("strike");
      }
    }
    this.panel();
  }

  private panel(): void {
    const m = this.match;
    const me = this.room.role;
    const them = me === "host" ? "guest" : "host";
    hudPanel(
      h(
        "div",
        { class: "scoreboard panel" },
        h(
          "div",
          { style: "font-weight:600; margin-bottom:6px" },
          `Turn ${Math.min(m.turn + 1, m.turns)}/${m.turns} · ${this.amPilot ? "you fly" : "you drive"}`,
        ),
        h(
          "div",
          { class: "row p1" },
          h("span", null, prefs().player.name),
          h("span", { class: "num" }, String(m.scores[me])),
        ),
        h(
          "div",
          { class: "row p2" },
          h("span", null, this.room.peer()?.name ?? "Opponent"),
          h("span", { class: "num" }, String(m.scores[them])),
        ),
        ...(this.amPilot && m.phase === "live"
          ? [
              h(
                "div",
                { class: "row muted" },
                h("span", null, "Aircraft left"),
                h("span", { class: "num" }, String(this.livesLeft)),
              ),
            ]
          : []),
      ),
    );
  }

  marks(): ReturnType<ModeRuntime["marks"]> {
    const p = this.ghost?.poseNow();
    return p
      ? [
          {
            lat: p.lat,
            lon: p.lon,
            kind: this.ghostVehicle === TRUCK ? "truck" : "peer",
            label: this.room.peer()?.name ?? "",
          },
        ]
      : [];
  }

  dispose(): void {
    this.abort.abort();
    for (const d of this.disposers) d();
    this.ghost?.dispose();
    this.flight.truckAhead = 0;
    void this.room.leave();
  }
}
