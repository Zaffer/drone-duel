// The duel's match (duel.js), in the COP: the room, the turns and the score.
//
// A match is up to MAX_ROUNDS rounds. In each round both players have a turn of the round's
// minutes as the pilot, the host first: they fly the mission with their own sim's drones,
// swarm or one at a time, and score a point for each drone that strikes the other player's
// truck (duelpilot.js). The other drives the truck in their COP (dueldriver.js). A turn ends
// when its time is up or the pilot has no drone left; then the players swap. The match ends
// after the last round, or sooner once the trailing player can no longer catch up.
//
// The host's page runs the match and sends it to the guest's; the guest asks it to start a
// turn of theirs (intent). Each turn starts with the pilot's drones on the ground at home: the
// sim restarts every copter when its pilot's turn ends, and when a match starts.

import { clock, cleanRoomCode, DuelLink, newRoomCode } from "./duel.js";
import { DriverRole } from "./dueldriver.js";
import { PilotRole, simPost } from "./duelpilot.js";
import { sfx } from "./sounds.js";

const $ = (id) => document.getElementById(id);

const MAX_ROUNDS = 5;
const DEFAULT_RULES = { rounds: MAX_ROUNDS, minutes: 2 };
/** The mission's five drones: the most one turn can score. Used to end a match once decided. */
const DRONES_PER_TURN = 5;
/** While a turn is on, the guest hears the match this often. */
const MATCH_SEND_MS = 1000;
/** After a sim reset, the drones' reports are stale this long before they go quiet. */
const RESET_SETTLE_MS = 5000;
/** A turn's last seconds tick. */
const COUNTDOWN_S = 10;
const RULES_KEY = "drone-duel-match-rules";
const NAME_KEY = "drone-duel-name";
const savedMatchKey = (room) => `drone-duel-match-${room}`;

/** Who flies turn `turn`: the host first, then they swap. */
const pilotOf = (turn) => (turn % 2 === 0 ? "host" : "guest");
const other = (side) => (side === "host" ? "guest" : "host");

/** Each side's total over finished turns. */
function totals(turns) {
  const sum = { host: 0, guest: 0 };
  for (const t of turns) sum[t.pilot] += t.hits;
  return sum;
}

/** The trailing side can no longer catch up, even with every drone striking in its turns left. */
function decided(turns, rounds) {
  const sum = totals(turns);
  const left = { host: 0, guest: 0 };
  for (let t = turns.length; t < 2 * rounds; t++) left[pilotOf(t)]++;
  const best = (side) => sum[side] + DRONES_PER_TURN * left[side];
  return best("host") < sum.guest || best("guest") < sum.host;
}

function lobby(rules, names = { host: "", guest: "" }) {
  return {
    phase: "lobby", // lobby | ready | live | over
    rounds: rules.rounds,
    minutes: rules.minutes,
    turn: 0,
    /** Finished turns: {pilot: "host" | "guest", hits}. */
    turns: [],
    /** The turn on's strikes. */
    hits: 0,
    leftS: 0,
    names,
    winner: null, // host | guest | draw
    /** Why the last turn ended, or why the match did. */
    note: "",
  };
}

export class DuelMatch {
  /**
   * @param {object} ctx fleet, map, config (the page's), toast(text, kind), setDriving(on)
   */
  constructor({ fleet, map, config, toast, setDriving }) {
    this.fleet = fleet;
    this.map = map;
    this.config = config;
    this.toast = toast;
    this.setDriving = setDriving;
    /** @type {DuelLink | null} */
    this.link = null;
    this.rules = loadRules();
    this.state = lobby(this.rules);
    /** The pilot's latest field, for the driver. */
    this.field = null;
    /** @type {PilotRole | DriverRole | null} */
    this.role = null;
    this.roleName = null;
    /** Host: the drones that have scored this turn, and when the turn ends (performance.now()). */
    this._counted = new Set();
    this._endsAt = 0;
    this._sentAt = 0;
    /** Lost drones seen with their link down since: back once it is up again. */
    this._down = new Set();
    this._resetAt = -Infinity;
    /** The turn's whole seconds left at the last tick: never ticks twice for one. */
    this._tickLeft = Infinity;
    this._wirePanel();
    setInterval(() => this._tick(), 250);
    const params = new URLSearchParams(location.search);
    if (params.get("room")) this.open(params.get("room"), params.get("as") === "host" ? "host" : "guest");
  }

  /** This page's side, "host" or "guest"; null out of a room. */
  get side() {
    return this.link?.role ?? null;
  }

  get panelShown() {
    return !$("duel").hidden;
  }

  togglePanel() {
    $("duel").hidden = !$("duel").hidden;
    if (!$("duel").hidden && !this.link) $("duel-code").focus();
    this._render();
  }

  /** Open room `code` as the host, who flies first, or join it as the guest. */
  async open(code, side) {
    this.close();
    code = cleanRoomCode(code);
    if (!code) return;
    const name = ($("duel-name").value.trim() || "Player").slice(0, 24);
    saveLocal(NAME_KEY, name);
    let link;
    try {
      link = await DuelLink.join(code, side, name);
    } catch (err) {
      this.toast(`Duel: Trystero did not load (${err.message}); run scripts/setup.sh`, "bad");
      return;
    }
    this.link = link;
    link.onError = (text) => this.toast(`Duel: ${text}`, "bad");
    link.onPeer = (connected) => this._peer(connected);
    link.on("match", (s) => this._received(s));
    link.on("intent", ({ action } = {}) => this._intent(action));
    link.on("field", (f) => {
      this.field = f;
      if (this.role instanceof DriverRole) this.role.onField(f);
    });
    const names = { host: side === "host" ? name : "", guest: side === "guest" ? name : "" };
    this.state = lobby(this.rules, names);
    if (side === "host") this._restore(code);
    this._setUrl(code, side);
    $("duel").hidden = false;
    this._apply(lobby(this.rules));
  }

  close() {
    if (!this.link) return;
    this.link.leave();
    this.link = null;
    const prev = this.state;
    this.state = lobby(this.rules);
    this._apply(prev);
    this._setUrl(null);
  }

  /** The opponent's drones to hear while driving (DriverRole.audible); none otherwise. */
  audibleDrones() {
    return this.role instanceof DriverRole ? this.role.audible() : [];
  }

  /** The driver's truck to hear while flying (PilotRole.audibleTruck); null otherwise. */
  audibleTruck() {
    return this.role instanceof PilotRole ? this.role.audibleTruck() : null;
  }

  /** Each telemetry frame. */
  frame() {
    this._revive();
    if (this.role instanceof PilotRole) this.role.frame();
  }

  // -- the host's match ------------------------------------------------------------

  startMatch() {
    if (this.side !== "host" || !this.link.connected) return;
    this._counted.clear();
    this._set({ ...lobby(this.rules, this.state.names), phase: "ready" });
    this.toast(`Match on: ${this.state.rounds} round${this.state.rounds === 1 ? "" : "s"}, ${this._name("host")} flies first`, "leader");
  }

  startTurn() {
    const s = this.state;
    if (this.side !== "host" || s.phase !== "ready" || !this.link.connected) return;
    this._counted.clear();
    this._endsAt = performance.now() + s.minutes * 60_000;
    this._set({ ...s, phase: "live", hits: 0, note: "" });
  }

  /** A drone struck the truck in a turn: the host scores it, once. */
  hitBy(sysid) {
    const s = this.state;
    if (this.side !== "host" || s.phase !== "live" || this._counted.has(sysid)) return;
    this._counted.add(sysid);
    this._set({ ...s, hits: s.hits + 1 });
  }

  /** The pilot has no drone left: the turn is over. */
  pilotOut() {
    if (this.side === "guest") this.link.send("intent", { action: "out" });
    else if (this.state.phase === "live") this._endTurn("every drone out");
  }

  _endTurn(why) {
    const s = this.state;
    const turns = [...s.turns, { pilot: pilotOf(s.turn), hits: s.hits }];
    const turn = s.turn + 1;
    const last = turn >= 2 * s.rounds;
    const early = !last && decided(turns, s.rounds);
    if (!last && !early) {
      this._set({ ...s, turns, turn, hits: 0, phase: "ready", note: why });
      return;
    }
    const sum = totals(turns);
    const winner = sum.host > sum.guest ? "host" : sum.guest > sum.host ? "guest" : "draw";
    const note = early ? `decided after round ${Math.ceil(turn / 2)} of ${s.rounds}` : why;
    this._set({ ...s, turns, turn, hits: 0, phase: "over", winner, note });
  }

  /** Back to the start of the turn on: its strikes do not count. */
  _callOff(why) {
    if (this.state.phase !== "live") return;
    this._set({ ...this.state, phase: "ready", hits: 0, note: why });
  }

  _intent(action) {
    const s = this.state;
    if (this.side !== "host" || pilotOf(s.turn) !== "guest") return;
    if (action === "start-turn") this.startTurn();
    else if (action === "out" && s.phase === "live") this._endTurn("every drone out");
  }

  _peer(connected) {
    const peer = this.link.peerName ?? "Your opponent";
    this.toast(connected ? `${peer} is in the room` : "Your opponent left the room", connected ? "" : "bad");
    sfx.play(connected ? "join" : "leave");
    if (this.side === "host") {
      if (connected) this._set({ ...this.state, names: { ...this.state.names, guest: peer } });
      else this._callOff("called off: the guest left");
    }
    if (connected && this.role instanceof PilotRole) this.role.sendField();
    this._render();
  }

  /** Set the match (the host), send it on, and act on the change. */
  _set(next) {
    const prev = this.state;
    this.state = next;
    if (this.side === "host") {
      this._send();
      saveSession(savedMatchKey(this.link.code), { state: next });
    }
    this._apply(prev);
  }

  _send() {
    const s = this.state;
    const leftS = s.phase === "live" ? Math.max(0, (this._endsAt - performance.now()) / 1000) : 0;
    this._sentAt = performance.now();
    this.link?.send("match", { ...s, leftS });
  }

  /** The guest: the host's match. */
  _received(s) {
    if (this.side !== "guest" || !s || typeof s !== "object" || !Array.isArray(s.turns)) return;
    this._endsAt = performance.now() + (Number(s.leftS) || 0) * 1000;
    const prev = this.state;
    this.state = s;
    this._apply(prev);
  }

  /** The host, back after a reload: the match as it was, a turn that was on to start again. */
  _restore(room) {
    const saved = loadSession(savedMatchKey(room))?.state;
    if (!saved || !Array.isArray(saved.turns)) return;
    this.state = { ...saved, phase: saved.phase === "live" ? "ready" : saved.phase, hits: 0 };
    this.state.names = { ...this.state.names, host: this.link.name };
  }

  _tick() {
    const s = this.state;
    if (this.side === "host" && s.phase === "live") {
      if (performance.now() >= this._endsAt) this._endTurn("time up");
      else if (performance.now() - this._sentAt >= MATCH_SEND_MS) this._send();
    }
    this._countdown();
    this._render();
  }

  /** A tick for each of a live turn's last COUNTDOWN_S seconds. The guest's clock jitters with
   * the host's updates, so a second ticks once. */
  _countdown() {
    if (this.state.phase !== "live") {
      this._tickLeft = Infinity;
      return;
    }
    const left = Math.ceil((this._endsAt - performance.now()) / 1000);
    if (left < this._tickLeft && left > 0 && left <= COUNTDOWN_S) sfx.play("tick", { urgent: left <= 3 });
    this._tickLeft = Math.min(this._tickLeft, left);
  }

  // -- both pages: roles, resets, news ----------------------------------------------

  _apply(prev) {
    const s = this.state;
    const me = this.side;
    const playing = me && (s.phase === "ready" || s.phase === "live");
    const wanted = playing ? (pilotOf(s.turn) === me ? "pilot" : "driver") : null;
    const wasPilot = me && prev.phase === "live" && pilotOf(prev.turn) === me;
    // A match starting, or my turn as pilot over: every drone back home for the next.
    if ((wasPilot && s.phase !== "live") || (s.phase === "ready" && (prev.phase === "lobby" || prev.phase === "over"))) {
      this._resetSim();
    }
    if (wanted !== this.roleName) {
      this.role?.destroy();
      this.role = null;
      this.roleName = wanted;
      const ctx = { fleet: this.fleet, map: this.map, config: this.config, link: this.link, match: this, toast: this.toast };
      if (wanted === "pilot") {
        if (!this.config.sim) this.fleet.lost.clear();
        this.role = new PilotRole(ctx);
      } else if (wanted === "driver") {
        this.role = new DriverRole({ ...ctx, banner: this.toast, setDriving: this.setDriving });
      }
    }
    if (s.phase !== prev.phase) {
      if (this.role instanceof DriverRole) this.role.phaseChanged(prev.phase, s.phase);
      if (this.role instanceof PilotRole && s.phase === "live") this.role.turnStarted();
      this._announce(prev);
    }
    if (s.phase === "over" && prev.phase !== "over") this._showFinal();
    this._render();
  }

  _announce(prev) {
    const s = this.state;
    if (prev.phase === "live" && s.turns.length > prev.turns.length) {
      const t = s.turns.at(-1);
      this.toast(`${this._name(t.pilot)} struck the truck ${t.hits} time${t.hits === 1 ? "" : "s"} (${s.note})`, "leader");
      if (s.phase !== "over") sfx.play("turnEnd"); // over: the match's own sound
    } else if (prev.phase === "live" && s.phase === "ready") {
      this.toast(`Turn ${s.note || "called off"}: it starts again`, "bad");
      sfx.play("turnEnd");
    }
    if (s.phase === "live") {
      sfx.play("turnStart");
      const mine = pilotOf(s.turn) === this.side;
      this.toast(mine ? `Your turn: strike the truck, ${clock(s.minutes * 60)} on the clock` : `Drive! ${this._name(pilotOf(s.turn))}'s drones are coming`, "leader");
    }
  }

  _resetSim() {
    this._resetAt = performance.now();
    if (this.config.sim) simPost("/sim/reset");
  }

  /** A lost drone whose sim copter has restarted (its link down, then up) is back. */
  _revive() {
    const fleet = this.fleet;
    for (const sysid of [...fleet.lost]) {
      const view = fleet.vehicles.get(sysid);
      if (!view?.data) continue;
      if (!view.data.link) this._down.add(sysid);
      else if (this._down.has(sysid)) {
        fleet.lost.delete(sysid);
        this._down.delete(sysid);
        view.trail.length = 0;
      }
    }
  }

  _name(side) {
    return this.state.names?.[side] || (side === "host" ? "Host" : "Guest");
  }

  _setUrl(code, side) {
    const url = new URL(location.href);
    if (code) {
      url.searchParams.set("room", code);
      url.searchParams.set("as", side);
    } else {
      url.searchParams.delete("room");
      url.searchParams.delete("as");
    }
    history.replaceState(null, "", url);
  }

  // -- panel ----------------------------------------------------------------------

  _wirePanel() {
    $("duel-code").value = newRoomCode();
    $("duel-name").value = loadLocal(NAME_KEY) ?? "";
    $("duel-join").addEventListener("submit", (e) => {
      e.preventDefault();
      this.open($("duel-code").value, e.submitter?.dataset.as === "host" ? "host" : "guest");
    });
    $("duel-new-code").addEventListener("click", () => {
      $("duel-code").value = newRoomCode();
    });
    $("duel-copy").addEventListener("click", async () => {
      try {
        await navigator.clipboard.writeText(this.link?.code ?? "");
        this.toast("Room code copied");
      } catch {
        this.toast("Copy blocked: read the code out", "bad");
      }
    });
    $("duel-action").addEventListener("click", () => this._action());
    $("duel-leave").addEventListener("click", () => this.close());
    $("duel-final-close").addEventListener("click", () => ($("duel-final").hidden = true));
    for (const [id, key, max] of [
      ["duel-rounds", "rounds", MAX_ROUNDS],
      ["duel-minutes", "minutes", 10],
    ]) {
      const input = $(id);
      input.value = this.rules[key];
      input.addEventListener("change", () => {
        const n = Math.round(Number(input.value));
        this.rules[key] = Number.isFinite(n) ? Math.min(Math.max(n, 1), max) : DEFAULT_RULES[key];
        input.value = this.rules[key];
        saveLocal(RULES_KEY, JSON.stringify(this.rules));
        if (this.side === "host" && (this.state.phase === "lobby" || this.state.phase === "over")) {
          this._set({ ...this.state, [key]: this.rules[key] });
        }
      });
    }
    this._render();
  }

  /** What the main button does now, and whether it can: {label, enabled, run, why}. */
  _button() {
    const s = this.state;
    const host = this.side === "host";
    const connected = this.link?.connected;
    if (s.phase === "lobby" || s.phase === "over") {
      const label = s.phase === "lobby" ? "Start match" : "Rematch";
      if (!host) return { label: `Waiting for ${this._name("host")} to start`, enabled: false };
      return { label, enabled: connected, run: () => this.startMatch(), why: connected ? "" : "Waiting for an opponent to join" };
    }
    if (s.phase === "live") {
      if (!host) return { label: "Turn on", enabled: false };
      return { label: "Call off turn", enabled: true, run: () => this._callOff("called off by the host") };
    }
    if (pilotOf(s.turn) !== this.side) return { label: `Waiting for ${this._name(pilotOf(s.turn))} to start`, enabled: false };
    const pilot = this.role instanceof PilotRole ? this.role : null;
    const r = pilot?.readiness() ?? { up: 0, total: 0, flying: 0 };
    let why = "";
    if (!connected) why = "Waiting for your opponent";
    else if (!pilot?.truckLive) why = "Waiting for the truck on the map";
    else if (performance.now() - this._resetAt < RESET_SETTLE_MS || r.up === 0) {
      why = "Your drones are starting up: each needs its GPS position (about 30 s)";
    }
    else if (r.flying) why = "Land your drones: a turn starts with them on the ground";
    const run = host ? () => this.startTurn() : () => this.link.send("intent", { action: "start-turn" });
    return { label: `Start turn · ${r.up}/${r.total} drones ready`, enabled: !why, run, why };
  }

  _action() {
    const b = this._button();
    if (b.enabled) b.run();
  }

  _render() {
    const link = this.link;
    $("duel-btn").classList.toggle("on", this.panelShown || link !== null);
    $("duel-join").hidden = link !== null;
    $("duel-room").hidden = link === null;
    if (!link) return;
    const s = this.state;
    $("duel-code-out").textContent = link.code;
    const peer = $("duel-peer");
    peer.className = `duel-peer ${link.connected ? "live" : "waiting"}`;
    const rtt = link.rttMs === null ? "…" : `${Math.round(link.rttMs)} ms`;
    peer.querySelector("span").textContent = link.connected
      ? `${link.peerName} is in · ${rtt}`
      : this.side === "host"
        ? "Waiting for an opponent: give them the code"
        : "Waiting for the host";
    const me = this.side;
    const pilot = pilotOf(s.turn);
    let status;
    if (s.phase === "lobby") status = `LOBBY · ${this._name("host")} flies first`;
    else if (s.phase === "over") {
      const sum = totals(s.turns);
      const score = `${Math.max(sum.host, sum.guest)}–${Math.min(sum.host, sum.guest)}`;
      status = s.winner === "draw" ? `DRAW · ${score}` : `${this._name(s.winner).toUpperCase()} WINS · ${score}`;
    } else {
      const you = pilot === me ? "YOU FLY" : "YOU DRIVE";
      status = `ROUND ${Math.floor(s.turn / 2) + 1}/${s.rounds} · ${you}`;
    }
    $("duel-status").textContent = status;
    $("duel-status").className = `duel-status ${s.phase}`;
    const live = s.phase === "live";
    $("duel-clock").hidden = !(live || s.phase === "ready");
    $("duel-time").textContent = clock(live ? (this._endsAt - performance.now()) / 1000 : s.minutes * 60);
    $("duel-strikes").textContent = live ? s.hits : "–";
    $("duel-flier").textContent = this._name(pilot);
    this._table($("duel-table"));
    const b = this._button();
    $("duel-action").textContent = b.label;
    $("duel-action").disabled = !b.enabled;
    $("duel-note").textContent = b.why || (s.note && s.phase !== "lobby" ? `Last turn: ${s.note}` : "");
    const editable = me === "host" && (s.phase === "lobby" || s.phase === "over");
    for (const [id, key] of [
      ["duel-rounds", "rounds"],
      ["duel-minutes", "minutes"],
    ]) {
      $(id).disabled = !editable;
      if (!editable && document.activeElement !== $(id)) $(id).value = s[key];
    }
  }

  /** The tally: a row per player, a column per round, and the totals. */
  _table(table) {
    const s = this.state;
    const sum = totals(s.turns);
    const head = ["", ...Array.from({ length: s.rounds }, (_, i) => i + 1), "TOTAL"];
    const row = (side) => {
      const cells = [];
      for (let r = 0; r < s.rounds; r++) {
        const turn = 2 * r + (side === "host" ? 0 : 1);
        const done = s.turns[turn];
        const now = turn === s.turn && (s.phase === "ready" || s.phase === "live");
        const text = done ? done.hits : now && s.phase === "live" ? s.hits : now ? "▸" : "·";
        cells.push(`<td class="${now ? "now" : done ? "done" : ""}">${text}</td>`);
      }
      const you = side === this.side ? ' <i>you</i>' : "";
      const win = s.phase === "over" && s.winner === side ? " win" : "";
      const total = sum[side] + (s.phase === "live" && pilotOf(s.turn) === side ? s.hits : 0);
      return `<tr class="${win}"><th>${escapeHtml(this._name(side))}${you}</th>${cells.join("")}<td class="total">${total}</td></tr>`;
    };
    table.innerHTML = `<tr>${head.map((h) => `<th>${h}</th>`).join("")}</tr>${row("host")}${row("guest")}`;
  }

  _showFinal() {
    const s = this.state;
    const sum = totals(s.turns);
    const title =
      s.winner === "draw"
        ? `Draw, ${sum.host}–${sum.guest}`
        : `${this._name(s.winner)} wins ${sum[s.winner]}–${sum[other(s.winner)]}`;
    $("duel-final-title").textContent = s.winner === this.side ? `You win! ${title}` : title;
    sfx.play(s.winner === "draw" ? "turnEnd" : s.winner === this.side ? "win" : "lose");
    $("duel-final-note").textContent = s.note ? `Match over: ${s.note}` : "Match over";
    this._table($("duel-final-table"));
    $("duel-final").hidden = false;
  }
}

function escapeHtml(text) {
  return String(text).replace(/[&<>"']/g, (c) => `&#${c.charCodeAt(0)};`);
}

function loadRules() {
  try {
    const saved = JSON.parse(loadLocal(RULES_KEY) ?? "{}");
    const rules = { ...DEFAULT_RULES, ...saved };
    rules.rounds = Math.min(Math.max(Math.round(rules.rounds) || 1, 1), MAX_ROUNDS);
    rules.minutes = Math.min(Math.max(Math.round(rules.minutes) || 2, 1), 10);
    return rules;
  } catch {
    return { ...DEFAULT_RULES };
  }
}

function loadLocal(key) {
  try {
    return localStorage.getItem(key);
  } catch {
    return null;
  }
}

function saveLocal(key, value) {
  try {
    localStorage.setItem(key, value);
  } catch {
    // Storage blocked: it lasts for this page only.
  }
}

function loadSession(key) {
  try {
    return JSON.parse(sessionStorage.getItem(key) ?? "null");
  } catch {
    return null;
  }
}

function saveSession(key, value) {
  try {
    sessionStorage.setItem(key, JSON.stringify(value));
  } catch {
    // Storage blocked: a reload starts the match again.
  }
}
