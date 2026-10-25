// The drone radio (swarm.radio's, in joystick mode) as the truck's controller, and its panel.
// Each job (throttle, steer, gear, handbrake, reset) is one channel of the radio, picked on the
// panel: Set, then move that stick or switch to where the panel asks. That records the
// channel and which way round it is, for any radio; the choice is kept in this browser.
//
// The throttle, its full travel, drives in the gear: forward, neutral or reverse (reverse
// brakes first). Its stick stays where it is left, so it drives nothing till it has been at the
// bottom once after each placing (`rearm`). The reset switch puts the truck back on its wheels
// on every flick.
//
// The channels come from the viewer (swarm.view's /radio), which reads the radio from Linux
// itself: Chrome's Gamepad API on Linux misses most of a drone radio's axes.

const STORE_KEY = "drive-radio";
/** Axis n is channel n + 1, each -1..1. swarm.radio's order: roll, pitch, throttle, yaw, then
 * switches; CH6 is its 3-position flight-mode switch. `sign` -1: the channel runs the other way. */
const DEFAULT_MAP = {
  throttle: { axis: 2, sign: 1 },
  steer: { axis: 3, sign: 1 },
  gear: { axis: 5, sign: 1 },
  handbrake: { axis: 4, sign: 1 },
  reset: { axis: 6, sign: 1 },
};
const JOBS = {
  throttle: { name: "Throttle", ask: "Throttle stick to the top and hold it" },
  steer: { name: "Steer", ask: "Steering stick to the right and hold it" },
  gear: { name: "Gear", ask: "Gear switch to forward (D)" },
  handbrake: { name: "Handbrake", ask: "Handbrake switch to on" },
  reset: { name: "Reset", ask: "Flick the reset switch" },
};
const DEADBAND = 0.1;
/** A channel moved this far from where it was has been picked (Set). */
const PICK_MOVE = 0.5;
/** It has stopped moving, at an end, this long: the end the panel asked for. */
const PICK_STILL_MS = 300;
/** The panel's live values this often. */
const PANEL_MS = 66;

/** A drone radio as Chrome sees it, not a game controller: no standard layout, and four sticks
 * and a switch at least. Its controls come from /radio instead (DriveSession skips it). */
export function isRadio(pad) {
  return pad?.mapping !== "standard" && pad?.axes.length >= 5;
}

/** The radio's channels from /radio, one stream for the page: [-1..1, ...] from channel 1, or
 * null with no radio. Open while a DriveRadio (or the wing page, wing.js) is. */
export const feed = {
  channels: null,
  source: null,
  users: 0,
  open() {
    if (this.users++ === 0) {
      this.source = new EventSource("/radio");
      this.source.onmessage = (e) => {
        const channels = JSON.parse(e.data);
        this.channels = Array.isArray(channels) ? channels : null;
      };
      this.source.onerror = () => {
        this.channels = null;
      };
    }
  },
  close() {
    if (--this.users === 0) {
      this.source.close();
      this.source = null;
      this.channels = null;
    }
  },
};

/** -1, 0 or 1: which third of its travel a channel is in. */
function position(v) {
  return v > 0.5 ? 1 : v < -0.5 ? -1 : 0;
}

/** Only a change: the panel is redrawn many times a second, under the mouse. */
function setText(el, text) {
  if (el.textContent !== text) el.textContent = text;
}

function loadMap() {
  try {
    const saved = JSON.parse(localStorage.getItem(STORE_KEY) ?? "{}");
    const map = structuredClone(DEFAULT_MAP);
    for (const job of Object.keys(map)) {
      const s = saved[job];
      if (Number.isInteger(s?.axis) && (s.sign === 1 || s.sign === -1)) map[job] = { axis: s.axis, sign: s.sign };
    }
    return map;
  } catch {
    return structuredClone(DEFAULT_MAP);
  }
}

export class DriveRadio {
  /**
   * @param {{panel?: HTMLElement, notice?: (text: string | null) => void}} ui `panel`: where
   *   the radio's panel goes, shown while a radio is plugged in
   */
  constructor({ panel, notice } = {}) {
    this.map = loadMap();
    this.notice = notice ?? (() => {});
    /** The throttle has been at the bottom since the last placing. */
    this.ready = false;
    this._asked = false;
    this._resetAt = null;
    /** Set in progress: {job, base: axes then, axis, value, stillSince}. */
    this._picking = null;
    this._panelT = 0;
    this._last = null;
    this._panel = panel ? this._buildPanel(panel) : null;
    feed.open();
  }

  destroy() {
    feed.close();
    if (this._panel) {
      this._panel.root.replaceChildren();
      this._panel.root.hidden = true;
    }
    if (this._asked) this.notice(null);
  }

  /** The truck has just been put down: the throttle waits for the bottom again. */
  rearm() {
    this.ready = false;
    this._asked = false;
  }

  /** The radio's inputs this frame, or null with no radio: {throttle, steer, handbrake, reset}
   * (reset true on the frame its switch is flicked). `placed`: the truck is on the ground. */
  read(placed) {
    const axes = feed.channels;
    this._showPanel(axes);
    if (!axes) return null;
    if (this._picking) {
      this._pick(axes);
      return { throttle: 0, steer: 0, handbrake: false, reset: false };
    }
    const value = (job) => {
      const { axis, sign } = this.map[job];
      return (axes[axis] ?? 0) * sign;
    };
    const lift = (value("throttle") + 1) / 2;
    const gear = position(value("gear"));
    const steer = value("steer");
    const resetAt = position(value("reset"));
    const reset = this._resetAt !== null && resetAt !== this._resetAt;
    this._resetAt = resetAt;
    const atBottom = lift < DEADBAND;
    if (!this.ready && atBottom) {
      this.ready = true;
      if (this._asked) this.notice(null);
    } else if (!this.ready && placed && !this._asked) {
      this._asked = true;
      this.notice("Throttle stick to the bottom to drive");
    }
    this._last = { lift, gear, steer, handbrake: value("handbrake") > 0.5, reset: resetAt };
    return {
      throttle: this.ready && !atBottom ? gear * lift : 0,
      steer: Math.abs(steer) > DEADBAND ? -steer : 0,
      handbrake: this._last.handbrake,
      reset,
    };
  }

  /** Start picking the channel for `job`. */
  _startPick(job) {
    const axes = feed.channels;
    if (!axes) return;
    this._picking = { job, base: [...axes], axis: null, value: 0, stillSince: 0 };
  }

  /** Watch for the channel that moves, then for where it stops. */
  _pick(axes) {
    const p = this._picking;
    const now = performance.now();
    if (p.axis === null) {
      let most = PICK_MOVE;
      axes.forEach((v, i) => {
        const moved = Math.abs(v - (p.base[i] ?? v));
        if (moved > most) {
          most = moved;
          p.axis = i;
        }
      });
      if (p.axis === null) return;
      p.value = axes[p.axis];
      p.stillSince = now;
    }
    const v = axes[p.axis];
    if (Math.abs(v - p.value) > 0.05) {
      p.value = v;
      p.stillSince = now;
      return;
    }
    // The reset switch needs only the flick; the rest, where it was put.
    if (p.job !== "reset" && (Math.abs(v) < 0.5 || now - p.stillSince < PICK_STILL_MS)) return;
    this.map[p.job] = { axis: p.axis, sign: p.job === "reset" || v > 0 ? 1 : -1 };
    try {
      localStorage.setItem(STORE_KEY, JSON.stringify(this.map));
    } catch {
      // kept for this page only
    }
    this._picking = null;
    this._resetAt = null;
    this.rearm();
  }

  _buildPanel(root) {
    root.replaceChildren();
    const el = (tag, cls, text) => {
      const e = document.createElement(tag);
      if (cls) e.className = cls;
      if (text) e.textContent = text;
      return e;
    };
    root.append(el("div", "radio-title", "RADIO"));
    const rows = {};
    for (const [job, { name }] of Object.entries(JOBS)) {
      const row = el("div", "radio-row");
      const value = el("span", "radio-value");
      const bar = job === "throttle" || job === "steer" ? el("span", `radio-bar ${job}`) : null;
      if (bar) {
        bar.append(el("i"));
        value.append(bar);
      } else {
        value.textContent = "–";
      }
      const channel = el("span", "radio-ch");
      const set = el("button", "radio-set", "Set");
      set.type = "button";
      set.addEventListener("click", () => {
        set.blur(); // Space is the handbrake, not this button
        if (this._picking?.job === job) this._picking = null;
        else this._startPick(job);
      });
      row.append(el("span", "radio-name", name), value, channel, set);
      root.append(row);
      rows[job] = { row, value, bar: bar?.firstChild, channel, set };
    }
    const ask = el("div", "radio-ask");
    const channels = el("div", "radio-channels");
    root.append(ask, channels);
    root.hidden = true;
    return { root, rows, ask, channels, chBars: [] };
  }

  _showPanel(axes) {
    const panel = this._panel;
    if (!panel) return;
    panel.root.hidden = !axes;
    const now = performance.now();
    if (!axes || now - this._panelT < PANEL_MS) return;
    this._panelT = now;
    const last = this._last;
    for (const [job, r] of Object.entries(panel.rows)) {
      const { axis } = this.map[job];
      const picking = this._picking?.job === job;
      setText(r.channel, `CH${axis + 1}`);
      r.row.classList.toggle("picking", picking);
      setText(r.set, picking ? "Cancel" : "Set");
      if (!last) continue;
      if (job === "throttle") {
        r.bar.style.width = `${last.lift * 100}%`;
        r.row.classList.toggle("waiting", !this.ready);
      } else if (job === "steer") {
        const s = Math.max(-1, Math.min(1, last.steer));
        r.bar.style.left = `${50 + Math.min(0, s) * 50}%`;
        r.bar.style.width = `${Math.abs(s) * 50}%`;
      } else if (job === "gear") {
        setText(r.value, ["R reverse", "N neutral", "D forward"][last.gear + 1]);
      } else if (job === "handbrake") {
        setText(r.value, last.handbrake ? "ON" : "off");
        r.row.classList.toggle("on", last.handbrake);
      } else {
        setText(r.value, ["low", "middle", "high"][last.reset + 1]);
      }
    }
    const ask = this._picking
      ? `${JOBS[this._picking.job].ask}…`
      : this.ready
        ? ""
        : "Throttle to the bottom to drive";
    setText(panel.ask, ask);
    // Every channel, live: flick a switch to see which it is.
    while (panel.chBars.length < axes.length) {
      const cell = document.createElement("span");
      cell.className = "radio-chan";
      const fill = document.createElement("i");
      const label = document.createElement("b");
      label.textContent = `${panel.chBars.length + 1}`;
      cell.append(fill, label);
      panel.channels.append(cell);
      panel.chBars.push(fill);
    }
    axes.forEach((v, i) => {
      panel.chBars[i].style.height = `${((v + 1) / 2) * 100}%`;
    });
  }
}
