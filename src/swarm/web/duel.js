// The duel: a match of turns between two players, each with their own sim and COP. In each
// turn one player flies the mission with the drones and strikes the truck; the other drives
// the truck, in their COP (dueldriver.js), and keeps away. The pilot scores a point for each
// drone that strikes it before the time runs out. Then they swap. The pages meet in a room
// over Trystero: WebRTC, its connection set up through public Nostr relays, then browser to
// browser with no server between them.
//
// The room's host, who opened it, runs the match and flies first (duelmatch.js). The pilot's
// page judges the strikes and the crashes, on what its pilot sees (duelpilot.js).
//
// Each message is its own Trystero action. Poses are Earth-fixed (ECEF): metres, and the
// rotation from the model's frame as a quaternion, so neither page needs the other's frame.
//   hello   both ways      {role, name, version}: role "host" or "guest"
//   match   host → guest   the match (DuelMatch.state), with leftS for a turn on
//   intent  guest → host   {action}: "start-turn" (the guest flies next and is ready) or
//                          "out" (the guest's drones are all out)
//   field   pilot → driver {b: [lat, lon], fence: [[lat, lon], ...]}: the truck's start, the
//                          fence it keeps inside (none: anywhere)
//   drones  pilot → driver [[sysid, x, y, z, qx, qy, qz, qw, armed, m1, m2, m3, m4], ...]: those
//                          still flying, with their motor outputs in µs (none: not reported)
//   car     driver → pilot [x, y, z, qx, qy, qz, qw, speed, throttle]: m/s, -1..1
//   hit     pilot → driver {sysid, at: [x, y, z], counted}: a drone struck the truck
//   lost    pilot → driver {sysid, at: [x, y, z], why}: a drone crashed; it is out

const C = Cesium;

/** Trystero's namespace for the duel. Pages of another protocol version say so and part. */
const APP_ID = "drone-duel";
const PROTOCOL = 2;
const MESSAGES = ["hello", "match", "intent", "field", "drones", "car", "hit", "lost"];
const PING_MS = 2000;
/** Room codes: no 0/o, 1/l/i to misread when said aloud. */
const CODE_LETTERS = "abcdefghjkmnpqrstuvwxyz23456789";

export const newRoomCode = () =>
  Array.from(crypto.getRandomValues(new Uint8Array(6)), (n) => CODE_LETTERS[n % CODE_LETTERS.length]).join("");

/** A room code as typed: lower case, no spaces. */
export const cleanRoomCode = (text) => text.trim().toLowerCase().replace(/\s+/g, "");

/** The other player, peer to peer. Join with DuelLink.join; `on(type, fn)` takes each message
 * the opponent sends; `onPeer(connected)` and `onError(text)` report the link. */
export class DuelLink {
  /** Join room `code` as `role`, "host" or "guest", called `name`. Rejects without Trystero
   * (setup.sh). */
  static async join(code, role, name) {
    const { joinRoom } = await import("trystero");
    return new DuelLink(joinRoom, code, role, name);
  }

  constructor(joinRoom, code, role, name) {
    this.code = code;
    this.role = role;
    this.name = name;
    /** The opponent's peer id and name, once its hello has come. */
    this.peer = null;
    this.peerName = null;
    this.rttMs = null;
    this.onPeer = null;
    this.onError = null;
    this._handlers = {};
    /** peerId -> {role, name}, of every page in the room that said hello. */
    this._roles = new Map();
    this.room = joinRoom({ appId: APP_ID }, code, {
      onJoinError: ({ error }) => this.onError?.(`could not reach a peer: ${error}`),
    });
    this._actions = Object.fromEntries(MESSAGES.map((type) => [type, this.room.makeAction(type)]));
    for (const [type, action] of Object.entries(this._actions)) {
      action.onMessage = (data, { peerId }) => {
        if (type === "hello") this._hello(data, peerId);
        else if (peerId === this.peer) this._handlers[type]?.(data);
      };
    }
    this.room.onPeerJoin = (peerId) =>
      this._actions.hello.send({ role, name, version: PROTOCOL }, { target: peerId }).catch(() => {});
    this.room.onPeerLeave = (peerId) => {
      this._roles.delete(peerId);
      if (peerId !== this.peer) return;
      this.peer = null;
      this.peerName = null;
      this.rttMs = null;
      // Another page of the opponent's role, waiting in the room, takes over.
      const next = [...this._roles].find(([, r]) => r.role !== this.role);
      if (next) [this.peer, this.peerName] = [next[0], next[1].name];
      this.onPeer?.(this.peer !== null);
    };
    this._pinger = setInterval(() => this._ping(), PING_MS);
    // Say goodbye on the way out: otherwise WebRTC notices only when the connection times out.
    this._onPageHide = () => this.leave();
    addEventListener("pagehide", this._onPageHide);
  }

  get connected() {
    return this.peer !== null;
  }

  on(type, fn) {
    this._handlers[type] = fn;
  }

  /** Send the opponent message `type`; nothing without one. Never throws: the pages send from
   * inside Cesium's render loop, which stops for good on an error. */
  send(type, data) {
    if (!this.peer) return;
    try {
      this._actions[type].send(data, { target: this.peer }).catch((err) => this._sendFailed(type, err));
    } catch (err) {
      this._sendFailed(type, err);
    }
  }

  _sendFailed(type, err) {
    console.warn(`duel: sending ${type} failed`, err);
  }

  leave() {
    clearInterval(this._pinger);
    removeEventListener("pagehide", this._onPageHide);
    this.room.leave();
    this.peer = null;
  }

  _hello({ role, name, version }, peerId) {
    if (version !== PROTOCOL) {
      this.onError?.(`the other page speaks duel version ${version}, this one ${PROTOCOL}: update both`);
      return;
    }
    this._roles.set(peerId, { role, name });
    if (role === this.role) {
      this.onError?.(`room ${this.code} has another ${role}: one opens it, the other joins`);
      return;
    }
    if (this.peer) return; // one opponent at a time; the others wait
    this.peer = peerId;
    this.peerName = String(name || "Opponent").slice(0, 24);
    this.onPeer?.(true);
  }

  async _ping() {
    if (!this.peer) return;
    try {
      this.rttMs = await this.room.ping(this.peer);
    } catch {
      // Gone: onPeerLeave says so.
    }
  }
}

/** [x, y, z, qx, qy, qz, qw] of an Earth-fixed position and rotation, to the centimetre. */
export function packPose(position, quaternion) {
  const cm = (v) => Math.round(v * 100) / 100;
  const q = (v) => Math.round(v * 1e5) / 1e5;
  return [cm(position.x), cm(position.y), cm(position.z), q(quaternion.x), q(quaternion.y), q(quaternion.z), q(quaternion.w)];
}

/** The position and rotation of a model matrix, packed (packPose). */
export function packModelMatrix(m) {
  const rotation = C.Matrix4.getMatrix3(m, scratchRotation);
  return packPose(
    C.Matrix4.getTranslation(m, scratchPosition),
    C.Quaternion.fromRotationMatrix(rotation, scratchQuaternion),
  );
}

const scratchRotation = new C.Matrix3();
const scratchPosition = new C.Cartesian3();
const scratchQuaternion = new C.Quaternion();

const KEEP_SAMPLES_S = 5;

/** A thing the other page moves: each pose it sends is drawn `delayS` after it comes, so the
 * drawing glides between poses instead of jumping, as the COP draws the telemetry. */
export class PoseTrack {
  constructor(delayS) {
    this.delayS = delayS;
    this.position = new C.SampledPositionProperty();
    this.orientation = new C.SampledProperty(C.Quaternion);
    for (const p of [this.position, this.orientation]) {
      p.forwardExtrapolationType = C.ExtrapolationType.HOLD;
      p.backwardExtrapolationType = C.ExtrapolationType.HOLD;
    }
    /** performance.now() of the last pose. */
    this.at = 0;
    this._pruned = 0;
  }

  /** Take pose [x, y, z, qx, qy, qz, qw, ...] (packPose); a malformed one is dropped. */
  add(pose) {
    if (!Array.isArray(pose) || pose.length < 7 || !pose.slice(0, 7).every(Number.isFinite)) return;
    const q = new C.Quaternion(pose[3], pose[4], pose[5], pose[6]);
    if (C.Quaternion.magnitude(q) < 0.5) return;
    // Back to unit length after packPose's rounding: Cesium interpolates rotations through
    // acos(w), which is NaN for a w past 1, and a NaN model stops the whole scene rendering.
    C.Quaternion.normalize(q, q);
    const when = C.JulianDate.addSeconds(C.JulianDate.now(), this.delayS, new C.JulianDate());
    this.position.addSample(when, new C.Cartesian3(pose[0], pose[1], pose[2]));
    this.orientation.addSample(when, q);
    this.at = performance.now();
    if (this.at - this._pruned > KEEP_SAMPLES_S * 1000) {
      this._pruned = this.at;
      const old = new C.TimeInterval({
        start: C.JulianDate.fromIso8601("2000-01-01T00:00:00Z"),
        stop: C.JulianDate.addSeconds(C.JulianDate.now(), -KEEP_SAMPLES_S, new C.JulianDate()),
      });
      this.position.removeSamples(old);
      this.orientation.removeSamples(old);
    }
  }

  /** Milliseconds since the last pose; Infinity before the first. */
  get age() {
    return this.at ? performance.now() - this.at : Infinity;
  }

  /** The model matrix as drawn now, or undefined before the first pose. */
  modelMatrix(result = new C.Matrix4()) {
    const now = C.JulianDate.now();
    const p = this.position.getValue(now, scratchPosition);
    const q = this.orientation.getValue(now, scratchQuaternion);
    if (!p || !q) return undefined;
    return C.Matrix4.fromRotationTranslation(C.Matrix3.fromQuaternion(q, scratchRotation), p, result);
  }
}

/** "4:05": seconds as minutes and seconds. */
export const clock = (s) => {
  const whole = Math.max(0, Math.ceil(s));
  return `${Math.floor(whole / 60)}:${String(whole % 60).padStart(2, "0")}`;
};
