// The other player, peer to peer (DESIGN.md §7): a room over Trystero, its WebRTC connection set
// up through public Nostr relays, then browser to browser with no server between them. Each
// message type is its own Trystero action. The room's host runs the match; the guest follows.
//
//   hello  both ways  {role, name, version}: role "host" or "guest"; a page of another protocol
//                     version says so and parts
//   ping   both ways  {t}: the sender's clock (Date.now()); answered by pong {t, at}
// and whatever the mode sends (duel: match, intent, pose, hit, lost; race: race, pose, gate, done).
// The clock offset from the pings lets both pages agree when a countdown ends.

import type { JsonValue, MessageAction, Room as TrysteroRoom } from "@trystero-p2p/core";
import { signal } from "alien-signals";

const APP_ID = "drone-duels";
const PROTOCOL = 1;
const PING_MS = 2000;
/** Room codes: no 0/o, 1/l/i to misread when said aloud. */
const CODE_LETTERS = "abcdefghjkmnpqrstuvwxyz23456789";

export type Role = "host" | "guest";

export const newRoomCode = (): string =>
  Array.from(
    crypto.getRandomValues(new Uint8Array(6)),
    (n) => CODE_LETTERS[n % CODE_LETTERS.length],
  ).join("");

/** A room code as typed: lower case, no spaces. */
export const cleanRoomCode = (text: string): string =>
  text.trim().toLowerCase().replace(/\s+/g, "");

class PeerMessage extends Event {
  constructor(
    type: string,
    readonly data: unknown,
  ) {
    super(type);
  }
}

export class Room extends EventTarget {
  /** The opponent, once their hello has come. */
  readonly peer = signal<{ id: string; name: string } | null>(null);
  readonly error = signal<string | null>(null);
  rttMs: number | null = null;
  /** The opponent's clock less ours (ms), from the pings. */
  clockOffset = 0;
  private actions = new Map<string, MessageAction<JsonValue>>();
  private roles = new Map<string, { role: Role; name: string }>();
  private pinger: ReturnType<typeof setInterval>;
  private onHide = () => void this.leave();

  /** Join room `code` of `game` ("duel", "race") as `role`, called `name`. */
  static async join(
    game: string,
    code: string,
    role: Role,
    name: string,
    types: string[],
  ): Promise<Room> {
    const { joinRoom } = await import("trystero");
    return new Room(joinRoom as unknown as JoinRoom, `${game}-${code}`, role, name, types);
  }

  private readonly room: TrysteroRoom;

  private constructor(
    joinRoom: JoinRoom,
    code: string,
    readonly role: Role,
    readonly name: string,
    types: string[],
  ) {
    super();
    this.room = joinRoom({ appId: APP_ID }, code, {
      onJoinError: ({ error }: { error: string }) => this.error(`could not reach a peer: ${error}`),
    });
    for (const type of ["hello", "ping", "pong", ...types]) {
      const action = this.room.makeAction<JsonValue>(type);
      action.onMessage = (data, { peerId }) => this.receive(type, data, peerId);
      this.actions.set(type, action);
    }
    this.room.onPeerJoin = (peerId) =>
      void this.actions
        .get("hello")!
        .send({ role, name, version: PROTOCOL }, { target: peerId })
        .catch(() => {});
    this.room.onPeerLeave = (peerId) => {
      this.roles.delete(peerId);
      if (peerId !== this.peer()?.id) return;
      this.rttMs = null;
      // Another page of the opponent's role, waiting in the room, takes over.
      const next = [...this.roles].find(([, r]) => r.role !== role);
      this.peer(next ? { id: next[0], name: next[1].name } : null);
    };
    this.pinger = setInterval(() => this.send("ping", { t: Date.now() }), PING_MS);
    addEventListener("pagehide", this.onHide);
  }

  get connected(): boolean {
    return this.peer() !== null;
  }

  /** Every message of `type` from the opponent. */
  on<T>(type: string): Observable<T> {
    return this.when<PeerMessage>(type).map((e) => e.data as T);
  }

  /** Send the opponent message `type`; nothing without one. Never throws. */
  send(type: string, data: JsonValue): void {
    const peer = this.peer();
    const action = this.actions.get(type);
    if (!peer || !action) return;
    try {
      action
        .send(data, { target: peer.id })
        .catch((err: unknown) => console.warn(`peer: ${type}`, err));
    } catch (err) {
      console.warn(`peer: ${type}`, err);
    }
  }

  /** The opponent's clock now, by ours. */
  peerNow(): number {
    return Date.now() + this.clockOffset;
  }

  async leave(): Promise<void> {
    clearInterval(this.pinger);
    removeEventListener("pagehide", this.onHide);
    this.peer(null);
    await this.room.leave().catch(() => {});
  }

  private receive(type: string, data: JsonValue, peerId: string): void {
    if (type === "hello") {
      const h = data as { role: Role; name: string; version: number };
      if (h.version !== PROTOCOL) {
        this.error(
          `the other page runs another version of the game (${h.version}, this is ${PROTOCOL})`,
        );
        return;
      }
      this.roles.set(peerId, { role: h.role, name: h.name });
      if (h.role === this.role) {
        this.error(`the room already has a ${this.role}`);
        return;
      }
      if (!this.peer()) {
        this.peer({ id: peerId, name: h.name });
        // Say hello back, in case ours went before they listened.
        void this.actions
          .get("hello")!
          .send({ role: this.role, name: this.name, version: PROTOCOL }, { target: peerId })
          .catch(() => {});
      }
      return;
    }
    if (peerId !== this.peer()?.id) return;
    if (type === "ping") {
      this.send("pong", { t: (data as { t: number }).t, at: Date.now() });
      return;
    }
    if (type === "pong") {
      const { t, at } = data as { t: number; at: number };
      const now = Date.now();
      const rtt = now - t;
      // Keep the offset from the quickest round trip: it has the least jitter in it.
      if (this.rttMs === null || rtt <= this.rttMs * 1.2) this.clockOffset = at - (t + rtt / 2);
      this.rttMs = this.rttMs === null ? rtt : 0.8 * this.rttMs + 0.2 * rtt;
      return;
    }
    this.dispatchEvent(new PeerMessage(type, data));
  }
}

type JoinRoom = (
  config: { appId: string },
  roomId: string,
  callbacks?: { onJoinError?: (e: { error: string }) => void },
) => TrysteroRoom;
