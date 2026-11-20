// The link to the game's server (/ws, DESIGN.md §8): JSON messages by channel, and the true pose
// as binary frames. It reconnects on its own. Everything that arrives is an event on `link`, so
// any module can take it as an Observable: `link.on("telemetry")`, `link.poses()`.
//
// The web version (DESIGN.md §18) has no server: its `link` is to the simulator in a worker of
// the page, which takes and sends the same messages (`WorkerLink`). Nothing above this module
// can tell which it has.

import { signal } from "alien-signals";
import type { Init as SimInit } from "../sim/worker.ts";
import { content } from "../state/app.ts";
import schema from "./protocol.schema.json";
import type {
  DynamicsData,
  DynamicsMsg,
  GroundMsg,
  HitMsg,
  LogMsg,
  OsdMsg,
  PhysicsMsg,
  PlanMsg,
  RadioMsg,
  RadioSourceMsg,
  RcMsg,
  SessionMsg,
  StatusMsg,
  TargetMsg,
  TelemetryMsg,
  VideoMsg,
  WindMsg,
} from "./protocol.ts";
import { asset, WEB } from "./target.ts";

export type ServerMsg =
  | StatusMsg
  | TelemetryMsg
  | OsdMsg
  | PlanMsg
  | DynamicsData
  | LogMsg
  | RadioMsg
  | PhysicsMsg;
export type ClientMsg =
  | SessionMsg
  | RcMsg
  | GroundMsg
  | WindMsg
  | VideoMsg
  | DynamicsMsg
  | RadioSourceMsg
  | TargetMsg
  | HitMsg;
type ByCh<C extends ServerMsg["ch"]> = Extract<ServerMsg, { ch: C }>;

/** protocol.POSE_FIELDS, the order of the doubles in a pose frame. */
export const POSE_FIELDS = schema.poseFields as readonly string[];

/** The true state of the vehicle, from its physics (never from telemetry). */
export interface Pose {
  /** Physics time, s. */
  t: number;
  lat: number;
  lon: number;
  /** Metres above sea level. */
  alt: number;
  /** Attitude, body (forward, right, down) to north-east-down: w, x, y, z. */
  q: [number, number, number, number];
  /** North, east, down, m/s. */
  v: [number, number, number];
  /** Body rates, rad/s. */
  rates: [number, number, number];
  /** Quad: rotor speeds, rad/s. Wing: elevons L, R (rad), prop (rad/s). */
  act: [number, number, number, number];
  /** performance.now() when it arrived. */
  at: number;
}

export function decodePose(buf: ArrayBuffer, at: number): Pose {
  const d = new Float64Array(buf);
  return {
    t: d[0]!,
    lat: d[1]!,
    lon: d[2]!,
    alt: d[3]!,
    q: [d[4]!, d[5]!, d[6]!, d[7]!],
    v: [d[8]!, d[9]!, d[10]!],
    rates: [d[11]!, d[12]!, d[13]!],
    act: [d[14]!, d[15]!, d[16]!, d[17]!],
    at,
  };
}

class MsgEvent<M> extends Event {
  constructor(
    type: string,
    readonly msg: M,
  ) {
    super(type);
  }
}

/** The link's two ends' common part: every message that arrives is an event on it. */
export abstract class Link extends EventTarget {
  readonly connected = signal(false);

  abstract open(): void;
  abstract send(msg: ClientMsg): void;

  protected message(msg: ServerMsg): void {
    this.dispatchEvent(new MsgEvent(msg.ch, msg));
  }

  protected pose(buf: ArrayBuffer): void {
    this.dispatchEvent(new MsgEvent("pose", decodePose(buf, performance.now())));
  }

  /** Every message on channel `ch`. */
  on<C extends ServerMsg["ch"]>(ch: C): Observable<ByCh<C>> {
    return this.when<MsgEvent<ByCh<C>>>(ch).map((e) => e.msg);
  }

  /** Every true pose, as it arrives. */
  poses(): Observable<Pose> {
    return this.when<MsgEvent<Pose>>("pose").map((e) => e.msg);
  }
}

/** The game's server, over its WebSocket. */
export class ServerLink extends Link {
  private ws: WebSocket | null = null;
  private retry = 500;

  constructor(private readonly url: string) {
    super();
  }

  open(): void {
    const ws = new WebSocket(this.url);
    ws.binaryType = "arraybuffer";
    this.ws = ws;
    ws.onopen = () => {
      this.retry = 500;
      this.connected(true);
      this.dispatchEvent(new Event("open"));
    };
    ws.onclose = () => {
      this.connected(false);
      this.ws = null;
      setTimeout(() => this.open(), this.retry);
      this.retry = Math.min(this.retry * 2, 5000);
    };
    ws.onmessage = (e: MessageEvent<string | ArrayBuffer>) => {
      if (typeof e.data === "string") this.message(JSON.parse(e.data) as ServerMsg);
      else this.pose(e.data);
    };
  }

  send(msg: ClientMsg): void {
    if (this.ws?.readyState === WebSocket.OPEN) this.ws.send(JSON.stringify(msg));
  }
}

/** The web version's simulator (sim/worker.ts), in a worker of this page: the same messages
 * as the server's, posted, not sent; a pose comes as its 18 doubles. */
export class WorkerLink extends Link {
  private worker: Worker | null = null;

  open(): void {
    const worker = new Worker(asset("sim-worker.js"), { type: "module", name: "simulator" });
    this.worker = worker;
    worker.onmessage = (e: MessageEvent<ServerMsg | Float64Array | { ch: "open" }>) => {
      const data = e.data;
      if (data instanceof Float64Array) {
        this.pose(data.buffer as ArrayBuffer);
      } else if (data.ch === "open") {
        this.connected(true);
        this.dispatchEvent(new Event("open"));
      } else {
        this.message(data);
      }
    };
    worker.onerror = (e) => {
      this.connected(false);
      console.error("the simulator's worker failed", e.message);
    };
    const init: SimInit = {
      ch: "init",
      content: content()!,
      simData: asset("data/sim.json"),
      module: asset("sim/sim.wasm"),
    };
    worker.postMessage(init);
  }

  send(msg: ClientMsg): void {
    this.worker?.postMessage(msg);
  }
}

export const link: Link = WEB
  ? new WorkerLink()
  : new ServerLink(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`);
