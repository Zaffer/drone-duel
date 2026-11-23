// The simulator's worker (DESIGN.md §18): the web version's session (session.ts) on a thread
// of its own, so the firmware and the physics never wait on the page's drawing. The page
// (net/worker-link.ts) posts `init` once, then the messages it would send the game's server;
// this posts back what the server would send, and each true pose as its 18 doubles
// (transferred, not copied).
//
// The quad runs on this thread's timer: every few milliseconds it is stepped up to now, a
// millisecond a frame. A worker's timers keep their rate when the tab is hidden, and a long
// stall (the machine asleep) is not caught up (session.ts's LATE_S).

import type { ContentMsg } from "../net/protocol.ts";
import { type Incoming, type Outgoing, type SimData, WebSession } from "./session.ts";

/** The page's first message: the content it loaded, and where the simulator's files are. */
export interface Init {
  ch: "init";
  content: ContentMsg;
  /** URLs of data/sim.json and sim/sim.wasm. */
  simData: string;
  module: string;
}

/** Open-Meteo's elevation API (Copernicus DEM, 90 m; no key, open to pages), as the game's
 * server asks it (drone_duels.world.elevation): the ground under a place the player typed in. */
const ELEVATION_URL = "https://api.open-meteo.com/v1/elevation";

async function groundAlt(lat: number, lon: number): Promise<number> {
  const url = `${ELEVATION_URL}?latitude=${lat.toFixed(6)}&longitude=${lon.toFixed(6)}`;
  let response: Response;
  try {
    response = await fetch(url, { signal: AbortSignal.timeout(10_000) });
  } catch (err) {
    throw new Error(`could not look it up (${String(err)})`);
  }
  if (!response.ok) throw new Error(`could not look it up (${response.status})`);
  const alt = ((await response.json()) as { elevation?: unknown[] }).elevation?.[0];
  if (typeof alt !== "number" || !Number.isFinite(alt)) throw new Error("no height in the answer");
  return alt;
}

/** Between runs of the quad, ms: a timer's least. */
const TICK_MS = 4;

interface Scope {
  postMessage(message: unknown, transfer?: Transferable[]): void;
  onmessage: ((e: MessageEvent<Init | Incoming>) => void) | null;
}
const scope = self as unknown as Scope;
const now = (): number => performance.now() / 1000;

let session: WebSession | null = null;
const early: Incoming[] = [];

async function start(init: Init): Promise<void> {
  const data = (await (await fetch(init.simData)).json()) as SimData;
  let module: Promise<WebAssembly.Module> | null = null;
  session = new WebSession({
    content: init.content,
    sim: data,
    module: () => {
      module ??= WebAssembly.compileStreaming(fetch(init.module)).catch((err: unknown) => {
        module = null; // offline, say: the next start asks again
        throw new Error(`could not load the simulator: ${String(err)}`);
      });
      return module;
    },
    send: (msg: Outgoing) => scope.postMessage(msg),
    pose: (frame) => scope.postMessage(frame, [frame.buffer]),
    yield: () => new Promise((done) => setTimeout(done, 0)),
    groundAlt,
  });
  for (const msg of early.splice(0)) session.handle(msg, now());
  setInterval(() => session?.tick(now()), TICK_MS);
  scope.postMessage({ ch: "open" });
}

scope.onmessage = (e) => {
  const msg = e.data;
  if (msg.ch === "init") {
    start(msg).catch((err: unknown) => {
      scope.postMessage({
        ch: "log",
        level: "error",
        source: "server",
        text: `the simulator did not start: ${String(err)}`,
      });
    });
  } else if (session) {
    session.handle(msg, now());
    // The sticks reach the firmware now, not at the next timer.
    if (msg.ch === "rc") session.tick(now());
  } else {
    early.push(msg);
  }
};
