// The FPV camera feed (DESIGN.md §10): the world as the vehicle's camera sees it, cropped to the
// camera's picture with black round it, as goggles show it, and the firmware's own OSD.
//
// Analog: NTSC, 4:3. The world is drawn into the 4:3 frame at about FEED_RENDER_WIDTH px (the
// picture is 754 × 480: more is wasted), then sent through analog-vision, signal by signal: the
// OSD goes into the video before the transmitter, as the flight controller's OSD chip puts it,
// so the link spoils it too. Its whole chain (AnalogFeed: the radio and the decoder's search
// for the sync) is more than a phone's GPU gets through live, so a phone has its light one
// (AnalogLight: the same signal and decoder's filters in two passes a field); the Video
// options say which (prefs.video.analog). The link's carrier to noise is free space from the
// pilot (setPilot: the operator's goggles, behind the start pad) at the VTX power, each end a
// dipole standing up from its frame, with slow fading on top. Where neither chain runs (no
// WebGL2 float targets), the world plain with the OSD laid over it.
//
// Digital: 16:9, letterboxed in the window, no effect, the HD OSD grid laid over the picture as
// DJI/HDZero/Walksnail goggles draw it.
//
// The rig (views/rigs.ts) sets the camera's field of view; Cesium takes it across the wider
// side, which is the picture's width here. The feed reads the picture in Cesium's postRender,
// while its drawing buffer is still there.

import { effect } from "alien-signals";
import { AnalogFeed, AnalogLight, linkCnrDb } from "analog-vision";
import type { OsdMsg } from "../net/protocol.ts";
import type { Pose } from "../net/server.ts";
import { prefs, type VideoSystem } from "../state/prefs.ts";
import { osd } from "../state/session.ts";
import { bodyAxesEnu, neFrom } from "../world/geo.ts";
import type { World } from "../world/world.ts";
import { type Box, fadeStep, fitFrame, linkGeometry } from "./link.ts";
import { OsdRaster, OsdView } from "./osd.ts";
import "./video.css";

/** The pictures' shapes. */
export const ANALOG_ASPECT = 4 / 3;
export const DIGITAL_ASPECT = 16 / 9;
type Chain = "full" | "light";
/** Each analog chain takes the world at about this width, device px: more is wasted. The whole
 * chain resamples it to 754 × 480 and is the better for a little more; the light one reads it
 * as it is, and copying more in is what costs a phone. */
const FEED_RENDER_WIDTH: Record<Chain, number> = { full: 1100, light: 754 };
/** An OSD screen older than this is gone (the firmware stopped drawing). */
const OSD_STALE_MS = 1000;

function pictureCanvas(): HTMLCanvasElement {
  const canvas = document.createElement("canvas");
  canvas.className = "cf-picture";
  return canvas;
}

/** A standard normal draw. */
const gauss = (): number =>
  Math.sqrt(-2 * Math.log(1 - Math.random())) * Math.cos(2 * Math.PI * Math.random());

export class CameraFeed {
  private readonly root: HTMLDivElement;
  private picture: HTMLCanvasElement;
  private readonly osdCanvas: HTMLCanvasElement;
  private readonly osdView: OsdView;
  private readonly raster = new OsdRaster();
  private analog: AnalogFeed | AnalogLight | null = null;
  /** The chain `analog` is, and the one that was asked for when it was made. */
  private chain: Chain = "full";
  private chainAsked: Chain | null = null;
  private on = false;
  private system: VideoSystem = "analog";
  private pilot: { lat: number; lon: number; alt: number } | null = null;
  private osdAt = 0;
  private osdIn = false;
  private fade = { db: 0, at: 0 };
  private cnr: number | null = null;
  /** What the world's container and the viewer had before the feed cropped them. */
  private saved: { style: Record<string, string>; scale: number } | null = null;
  private readonly stops: (() => void)[] = [];

  constructor(
    private readonly world: World,
    private readonly host: HTMLElement,
  ) {
    this.picture = pictureCanvas();
    this.osdCanvas = document.createElement("canvas");
    this.osdCanvas.className = "cf-osd";
    this.root = document.createElement("div");
    this.root.className = "camera-feed";
    this.root.hidden = true;
    // Inline: a host's own rules (`#app > *`) would otherwise win.
    this.root.style.pointerEvents = "none";
    this.root.append(this.picture, this.osdCanvas);
    host.append(this.root);
    this.osdView = new OsdView(this.osdCanvas, "analog");

    this.stops.push(
      effect(() => {
        osd();
        this.osdAt = performance.now();
      }),
    );
    this.stops.push(
      effect(() => {
        if (this.wanted() !== this.chainAsked && this.on && this.system === "analog") {
          this.ensureAnalog()?.reset();
          this.layout();
        }
      }),
    );
    this.stops.push(world.scene.postRender.addEventListener(() => this.afterRender()));
    const relayout = (): void => {
      if (this.on) this.layout();
    };
    window.addEventListener("resize", relayout);
    this.stops.push(() => window.removeEventListener("resize", relayout));
    const observer = new ResizeObserver(relayout);
    observer.observe(host);
    this.stops.push(() => observer.disconnect());
  }

  /** The video link's carrier to noise now, dB (analog; null before the first pose). */
  get cnrDb(): number | null {
    return this.cnr;
  }

  /** Whether an analog chain runs here (WebGL2 with float targets); null until it is first
   * needed. */
  get analogOk(): boolean | null {
    return this.analog?.ok ?? null;
  }

  /** On in the FPV view only. */
  setOn(on: boolean): void {
    if (on === this.on) return;
    this.on = on;
    this.root.hidden = !on;
    document.documentElement.classList.toggle("camera-feed-on", on);
    if (on) {
      if (this.system === "analog") this.ensureAnalog()?.reset();
      this.layout();
    } else {
      this.osdView.clear();
      this.restore();
    }
  }

  setSystem(system: VideoSystem): void {
    if (system === this.system) return;
    this.system = system;
    this.osdView.clear();
    if (this.on) {
      if (system === "analog") this.ensureAnalog()?.reset();
      this.layout();
    }
  }

  /** Where the receiver is: the pilot's goggles' antenna at the pad (`alt` its sim height, m:
   * the ground there and the pilot's eye height). */
  setPilot(lat: number, lon: number, alt: number): void {
    this.pilot = { lat, lon, alt };
  }

  /** Each animation frame: the OSD, and the link from the vehicle's true `pose`. The analog
   * picture itself is made in Cesium's postRender. */
  frame(now: number, pose: Pose | null): void {
    if (!this.on) return;
    if (pose && this.pilot) this.cnr = this.linkCnr(now, pose, this.pilot);
    const screen = this.liveOsd(now);
    const analogRuns = this.system === "analog" && this.analog?.ok === true;
    this.osdIn = analogRuns && screen !== null;
    if (this.osdIn && screen) {
      this.raster.draw(screen);
      this.osdView.clear();
    } else if (screen) {
      this.osdView.layout = this.system;
      this.osdView.draw(screen);
    } else {
      this.osdView.clear();
    }
  }

  dispose(): void {
    this.setOn(false);
    for (const stop of this.stops) stop();
    this.stops.length = 0;
    this.root.remove();
    this.analog = null;
  }

  // -- inside --------------------------------------------------------------------------------

  /** The analog chain the options ask for: by the device, the light one where the pointer is a
   * finger (a phone, a tablet). */
  private wanted(): Chain {
    const asked = prefs().video.analog;
    if (asked !== "auto") return asked;
    return window.matchMedia?.("(pointer: coarse)").matches ? "light" : "full";
  }

  /** The chain asked for, made when first needed and again when the choice changes; the other
   * where that one cannot run (the light one needs less of WebGL2). */
  private ensureAnalog(): AnalogFeed | AnalogLight | null {
    const asked = this.wanted();
    if (this.chainAsked === asked) return this.analog;
    this.chainAsked = asked;
    if (this.analog) {
      // A chain keeps its canvas' context: a new canvas for the next.
      const old = this.picture;
      this.picture = pictureCanvas();
      this.picture.hidden = old.hidden;
      old.replaceWith(this.picture);
      old.getContext("webgl2")?.getExtension("WEBGL_lose_context")?.loseContext();
      this.analog = null;
    }
    const order: Chain[] = asked === "light" ? ["light", "full"] : ["full", "light"];
    for (const chain of order) {
      try {
        const made =
          chain === "light" ? new AnalogLight(this.picture) : new AnalogFeed(this.picture);
        this.analog = made;
        this.chain = chain;
        if (made.ok) break;
      } catch (err) {
        console.error(`analog feed (${chain}) off:`, err);
      }
    }
    return this.analog;
  }

  private liveOsd(now: number): OsdMsg | null {
    if (!prefs().video.osd) return null;
    const screen = osd();
    return screen && now - this.osdAt < OSD_STALE_MS ? screen : null;
  }

  /** The analog picture, from the world Cesium has just drawn. */
  private afterRender(): void {
    if (!this.on || this.system !== "analog") return;
    const analog = this.analog;
    if (!analog?.ok) return;
    if (this.cnr !== null) analog.cnrDb = this.cnr;
    analog.frame(this.world.scene.canvas, this.osdIn ? this.raster : null, performance.now());
  }

  /** The link's carrier to noise, dB: free space at the VTX power, the dipoles' gains, fading. */
  private linkCnr(
    now: number,
    pose: Pose,
    pilot: { lat: number; lon: number; alt: number },
  ): number {
    const [n, e] = neFrom(pilot.lat, pilot.lon, pose.lat, pose.lon);
    const d: [number, number, number] = [e, n, pose.alt - pilot.alt];
    const { range, txGainDbi, rxGainDbi } = linkGeometry(d, bodyAxesEnu(pose.q).up);
    const dt = this.fade.at ? (now - this.fade.at) / 1000 : 0;
    this.fade = { db: fadeStep(this.fade.db, dt, gauss()), at: now };
    return linkCnrDb(range, prefs().video.vtxMw, txGainDbi, rxGainDbi) + this.fade.db;
  }

  /** Crop the world to the picture's frame in the host, the feed over it. */
  private layout(): void {
    const analog = this.system === "analog" && (this.ensureAnalog()?.ok ?? false);
    const area = this.host.getBoundingClientRect();
    const W = area.width || window.innerWidth;
    const H = area.height || window.innerHeight;
    const f = fitFrame(W, H, this.system === "analog" ? ANALOG_ASPECT : DIGITAL_ASPECT);
    const box: Box = { ...f, left: f.left + area.left, top: f.top + area.top };
    const container = this.world.viewer.container as HTMLElement;
    const viewer = this.world.viewer;
    if (!this.saved) {
      const style: Record<string, string> = {};
      for (const k of ["left", "top", "right", "bottom", "width", "height"]) {
        style[k] = container.style.getPropertyValue(k);
      }
      this.saved = { style, scale: viewer.resolutionScale };
    }
    const px = (v: number): string => `${Math.round(v)}px`;
    const place = {
      left: px(box.left),
      top: px(box.top),
      width: px(box.width),
      height: px(box.height),
    };
    Object.assign(container.style, { ...place, right: "auto", bottom: "auto" });
    Object.assign(this.root.style, place);
    this.picture.hidden = !analog;
    const dpr = window.devicePixelRatio || 1;
    viewer.resolutionScale = analog
      ? Math.min(this.saved.scale, FEED_RENDER_WIDTH[this.chain] / Math.max(box.width * dpr, 1))
      : this.saved.scale;
    viewer.resize();
  }

  /** The world back to the whole window. */
  private restore(): void {
    const saved = this.saved;
    if (!saved) return;
    this.saved = null;
    const container = this.world.viewer.container as HTMLElement;
    for (const [k, v] of Object.entries(saved.style)) {
      if (v) container.style.setProperty(k, v);
      else container.style.removeProperty(k);
    }
    this.world.viewer.resolutionScale = saved.scale;
    this.world.viewer.resize();
  }
}
