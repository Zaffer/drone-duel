// The FPV camera feed (DESIGN.md §10): the world as the vehicle's camera sees it, cropped to the
// camera's picture with black round it, as goggles show it, and the firmware's own OSD.
//
// Analog: NTSC, 4:3. The world is drawn into the 4:3 frame at about FEED_RENDER_WIDTH px (the
// analog chain resamples it to 754 × 480: more is wasted), then sent through analog-vision's
// AnalogFeed, signal by signal: the OSD goes into the video before the transmitter, as the
// flight controller's OSD chip puts it, so the link spoils it too. The link's carrier to noise
// is free space from the pilot (setPilot: the goggles at the start pad) at the VTX power, each end
// a dipole standing up from its frame, with slow fading on top. Without WebGL2's float targets,
// the world plain with the OSD laid over it.
//
// Digital: 16:9, letterboxed in the window, no effect, the HD OSD grid laid over the picture as
// DJI/HDZero/Walksnail goggles draw it.
//
// The rig (views/rigs.ts) sets the camera's field of view; Cesium takes it across the wider
// side, which is the picture's width here. The feed reads the picture in Cesium's postRender,
// while its drawing buffer is still there.

import { effect } from "alien-signals";
import { AnalogFeed, linkCnrDb } from "analog-vision";
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
/** The analog chain takes the world at about this width, device px: more is wasted. */
const FEED_RENDER_WIDTH = 1100;
/** An OSD screen older than this is gone (the firmware stopped drawing). */
const OSD_STALE_MS = 1000;

/** A standard normal draw. */
const gauss = (): number =>
  Math.sqrt(-2 * Math.log(1 - Math.random())) * Math.cos(2 * Math.PI * Math.random());

export class CameraFeed {
  private readonly root: HTMLDivElement;
  private readonly picture: HTMLCanvasElement;
  private readonly osdCanvas: HTMLCanvasElement;
  private readonly osdView: OsdView;
  private readonly raster = new OsdRaster();
  private analog: AnalogFeed | null = null;
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
    this.picture = document.createElement("canvas");
    this.picture.className = "cf-picture";
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

  /** Whether the analog chain runs here (WebGL2 with float targets); null until it is first
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

  private ensureAnalog(): AnalogFeed | null {
    if (!this.analog) {
      try {
        this.analog = new AnalogFeed(this.picture);
      } catch (err) {
        console.error("analog feed off:", err);
        return null;
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
      ? Math.min(this.saved.scale, FEED_RENDER_WIDTH / Math.max(box.width * dpr, 1))
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
