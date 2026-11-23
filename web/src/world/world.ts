// The Cesium viewer and its ground: a globe of Cesium World Terrain + Bing imagery + OSM
// buildings when a Cesium ion token is set, else Esri World Imagery on Esri World Elevation (no
// key needed); and, while flying, Google Photorealistic 3D Tiles over it when a Google key is
// set. The menus' backdrop stays on the globe: Google bills each 3D Tiles session.

import * as C from "cesium";
import "cesium/Build/Cesium/Widgets/widgets.css";
import { loop } from "../engine/loop.ts";
import type { MapKeys } from "../net/protocol.ts";
import { BACKSTOP_M, BACKSTOP_SSE, backstopTerrain } from "./backstop.ts";

const ESRI_IMAGERY =
  "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}";
const ESRI_TERRAIN =
  "https://elevation3d.arcgis.com/arcgis/rest/services/WorldElevation3D/Terrain3D/ImageServer";
const MODEL_CREDITS = [
  'Drone model: "FPV Drone" by PinaPipe (CC BY 4.0)',
  'Truck model: "Military Truck" by Raphael Escamilla (CC BY 4.0)',
  'Operator model: "Quantum Modular Character Free Sample" (Fab)',
];

/** A touch screen's device pixels to a CSS pixel, at most: the rest are not drawn. */
const PHONE_PIXEL_RATIO = 1.5;

/** The ground's detail (Options → Graphics), a step a level of the map's tiles: each level down
 * is a tile twice as wide for the same ground, at half the sharpness. `globe` and `tiles` are the
 * screen pixels a tile may be out by before the next level is drawn (the globe's, and Google's
 * tiles'): doubled, everything is drawn a level coarser. `deepest` is the last level of Esri's
 * globe drawn at all. Esri's terrain ends at level 16 and its imagery at 19, which a level 18
 * tile already carries (below 69° of latitude), but Cesium goes on splitting tiles for as long
 * as the camera is near, past level 27 from a metre up, and every tile is 131 072 triangles
 * whatever it shows: flying low on a phone's screen, 6 million triangles a frame, half of them
 * the same ground drawn on smaller tiles (measured 2026-11-22). High stops at 18 and looks the
 * same; Medium and Low stop a level and two sooner. The colliders are built from the drawn
 * tiles' heights (engine/terrain.ts), so the ground driven on follows the ground seen. */
export type Detail = "low" | "medium" | "high";
const DETAIL: Record<Detail, { globe: number; deepest: number; tiles: number }> = {
  high: { globe: 2, deepest: 18, tiles: 16 },
  medium: { globe: 4, deepest: 17, tiles: 32 },
  low: { globe: 8, deepest: 16, tiles: 64 },
};

export type GroundSource = "google" | "ion" | "esri";

// Cesium 1.145 copies a 3D tileset's statistics into two scratch objects every frame, and with
// them, by object spread, the reference count of every texture loaded: thousands on Google's
// tiles, twice a frame, and nothing reads the copies' counts (only the tileset's own statistics
// keep them). The copy is left out here; everything else is copied as Cesium does.
type Stats = { texturesReferenceCounterById: Record<string, number> };
const TilesetStatistics = (
  C as unknown as { Cesium3DTilesetStatistics: { clone(s: Stats, r: Stats): void } }
).Cesium3DTilesetStatistics;
const cloneStatistics = TilesetStatistics.clone;
const NO_COUNTS: Record<string, number> = Object.freeze({}) as Record<string, number>;
TilesetStatistics.clone = (statistics: Stats, result: Stats): void => {
  const counts = statistics.texturesReferenceCounterById;
  statistics.texturesReferenceCounterById = NO_COUNTS;
  try {
    cloneStatistics(statistics, result);
  } finally {
    statistics.texturesReferenceCounterById = counts;
  }
};

export class World {
  readonly viewer: C.Viewer;
  readonly scene: C.Scene;
  /** The 3D tileset when it is the ground now (flying, with a Google key), else null. */
  tileset: C.Cesium3DTileset | null = null;
  /** The ground now. */
  source: GroundSource;
  /** The globe's source, under the tiles. */
  globeSource: "ion" | "esri";
  /** Why a configured source was not used, for the page to show. */
  fallbackReason: string | null = null;
  /** Settles once the ground now (globe or tiles) can be sampled. */
  ready: Promise<void>;
  private readonly globeReady: Promise<void>;
  private google: Promise<C.Cesium3DTileset | null> | null = null;
  private readonly googleKey: string | null;
  /** Counts useTiles calls, so a late tileset is not shown after a later "off". */
  private tilesAsked = 0;
  /** The globe's own terrain (the menus), kept while the backstop stands in for it. */
  private globeTerrain: C.TerrainProvider | null = null;
  /** The globe under Google's tiles, flying (backstop.ts). */
  private readonly backstop = backstopTerrain();
  private detail: Detail = "high";
  /** Whether the free mouse camera is kept above the ground. Cesium does it by casting a ray
   * through the meshes under the camera every frame (Scene's getGlobeHeight; through Google's
   * tiles, read back from the GPU): a fifth of the page's time measured (2026-11-04), so only
   * while the mouse moves the camera; the pilot views place it themselves. */
  private collide = true;
  /** Everything the page draws itself (world.add): never ground to a lookup here. */
  private readonly ours = new Set<object>();

  constructor(container: HTMLElement, keys: MapKeys) {
    C.Ion.defaultAccessToken = keys.cesium_ion_token ?? "";
    this.viewer = new C.Viewer(container, {
      baseLayer: false,
      animation: false,
      timeline: false,
      baseLayerPicker: false,
      geocoder: false,
      homeButton: false,
      sceneModePicker: false,
      navigationHelpButton: false,
      fullscreenButton: false,
      infoBox: false,
      selectionIndicator: false,
      shouldAnimate: true,
      useBrowserRecommendedResolution: false,
      // MSAA draws a hairline seam across the middle of the view in this build; FXAA instead.
      msaaSamples: 1,
      // A pilot's view needs no picking of entities, and the page owns double clicks.
      requestRenderMode: false,
      // The page's one frame loop draws, after the views have placed everything (§3.1).
      useDefaultRenderLoop: false,
    });
    const viewer = this.viewer;
    loop.draw(() => {
      try {
        viewer.resize();
        viewer.render();
      } catch (err) {
        // As Cesium's own loop does: say so and stop, rather than throw every frame.
        loop.draw(null);
        console.error("rendering stopped", err);
        viewer.cesiumWidget.showErrorPanel("Rendering has stopped.", undefined, String(err));
      }
    });
    this.scene = viewer.scene;
    // A phone has three device pixels to a CSS pixel and a GPU a fraction of a desktop's: drawn
    // at PHONE_PIXEL_RATIO at most (a judgement, not measured on a phone: about a megapixel on
    // today's screens).
    const ratio = window.devicePixelRatio || 1;
    if (window.matchMedia?.("(pointer: coarse)").matches && ratio > PHONE_PIXEL_RATIO) {
      viewer.resolutionScale = PHONE_PIXEL_RATIO / ratio;
    }
    viewer.clock.clockStep = C.ClockStep.SYSTEM_CLOCK;
    viewer.scene.globe.depthTestAgainstTerrain = true;
    viewer.scene.globe.baseColor = C.Color.fromCssColorString("#151724");
    viewer.scene.postProcessStages.fxaa.enabled = true;
    if (viewer.scene.skyAtmosphere) viewer.scene.skyAtmosphere.show = true;
    viewer.scene.fog.enabled = true;
    viewer.camera.frustum.near = 0.05;
    for (const credit of MODEL_CREDITS) viewer.creditDisplay.addStaticCredit(new C.Credit(credit));
    // Nothing in the game is picked by a click: the Viewer's own pick-and-select on every click
    // is a render of its own (it held the click that takes Shoot Down's mouse 240 ms, measured
    // 2026-11-07), and its double click flies to what it picks.
    for (const click of [
      C.ScreenSpaceEventType.LEFT_CLICK,
      C.ScreenSpaceEventType.LEFT_DOUBLE_CLICK,
    ]) {
      viewer.cesiumWidget.screenSpaceEventHandler.removeInputAction(click);
    }
    // Cesium looks up the ground under its camera every frame the camera has moved (Scene's
    // globe height, then a callback for when finer tiles come): a ray through the terrain tile
    // there, and Esri's are 131 072 triangles each. It is what keeps the mouse's camera above
    // the ground and tells Cesium the camera is under it; the pilot views place the camera
    // themselves, so while they do it is not asked (a quarter of the frame on a phone-speed
    // thread, 1.4 ms of 5.2, measured 2026-11-22). Nothing of the page's is clamped to the
    // ground by Cesium (engine/ground.ts does that), so nothing else waits on these.
    const lookups = this.scene as unknown as {
      getHeight(...args: unknown[]): number | undefined;
      updateHeight(...args: unknown[]): () => void;
    };
    const getHeight = lookups.getHeight.bind(lookups);
    const updateHeight = lookups.updateHeight.bind(lookups);
    lookups.getHeight = (...args) => (this.collide ? getHeight(...args) : undefined);
    lookups.updateHeight = (...args) => (this.collide ? updateHeight(...args) : () => {});
    // The deepest level of Esri's globe (DETAIL): from there a tile says it has no error left
    // to split for, which is true, and is how Cesium is told a tile is good enough to draw.
    // Less than none, in fact: Cesium divides the error by the camera's distance from the tile,
    // which is 0 inside it (FPV on the ground), and 0 / 0 is not small enough for it.
    const tiles = (
      this.scene.globe as unknown as {
        _surface: { tileProvider: { getLevelMaximumGeometricError(level: number): number } };
      }
    )._surface.tileProvider;
    const errorAt = tiles.getLevelMaximumGeometricError.bind(tiles);
    tiles.getLevelMaximumGeometricError = (level) =>
      this.source === "esri" && level >= DETAIL[this.detail].deepest ? -1 : errorAt(level);
    this.googleKey = keys.google_maps_api_key ?? null;
    this.globeSource = keys.cesium_ion_token ? "ion" : "esri";
    this.source = this.globeSource;
    this.globeReady = this.load();
    this.ready = this.globeReady;
  }

  /** The Google 3D tiles as the ground (flying) or the globe (the menus). The tileset is loaded
   * the first time it is wanted and kept, hidden, after. No-op without a Google key. */
  useTiles(on: boolean): Promise<void> {
    if (!this.googleKey) return this.ready;
    const asked = ++this.tilesAsked;
    if (!on) {
      if (this.tileset) this.tileset.show = false;
      this.tileset = null;
      if (this.globeTerrain) this.scene.terrainProvider = this.globeTerrain;
      this.scene.globe.maximumScreenSpaceError = DETAIL[this.detail].globe;
      this.source = this.globeSource;
      this.ready = this.globeReady;
      return this.ready;
    }
    this.google ??= this.loadGoogle(this.googleKey);
    const loading = this.google;
    this.ready = Promise.all([this.globeReady, loading]).then(([, tiles]) => {
      // Back in the menus before the tiles came: leave them hidden.
      if (!tiles || asked !== this.tilesAsked) return;
      tiles.show = true;
      tiles.enableCollision = this.collide;
      this.tileset = tiles;
      this.globeTerrain ??= this.scene.terrainProvider;
      this.scene.terrainProvider = this.backstop;
      this.scene.globe.maximumScreenSpaceError = BACKSTOP_SSE;
      this.source = "google";
    });
    return this.ready;
  }

  /** How finely the ground is drawn (Options → Graphics). */
  setDetail(detail: Detail): void {
    this.detail = detail;
    // Under Google's tiles the globe is the backstop, at its own coarse setting.
    if (!this.tileset) this.scene.globe.maximumScreenSpaceError = DETAIL[detail].globe;
    void this.google?.then((tiles) => {
      if (tiles) tiles.maximumScreenSpaceError = DETAIL[this.detail].tiles;
    });
  }

  /** The sun at `hour` (0–24) of local solar time at longitude `lon`, today, and held there; or,
   * with null, the real time. Cesium lights the models and the sky by the clock's sun. */
  setTimeOfDay(hour: number | null, lon: number): void {
    const clock = this.viewer.clock;
    if (hour === null) {
      clock.clockStep = C.ClockStep.SYSTEM_CLOCK;
      return;
    }
    const now = new Date();
    const midnight = Date.UTC(now.getUTCFullYear(), now.getUTCMonth(), now.getUTCDate());
    clock.clockStep = C.ClockStep.TICK_DEPENDENT;
    clock.multiplier = 0;
    clock.currentTime = C.JulianDate.fromDate(new Date(midnight + (hour - lon / 15) * 3_600_000));
  }

  /** True with a Google key: flights draw the ground in 3D tiles. */
  get hasTiles(): boolean {
    return this.googleKey !== null;
  }

  /** True when the ground is a 3D tileset rather than the globe surface. */
  get groundIsTiles(): boolean {
    return this.tileset !== null;
  }

  /** The free mouse camera (map, orbit views) on or off (pilot views drive the camera). */
  set mouseCamera(on: boolean) {
    const ctl = this.scene.screenSpaceCameraController;
    ctl.enableInputs = on;
    this.collide = on;
    if (this.tileset) this.tileset.enableCollision = on;
  }

  private async loadGoogle(key: string): Promise<C.Cesium3DTileset | null> {
    try {
      C.GoogleMaps.defaultApiKey = key;
      // No geocoder is used at all, which is within Google's terms for these tiles.
      const tiles = await C.createGooglePhotorealistic3DTileset({
        onlyUsingWithGoogleGeocoder: true,
      });
      tiles.show = false;
      tiles.maximumScreenSpaceError = DETAIL[this.detail].tiles;
      this.scene.primitives.add(tiles);
      return tiles;
    } catch (err) {
      console.error("Google Photorealistic 3D Tiles failed", err);
      this.fallbackReason = "Google 3D Tiles failed: check GOOGLE_MAPS_API_KEY";
      return null;
    }
  }

  private async load(): Promise<void> {
    if (this.source === "ion") {
      try {
        await C.IonResource.fromAssetId(1);
      } catch (err) {
        console.error("Cesium ion failed", err);
        this.fallbackReason = "Cesium ion failed: check CESIUM_ION_TOKEN";
        this.globeSource = "esri";
        this.source = "esri";
      }
    }
    await this.loadGlobe();
  }

  private async loadGlobe(): Promise<void> {
    let terrain: C.Terrain;
    if (this.source === "ion") {
      terrain = C.Terrain.fromWorldTerrain();
      this.viewer.imageryLayers.add(C.ImageryLayer.fromWorldImagery({}));
      this.scene.primitives.add(await C.createOsmBuildingsAsync());
    } else {
      terrain = new C.Terrain(C.ArcGISTiledElevationTerrainProvider.fromUrl(ESRI_TERRAIN));
      this.viewer.imageryLayers.add(
        new C.ImageryLayer(
          new C.UrlTemplateImageryProvider({
            url: ESRI_IMAGERY,
            maximumLevel: 19,
            credit: "Imagery: Esri, Maxar, Earthstar Geographics, and the GIS User Community",
          }),
        ),
      );
    }
    this.scene.setTerrain(terrain);
    await new Promise<void>((resolve) => {
      if (terrain.ready) resolve();
      terrain.readyEvent.addEventListener(() => resolve());
      terrain.errorEvent.addEventListener((err) => {
        console.warn("terrain failed, using the ellipsoid", err);
        resolve();
      });
    });
  }

  /** Draw `p` (a model, a primitive collection): the page's own, so never ground to a lookup.
   * Everything the page draws comes through here (a test holds it). */
  add<T extends object>(p: T): T {
    this.scene.primitives.add(p);
    this.ours.add(p);
    return p;
  }

  /** Stop drawing `p` (destroying it, as Cesium's remove does). */
  remove(p: object): boolean {
    this.ours.delete(p);
    return this.scene.primitives.remove(p);
  }

  // The lookups below are Cesium's: each a render of its own, and the ones that wait load the
  // finest tiles first, so they are for where the ground service (engine/ground.ts) has nothing
  // built yet: a pad before anything is drawn there, the map view's far-off vehicle. Each is
  // metered in the frame loop ("map lookup", "screen pick").

  /** Height of the drawn ground at a point, m above the ellipsoid, from the most detailed tiles
   * there (coarser ones, drawn while those load, can be kilometres out); never what the page
   * draws itself. Undefined when nothing is there yet. */
  async groundHeight(lat: number, lon: number): Promise<number | undefined> {
    const [h] = await this.groundHeights([[lat, lon]]);
    return h;
  }

  /** The ground heights at several points (lat, lon), in one look. */
  async groundHeights(points: [number, number][]): Promise<(number | undefined)[]> {
    await this.ready;
    const cartos = points.map(([lat, lon]) => C.Cartographic.fromDegrees(lon, lat));
    try {
      if (this.groundIsTiles) {
        const t0 = performance.now();
        const asked = this.scene.sampleHeightMostDetailed(cartos, [...this.ours]);
        loop.meter("map lookup", performance.now() - t0);
        const hits = await asked;
        // Through a slit between tiles, down to the backstop: no ground there, as before it.
        return hits.map((hit) =>
          hit?.height !== undefined && hit.height > BACKSTOP_M + 1 ? hit.height : undefined,
        );
      }
      const provider = this.viewer.terrainProvider;
      if (provider instanceof C.EllipsoidTerrainProvider) return points.map(() => 0);
      const hits = await C.sampleTerrainMostDetailed(provider, cartos);
      return hits.map((hit) => hit?.height);
    } catch (err) {
      console.warn("ground height lookup failed", err);
      return points.map(() => undefined);
    }
  }

  /** The ground height at a point, retrying while its tiles load (up to `tries` × 0.5 s). */
  async groundHeightSettled(lat: number, lon: number, tries = 20): Promise<number> {
    for (let i = 0; i < tries; i++) {
      const h = await this.groundHeight(lat, lon);
      if (h !== undefined) return h;
      await new Promise((r) => setTimeout(r, 500));
    }
    return 0;
  }

  /** WGS84 position under a screen point, or undefined off the ground. */
  pickGround(windowPosition: C.Cartesian2): C.Cartographic | undefined {
    const cart = this.pickPoint(windowPosition);
    return cart ? C.Cartographic.fromCartesian(cart) : undefined;
  }

  /** The earth-fixed point drawn under a screen point (anything drawn), or undefined. */
  pickPoint(windowPosition: C.Cartesian2): C.Cartesian3 | undefined {
    const scene = this.scene;
    return loop.timed("screen pick", () => {
      if (this.groundIsTiles) {
        return scene.pickPositionSupported ? scene.pickPosition(windowPosition) : undefined;
      }
      const ray = this.viewer.camera.getPickRay(windowPosition);
      return ray ? scene.globe.pick(ray, scene) : undefined;
    });
  }

  /** Fly the camera to look at a place from above (menus, map view). */
  overview(lat: number, lon: number, alt: number, range = 600, duration = 0): void {
    const target = C.Cartesian3.fromDegrees(lon, lat, alt);
    this.viewer.camera.flyToBoundingSphere(new C.BoundingSphere(target, 1), {
      offset: new C.HeadingPitchRange(C.Math.toRadians(20), C.Math.toRadians(-35), range),
      duration,
    });
  }
}
