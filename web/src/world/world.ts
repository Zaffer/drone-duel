// The Cesium viewer and its ground: a globe of Cesium World Terrain + Bing imagery + OSM
// buildings when a Cesium ion token is set, else Esri World Imagery on Esri World Elevation (no
// key needed); and, while flying, Google Photorealistic 3D Tiles over it when a Google key is
// set. The menus' backdrop stays on the globe: Google bills each 3D Tiles session.

import * as C from "cesium";
import "cesium/Build/Cesium/Widgets/widgets.css";
import type { MapKeys } from "../net/protocol.ts";
import { drawEachFrame } from "../streams/time.ts";
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

export type GroundSource = "google" | "ion" | "esri";

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
  /** The globe's screen-space error in the menus, kept while the backstop stands in. */
  private globeSse: number | null = null;
  /** Whether the free mouse camera is kept above Google's tiles. Cesium does it by casting a ray
   * through the tiles' meshes under the camera every frame, read back from the GPU (Scene's
   * getGlobeHeight): a fifth of the page's time measured (2026-11-04), so only while the mouse
   * moves the camera; the pilot views place it themselves. */
  private collide = true;

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
    drawEachFrame(() => {
      try {
        viewer.resize();
        viewer.render();
      } catch (err) {
        // As Cesium's own loop does: say so and stop, rather than throw every frame.
        drawEachFrame(null);
        console.error("rendering stopped", err);
        viewer.cesiumWidget.showErrorPanel("Rendering has stopped.", undefined, String(err));
      }
    });
    this.scene = viewer.scene;
    viewer.clock.clockStep = C.ClockStep.SYSTEM_CLOCK;
    viewer.scene.globe.depthTestAgainstTerrain = true;
    viewer.scene.globe.baseColor = C.Color.fromCssColorString("#151724");
    viewer.scene.postProcessStages.fxaa.enabled = true;
    if (viewer.scene.skyAtmosphere) viewer.scene.skyAtmosphere.show = true;
    viewer.scene.fog.enabled = true;
    viewer.camera.frustum.near = 0.05;
    for (const credit of MODEL_CREDITS) viewer.creditDisplay.addStaticCredit(new C.Credit(credit));
    viewer.cesiumWidget.screenSpaceEventHandler.removeInputAction(
      C.ScreenSpaceEventType.LEFT_DOUBLE_CLICK,
    );
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
      if (this.globeSse !== null) this.scene.globe.maximumScreenSpaceError = this.globeSse;
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
      this.globeSse ??= this.scene.globe.maximumScreenSpaceError;
      this.scene.terrainProvider = this.backstop;
      this.scene.globe.maximumScreenSpaceError = BACKSTOP_SSE;
      this.source = "google";
    });
    return this.ready;
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

  /** Height of the drawn ground at a point, m above the ellipsoid, from the most detailed tiles
   * there (coarser ones, drawn while those load, can be kilometres out). `exclude`: primitives
   * that are not ground (vehicles). Undefined when nothing is there yet. */
  async groundHeight(
    lat: number,
    lon: number,
    exclude: object[] = [],
  ): Promise<number | undefined> {
    await this.ready;
    const carto = C.Cartographic.fromDegrees(lon, lat);
    try {
      if (this.groundIsTiles) {
        const [hit] = await this.scene.sampleHeightMostDetailed([carto], exclude);
        // Through a slit between tiles, down to the backstop: no ground there, as before it.
        return hit?.height !== undefined && hit.height > BACKSTOP_M + 1 ? hit.height : undefined;
      }
      const provider = this.viewer.terrainProvider;
      if (provider instanceof C.EllipsoidTerrainProvider) return 0;
      const [hit] = await C.sampleTerrainMostDetailed(provider, [carto]);
      return hit?.height;
    } catch (err) {
      console.warn("ground height lookup failed", err);
      return undefined;
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
    const scene = this.scene;
    if (this.groundIsTiles) {
      const cart = scene.pickPositionSupported ? scene.pickPosition(windowPosition) : undefined;
      return cart ? C.Cartographic.fromCartesian(cart) : undefined;
    }
    const ray = this.viewer.camera.getPickRay(windowPosition);
    const cart = ray ? scene.globe.pick(ray, scene) : undefined;
    return cart ? C.Cartographic.fromCartesian(cart) : undefined;
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
