// The Cesium viewer and its ground: Google Photorealistic 3D Tiles when a Google key is set,
// Cesium World Terrain + Bing imagery + OSM buildings when a Cesium ion token is set, and Esri
// World Imagery on Esri World Elevation otherwise (no key needed).

import * as C from "cesium";
import "cesium/Build/Cesium/Widgets/widgets.css";
import type { MapKeys } from "../net/protocol.ts";

const ESRI_IMAGERY =
  "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}";
const ESRI_TERRAIN =
  "https://elevation3d.arcgis.com/arcgis/rest/services/WorldElevation3D/Terrain3D/ImageServer";
const MODEL_CREDITS = [
  'Drone model: "FPV Drone" by PinaPipe (CC BY 4.0)',
  'Truck model: "Military Truck" by Raphael Escamilla (CC BY 4.0)',
];

export type GroundSource = "google" | "ion" | "esri";

export class World {
  readonly viewer: C.Viewer;
  readonly scene: C.Scene;
  tileset: C.Cesium3DTileset | null = null;
  source: GroundSource;
  /** Why a configured source was not used, for the page to show. */
  fallbackReason: string | null = null;
  readonly ready: Promise<void>;

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
    });
    const viewer = this.viewer;
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
    this.source = keys.google_maps_api_key ? "google" : keys.cesium_ion_token ? "ion" : "esri";
    this.ready = this.load(keys);
  }

  /** True when the ground is a 3D tileset rather than the globe surface. */
  get groundIsTiles(): boolean {
    return this.tileset !== null;
  }

  /** The free mouse camera (map, orbit views) on or off (pilot views drive the camera). */
  set mouseCamera(on: boolean) {
    const ctl = this.scene.screenSpaceCameraController;
    ctl.enableInputs = on;
  }

  private async load(keys: MapKeys): Promise<void> {
    if (this.source === "google") {
      try {
        C.GoogleMaps.defaultApiKey = keys.google_maps_api_key ?? undefined;
        // No geocoder is used at all, which is within Google's terms for these tiles.
        this.tileset = await C.createGooglePhotorealistic3DTileset({
          onlyUsingWithGoogleGeocoder: true,
        });
        this.scene.primitives.add(this.tileset);
        this.scene.globe.show = false;
        return;
      } catch (err) {
        console.error("Google Photorealistic 3D Tiles failed", err);
        this.fallbackReason = "Google 3D Tiles failed: check GOOGLE_MAPS_API_KEY";
        this.source = keys.cesium_ion_token ? "ion" : "esri";
      }
    }
    if (this.source === "ion") {
      try {
        await C.IonResource.fromAssetId(1);
      } catch (err) {
        console.error("Cesium ion failed", err);
        this.fallbackReason = "Cesium ion failed: check CESIUM_ION_TOKEN";
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
        return hit?.height;
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
