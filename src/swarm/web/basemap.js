// The Cesium viewer and its ground: Google Photorealistic 3D Tiles when a Google key is set,
// Cesium World Terrain + Bing imagery + OSM buildings when a Cesium ion token is set, and
// Esri World Imagery on Esri World Elevation otherwise (no key needed).

const C = Cesium;

const ESRI_IMAGERY =
  "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}";
const ESRI_TERRAIN =
  "https://elevation3d.arcgis.com/arcgis/rest/services/WorldElevation3D/Terrain3D/ImageServer";
const OSM_STREETS = "https://tile.openstreetmap.org/{z}/{x}/{y}.png";
const MODEL_CREDITS = [
  'Drone model: "FPV Drone" by PinaPipe (CC BY 4.0)',
  'Truck model: "Military Truck" by Raphael Escamilla (CC BY 4.0)',
];

/**
 * Mouse as Google Earth, Mapbox and ArcGIS have it: left drag pans, right or middle drag (or
 * Ctrl + left drag) orbits, wheel and trackpad pinch zoom, Shift + left drag looks around.
 * Cesium's own default zooms on right drag and orbits only on middle drag.
 */
function useMapMouse(viewer) {
  const { CameraEventType: E, KeyboardEventModifier: K } = C;
  const ctl = viewer.scene.screenSpaceCameraController;
  ctl.rotateEventTypes = E.LEFT_DRAG;
  ctl.tiltEventTypes = [E.RIGHT_DRAG, E.MIDDLE_DRAG, { eventType: E.LEFT_DRAG, modifier: K.CTRL }];
  // A trackpad pinch reaches the page as a Ctrl + wheel.
  ctl.zoomEventTypes = [E.WHEEL, E.PINCH, { eventType: E.WHEEL, modifier: K.CTRL }];
  ctl.lookEventTypes = { eventType: E.LEFT_DRAG, modifier: K.SHIFT };
  // The page decides what a double click does, not Cesium's track-the-entity default.
  viewer.cesiumWidget.screenSpaceEventHandler.removeInputAction(C.ScreenSpaceEventType.LEFT_DOUBLE_CLICK);
}

export class BaseMap {
  /** @param {HTMLElement} container */
  constructor(container, config) {
    C.Ion.defaultAccessToken = config.cesiumIonToken ?? "";
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
    });
    const viewer = this.viewer;
    viewer.clock.clockStep = C.ClockStep.SYSTEM_CLOCK;
    viewer.scene.globe.depthTestAgainstTerrain = true;
    viewer.scene.globe.baseColor = C.Color.fromCssColorString("#1b2229");
    viewer.scene.postProcessStages.fxaa.enabled = true;
    for (const credit of MODEL_CREDITS) viewer.creditDisplay.addStaticCredit(new C.Credit(credit));
    useMapMouse(viewer);
    this.tileset = null;
    this.imageryLayers = {};
    /** Why a configured source was not used, for the page to show. */
    this.fallbackReason = null;
    /** "google" | "ion" | "esri" */
    this.source = config.googleMapsApiKey ? "google" : config.cesiumIonToken ? "ion" : "esri";
    this.ready = this._load(config);
  }

  /** True when the ground is a 3D tileset rather than the globe surface. */
  get groundIsTiles() {
    return this.tileset !== null;
  }

  /** How ground-clamped graphics classify against what is drawn as ground. */
  get classification() {
    return this.groundIsTiles ? C.ClassificationType.CESIUM_3D_TILE : C.ClassificationType.TERRAIN;
  }

  /** Load the best configured source, falling back when a key is rejected. */
  async _load(config) {
    if (this.source === "google") {
      try {
        C.GoogleMaps.defaultApiKey = config.googleMapsApiKey;
        // No geocoder is used at all, which is within Google's terms for these tiles.
        this.tileset = await C.createGooglePhotorealistic3DTileset({
          onlyUsingWithGoogleGeocoder: true,
        });
        this.viewer.scene.primitives.add(this.tileset);
        this.viewer.scene.globe.show = false;
        return;
      } catch (err) {
        console.error("Google Photorealistic 3D Tiles failed", err);
        this.fallbackReason = "Google 3D Tiles failed: check GOOGLE_MAPS_API_KEY";
        this.source = config.cesiumIonToken ? "ion" : "esri";
      }
    }
    if (this.source === "ion") {
      try {
        // A rejected token fails here, before any layer is added.
        await C.IonResource.fromAssetId(1);
      } catch (err) {
        console.error("Cesium ion failed", err);
        this.fallbackReason = "Cesium ion failed: check CESIUM_ION_TOKEN";
        this.source = "esri";
      }
    }
    await this._loadGlobe();
  }

  async _loadGlobe() {
    const viewer = this.viewer;
    let terrain;
    if (this.source === "ion") {
      terrain = C.Terrain.fromWorldTerrain();
      this.imageryLayers.satellite = C.ImageryLayer.fromWorldImagery({});
      viewer.scene.primitives.add(await C.createOsmBuildingsAsync());
    } else {
      terrain = new C.Terrain(C.ArcGISTiledElevationTerrainProvider.fromUrl(ESRI_TERRAIN));
      this.imageryLayers.satellite = new C.ImageryLayer(
        new C.UrlTemplateImageryProvider({
          url: ESRI_IMAGERY,
          maximumLevel: 19,
          credit: "Imagery: Esri, Maxar, Earthstar Geographics, and the GIS User Community",
        }),
      );
    }
    this.imageryLayers.streets = new C.ImageryLayer(
      new C.UrlTemplateImageryProvider({
        url: OSM_STREETS,
        maximumLevel: 19,
        credit: "© OpenStreetMap contributors",
      }),
      { show: false },
    );
    viewer.imageryLayers.add(this.imageryLayers.satellite);
    viewer.imageryLayers.add(this.imageryLayers.streets);
    viewer.scene.setTerrain(terrain);
    await new Promise((resolve) => {
      if (terrain.ready) resolve();
      terrain.readyEvent.addEventListener(resolve);
      terrain.errorEvent.addEventListener((err) => {
        console.warn("terrain failed, using the ellipsoid", err);
        resolve();
      });
    });
  }

  /** Switch the imagery between "satellite" and "streets". No effect on 3D tiles. */
  setImagery(name) {
    for (const [key, layer] of Object.entries(this.imageryLayers)) layer.show = key === name;
  }

  /** Height of the drawn ground at a point, metres above the ellipsoid. */
  async groundHeight(lat, lon) {
    await this.ready;
    const carto = C.Cartographic.fromDegrees(lon, lat);
    try {
      if (this.groundIsTiles) {
        for (let attempt = 0; attempt < 20; attempt++) {
          const [hit] = await this.viewer.scene.sampleHeightMostDetailed([carto]);
          if (hit?.height !== undefined) return hit.height;
          await new Promise((r) => setTimeout(r, 500));
        }
        return 0;
      }
      const provider = this.viewer.terrainProvider;
      if (provider instanceof C.EllipsoidTerrainProvider) return 0;
      const [hit] = await C.sampleTerrainMostDetailed(provider, [carto]);
      return hit.height ?? 0;
    } catch (err) {
      console.warn("ground height lookup failed", err);
      return 0;
    }
  }

  /** WGS84 position under a screen point, or undefined off the ground. */
  pickGround(windowPosition) {
    const scene = this.viewer.scene;
    if (this.groundIsTiles) {
      const cart = scene.pickPositionSupported ? scene.pickPosition(windowPosition) : undefined;
      return cart ? C.Cartographic.fromCartesian(cart) : undefined;
    }
    const ray = this.viewer.camera.getPickRay(windowPosition);
    const cart = ray ? scene.globe.pick(ray, scene) : undefined;
    return cart ? C.Cartographic.fromCartesian(cart) : undefined;
  }
}
