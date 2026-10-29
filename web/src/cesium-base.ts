// Cesium finds its workers and assets here (served by the server from its npm package). This
// module runs before Cesium's, being imported first by main.ts.
(window as unknown as { CESIUM_BASE_URL: string }).CESIUM_BASE_URL = "/cesium/";
