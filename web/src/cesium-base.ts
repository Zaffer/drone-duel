// Cesium finds its workers and assets here (served by the server from its npm package; copied
// beside the web version's page by its build). This module runs before Cesium's, being
// imported first by main.ts.
import { asset, WEB } from "./net/target.ts";

(window as unknown as { CESIUM_BASE_URL: string }).CESIUM_BASE_URL = WEB
  ? asset("cesium/")
  : "/cesium/";
