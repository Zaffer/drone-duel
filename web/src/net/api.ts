// The server's REST API: content, config, the player's preferences, and quitting. The web
// version (DESIGN.md §18) has no server: its content and config are files beside the page
// (written by `duels web`: drone_duels.webdata), its preferences and the player's own map keys
// are kept by the browser.

import type { ConfigMsg, ContentMsg } from "./protocol.ts";
import { asset, WEB } from "./target.ts";

async function json<T>(path: string, init?: RequestInit): Promise<T> {
  const res = await fetch(path, init);
  if (!res.ok) throw new Error(`${path}: ${res.status} ${await res.text()}`);
  return (await res.json()) as T;
}

const server = {
  content: () => json<ContentMsg>("/api/content"),
  config: () => json<ConfigMsg>("/api/config"),
  prefs: () => json<Record<string, unknown>>("/api/prefs"),
  savePrefs: (prefs: Record<string, unknown>) =>
    json<Record<string, unknown>>("/api/prefs", {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(prefs),
    }),
  /** Stop the game: the server closes the session (the firmware) and exits. */
  quit: () => json<Record<string, never>>("/api/quit", { method: "POST" }),
};

const PREFS_KEY = "drone-duels.prefs";
const KEYS_KEY = "drone-duels.map-keys";

/** The player's own map keys (the web version): theirs alone, kept in this browser. */
export function ownMapKeys(): ConfigMsg["map_keys"] {
  try {
    const kept = JSON.parse(localStorage.getItem(KEYS_KEY) ?? "{}") as ConfigMsg["map_keys"];
    return {
      google_maps_api_key: kept.google_maps_api_key || null,
      cesium_ion_token: kept.cesium_ion_token || null,
    };
  } catch {
    return { google_maps_api_key: null, cesium_ion_token: null };
  }
}

/** Keep the player's own map keys (empty: forget them). The world reads them as it is built,
 * so the page is loaded again to use them. */
export function saveOwnMapKeys(keys: ConfigMsg["map_keys"]): void {
  localStorage.setItem(KEYS_KEY, JSON.stringify(keys));
}

function kept(): Record<string, unknown> {
  try {
    return JSON.parse(localStorage.getItem(PREFS_KEY) ?? "{}") as Record<string, unknown>;
  } catch {
    return {};
  }
}

const web: typeof server = {
  content: () => json<ContentMsg>(asset("data/content.json")),
  config: async () => ({
    ...(await json<ConfigMsg>(asset("data/config.json"))),
    map_keys: ownMapKeys(),
  }),
  prefs: async () => kept(),
  savePrefs: async (prefs) => {
    localStorage.setItem(PREFS_KEY, JSON.stringify(prefs));
    return prefs;
  },
  quit: async () => ({}),
};

export const api = WEB ? web : server;
