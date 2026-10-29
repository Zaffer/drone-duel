// The server's REST API: content, config and the player's preferences.

import type { ConfigMsg, ContentMsg } from "./protocol.ts";

async function json<T>(path: string, init?: RequestInit): Promise<T> {
  const res = await fetch(path, init);
  if (!res.ok) throw new Error(`${path}: ${res.status} ${await res.text()}`);
  return (await res.json()) as T;
}

export const api = {
  content: () => json<ContentMsg>("/api/content"),
  config: () => json<ConfigMsg>("/api/config"),
  prefs: () => json<Record<string, unknown>>("/api/prefs"),
  savePrefs: (prefs: Record<string, unknown>) =>
    json<Record<string, unknown>>("/api/prefs", {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(prefs),
    }),
};
