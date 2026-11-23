// A place of the player's own (the main menu's GPS point, ui/app.ts): typed in as a latitude
// and longitude, made into a location as content/locations/*.toml would hold it, with its
// ground's height left "auto" for the session to look up (drone_duels.world.elevation on the
// server; Open-Meteo from the worker in the web version). The session's start carries it whole
// (protocol.SessionMsg.place), since the content has no such id.

import type { Location } from "../net/protocol.ts";

/** Ids of the player's places start so; the content's never do (they are file names). */
export const GPS_PREFIX = "gps:";
/** Player 2's pad, this far south of player 1's, m (as the content's places have it). */
const PAD_SPACING_M = 10;
const M_PER_DEG_LAT = 111_320;

/** The wind at a place of the player's own until they set one (the Wind options tab). */
const CALM: Location["wind"] = {
  speed_ms: 2,
  from_deg: 270,
  turbulence: 1,
  eddy_scale: 1,
  gust_factor: 1,
  gust_every_s: 10,
};

const NUMBER = String.raw`[-+−]?\d+(?:\.\d+)?`;

/** A latitude and longitude from what the player typed or pasted: "45.3198, -75.7586" (Google
 * Maps' own copy), "45.3198 -75.7586", "45.3198° N, 75.7586° W", or a map's link holding
 * "@45.3198,-75.7586" or "q=45.3198,-75.7586". Null if it is none of those, or out of range. */
export function parseLatLon(text: string): [number, number] | null {
  const t = text.trim();
  const link = t.match(new RegExp(`[@=](${NUMBER}),\\s*(${NUMBER})`));
  const plain = t.match(
    new RegExp(`^(${NUMBER})\\s*°?\\s*([NS])?\\s*[,;\\s]\\s*(${NUMBER})\\s*°?\\s*([EW])?$`, "i"),
  );
  const num = (s: string) => Number(s.replace("−", "-").replace("+", ""));
  let lat: number;
  let lon: number;
  if (plain) {
    lat = num(plain[1]!) * (plain[2]?.toUpperCase() === "S" ? -1 : 1);
    lon = num(plain[3]!) * (plain[4]?.toUpperCase() === "W" ? -1 : 1);
  } else if (link) {
    lat = num(link[1]!);
    lon = num(link[2]!);
  } else {
    return null;
  }
  if (!(Math.abs(lat) <= 90 && Math.abs(lon) <= 180)) return null;
  return [lat, lon];
}

/** The location at a GPS point: both pads there, facing north, player 2's 10 m south. */
export function gpsPlace(lat: number, lon: number): Location {
  const at = (n: number): [number, number] => [Number(n.toFixed(6)), Number(lon.toFixed(6))];
  const origin = at(lat);
  const name = `${lat.toFixed(5)}, ${lon.toFixed(5)}`;
  return {
    id: `${GPS_PREFIX}${origin[0]},${origin[1]}`,
    name,
    blurb: "Your own GPS point.",
    origin,
    ground_alt_m: "auto",
    heading_deg: 0,
    pads: [
      { at: origin, heading_deg: 0 },
      { at: at(lat - PAD_SPACING_M / M_PER_DEG_LAT), heading_deg: 0 },
    ],
    wind: CALM,
  };
}

export const isGpsPlace = (id: string | null | undefined): boolean => !!id?.startsWith(GPS_PREFIX);
