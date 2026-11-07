// Dust in the air (the old smoke.js): a burst of soft discs that flies out, stops in the air
// within a second, then hangs, swells a little and fades, and clears itself away.
//
// - `puff`: white cornstarch dust where a drone strikes the truck.
// - `crash`: a darker, heavier burst of earth and debris dust where a vehicle crashes: more of
//   it, flung harder, settling rather than rising.

import * as C from "cesium";
import { toFixed } from "../world/geo.ts";
import type { World } from "../world/world.ts";

interface Dust {
  color: C.Color;
  /** Seconds it keeps billowing after its first burst, and particles a second meanwhile. */
  billowS: number;
  billowRate: number;
  /** Seconds each particle lives. */
  lifeS: [number, number];
  burst: [number, number];
  /** Burst speed, m/s. */
  speedMs: [number, number];
  /** The fraction of its speed a particle keeps after one second. */
  dragKeep: number;
  /** Up (+) or down (-) drift, m/s each second. */
  riseMs2: number;
  /** A particle's size when made, m, and how many times that it grows to. */
  sizeM: number;
  growth: number;
  emitterM: number;
  startAlpha: number;
}

const PUFF: Dust = {
  color: C.Color.WHITE,
  billowS: 1,
  billowRate: 5,
  lifeS: [2.5, 4.5],
  burst: [30, 40],
  speedMs: [2, 6],
  dragKeep: 0.05,
  // Dust, not hot smoke: it drifts up slowly.
  riseMs2: 0.15,
  sizeM: 1.2,
  growth: 3.5,
  emitterM: 0.6,
  startAlpha: 0.9,
};

const CRASH: Dust = {
  color: C.Color.fromCssColorString("#5B4D3F"),
  billowS: 1.5,
  billowRate: 10,
  lifeS: [3, 6],
  burst: [55, 75],
  speedMs: [3, 9],
  dragKeep: 0.08,
  riseMs2: -0.25,
  sizeM: 1.6,
  growth: 4,
  emitterM: 0.8,
  startAlpha: 0.85,
};

let disc: string | null = null;
/** A soft white disc, opaque in the middle and clear at the edge, as a data URL. */
function discImage(): string {
  if (disc) return disc;
  const size = 64;
  const canvas = document.createElement("canvas");
  canvas.width = canvas.height = size;
  const g = canvas.getContext("2d")!;
  const gradient = g.createRadialGradient(size / 2, size / 2, 0, size / 2, size / 2, size / 2);
  gradient.addColorStop(0, "rgba(255,255,255,1)");
  gradient.addColorStop(0.5, "rgba(255,255,255,0.55)");
  gradient.addColorStop(1, "rgba(255,255,255,0)");
  g.fillStyle = gradient;
  g.fillRect(0, 0, size, size);
  disc = canvas.toDataURL();
  return disc;
}

function dust(world: World, position: C.Cartesian3, d: Dust): void {
  const up = C.Ellipsoid.WGS84.geodeticSurfaceNormal(position, new C.Cartesian3());
  const rise = new C.Cartesian3();
  const drag = Math.log(d.dragKeep);
  const system = world.add(
    new C.ParticleSystem({
      image: discImage(),
      modelMatrix: C.Transforms.eastNorthUpToFixedFrame(position),
      emitter: new C.SphereEmitter(d.emitterM),
      bursts: [new C.ParticleBurst({ time: 0, minimum: d.burst[0], maximum: d.burst[1] })],
      emissionRate: d.billowRate,
      lifetime: d.billowS,
      loop: false,
      minimumSpeed: d.speedMs[0],
      maximumSpeed: d.speedMs[1],
      minimumParticleLife: d.lifeS[0],
      maximumParticleLife: d.lifeS[1],
      imageSize: new C.Cartesian2(d.sizeM, d.sizeM),
      sizeInMeters: true,
      startScale: 1,
      endScale: d.growth,
      startColor: d.color.withAlpha(d.startAlpha),
      endColor: d.color.withAlpha(0),
      // Velocities are in world coordinates.
      updateCallback: (particle: C.Particle, dt: number) => {
        C.Cartesian3.multiplyByScalar(particle.velocity, Math.exp(drag * dt), particle.velocity);
        C.Cartesian3.multiplyByScalar(up, d.riseMs2 * dt, rise);
        C.Cartesian3.add(particle.velocity, rise, particle.velocity);
      },
    }),
  ) as C.ParticleSystem;
  // The system's complete event comes when it stops billowing, its particles still alive.
  setTimeout(() => world.remove(system), (d.billowS + d.lifeS[1] + 1) * 1000);
}

/** White dust where a drone strikes something, at sim height `altAmsl` (m above sea level). */
export function puff(world: World, lat: number, lon: number, altAmsl: number): void {
  dust(world, toFixed(lat, lon, altAmsl), PUFF);
}

/** A dark burst of dust where a vehicle crashes, at sim height `altAmsl`. */
export function crash(world: World, lat: number, lon: number, altAmsl: number): void {
  dust(world, toFixed(lat, lon, altAmsl), CRASH);
}
