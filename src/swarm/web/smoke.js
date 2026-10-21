// A puff of white cornstarch dust where a drone strikes something: a burst of soft white
// discs that flies out, stops in the air within a second, then hangs, swells a little and
// fades.

const C = Cesium;

/** Seconds the puff keeps billowing after its first burst. */
const BILLOW_S = 1;
/** Seconds each particle lives, the puff's whole life after the billowing. */
const PARTICLE_LIFE_S = { min: 2.5, max: 4.5 };
const BURST = { min: 30, max: 40 };
/** Particles a second while billowing, after the burst. */
const BILLOW_RATE = 5;
/** Burst speed, metres a second. */
const SPEED_MS = { min: 2, max: 6 };
/** Air drag: the fraction of its speed a particle keeps after one second. */
const DRAG_KEEP = 0.05;
/** How fast the stopped cloud drifts up, metres a second each second: dust, not hot smoke. */
const RISE_MS2 = 0.15;
/** A particle's size when it is made, metres, and how many times that it grows to. */
const SIZE_M = 1.2;
const GROWTH = 3.5;

let disc = null;
/** A soft white disc, opaque in the middle and clear at the edge, as a data URL. */
function puffImage() {
  if (disc) return disc;
  const size = 64;
  const canvas = document.createElement("canvas");
  canvas.width = canvas.height = size;
  const g = canvas.getContext("2d");
  const gradient = g.createRadialGradient(size / 2, size / 2, 0, size / 2, size / 2, size / 2);
  gradient.addColorStop(0, "rgba(255,255,255,1)");
  gradient.addColorStop(0.5, "rgba(255,255,255,0.55)");
  gradient.addColorStop(1, "rgba(255,255,255,0)");
  g.fillStyle = gradient;
  g.fillRect(0, 0, size, size);
  disc = canvas.toDataURL();
  return disc;
}

/** Puff white dust at `position` (Cartesian3); it clears itself away when it has faded. */
export function poof(scene, position) {
  const up = C.Ellipsoid.WGS84.geodeticSurfaceNormal(position, new C.Cartesian3());
  const rise = new C.Cartesian3();
  const drag = Math.log(DRAG_KEEP);
  const system = scene.primitives.add(
    new C.ParticleSystem({
      image: puffImage(),
      modelMatrix: C.Transforms.eastNorthUpToFixedFrame(position),
      emitter: new C.SphereEmitter(0.6),
      bursts: [new C.ParticleBurst({ time: 0, minimum: BURST.min, maximum: BURST.max })],
      emissionRate: BILLOW_RATE,
      lifetime: BILLOW_S,
      loop: false,
      minimumSpeed: SPEED_MS.min,
      maximumSpeed: SPEED_MS.max,
      minimumParticleLife: PARTICLE_LIFE_S.min,
      maximumParticleLife: PARTICLE_LIFE_S.max,
      imageSize: new C.Cartesian2(SIZE_M, SIZE_M),
      sizeInMeters: true,
      startScale: 1,
      endScale: GROWTH,
      startColor: C.Color.WHITE.withAlpha(0.9),
      endColor: C.Color.WHITE.withAlpha(0),
      // Velocities are in world coordinates.
      updateCallback: (particle, dt) => {
        C.Cartesian3.multiplyByScalar(particle.velocity, Math.exp(drag * dt), particle.velocity);
        C.Cartesian3.multiplyByScalar(up, RISE_MS2 * dt, rise);
        C.Cartesian3.add(particle.velocity, rise, particle.velocity);
      },
    }),
  );
  // The system's complete event comes when it stops billowing, its particles still alive.
  setTimeout(() => scene.primitives.remove(system), (BILLOW_S + PARTICLE_LIFE_S.max + 1) * 1000);
}
