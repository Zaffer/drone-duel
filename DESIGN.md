# Drone Duels: design

Status: built on branch `v2` (2026-10-29). Where the build found something the draft got wrong, the
text below says what was done instead.

Drone Duels is a flight simulator game and practice sim. Every aircraft flies on its real flight
controller firmware in SITL (ArduPilot, INAV or Betaflight), on the best physics we can
model, over real places on Earth drawn from Google Photorealistic 3D Tiles. You can fly around
on your own, fight a 1v1 duel, or race a friend through gates. You can switch vehicle at any
time, see the firmware's real OSD through an analog or digital video link, and change the wind
as you fly. It is also where we practise new airframes and ArduPilot missions before flying them
for real.

## 1. Goals and non-goals

**Goals for v1**

- The real firmware flies every aircraft: ArduCopter 4.7 SITL or Betaflight 2026.6 SITL for the
  5" quad (two vehicles, one airframe) and INAV 9.1 SITL for the TBS Chupito. Nothing we write
  stands in for the flight controller.
- Physics we own and can defend: `wingsim` (strips over a vortex lattice, MIL-F-8785C wind) for
  the Chupito, and SkyFlow-Dynamics' `racer5in` plant for the quad. One wind model drives both.
- Modes: **Free Fly**, **Duel** (a drone or the Chupito against the truck) and **Race** (live
  head to head through gates). Modes are plugins, so more can follow.
- A mode (the menu's scenario), a place, then a vehicle, picked from a menu: Free Fly at
  Toronto's Billy Bishop, or a Race at AreaXO (the race runs the place's course).
- Vehicle select: the 5" quad, the Chupito or the truck, in the menu or in flight (the pause
  menu). Switching respawns you at the place's start.
- Views: FPV (the camera feed with the real OSD), chase near and far, a line-of-sight spotter,
  Map/COP, a free orbit camera, and a minimap PiP with the aviation dials.
- Video: analog (analog-vision's NTSC chain, SD OSD) or digital (clean 16:9, HD OSD).
- Controllers: Chrome's Gamepad API, with profiles for the BETAFPV LiteRadio 2, Radiomaster TX15
  and Radiomaster T8L, and a calibration wizard for any other.
- Wind and gusts, adjustable live.
- Dynamics visualisation for the Chupito, as wingdynamics.js does today.
- ArduPilot practice: mavlink-router with a QGroundControl port. The game draws the vehicle's
  uploaded mission and fence.

**Not in v1** (see ROADMAP.md): the drone swarm and its missions, Betaflight, ArduPilot Rover on
the truck, an in-game course editor or mission planner, dynamics visualisation for the quad and
the truck, Windows/WSL and macOS, and flying real hardware.

## 2. Principles

1. **Firmware is real, physics is ours.** Every aircraft is a firmware process plus a physics
   process joined by that firmware's own simulator protocol. The game never commands the
   aircraft except through its RC input or MAVLink, as a pilot or a GCS would.
2. **Two kinds of truth, kept apart.**
   - *What the pilot sees from the aircraft:* the camera, drawn from the physics' true pose, and
     the OSD, drawn from the firmware's own MSP DisplayPort screen.
   - *What a ground station sees:* the map, the dials and the plans, all from telemetry (MAVLink,
     or MSP for INAV).

   This is the rule today's code follows ("the map, the FPV view and the dials stay on
   MAVLink"). Keeping to it lets the Map/COP view later run against real aircraft unchanged.
3. **Content is data, not code.** Locations, vehicles, courses and controller
   profiles are TOML files in `content/`. Each loads into a typed schema that rejects unknown and
   missing keys, as `swarm.settings` does today. CLI flags only override a file.
4. **Physics stays lean and borrowed models stay as they are.** `drone_duels.physics.core` and
   `physics.wing` import only themselves and the standard library, as `swarm.wingsim` does
   today. The quad uses SkyFlow-Dynamics' own JAX backend unchanged, on the CPU (§6.2), so it
   follows upstream and its golden tests with no copy to drift. JAX loads only in the quad's
   physics process.
5. **One process to launch.** `uv run duels` starts the server and opens Chrome. Everything else
   (firmware, physics, router) is started and stopped by the session, from the menu.
6. **Chrome only.** We use what Chrome ships: WebGPU/WebGL2, the Gamepad API, native Observables.
   No polyfills.

## 3. Architecture

```
 Chrome ── one WebSocket (/ws) + static app ──────────────────────────────────┐
   │  app shell, menus, HUD          (alien-signals: state)                  │
   │  views, video, OSD, dials       (Observables: pose/osd/telemetry/rc)    │
   │  frame loop, ground, truck      (engine/: Rapier, in the page, §3.2)    │
   │  duel / race peer link          (Trystero over Nostr ⇄ other player)   │
   └─────────────────────────────────────────────────────────────────────────┘
                                   │
 duels server (Python, FastAPI on uvicorn, pydantic)
   ├─ Session          mode-agnostic: spawn / switch / reset / destroy a vehicle at a location
   ├─ ContentStore     content/*.toml → typed specs
   ├─ Prefs            ~/.config/drone-duels/prefs.json (controls, video, graphics, audio)
   ├─ Pilot            the radio read through Linux (evdev), flown straight to the firmware
   ├─ RcSink           sticks (the server's radio, or the page's) → the firmware's RC input,
   │                   with failsafe on silence
   ├─ TelemetryClient  MAVLink via mavlink-router (sysid 254) │ MSP for INAV, Betaflight
   ├─ OsdClient        MSP DisplayPort from the firmware → grid screens
   └─ per vehicle slot:
        Firmware process  ⇄  bridge protocol  ⇄  Physics process
        ArduCopter SITL   ⇄  ArduPilot JSON   ⇄  quad (SkyFlow-Dynamics plant)
        Betaflight SITL   ⇄  its sim UDP      ⇄  quad (the same plant)
        INAV SITL         ⇄  X-Plane UDP      ⇄  wing (wingsim)
        (truck: no firmware; physics runs in the page)

 mavlink-router: SITL serial0 → UDP 14550 QGroundControl (255) · UDP 14570 + slot duels (254)
```

**Processes.** Each physics process is a child of the server, one per vehicle slot, so a slow
render or a GC pause in the server never stalls the lockstep with the firmware. It talks to the
server over a local UDP control socket: wind, ground height and resets in; true pose and
dynamics out. v1 uses slot 0 only. Slots exist so the swarm (and two local players for testing)
can come back without redesign.

**Ports** live in one module, `drone_duels.ports`, as a block per slot: firmware MAVLink/MSP, CLI,
OSD, RC in, physics bridge and physics control. Nothing else hard-codes a port. Today ArduPilot's
SITL serial0 and INAV's CLI both default to 5760; the port map ends that clash. Betaflight's
SITL fixes its own (TCP 5760 up, UDP 9001-9004: ArduPilot's), so it is built with
`scripts/betaflight-sitl.patch`, which makes them arguments.

**Time.** ArduPilot's JSON backend and INAV's X-Plane link both run in lockstep: the physics
steps when the firmware sends its outputs, so the sim never drifts from the firmware's clock.
Betaflight's SITL does not wait: its clock runs at the rate the physics' timestamps advance
against the wall clock. So under Betaflight the physics paces itself, 1000 frames a second in
real time, and Betaflight follows (§6.2).
The physics streams the true pose to the server at 120 Hz, each stamped with its sim time `t`.

### 3.1 Timing and latency

Measured (2026-11-04, the table below): the physics kept to the wall clock within 2 ms and sent
a pose every 8.4 ms, but the page handled 63% of them in clumps between frames, up to 100 ms
late, because drawing a frame holds the page's one thread (45 fps). The page stamped each pose
when it handled it and carried it on along its velocity from there, so the aircraft surged,
stalled and stepped back (rubber banding). And the sticks went through that same busy thread
(the server's radio to the page and back): from the page alone, 100 ms from a stick step to the
elevons moving. Hence four paths, each with its own need:

| Path | Needs | How |
|---|---|---|
| Sticks → firmware | shortest, steady, never waiting on drawing | the server's radio flies directly (§12) |
| Your aircraft → screen | exact, no added delay | a timeline on sim time, sampled at "sim now" |
| The other player → screen | smooth over the internet's jitter | the same timeline, drawn a buffer behind (§7) |
| Ground station (COP) | what a GCS really sees | telemetry, as it comes (§2.2) |

**The pose timeline** (`web/src/vehicles/timeline.ts`) keeps the latest second of samples by
the time they carry, never by when the page handled them. The page learns how that clock maps
to its own from the quickest deliveries: the least of (arrival − `t`) over the last 2 s, which
the clumps cannot raise. The estimate moves at most 5% of real time (it slews), so a change in
it never shows as a jump; only when every sample has come 150 ms late for 250 ms (the physics
stalled, its clock slipped) does it jump to them, and one late clump (the page itself stalled)
does not. Sampled between two samples, the position follows the cubic through both with their
velocities and the attitude turns between them (slerp); past the newest, it goes on along the
velocity and body rates for up to 100 ms, then holds. Your own aircraft is sampled at "sim now",
with no delay added: it is carried on 6 ms at the median, 10 to 30 ms at the 95th percentile.

**One frame loop** (§3.2). Cesium's own render loop is off. Each animation frame runs every
system of the loop in its phase (the views place the camera and the vehicle from the
timeline), then draws, so a frame shows what it placed, not the frame before's.

**Measured** (2026-11-04, headless Chrome on the RTX 4090, the Chupito at 33 m/s in chase view,
the old code and the new flown by the same script):

| | before | after |
|---|---|---|
| Drawn speed, 1st / 50th / 99th percentile | 3 / 32 / 96 m/s | 30 / 33 / 35 m/s |
| Frames more than 10% off the true speed | 895 of 1129 | 2 of 1055 |
| Frames stepping back along the path | 20 | 2 |
| Stick to elevons, through INAV (median) | 100 ms, sticks from the page, elevons as the page saw them | 30 ms, the server's radio, elevons as the server sent them: under 0.3 ms to the firmware, the rest INAV's loop |

A gamepad or the keyboard still flies through the page (100 ms as above); the radio does not.
In the game, the performance panel (P) shows the frame rate, the frame time, how far the pose
is carried on and where the sticks come from. `web/test/timeline.test.ts` replays the poses as
Chrome got them (`web/test/fixtures/pose-arrivals.json`), `tests/test_pilot.py` times the
server's radio to the firmware, and `tests/test_session_chupito.py` a stick step to INAV's
elevons.

**The frame's cost** is Cesium choosing the ground's tiles, on the main thread, and it grows
with the window (a 3440×1440 window needs more tiles than 1600×900). Two things cost most of it
until 2026-11-04, measured at 3440×1400 in chase view:

- The globe drawn under Google's tiles (`world/backstop.ts`) had 2 heights per tile side.
  Cesium judges a heightmap's error by that, so it split the globe about five levels deeper than
  usual: 20 to 100 thousand tiles visited each frame, 5 to 20 fps. With 65, as ordinary
  heightmaps have, it visits a few hundred (`web/test/backstop.test.ts` holds it).
- Google's tileset comes with `enableCollision` on: every frame Cesium casts a ray through its
  meshes under the camera, read back from the GPU, to keep the camera above them. That is now on
  only while the mouse moves the camera (Map, orbit), which takes about a fifth of the main thread.

Uncapped, the frame is now 3.8 ms in FPV and 10.3 ms in chase at 3440×1400 (14 fps and 67 ms in
chase before).

**All in the browser** is the web version (§18): the same seams, input → sim → timeline →
renderer, with the firmware and the physics (one WebAssembly module) in a dedicated Worker on
a fixed step, so the sticks still never wait on drawing. The poses reach the same timeline as
posted messages, stamped with sim time as the server's are.

### 3.2 The page's engine

Built 2026-11-06 after Shoot Down's "somersaults" (a ground lookup per bullet step spiralling
into seconds-long frames, and a spring stepped a frame at a time running away on them). The
rule: what costs a render, what runs each frame, and what moves are each in one place, metered
there, and a test fences them in. In `web/src/engine/`, `input/actions.ts` and `world/world.ts`:

- **One frame loop** (`engine/loop.ts`). Each animation frame runs its systems in phases:
  `input` (the controls), `sim` (what moves by its own rules), `place` (the camera, the models,
  the HUD), then Cesium draws. Every system has a name and is timed, and the performance panel
  (P) lists the dearest. A fixed-rate system (`loop.fixed`) steps by real time whatever the frame
  rate (the physics at 60 Hz, the shooter's body at 240 Hz), and catches up at most 0.1 s a
  frame: a stall is dropped and counted, never spent, so a slow frame cannot make the next one
  slower. Pausing stops input, not time: the firmware cannot stop, so nothing in the world does.
- **Springs are exact** (`engine/spring.ts`): a damped spring is stepped by its closed-form
  solution, stable at any step; the recoil, the heartbeat and the knees are these. Exponential
  approaches use `approach(dt, τ)`, exact too.
- **One local frame per place** (`engine/frame.ts`): east-north-up metres at the pad, from the
  pad's map height; it converts to and from earth-fixed, map heights and sim heights. What moves
  or collides works in it; `geo.ts` keeps the small lat/lon offsets (a pose carried on).
- **One ground** (`engine/ground.ts`): the map drawn within 150 m of a focus (the vehicle, the
  shooter, the truck) is read back into Rapier trimeshes, dropped 400 m from every focus. Rays,
  segments and heights are a walk of Rapier's tree: microseconds, no render. Two builders:
  Google's 3D tiles, each drawn tile's own triangles following Cesium's level of detail
  (`engine/tiles.ts`); and on the free map, the globe's terrain, 64 m patches on a 2 m grid
  sampled from the drawn terrain tiles' own heights (`engine/terrain.ts`: Esri's tiles are
  131 072 triangles each, 40 ms to read and up to 630 ms for Rapier to take in; the globe's
  `getHeight` is milliseconds a point; its tiles' data, interpolated, a microsecond). Building is
  a few milliseconds a frame at most ("ground build"). It owns the page's one physics world and
  steps it ("physics"); the truck drives in it. Coarse tiles or terrain (metres out) are never
  built: past the built ground there is none to ask, and the rounds stop looking (§7.1).
- **Cesium's own lookups** (`sampleHeight*`, `pickPosition`, `drillPick`, `globe.pick`) are each
  a render, and the ones that wait load the finest tiles first. They are only in `world.ts`,
  metered ("map lookup", "screen pick"), for where nothing is built yet: a pad before anything
  is drawn there, the drone in the map view. Everything the page draws goes through
  `world.add`, so those lookups look through it themselves: no exclusion lists to forget (a
  tracer once "landed" the drone in mid-air).
- **Controls as tables** (`input/actions.ts`, §12): one listener for the keyboard, the locked
  mouse and a standard gamepad (read once a frame); each part of the game mounts a table of its
  actions, and the Inputs tab and the key strip are drawn from the same tables.
- **Fences** (`web/test/fence.test.ts`): Cesium's lookups only in `world.ts`; scene adds only
  through `world.add`; `requestAnimationFrame` only in the loop; no hand-cut spring substeps.
  `web/test/actions.test.ts` holds tables mounted together never to answer one press twice.
- **Measured** (`scripts/perf.py`, `scripts/check.sh --perf`): headless Chrome on the GPU at
  3440×1400, uncapped, plays Shoot Down (stand, walk, sprint, ten quick shots, three into the
  ground, which must stop there) and the truck (it must drive), on the free map (port 8610,
  slot 6: beside the game, and free) and on Google's tiles (8600), and fails a frame over 40 ms
  at the 95th percentile, any over 250 ms, a system over 6 ms at the 95th, or time dropped. Each
  step prints its longest frame's make-up from Chrome's long-animation-frame record (which
  scripts, how long the render).
  2026-11-06: Shoot Down ~110 fps walking; ten quick shots took the frame's own work from
  8.8 ms (rounds' ground picks) to 0.2 ms, and the longest frame from 150 ms to 19 ms (the
  tracer's shader kept, §7.1); the truck ~300 fps. Cesium's render is the rest of every frame.

Two of Cesium's costs are taken out in `world.ts`: the Viewer's pick-and-select on every left
click (a render of its own: the click that takes Shoot Down's mouse held a frame 240 ms), and a
copy of every texture's reference count into two scratch objects each frame that nothing reads.
Coming off the sight held four frames 40 to 60 ms each (2026-11-07). Not Cesium, though its
tiles came back into view in the same frames: its tile work was 11 ms as always, and the WebGL
calls themselves stalled. Chrome's GPU thread was busy repainting the sight picture, a 2D canvas
the size of the window cleared and drawn each frame. Hidden, there were no long frames.
`shoot/sights.ts` now draws each sight on a small canvas moved by a CSS transform and faded by
its opacity (the compositor's work); `perf.py` takes the sight on and off three times, and the
longest frame there is now about 32 ms.

A later step (§3.1) moves the physics world into a Worker; the ground's questions are already
the only way in.

## 4. Repository layout

```
drone-duels/
  pyproject.toml            name drone-duels · package drone_duels · script `duels`
  DESIGN.md  ROADMAP.md  CLAUDE.md  README.md
  content/
    locations/              billy-bishop.toml, area-xo.toml          (place, ground, start pads)
    vehicles/               quad5.toml, quad5-betaflight.toml, chupito.toml, truck.toml
    courses/                areaxo-1.toml                            (race gates)
    controllers/            literadio2.toml, tx15.toml, t8l.toml     (default channel maps)
  firmware/
    ardupilot/              copter-quad5.parm, sitl.parm, json-frame notes
    inav/                   chupito.cli                              (INAV CLI settings)
    betaflight/             quad5.cli (+ .analog/.digital: the OSD)  (Betaflight CLI settings)
  router/sitl.conf
  scripts/setup.sh          ArduPilot, INAV, Betaflight (built with betaflight-sitl.patch),
                            mavlink-router, bun, web deps, Chupito GLB
  scripts/setup-web.sh      the web version's simulator (§18): emsdk, cudaflight (built with
                            cudaflight-web.patch) and web/wasm/ to external/web/sim/sim.wasm
  tools/                    gen_protocol_ts.py (pydantic → web/src/net/protocol.ts),
                            models/ (fpvmodel, truckmodel, operatormodel, operatortextures,
                            gltf, fbx: GLB asset builders)
  src/drone_duels/
    cli.py                  `duels` (serve, doctor, physics, record)
    ports.py
    content/                pydantic schemas + loader for content/*.toml, prefs
    server/                 FastAPI app, /ws protocol (pydantic models), REST, static files
    session/                Session, slots, vehicle lifecycle, mode hooks on the server side
    firmware/               base.py (Firmware protocol), ardupilot.py, inav.py, betaflight.py
    bridges/                ardupilot_json.py, xplane.py, betaflight.py
    physics/
      core/                 (stdlib only) rigidbody, atmosphere, weather (MIL-F-8785C), ground
      wing/                 (stdlib only) today's wingsim: strips, aero, propulsion, airframe
      quad/                 racer5in on skyflow-dynamics' JAX backend (CPU), ESC/motor map,
                            battery, ground contact
      runner.py             the physics process: a bridge loop + control socket
    telemetry/              mavlink link, vehicle state, plan reader; msp for INAV
    video/                  osd (MSP DisplayPort, fonts)
    rc/                     RcSink, failsafe, channel maps
    world/                  geo, elevation
    webdata.py              the web version's data, and what is generated for it (§18)
    record/                 flight recorder (logs/flights/)
  web/
    package.json  tsconfig.json  biome.json
    src/
      main.ts
      theme/                tokens.css (palette, §11), base.css
      state/                alien-signals stores (§9)
      engine/               frame loop, exact spring, local frame, ground + physics (§3.2)
      streams/              Observable sources: ws channels, frames, timers
      net/                  server ws client, protocol types (generated, §8), peer link
      ui/                   menu (scenario, location, vehicle), pause menu, top bar, toasts,
                            a touch screen's sticks
      world/                Cesium viewer, basemap, world.add, Cesium's lookups (metered)
      vehicles/             quad, wing, truck: models, animation; truck physics (Rapier)
      people/               the operator standing behind the pad
      views/                fpv, chase, spotter, map, orbit, minimap, dials
      video/                analog (analog-vision), digital, osd renderer
      modes/                freefly/, duel/, race/, shoot/
      shoot/                Shoot Down: ballistics, air, rounds, recoil, hold, shooter (§7.1)
      dynamics/             wing dynamics overlay
      input/                actions (the controls' tables), radio, mapping, calibration, touch
      audio/
      sim/                  the web version's simulator, in a worker (§18): module (the
                            WebAssembly), quad, weather, msp, vehicle, session, worker
    wasm/                   the simulator's C: bfweb.c (Betaflight's harness), plant.c,
                            sfd.c/.h (SkyFlow-Dynamics' generated C backend)
    test/                   bun test
  tests/                    pytest (physics goldens, bridges, content, session, protocol)
```

`external/` (git-ignored, from `scripts/setup.sh`) keeps ArduPilot, INAV, Betaflight and
mavlink-router; `scripts/setup-web.sh` adds emsdk, cudaflight and the built simulator.
The web libraries (Cesium, Rapier, Trystero, milsymbol, alien-signals) become bun dependencies
pinned in `web/package.json`, no longer tarballs copied by hand. analog-vision is a git
dependency, `github:Synetic-Labs/Analog-Vision#<commit>` (its `package.json` exports
`analog.js`), imported as `"analog-vision"`. To try a local change, `bun link` the checkout beside
this one. skyflow-dynamics is a uv git dependency pinned to a commit, with its `jax` extra
(CPU jaxlib).

## 5. Content model

```toml
# content/locations/billy-bishop.toml
name = "Toronto: Billy Bishop Airport"
origin = [43.630930, -79.389450]
ground_alt_m = 77.0                       # or "auto" (Open-Meteo, cached)
heading_deg = 270                         # the start pads face this way
[wind]                                    # defaults; live-adjustable
speed_ms = 3.0
from_deg = 250
turbulence = "light"                      # none | light | moderate | severe (MIL-F-8785C)
gusts = false

# content/vehicles/quad5.toml
name = "5\" FPV quad"
kind = "multirotor"
firmware = { kind = "ardupilot-copter", params = ["firmware/ardupilot/copter-quad5.parm"] }
physics = { kind = "quad", airframe = "racer5in" }
model = "models/fpv-drone.glb"
camera = { tilt_deg = 30, hfov_deg = 120, offset_m = [0.06, 0, -0.02] }
launch = "ground"                         # ground | hand (Chupito) | drive (truck)
dynamics_overlay = false

# content/courses/areaxo-1.toml
name = "AreaXO: the loop"
location = "area-xo"
laps = 2
alt_ref = "agl"                           # agl | amsl
[[gate]]
at = [45.319287, -75.758596]
alt_m = 3.0
ypr_deg = [90, 0, 0]                      # yaw true, pitch, roll
size_m = [2.0, 2.0]
```

Each file loads into a frozen pydantic model (`extra="forbid"`, no defaults where a value
matters). An unknown or missing key is an error naming the file and the key. `duels doctor` loads
every file, so a bad edit shows up before anyone flies. The same models feed the REST API, so the
menu lists exactly what loaded.

A game is a mode at a place; there is no scenario file. Which vehicles a mode allows (Race: no
truck) and where it can be played (Race: where a course is) are the mode's, in the page. "Mission"
means only an ArduPilot mission.

## 6. Vehicles

### 6.1 Chupito (INAV + wingsim)

Ported as it is: `swarm.wingsim` → `drone_duels.physics.wing`, with `inav.py` and `wingrun.py`
split into `firmware/inav.py`, `bridges/xplane.py` and the session. The tests that hold the model
to its sources (`tests/test_wingsim.py`) move with it unchanged. The Nanawing arcade port
(`wingflight.js`) is dropped. The page still posts the ground height under the wing
(`sampleHeightMostDetailed`, never the sync `sampleHeight`).

### 6.2 5" quad (ArduCopter + SkyFlow-Dynamics)

ArduCopter 4.7 SITL runs with `--model JSON`. Each step it sends a servo packet (magic 18458, 16
PWM) to the physics, and the physics answers with its JSON state (time, IMU, position,
attitude, velocity), lockstep.

The plant is SkyFlow-Dynamics' `racer5in` (the UZH-RPG NeuroBEM/Agilicious quad, fitted to its
flight data): rigid body, motor lag, rotor thrust and drag, frame drag, inflow damping, ground
effect. It is close enough to our own 750 g 6S build. It runs on SkyFlow-Dynamics' JAX backend
as published, on the CPU:

- ArduPilot's SITL physics runs at 1200 Hz (`SIM_RATE_HZ 1200` in `sitl.parm`): at 400 Hz
  ArduCopter refuses to arm ("Gyro 0 rate 400Hz < loop rate*1.8"). Each JSON frame is one jitted
  call: an `exact_exp_step_fn` step (or as many as fit a slower frame) and `imu_fn` for the
  accelerometer and gyro. Both are jitted once at spawn.
- Measured (2026-10-29, x64): **88 µs median, 197 µs p99** per 1200 Hz frame, about 10 % of a
  core. The same spec lambdified to plain Python takes 167 µs, so JAX is not the slow choice.
- The cost is at spawn: about 1.5 s to import JAX and 2.3 s to build and jit, hidden behind
  ArduPilot's own boot and EKF start. JAX's persistent compilation cache (`logs/jax-cache/`)
  cuts the jit on later spawns.
- `JAX_PLATFORMS=cpu` is set in that process, so it never takes the GPU from Chrome.

The ESC map (ArduPilot's PWM to `Ω_c` through the backend's `throttle_to_speed_fn`), the battery
sag, the wind (core weather, §13) and ground contact are harness-side, as SkyFlow-Dynamics
requires, and live in `physics/quad/`. The tests replay SkyFlow-Dynamics' golden step vectors
through our harness, so the wiring cannot drift.

The ArduPilot tune (`copter-quad5.parm`, from today's `fpv5.parm`) is re-fitted to racer5in's
mass and thrust. `MOT_THST_HOVER` and `MOT_THST_EXPO` come from the plant, not by hand.

**Flying itself** (a target, `target` in §8): the game flies it as a ground station would, over
MAVLink as system 254, while the idle receiver plays on (nobody on the sticks): GUIDED, arm,
`MAV_CMD_NAV_TAKEOFF` to the height above the pad, each waiting for its `COMMAND_ACK`, tried
again until telemetry shows it climbing (`ArduCopter.hover`, on a thread of its own); up
already, a position target at the height; `land` is LAND. Nothing about the RC changes: the arm
and mode switches act only when they move, and the idle 1000 µs throttle is above
`FS_THR_VALUE`. Measured (2026-11-06, SITL): 30 m held 16 s after asking from the pad (climbing
at `WP_SPD_UP`'s 2.5 m/s); a reset respawns ArduCopter (48 s to ready, 32 of them waiting for
the EKF), then 10 m in 8 s.

**Hits** (`hit` in §8) damage the plant by part (`physics/quad/model.py`, `Damage`), four of
them: an **arm** (the arm, its motor and ESC, its prop) loses that rotor: its `ct2` and `cq2` to
0, repacked with `pack_params` (the same shape, so nothing is jitted again), and its motor
unpowered; the other three fly on, and the page stops drawing its prop. A prop with one blade
shot off is no better: out of balance by some six times the quad's weight at hover, it shakes
the motor off or floods the gyros (the reasoning is beside `ROTOR_PARTS`). The **body** (the
stack, the plates and the wiring between them, the camera) cuts the power to all four at once.
The **battery**, pierced, holds 0.3 s and then dies (EST, the reasoning beside
`BATTERY_HOLD_S`), reading dead to the flight controller too. A **payload** (content/payloads,
fitted by a mode to whichever vehicle it starts: Shoot Down's 0.4 kg charge, carried in the
plant's mass and inertia by `plant.with_payload`, with its firmware's files after the vehicle's:
ArduCopter's hover throttle, 0.35) goes off when hit, or one time in two when the quad comes
down (crashed, or onto the ground after a hit: the game's rule); gone off, the power is off and
the page draws the blast instead of the quad. The kick each part takes is its share of the
bullet's momentum (`shoot/target.ts`, zone by zone: a pack's slight, 0.12, the reasoning beside
it). Every hit leaves its momentum as SkyFlow's
external force over the next frame (F = J / dt, W) and its moment about the centre of mass as
the external torque (B). ArduCopter then flies what is left, as it would: one rotor gone, the
quad yaws away, tumbles and falls (SITL, from 30 m: 18 m/s into the ground 4 s later, crashed).
A reset mends it all; the physics' status carries the damage, and `physics` (§8) passes it to
the page.

**On Betaflight** (`quad5-betaflight`, the same airframe and plant): Betaflight 2026.6's SITL,
built from source by `scripts/setup.sh` with `scripts/betaflight-sitl.patch`. The patch makes
its ports arguments, keeps it to 127.0.0.1, builds in the OSD (MSP DisplayPort; stock SITL has
none), and drops state that arrives before its virtual sensors exist (which crashed it as it
booted, the physics already running). Its settings (`firmware/betaflight/quad5.cli`, then the
video system's OSD layout) go into a fresh eeprom by SITL's own `--config` before it boots;
Betaflight's own PIDs and rates fly it. `bridges/betaflight.py`:

- Betaflight keeps its own clock, scaled to the rate the state's timestamps advance against the
  wall clock, and sends its motors once for each state packet. So the physics paces itself at
  1000 frames a second in real time, steps with the motors last sent, and sends the state.
  Measured (2026-11-06): Betaflight 3 % of a core, the physics process about 50 %.
- The frames are those of Gazebo's BetaflightPlugin, which the default SITL build reads, held
  by `tests/test_bridge_betaflight.py` to Betaflight's own SITL harness for any attitude: gyro
  (p, q, −r); the accelerometer the negated FLU specific force; the attitude FLU to NWU, turned
  −90° and conjugated by a half turn about x; velocity ENU; GPS mirrored about the pad (the first
  fix Betaflight takes, and mirrors back about); the barometer from the height.
- Its motors come as µs (1000 stopped, 1055 armed idle, 2000 full), the range the ESC map reads,
  reordered from Betaflight's quad X (rear right first) to ArduPilot's.
- RC goes over UDP from the adapter (an idle receiver until the page's sticks come), telemetry
  over MSP, the OSD over MSP DisplayPort polled as a DJI air unit polls it (INAV's `AirUnit`),
  missions over MAVLink on UART4 (`telemetry/mission.py`; stock SITL builds without telemetry,
  so scripts/betaflight-sitl.patch adds it).
- The mode switch is laid out as ArduCopter's: low POSHOLD (Betaflight's altitude hold and GPS
  position hold together), middle ANGLE, high ACRO. Betaflight arms only with POSHOLD off, and
  the hold modes take over once the throttle has been raised after arming; in them the
  throttle stick asks for a climb or a descent, nothing within 20 % of its travel about the
  hover throttle (1275 µs, where this quad hovers in ANGLE).
- A link lost (§8): Betaflight's own failsafe lands it where it is (AUTO-LAND, holding the
  throttle through the guard time; its altitude and position hold fly the descent) and disarms
  on the touchdown's jerk (`landing_disarm_threshold` 150: 1500 g/s; a mission starting jerks it
  past 500 on our physics, and a touchdown past 2500, measured 2026-11-08). SITL has no battery: no pack voltage. A reset leaves it running (no
  EKF to upset).
- **Flying itself** (a target, `target` in §8): Betaflight takes no commands from a ground
  station (its MAVLink takes missions but no arming or modes), so the game flies it as a pilot
  on its own modes would (`Betaflight.hover`, `land`), reading it
  over MSP: arm switch up in ANGLE, POSHOLD, then the throttle stick for the climb asked (0.6 m/s
  a metre below the height, at most 3 m/s), back in the deadband within 0.3 m, and on nudging it
  there (its height target trails a fast climb). Landing comes down in POSHOLD until
  Betaflight's touchdown detection disarms it. A reset in place (`Firmware.reset_in_place`)
  puts the arm switch low, or it would fly off the pad again.
- **A path** (`Betaflight.path`, `target path`) is a mission its own autopilot flies: up and
  holding in POSHOLD, the game uploads the waypoints over MAVLink, as a ground station would,
  and sets AUTOPILOT's switch (channel 10, which no controller here maps) high. Betaflight denies
  a mission while it flies one, so a new one goes in with the switch low (POSHOLD holds it), in
  about 0.1 s. It flies the legs at `ap_max_velocity` (7 m/s; also POSHOLD's full stick),
  cutting the corners of the waypoints before the last and holding at the last; the switch low
  and high again starts it over, so a path goes round until a hover or a land takes it back.
  Measured (2026-11-08, SITL): a 40 m square, within 0.6 to 2.5 m of its corners, up to 11 m/s,
  20 m ± 1.5; round again; after a hover, still within 1.5 m.
- Measured (2026-11-06, SITL): ready 8 s after the start; Betaflight's attitude within 2° of the
  truth in flight (its compass has a 2° declination). In ANGLE the place's wind took it 25 m in
  10 s; in POSHOLD it held within 0.6 m. As a target: 30 m held 11 s after asking from the pad,
  within 0.5 m up and down after one 1.8 m dip, within 0.5 m of the pad across; after a reset, 10 m
  4 s after asking; down from 30 m in 11 s, disarmed by Betaflight on touchdown.

### 6.3 Truck

Today's Rapier ray-cast vehicle (car.js and drivesession.js) on the place's ground (§3.2: the
colliders built from the tiles, and the physics world stepped at 60 Hz), in the page, with no
firmware. It gets the same Vehicle interface in the page as the aircraft
(pose stream, controls, reset), so views and modes do not care that its physics is local.

### 6.4 Vehicle lifecycle

`Session.spawn(location, vehicle)`:

1. stop the slot's current firmware and physics, if any;
2. start the physics at the location's start pad;
3. start the firmware with its params and wait until it is ready (ArduPilot: EKF and prearm
   checks; INAV: no arming blockers on two polls in a row, a known quirk; Betaflight: none but
   the arm switch, twice);
4. report `ready`.

Switching vehicle is a spawn at the location's start. Reset re-spawns the same vehicle. Destroy
(a crash in Duel or Race) cuts the physics, so the firmware's link goes quiet as a destroyed
aircraft's would.

### 6.5 The operator

The pilot stands 3 m behind the start pad and 1.5 m to its left (clear of the chase camera), facing
the vehicle's heading: Fab's "Quantum Modular Character" posed by `tools/models/operatormodel.py`
(goggles on, a radio with two long-range antennas in both hands) and baked into a static GLB,
textured from the listing's maps (shrunk into `assets/quantum_textures/` by
`tools/models/operatortextures.py`). Drawn only, for the aircraft (the truck's driver is in it). The
spotter view is their eye, so they are hidden in it; ground sampling looks through them, and the
analog link's far end is their goggles.

## 7. Modes

A mode is a pair of plugins with one lifecycle, `setup → running → finished`:

- **In the page** (`web/src/modes/<id>/`), registered in the mode registry: menu options, HUD
  widgets, rules, scoring, peer messages.
- **On the server** (`session/modes/<id>.py`): optional hooks only (allowed vehicles, what a
  crash does). Most rules live in the page, beside the world they judge.

| Mode | Players | Rules |
|---|---|---|
| Free Fly | 1 | Any vehicle, switch at will (the pause menu), no score. Practice missions here with QGC. |
| Duel | 2 (P2P) | Turns. The pilot (quad or Chupito) strikes the truck; the driver (truck) evades. A strike scores; a crash spends the aircraft. Swap each turn. Today's duelmatch rules carry over. |
| Race | 2 (P2P) | Live, side by side. Each player flies their own sim and sees the other as a ghost (no collisions). Countdown start from the host. A gate counts when the segment between two true poses crosses its rectangle in the right direction, from GO (the countdown waits for the session it started to be ready). The next gate wears a chevron on each side, the one after it dots; the next three shade from ember to the rest's pale. Lap and total times; first to finish wins. |
| Shoot Down | 1 | On foot with a rifle; a real quad 30 m over the pad, 100 m in front, level by level: it hovers, flies a line, a circle, a figure eight, carrying a charge. Bring it down. No score but shots, hits, downs, deaths and the time from the first shot to a down (§7.1). |

The peer link stays Trystero over Nostr relays (WebRTC, no server), one room per match. Peers
exchange poses at 30 Hz for the ghost or truck, events (strike, gate, crash, finish) with a
sequence number, and a clock offset from pings, so the countdown and timers agree.

Poses go on a data channel of their own, unordered and never resent (`negotiated`, beside
Trystero's), so a lost packet costs one pose rather than stalling those behind it, as Trystero's
one reliable channel would; events stay on the reliable one. Each pose carries the sender's
clock at the moment it shows. The receiver keeps them in the same timeline as its own aircraft
(§3.1) and draws the other player a buffer behind: 100 ms to start, then the 95th percentile of
the lateness plus one interval, between 60 and 250 ms, changed slowly. Judging is as before:
gates on your own true poses; the pilot's page judges a strike against the truck as the pilot
saw it (favouring the shooter).

### 7.1 Shoot Down

The player is a person on the ground with an 11.5" AR-15 (content/weapons/ar15-sbr.toml: its
mass, twist, magazine, sights, zero, and the cartridge, M193, with each number's source). The
target is the session's own vehicle, the 5" quad on ArduCopter or Betaflight and
SkyFlow-Dynamics, with Shoot Down's charge fitted (`ModeDef.payload`: the start names it, the
session fits it, and its status says so, for the page to draw it): the page asks for
`target hover` and the server flies it up, as a ground station would (ArduCopter) or as a pilot
on its switches would (Betaflight, §6.2). Nobody flies it from the page (`ModeDef.ownsView`: no
rig, feed, operator, sticks or motor in the player's ears).

**Levels** (`shoot/levels.ts`): 1 hover; 2 a line across the view, 50 m; 3 a 40 m circle; 4 a
figure eight 50 m across. Down, the next level comes up (after the last, the first); killed, the
same again. From level 2 the page flies ArduCopter (up to 6, 7 and 8 m/s), 3 s after it reaches
its hover: `target go` five times a second, a point on the path with the path's velocity there
as the feed-forward (ArduCopter's position controller follows the path rather than chasing a
point). ArduCopter's own guidance shapes what it flies (copter-quad5.parm's WP_ACC 5 m/s², its
60° lean), as it would a real one's; each path eases up to its speed over 4 s. Measured
(2026-11-07, SITL, the free map): the line ±27 m at up to 6.4 m/s, the circle 46 m across at up
to 8.6, the eight 54 × 31 m at up to 9.0. Betaflight flies them as missions (§6.2): `target
path`, once, the path's 30 waypoints (whole laps: the line end to end, the circle 10 a lap, the
eight 15), at its own 7 m/s. Measured (2026-11-08, the free map): the line ±25 m, at up to 8.8
m/s. With the charge Betaflight climbs at half its rate and holds a metre or two low at first:
its hover throttle (ap_hover_throttle) is the bare quad's. The charge goes off when a bullet
hits it, or one time in two when the quad comes down (the physics decides, §6.2); within its
`lethal_m` (5 m, the M67's fatality radius) the shooter is killed.

The page's modules are in `web/src/shoot/`:

- **ballistics.ts**: a point mass on the G7 drag table scaled by the BC, through the moving air
  (ISA density at the place's height), WGS84 gravity, Coriolis, and Litz's spin drift and
  aerodynamic jump, stepped by RK4 at 0.5 ms. The sights are zeroed at 50 m by solving for the
  bore's angle; the tests hold it to Didion's lag rule, a vacuum, and the same flight on G1 with
  M193's published G1 coefficients.
- **air.ts**: the place's wind made again in the page as physics/core/weather.py makes it
  (MIL-F-8785C's log profile, Dryden-sized turbulence, 1 − cos gusts). The bullets fly through
  it and the wind meter reads it; the drone's physics draws its own gusts from the same settings.
- **target.ts**: the quad's parts as the drawn model lays them out (props as discs a blade crosses
  with the chance the blades cover, motors, ESC sleeves, arms, pack, stack, camera, plates, and
  a payload's tube), each zone the part it belongs to (arm, body, battery, payload) and the share
  of the bullet's momentum it takes as it passes through.
- **blast.ts**: a charge going off: a flash, a fireball, fragments, smoke for some seconds, dust
  near the ground; heard when its sound arrives. The quad and its payload (drawn under it by
  vehicles/aircraft.ts) are no longer drawn.
- **rounds.ts**: rounds stepped in real time against the drone's drawn pose at the same moment
  (favouring the shooter, as the duel's strikes) and the ground; tracers, dust, sparks. A hit
  goes to the server as `hit` (part, motor, impulse, where) and the physics does the rest. Each
  frame, a round's way since the last is a segment cast through the place's colliders (§3.2):
  the point where it went in, exact, in microseconds. Past 400 m from the shooter none are built,
  so it is no longer checked, and flies on, tracer and all, until spent (5 s, 120 m/s or 3 km).
  A clear tracer, flash, spark and dust are kept drawn: Cesium frees a material's shaders once
  nothing uses them, and relinking the tracer's held the first shot's frame 150 to 350 ms.
- **recoil.ts**: the rifle's momentum, bullet plus powder gas at Hatcher's 1.75 × muzzle speed
  (5.5 N·s for M193 here), into a body of damped springs by stance: the shoulder pocket, the
  torso pushed back and tipped about the hips (or elbows, prone), the shoulders turned by the
  push landing right of the spine. **hold.ts**: breathing, the breath held (still for 6 s, then
  the shakes, a forced breath at 14 s), the hold's wander and shiver by stance, heartbeats, the
  trigger's jerk, and a heart rate that walking and running raise.
- **shooter.ts**: the person (stances; walking, sprinting at 6 m/s with the rifle down, jumping;
  the camera on the sight or head up) and the shot: the trigger breaks, the jerk, 8 ms of lock
  time, the bullet leaves along the sight line tipped by the zero and spread by the
  ammunition's dispersion. Three systems in the frame loop: its controls (shoot/controls.ts),
  its body at 240 Hz, and the eye and rifle placed. Its ground is the place's colliders, asked
  as it goes, down from 2.8 m over the feet: a tree's crown or an overhang over them is not in
  their way; inside something taller (a building) nothing is found below, and its top is a wall.
- **obstacles.ts**: what they get over, by its rise over the feet: a step (to 0.45 m) walked
  onto; a vault (to 1.1 m, waist high) by walking on into it or a jump; a climb (to 2.3 m) by a
  jump, a grab of the top and a pull up, the rifle slung; higher, a wall, unless it is thin with
  ground within 4 m beyond, which leaning into for 0.6 s pushes through. The tiles' ground is
  heights, so bushes, hedges, low crowns of trees and the mesh's glitches are solid blobs to it:
  pushing through is the way out of them (and a thin wall too high to climb gives way the
  same). Down, any drop: a fall, and landing over 6 m/s (some 1.8 m) a stagger. The HUD's
  "Ahead" row says what is in front.
- **rifle.ts**, **sights.ts**, **gunsounds.ts**: the first-person rifle and arms
  (tools/models/riflemodel.py; its measured points in the GLB's extras), the holo's reticle
  (at infinity, through its window) and the irons' aperture (each a small canvas over the view,
  moved by the compositor, §3.2), and the rifle's sounds, those from out there arriving at the
  speed of sound.

Every spring (recoil, the heartbeat, the knees) is the exact one (§3.2), and the view's pitch is
held off straight up and down: stepped a frame at a time by Euler, the heartbeat's grew without
end under about 20 fps and turned the view over.

The numbers no source gives are marked EST where they are set. Not modelled yet: the spatial
variation of the wind along a shot, the AR's bolt carrier cycling in the recoil, a payload's
drag and the centimetre it lowers the centre of mass, wounds (only death, inside the lethal
radius), and ricochets. Walls
are the tiles' heights as the feet meet them: an overhang lower than 2.8 m over the feet is a
wall, and nothing hurts in a fall.

## 8. Server ⇄ page protocol

One WebSocket at `/ws` carries JSON envelopes `{ch, t, data}`, plus binary frames for the
high-rate pose channel.

| Direction | Channel | Rate | Content |
|---|---|---|---|
| page → server | `session` | on demand | start (with a payload fitted, if named), switch, reset, destroy |
| page → server | `rc` | 250 Hz | 16 channels in µs, sequence number; only while the page's controller flies (§12) |
| page → server | `ground` | 20 Hz | ground height under the aircraft |
| page → server | `wind` | on change | speed, direction, turbulence, gust trigger |
| page → server | `video` | on change | analog or digital (sets the OSD grid, §10) |
| page → server | `radio_source` | on change | stream the radio as the server reads it; and whether it flies (§12) |
| page → server | `target` | on demand; `go` 5 Hz | the quad flies itself: hover at a height over the pad, or land (§6.2); go on along a path, a point on it with its velocity (ArduCopter, §7.1); fly a path round and round as a mission (Betaflight, §6.2) |
| page → server | `hit` | per hit | a bullet struck a part (arm with its motor, body, battery, payload): the impulse, where; straight to the physics (§6.2). A payload reaching the shooter is sent as a hit too |
| server → page | `status` | on change | slot, vehicle, firmware state, arming blockers |
| server → page | `pose` | 120 Hz | sim time, true position, attitude, velocity, body rates, rotor/servo state (binary) |
| server → page | `osd` | per draw | the firmware's DisplayPort screen |
| server → page | `telemetry` | 10 Hz | what a GCS sees: mode, armed, GPS, battery, attitude, alt |
| server → page | `plan` | on change | mission, fence, rally (ArduPilot) |
| server → page | `dynamics` | 30 Hz | wing internals (when the overlay is on) |
| server → page | `physics` | on change | what the physics says it is doing: ground, hand, flying, crashed; what hits have taken |
| server → page | `radio` | 60 Hz | the radio's axes, read through Linux, and its channels as the server maps them (while asked) |

The protocol is defined once, in `server/protocol.py`, as pydantic models: a discriminated union
on `ch`, validated on the way in. `tools/gen_protocol_ts.py` writes their JSON Schema and
generates `web/src/net/protocol.ts` from it, and a test fails if the generated file is out of
date. The REST endpoints (`/api/content`, `/api/config`, `/api/prefs`, …) are FastAPI routes
on the same models, with OpenAPI docs at `/docs`.

Why FastAPI rather than aiohttp: the pose stream is a few hundred small messages a second on one
socket, which either serves easily. The deciding win is pydantic at the edges. One set of models
validates content files, prefs, REST and WebSocket messages, and generates the TypeScript types,
so the server and the page cannot disagree silently.

RC failsafe: if no sticks arrive for 200 ms (the radio unplugged; or, while the page's controller
flies, the tab hidden or crashed; or the page gone), the session tells the firmware its link is
lost, and the firmware's own failsafe runs, as on a real link. ArduPilot
SITL would hold the last pulses forever, so the adapter sets `SIM_RC_FAIL`; INAV's X-Plane link
stops reporting a joystick, and its receiver times out; Betaflight's adapter stops sending its
RC packets (RXLOSS). All come back with the next sticks.

The WebSocket takes only the game's own page (a local Origin), so another site open in the same
browser cannot fly the sim.

## 9. Web app state

- **alien-signals** holds every value the UI reads now: the screen (menu, flying, paused),
  mode, location, vehicle, view, video system, prefs, session status, match state, and the latest
  telemetry at 10 Hz. `computed` derives from them (for example, the dials are hidden when the
  view is FPV), and `effect` writes them to the DOM. Stores are plain modules in `state/`. No UI
  framework: a small `h()` helper plus effects.
- **Native Observables** carry everything that happens over time: the WebSocket channels
  (`ws.when("message")` split by channel), gamepad samples, animation frames, the race clock and
  countdown, and peer messages. The render loop subscribes to `pose` directly and does not go
  through signals. Where the UI needs only the latest value, an Observable writes into a signal
  (`telemetry$.subscribe(v => telemetry(v))`).
- TypeScript's DOM lib does not declare `Observable` and `EventTarget.when` yet, so
  `web/src/types/observable.d.ts` declares the subset we use.

Two games on one machine (to test a duel or race) each take a vehicle slot of their own:
`duels serve --port 8601 --slot 3`.

Toolchain: bun builds and serves the bundle (`bun build`, with watch in dev). Types check with
`bunx tsc --noEmit`, lint and format with `bunx biome check`, and tests run with `bun test`.

## 10. Video and OSD

- **FPV** draws the world from the vehicle's camera (tilt, FOV and offset from its spec) at the
  physics' true pose, with no map symbols.
- **Analog**: analog-vision's AnalogFeed (NTSC, 4:3). Link noise follows range from the pilot
  and VTX power (Video options), and both antennas' angles (dipoles, their nulls filled to
  -8 dBi). INAV's analog OSD is its Betaflight-compatible 30×16 mode (BF43COMPAT), laid out for
  the 13 rows NTSC shows. The OSD is the firmware's SD grid (30×16, 13 rows shown on NTSC)
  laid into the video before the analog chain, as the OSD chip does.
- **Digital**: 16:9, no effect, and the HD OSD grid laid over the picture as a DJI/HDZero/Walksnail
  goggle does (ArduPilot `OSD1_TXT_RES 1`; INAV's HD DisplayPort). A later item models digital
  link breakup.
- Switching the video system changes the firmware's OSD resolution. ArduPilot takes it in flight
  (it reads `OSD1_TXT_RES` each time it clears the screen), so its vehicle flies on. INAV takes
  it only at boot, so the switch respawns that vehicle at the start, as a vehicle switch does;
  so does Betaflight (NTSC's 30×13 for analog, its 53×20 HD canvas for digital).
- The OSD fonts are the firmware's own (ArduPilot's OSD_FONT, and Betaflight's, in which INAV's
  DisplayPort draws too), rendered from their .mcm files as `swarm.osd` does today.

## 11. UI

**Flow.** The main menu is three columns, each opened by a pick in the one before: the scenario
(Free Fly · Duel · Race), the location (Race: those with a course), the vehicle (Duel: an
aircraft, for your turns as the pilot). Free Fly flies on the vehicle's pick; the pause menu's
Vehicle tab switches it. Duel and Race draw their lobby under the vehicles: name, solo, host or
join, then the room code, peer status and ready. Esc steps back a column; the picks are kept for
the next visit. Options and Exit sit in the top right corner; Exit asks once, then stops the
server (`POST /api/quit`, as Ctrl-C would, so the session closes the firmware) and closes the
window.

**In flight.** A thin top bar shows mode · place · vehicle · view · video · wind · timer/score.
It fades after 3 s in FPV and comes back on mouse movement or a menu key. In the corners:

- bottom right: the minimap PiP (M toggles it);
- bottom left: the dials (today's instruments.js: throttle, altitude, climb, speed, compass,
  attitude, with the stick meters).

Both hide in FPV, where the OSD is the instrument panel. The quick keys: V cycles views, 1–6 jump
to a view, R resets, B switches vehicle, D shows the dynamics overlay (Chupito), Esc pauses.

**On a phone** (`ui/ui.css`'s last sections; a touch screen with no controller, §12). The same
page, laid out again by size and by touch, not another one:

- A small screen (under 900 px wide or 560 px high) gets the menu's three columns sharing the
  width, or one under the other below 700 px with the menu itself scrolling and each column
  brought into view as it opens; Options and the pause menu fill the screen, their tabs a strip
  along the top when held upright; the top bar keeps only the view, the mode's clock and the
  menu.
- A touch screen (`<html class="touch">`, kept to what the player last used: `input/touch.ts`)
  gets what the keys did as buttons: Reset on the top bar, Full screen beside Options, a tap
  on "Crashed" to reset. The top bar does not fade: it is the only way to the menu.
- While the sticks on the glass fly (`<html class="touch-sticks">`, `ui/sticks.ts`), the bottom
  of the screen is theirs: the minimap and the dials move up under the top bar, a mode's panel
  under the dials. Each stick is as big as its side of the screen holds (`--stick`, to 340 px:
  a thumb's travel for the whole stick is 0.31 of it), beside a middle column kept for the top
  bar and the switches. On a phone on its side that is nearly the screen's height, so the
  minimap and the dials go in the top corners, beside the top bar. Held upright, each stick is
  half the width, the switches sit in a row over them and the camera's picture keeps to what
  is left above them (the feed fits its host, §10).
- The page is not pinched, double-tapped bigger, pulled down to reload or selected by a long
  press (`theme/base.css`), and it keeps clear of a notch (`viewport-fit=cover` and the
  safe-area insets).
- A touch screen draws at 1.5 device pixels to a CSS pixel at most (`world.ts`): a phone has
  three, and a fraction of a desktop's GPU. A judgement: not yet measured on a phone.

A browser without Observables (§9: Safari, so every browser on an iPhone or iPad, and Firefox)
is told so in place of the menu (`main.ts`).

**Pause/Options (Esc)**, in tabs. Pausing stops the page's own world (the truck), not the
firmware: the radio keeps flying the aircraft, as a lost link would otherwise trip its failsafe.

| Tab | Holds |
|---|---|
| Vehicle | switch vehicle (respawns at start), reset |
| View | default view, chase distance, spotter position, FOV for the non-FPV views |
| Video | analog/digital, VTX power, camera tilt override, OSD on/off |
| Wind | speed, direction, turbulence level, gusts on/off, "gust now" |
| Controls | controller profile, calibration wizard, channel map, expo/rates for the truck, keyboard |
| Inputs | every hotkey, and each vehicle's sticks and switches on a radio, gamepad and keyboard, or on a touch screen its sticks' (`ui/inputs.ts`, which the HUD's key strip also reads) |
| Graphics | map source and keys (Google, Cesium ion, Esri), quality, frame cap, time of day (the sun's local solar hour, noon by default, or the real time) |
| Audio | master, effects, motor sound |
| Session | QGroundControl port info, restart firmware, quit to menu |

**Views.**

| # | View | What |
|---|---|---|
| 1 | FPV | the camera feed with the OSD, analog or digital |
| 2 | Chase near | behind and above, lagged spring arm |
| 3 | Chase far | the far chase from today's wing.html |
| 4 | Spotter | the pilot's eye at the start pad, tracking the vehicle (LOS flying) |
| 5 | Map/COP | tilted map, trails, milsymbol marker, the uploaded mission and fence, the dials |
| 6 | Orbit | free orbit around the vehicle (mouse) |

**Palette, "Dusk".** It is new and warm where the old one was cyan on black. The tokens live in
`web/src/theme/tokens.css`. There is no light theme: the game runs dark.

| Token | Hex | Use |
|---|---|---|
| `--bg` | `#0D0E15` | page and menu background |
| `--surface` | `#151724` | panels (at 86 % over the 3D view) |
| `--raised` | `#1E2133` | buttons, cards, inputs |
| `--line` | `#2C3049` | borders, dial outlines |
| `--text` | `#EEF0F7` | text |
| `--muted` | `#8E93AD` | secondary text, idle needles |
| `--ember` | `#FF6B35` | primary: actions, selection, the pilot / player 1 |
| `--tide` | `#22C7B8` | secondary: the driver / player 2, links |
| `--sun` | `#FFC845` | timers, countdown, focus ring, caution |
| `--rose` | `#FF4F6E` | crash, failsafe, disarm, errors |
| `--leaf` | `#8BE07A` | armed, ready, gate passed |
| `--sky` | `#6EA8FF` | telemetry traces, stick input meters, trails |

Type: Inter for the UI, JetBrains Mono for numbers, both bundled through `@fontsource` (no network
at runtime). The firmware OSD keeps its own fonts and colours, as real goggles show it.

## 12. Controllers

- The server reads a radio through Linux itself (`rc/evdev.py`): Chrome on Linux may not expose
  a radio until a stick moves, or all its axes (it lists the LiteRadio 2's CH5 and CH8 but never
  moves them). **When the server's radio flies, it flies from the server** (`rc/pilot.py`): each
  stick movement is mapped (profile, calibration: `rc/mapping.py`) and sent to the firmware at
  once, and the sticks are sent again every 4 ms (250 Hz, as ELRS) between movements. They never
  pass through the page, whose thread is busy drawing (§3.1). The page still gets the axes and
  the mapped channels at 60 Hz, for the HUD, the Controls tab, the calibration wizard and the
  truck (its physics is in the page).
- `input/radio.ts` picks the controller: the source that sees the most axes, the server's on a
  tie, a touch screen's sticks or the keyboard when there is none, the keyboard when the player
  chose it. It tells the server whether its
  radio flies (`radio_source`). Otherwise the page flies: it samples `navigator.getGamepads()` or
  the keys at 250 Hz with a timer and sends `rc`. Channels are -1..1, "natural" for every
  firmware (stick forward +1); each firmware's conventions (ArduPilot's pitch) are applied on the
  way out, in the page (`flight.ts`) and the server (`rc/mapping.py`) alike, and both pass one
  set of test cases (`tests/fixtures/mapping.json`).
- A hand-launched wing is thrown when its launch channel flicks high, whoever flies it: the
  session watches the sticks it forwards.
- A profile (`content/controllers/*.toml`) matches a gamepad by its USB vendor and product id,
  and maps axes and buttons to AETR plus AUX1–AUX8 with inversions. We ship LiteRadio 2 (from
  today's radio.py), TX15 and T8L. EdgeTX radios in USB joystick mode order the axes by their
  mixer, so their profiles assume EdgeTX's default joystick setup and say so.
- The calibration wizard records centre and endpoints and which way each stick and switch moves
  ("move the throttle to full", as driveradio.js does). It saves to the server's prefs, keyed by
  the gamepad's id, so the result survives a browser reset.
- The keyboard and an Xbox-style gamepad are fallbacks, mainly for the truck. Their mode
  switch is made up (M, X), and so is a touch screen's: it starts at the first mode the vehicle
  arms in (`startMode`: Betaflight will not arm in POSHOLD, its switch's low end). The truck
  takes a radio's channels only from a radio; the keyboard and a gamepad drive it by its own
  keys and triggers (`vehicles/truck/controls.ts`).
- **A touch screen with no controller flies by two sticks on the glass** (`input/touch.ts`
  their state, `ui/sticks.ts` the thumbs and the drawing), a mode 2 radio: the left stick
  throttle and yaw, the right roll and pitch, and the two switches as buttons, Arm and the
  flight mode. Each stick is the whole of its side of the bottom of the screen to a thumb, and
  moves by the thumb's travel from where it came down, so nothing jumps to the thumb. Let go,
  the sprung axes centre; the throttle stays, as a radio's. A reset by a tap lets the sticks go
  first (disarmed, the throttle down), or the aircraft would leap off the pad. The truck reads
  the sticks itself (`flight.ts`): the left one, sprung, forward and back, the right one
  steers, and a Brake button held is the handbrake. A radio or a gamepad plugged into the phone
  takes over, and the sticks go.
- Everything else (hotkeys, Shoot Down's controls) is an action in a table
  (`input/actions.ts`, §3.2): `ui/keys.ts` (the menu, flight, on foot, the dynamics overlay) and
  `shoot/controls.ts`. Codes are KeyboardEvent codes, `Mouse0`/`Mouse2` (only while the mouse is
  locked to the map, Shoot Down) and `Pad0`–`Pad16` (a standard gamepad's buttons). The Inputs
  tab and the HUD's key strip are drawn from the tables.
- Arm and mode switches go on AUX channels into the firmware, so you arm the way you would for
  real. The vehicle spec says which AUX does what for each firmware.

## 13. Wind

`physics/core/weather.py` is today's MIL-F-8785C model: log shear with a 6 m reference, Dryden
turbulence including the rotational terms, and the discrete 1−cos gust. It is shared by the wing
and the quad. The Wind tab and the location's defaults set the mean speed and direction, the
turbulence level (none/light/moderate/severe, giving σ from MIL-F-8785C) and gusts (random
1−cos gusts on or off, plus "gust now"). Changes reach the physics live over the control socket.
The truck ignores wind. A wind sock and streamers in the world show it, and the top bar shows
speed and direction.

## 14. Quality

- `uv run ruff check src tests tools`, `uv run basedpyright src tests tools`, `uv run pytest -q`.
- `cd web && bunx tsc --noEmit && bunx biome check && bun test`.
- One `scripts/check.sh` runs all of it. CLAUDE.md is updated to say so.
- Physics tests: wingsim's tests against its sources (moved as they are), our quad harness
  replaying SkyFlow-Dynamics' golden step vectors, and the bridges against recorded packets.
- Integration: `duels doctor` starts each firmware, spawns each vehicle headless, arms and
  reports. A Playwright smoke test (headless Chrome on the 4090 with `--use-angle=gl`, port 8600)
  runs menu → Free Fly → each vehicle → each view.
- The page's frame time: `scripts/check.sh --perf` (`scripts/perf.py`, §3.2) plays Shoot Down
  and the truck on the GPU at the user's window size, on the free map and on Google's tiles,
  and holds frames and systems to budget. Google's run serves on 8600 and costs money, so it is
  not in the ordinary checks; `perf.py --map free` alone is free and runs beside the game.
- Every flight is recorded to `logs/flights/` (today's FlightRecorder, generalised).

## 15. Decided in review (2026-10-29)

1. "Mission" is kept for ArduPilot missions. (Scenarios, a mode at a place, were dropped on
   2026-11-01: a game is a mode and a place, the vehicle picked in flight or in the lobby.
   Since 2026-11-05 the menu's first column calls the mode a "scenario"; the code says mode.)
2. Synetic-Labs/Analog-Vision has a `package.json` (commit d5548dc), so it is a bun git
   dependency.
3. racer5in is close enough to our 5" for now (a row for our own build is on the roadmap).
4. The quad runs SkyFlow-Dynamics' JAX backend as published, on the CPU, not generated code.
5. The server is FastAPI with pydantic, not aiohttp.

## 16. Migration from `main`

The branch starts from an empty `src/`. Each module is ported by moving it, adapting it and
re-testing it, so git history follows it where it can.

| Today (`swarm`) | v2 (`drone_duels`) |
|---|---|
| `wingsim/*` | `physics/wing/*`, `physics/core/weather.py` |
| `wingsim/inav.py`, `wingrun.py` | `firmware/inav.py`, `bridges/xplane.py`, `session/` |
| `sim.py` | `firmware/ardupilot.py` (with `--model JSON`) |
| `physics.py` (FlightGear FDM feed) | replaced: we own the physics, so the pose comes from it |
| `osd.py` | `video/osd.py` |
| `radio.py` (RcLink, failsafe) | `rc/` (input now from the page) |
| `link.py`, `state.py`, `mission.py` | `telemetry/` |
| `geo.py`, `elevation.py` | `world/` |
| `view.py` | `server/` (FastAPI, WebSocket) |
| `fpvmodel.py`, `truckmodel.py`, `gltf.py` | `tools/models/` |
| `settings.py`, `mission.toml` | `content/` + `content/` loader + prefs |
| web: basemap, tilecolliders, car, drivesession | `world/`, `vehicles/truck/` |
| web: duel, duelmatch, duelpilot, dueldriver | `modes/duel/`, `net/peer.ts` |
| web: osd, instruments, sensor, plans, symbols, fleet (VehicleView) | `video/`, `views/` |
| web: wing.js, wingdynamics.js | `vehicles/wing/`, `dynamics/` |
| web: driveradio.js | `input/` (profiles, calibration) |
| web: smoke, sounds | `world/fx`, `audio/` |
| **dropped** (back with the swarm, see ROADMAP): launch, follow, formation, fence upload, panel, paneldemo, termpanel, console, rehearse, watcher, telemetry publisher, plot, director.js | |
| **dropped for good:** wingflight.js (Nanawing arcade port) | |

## 17. Build phases

Each phase ends green on `scripts/check.sh` and is committed.

0. **Scaffold.** Clear the tree; add pyproject (`drone-duels`/`duels`), the web toolchain, the
   content schemas and loader, `ports.py`, the FastAPI server, `/ws` and the protocol's TS
   generation, the app shell with the palette and main menu, and `scripts/check.sh`.
1. **Free Fly with the Chupito.** wingsim, INAV and the X-Plane bridge behind Session; the pose
   and OSD streams; FPV (analog and digital) and chase; the gamepad with its profiles and
   calibration; live wind; the dynamics overlay.
2. **The quad.** racer5in on the JAX backend with its harness and golden replays; the ArduPilot
   JSON bridge; the router and QGC; Map/COP with plans.
3. **Truck and switching.** The truck behind the Vehicle interface; switching and reset; the
   spotter, orbit, minimap and dials; the pause menu complete.
4. **Duel.** The peer link, lobby and turns, the pilot choosing the quad or the Chupito.
5. **Race.** Course files, gate rendering, crossing detection, live ghosts, timing and results.
6. **Polish.** Audio, the doctor, the Playwright smoke test, README, CLAUDE.md.

## 18. The web version

The same game as a static page: no server, no install. `uv run duels web` builds it into
`web/dist-web/` and serves it (from a folder, as GitHub Pages serves a repository's);
`.github/workflows/pages.yml` publishes it to GitHub Pages on every push to `main`. It flies **Betaflight's 5" quad** and
drives the truck, in **Free Fly, Race and Duel**. Not in it: ArduPilot and INAV (their SITLs
are not built for the browser), so the ArduCopter quad and the Chupito; Shoot Down (its target
flies itself, over MAVLink or on the server's switches, and takes hits); the dynamics overlay;
QGroundControl; the radio read through Linux.

```
 Chrome, one page
   main thread: the page, as it is (§3, §9-§12)
        │  net/server.ts `link`: the server's messages (§8), posted, not sent
   worker "simulator" (web/src/sim/)
        session.ts    what drone_duels.session does: start, switch, reset, status, RC watchdog
        vehicle.ts    one quad: a millisecond of the model, then a millisecond of Betaflight
        quad.ts       physics/quad/model.py and esc.py: ESCs, pack, ground contact, crash
        weather.ts    physics/core/weather.py: shear, Dryden turbulence, gusts
        msp.ts        the ground station's MSP (UART2) and the air unit's DisplayPort (UART3)
        module.ts ──► sim.wasm: Betaflight + the plant, one instance a quad
```

**One seam.** The page talks to its server through `link` (messages by channel, poses) and
`api` (content, config, prefs). The web build (`DUELS_WEB`, `net/target.ts`) swaps both: `link`
is a `WorkerLink` whose worker takes and sends the same protocol messages, and `api` reads files
beside the page (`data/`, written by `drone_duels.webdata`: the content it can fly, each quad's
CLI settings and airframe, the OSD font) and keeps prefs in the browser. Nothing above that
seam knows which game it is in; the web-only differences in the UI are a handful of `WEB`
checks (no Shoot Down, no Exit, the player's own map key). `sim/` imports nothing of the
page's (`web/test/fence.test.ts`), so the worker's bundle has no Cesium and no DOM.

**Betaflight** is cudaflight's `SITL_LOCKSTEP` target (Synetic-Labs/cudaflight, on Betaflight
2026.6.1; the game's native SITL is 2026.6.2), compiled with Emscripten. That target is
Betaflight on a virtual clock that advances only inside a step call, single threaded, with no
sockets: sensors and RC go in and motors come out by direct call, which is what a browser can
run. `scripts/cudaflight-web.patch` adds what the page needs (wasm-ld takes no linker script, so
the parameter groups' registry is found by section name; the OSD over MSP DisplayPort, SD and
HD, as the native build's patch does) and `web/wasm/bfweb.c` is its harness: the boot, the
sensors set by stock SITL's own scaling and conventions, an eeprom in memory, and UARTs as
rings in memory. So the firmware is read the way a real one is: telemetry over MSP on UART2
and the OSD over MSP DisplayPort on UART3 (§2.2 holds: the dials and the map never see the
physics). The LLVM instancer cudaflight uses for fleets is not needed: another quad is another
instance of the module.

A setting takes effect at boot, as on a board. So a start boots one instance on defaults, types
the vehicle's CLI files in through Betaflight's own CLI parser (a line it rejects fails the
start) and keeps the eeprom image; the instance that flies boots on it. Its first seconds (the
gyro's calibration, the arming grace) are stepped through at once rather than waited for.
Measured (2026-11-20, bun on this machine): 70 ms from the start message to ready to arm, and
18.5 s of flight stepped in 280 ms, 65 times real time.

**The plant** is SkyFlow-Dynamics' racer5in, the same numbers the server flies, from a
generated C backend (`skyflow_dynamics/backends/c.py`: SymPy's C printer over the same
expressions the JAX backend is lambdified from; its tests replay the golden vectors and agree
with the JAX backend to 1e-12 on a step). `web/wasm/sfd.c` is that output, checked in;
`tests/test_web.py` compiles it and holds it to the JAX backend at the pin the server runs
(1e-11 on a step, 1e-8 over a thousand), and `duels gen-web` rewrites it once the pinned
skyflow-dynamics has the C backend. The model around the plant is `sim/quad.ts`, a port of
`physics/quad/model.py` and `esc.py` held to them by a flight: `webdata.reference_flight` flies
Python's model through a script of motor outputs (the pad, up, a push on each axis, down onto
its legs, stopped) and `web/test/sim-quad.test.ts` flies the page's through the same, within a
micrometre after 7.5 s.

**Time.** The native Betaflight SITL keeps its own clock and follows the physics' pacing (§3).
Here they are in lockstep: each frame is one millisecond of the model, then one of the
firmware, in one module. The worker's timer runs every 4 ms and steps the quad up to now; the
sticks' message steps it at once, so they reach the firmware within a frame. A stall longer
than 100 ms (the machine asleep) is not caught up. Measured in Chrome (2026-11-20, headless on
the RTX 4090, 1600×900, the free map): 60 fps, the pose carried on 6 ms at the median, the same
as from the server.

**The radio** is the page's: Chrome's Gamepad API, a gamepad, the keyboard, or on a phone the
sticks on the glass (§11, §12), sent to the worker as `rc`. No sticks for 0.2 s and nothing more is fed to Betaflight's receiver, so its
own failsafe flies, as with the server. Chrome's Gamepad API zeroes the LiteRadio 2's CH5 and
CH8 (its arm switch), which the server works round by reading it through Linux; the web
version cannot, until the radio is read over WebHID (ROADMAP.md).

**The map** is the free globe (Esri). The page is public, so it ships no key; a player may
type their own Google Maps key into Options → Graphics, kept in that browser only, for Google's
3D tiles in flight.

**Licence.** `sim.wasm` contains Betaflight, so it is GPL-3.0, as is `bfweb.c`; publishing the
page distributes it, and the source is cudaflight at the pinned commit with this repository's
patch and harness.

**Tests.** `web/test/sim-session.test.ts` flies the whole thing under bun as
`tests/test_sitl_betaflight.py` flies the native one: ready to arm, armed on its switch, up in
ANGLE and level, the sticks' senses, POSHOLD holding in the place's wind, the radio lost and
Betaflight's failsafe landing and disarming it, a reset, the other video system's OSD grid.
They need the simulator built and are skipped without it.

