# Drone Duels: design

Status: built on branch `v2` (2026-10-29). Where the build found something the draft got wrong, the
text below says what was done instead.

Drone Duels is a flight simulator game and practice sim. Every aircraft flies on its real flight
controller firmware in SITL (ArduPilot or INAV, later Betaflight), on the best physics we can
model, over real places on Earth drawn from Google Photorealistic 3D Tiles. You can fly around
on your own, fight a 1v1 duel, or race a friend through gates. You can switch vehicle at any
time, see the firmware's real OSD through an analog or digital video link, and change the wind
as you fly. It is also where we practise new airframes and ArduPilot missions before flying them
for real.

## 1. Goals and non-goals

**Goals for v1**

- The real firmware flies every aircraft: ArduCopter 4.7 SITL for the 5" quad and INAV 9.1 SITL
  for the TBS Chupito. Nothing we write stands in for the flight controller.
- Physics we own and can defend: `wingsim` (strips over a vortex lattice, MIL-F-8785C wind) for
  the Chupito, and SkyFlow-Dynamics' `racer5in` plant for the quad. One wind model drives both.
- Modes: **Free Fly**, **Duel** (a drone or the Chupito against the truck) and **Race** (live
  head to head through gates). Modes are plugins, so more can follow.
- Scenarios: a mode at a saved place, picked from a menu. For example "Free Fly: Toronto from
  Billy Bishop" or "Race: AreaXO course 1".
- Vehicle select: the 5" quad, the Chupito or the truck. Switching respawns you at the
  scenario's start.
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
3. **Content is data, not code.** Locations, scenarios, vehicles, courses and controller
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
   │  truck physics, tile colliders  (Rapier, in the page)                   │
   │  duel / race peer link          (Trystero over Nostr ⇄ other player)   │
   └─────────────────────────────────────────────────────────────────────────┘
                                   │
 duels server (Python, FastAPI on uvicorn, pydantic)
   ├─ Session          mode-agnostic: spawn / switch / reset / destroy a vehicle at a scenario
   ├─ ContentStore     content/*.toml → typed specs
   ├─ Prefs            ~/.config/drone-duels/prefs.json (controls, video, graphics, audio)
   ├─ RadioStream      the radio read through Linux (evdev), for when Chrome can't see it all
   ├─ RcSink           sticks from the page → the firmware's RC input, with failsafe on silence
   ├─ TelemetryClient  MAVLink via mavlink-router (sysid 254) │ MSP for INAV
   ├─ OsdClient        MSP DisplayPort from the firmware → grid screens
   └─ per vehicle slot:
        Firmware process  ⇄  bridge protocol  ⇄  Physics process
        ArduCopter SITL   ⇄  ArduPilot JSON   ⇄  quad (SkyFlow-Dynamics plant)
        INAV SITL         ⇄  X-Plane UDP      ⇄  wing (wingsim)
        (truck: no firmware; physics runs in the page)

 mavlink-router: SITL serial0 → UDP 14550 QGroundControl (255) · UDP 14570 duels (254)
```

**Processes.** Each physics process is a child of the server, one per vehicle slot, so a slow
render or a GC pause in the server never stalls the lockstep with the firmware. It talks to the
server over a local UDP control socket: wind, ground height and resets in; true pose and
dynamics out. v1 uses slot 0 only. Slots exist so the swarm (and two local players for testing)
can come back without redesign.

**Ports** live in one module, `drone_duels.ports`, as a block per slot: firmware MAVLink/MSP, CLI,
OSD, RC in, physics bridge and physics control. Nothing else hard-codes a port. Today ArduPilot's
SITL serial0 and INAV's CLI both default to 5760; the port map ends that clash.

**Time.** ArduPilot's JSON backend and INAV's X-Plane link both run in lockstep: the physics
steps when the firmware sends its outputs, so the sim never drifts from the firmware's clock.
The physics streams the true pose to the server at the firmware's loop rate. The server forwards
the newest pose per browser frame and drops the rest, so the camera is never more than a frame
late.

## 4. Repository layout

```
drone-duels/
  pyproject.toml            name drone-duels · package drone_duels · script `duels`
  DESIGN.md  ROADMAP.md  CLAUDE.md  README.md
  content/
    locations/              billy-bishop.toml, area-xo.toml          (place, ground, start pads)
    scenarios/              freefly-toronto.toml, duel-areaxo.toml…  (mode + location + rules)
    vehicles/               quad5.toml, chupito.toml, truck.toml
    courses/                areaxo-1.toml                            (race gates)
    controllers/            literadio2.toml, tx15.toml, t8l.toml     (default channel maps)
  firmware/
    ardupilot/              copter-quad5.parm, sitl.parm, json-frame notes
    inav/                   chupito.cli                              (INAV CLI settings)
  router/sitl.conf
  scripts/setup.sh          ArduPilot, INAV, mavlink-router, bun, web deps, Chupito GLB
  tools/                    gen_protocol_ts.py (pydantic → web/src/net/protocol.ts),
                            models/ (fpvmodel, truckmodel, gltf: GLB asset builders)
  src/drone_duels/
    cli.py                  `duels` (serve, doctor, physics, record)
    ports.py
    content/                pydantic schemas + loader for content/*.toml, prefs
    server/                 FastAPI app, /ws protocol (pydantic models), REST, static files
    session/                Session, slots, vehicle lifecycle, mode hooks on the server side
    firmware/               base.py (Firmware protocol), ardupilot.py, inav.py
    bridges/                ardupilot_json.py, xplane.py
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
    record/                 flight recorder (logs/flights/)
  web/
    package.json  tsconfig.json  biome.json
    src/
      main.ts
      theme/                tokens.css (palette, §11), base.css
      state/                alien-signals stores (§9)
      streams/              Observable sources: ws channels, gamepad, frames, timers
      net/                  server ws client, protocol types (generated, §8), peer link
      ui/                   menu, scenario/vehicle pickers, pause menu, top bar, toasts
      world/                Cesium viewer, basemap, tile colliders, ground sampling
      vehicles/             quad, wing, truck: models, animation; truck physics (Rapier)
      views/                fpv, chase, spotter, map, orbit, minimap, dials
      video/                analog (analog-vision), digital, osd renderer
      modes/                freefly/, duel/, race/
      dynamics/             wing dynamics overlay
      input/                gamepad, profiles, calibration, keyboard
      audio/
    test/                   bun test
  tests/                    pytest (physics goldens, bridges, content, session, protocol)
```

`external/` (git-ignored, from `scripts/setup.sh`) keeps ArduPilot, INAV and mavlink-router.
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

# content/scenarios/freefly-toronto.toml
name = "Free Fly: Toronto"
mode = "freefly"
location = "billy-bishop"
vehicles = ["quad5", "chupito", "truck"]  # what the picker offers
default_vehicle = "quad5"

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

"Scenario" is the word in the code and in the menu. "Mission" means only an ArduPilot mission.

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

### 6.3 Truck

Today's Rapier ray-cast vehicle (car.js and drivesession.js) on colliders built from the tiles,
in the page, with no firmware. It gets the same Vehicle interface in the page as the aircraft
(pose stream, controls, reset), so views and modes do not care that its physics is local.

### 6.4 Vehicle lifecycle

`Session.spawn(scenario, vehicle)`:

1. stop the slot's current firmware and physics, if any;
2. start the physics at the location's start pad;
3. start the firmware with its params and wait until it is ready (ArduPilot: EKF and prearm
   checks; INAV: no arming blockers on two polls in a row, a known quirk);
4. report `ready`.

Switching vehicle is a spawn at the scenario's start. Reset re-spawns the same vehicle. Destroy
(a crash in Duel or Race) cuts the physics, so the firmware's link goes quiet as a destroyed
aircraft's would.

## 7. Modes

A mode is a pair of plugins with one lifecycle, `setup → running → finished`:

- **In the page** (`web/src/modes/<id>/`), registered in the mode registry: menu options, HUD
  widgets, rules, scoring, peer messages.
- **On the server** (`session/modes/<id>.py`): optional hooks only (allowed vehicles, what a
  crash does). Most rules live in the page, beside the world they judge.

| Mode | Players | Rules |
|---|---|---|
| Free Fly | 1 | Any vehicle the scenario lists, switch at will, no score. Practice missions here with QGC. |
| Duel | 2 (P2P) | Turns. The pilot (quad or Chupito) strikes the truck; the driver (truck) evades. A strike scores; a crash spends the aircraft. Swap each turn. Today's duelmatch rules carry over. |
| Race | 2 (P2P) | Live, side by side. Each player flies their own sim and sees the other as a ghost (no collisions). Countdown start from the host. A gate counts when the segment between two true poses crosses its rectangle in the right direction. Lap and total times; first to finish wins. |

The peer link stays Trystero over Nostr relays (WebRTC, no server), one room per match. Peers
exchange poses at 20 Hz for the ghost or truck, events (strike, gate, crash, finish) with a
sequence number, and a clock offset estimated at the start, so the countdown and timers agree.

## 8. Server ⇄ page protocol

One WebSocket at `/ws` carries JSON envelopes `{ch, t, data}`, plus binary frames for the
high-rate pose channel.

| Direction | Channel | Rate | Content |
|---|---|---|---|
| page → server | `session` | on demand | start, switch, reset, destroy |
| page → server | `rc` | 250 Hz | 16 channels in µs, sequence number |
| page → server | `ground` | 20 Hz | ground height under the aircraft |
| page → server | `wind` | on change | speed, direction, turbulence, gust trigger |
| page → server | `video` | on change | analog or digital (sets the OSD grid, §10) |
| page → server | `radio_source` | on open | stream the radio as the server reads it |
| server → page | `status` | on change | slot, vehicle, firmware state, arming blockers |
| server → page | `pose` | each frame | true position, attitude, velocity, rotor/servo state (binary) |
| server → page | `osd` | per draw | the firmware's DisplayPort screen |
| server → page | `telemetry` | 10 Hz | what a GCS sees: mode, armed, GPS, battery, attitude, alt |
| server → page | `plan` | on change | mission, fence, rally (ArduPilot) |
| server → page | `dynamics` | 30 Hz | wing internals (when the overlay is on) |
| server → page | `physics` | on change | what the physics says it is doing: ground, hand, flying, crashed |
| server → page | `radio` | 250 Hz | the radio's axes, read through Linux (while asked) |

The protocol is defined once, in `server/protocol.py`, as pydantic models: a discriminated union
on `ch`, validated on the way in. `tools/gen_protocol_ts.py` writes their JSON Schema and
generates `web/src/net/protocol.ts` from it, and a test fails if the generated file is out of
date. The REST endpoints (`/api/scenarios`, `/api/vehicles`, `/api/prefs`, …) are FastAPI routes
on the same models, with OpenAPI docs at `/docs`.

Why FastAPI rather than aiohttp: the pose stream is a few hundred small messages a second on one
socket, which either serves easily. The deciding win is pydantic at the edges. One set of models
validates content files, prefs, REST and WebSocket messages, and generates the TypeScript types,
so the server and the page cannot disagree silently.

RC failsafe: if no `rc` message arrives for 200 ms (page hidden, tab crashed), the session tells
the firmware its link is lost, and the firmware's own failsafe runs, as on a real link. ArduPilot
SITL would hold the last pulses forever, so the adapter sets `SIM_RC_FAIL`; INAV's X-Plane link
stops reporting a joystick, and its receiver times out. Both come back with the next sticks.

The WebSocket takes only the game's own page (a local Origin), so another site open in the same
browser cannot fly the sim.

## 9. Web app state

- **alien-signals** holds every value the UI reads now: the screen (menu, flying, paused),
  scenario, vehicle, view, video system, prefs, session status, match state, and the latest
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
- Switching the video system changes the firmware's OSD resolution. ArduPilot needs a reboot for
  that, so the switch respawns the vehicle at the start, as a vehicle switch does, after a
  confirmation.
- The OSD fonts are the firmware's own (ArduPilot's OSD_FONT and INAV's), rendered from their
  .mcm files as `swarm.osd` does today.

## 11. UI

**Flow.** Main menu: Free Fly · Duel · Race · Options. Then a scenario list for that mode, with
location, a thumbnail and its vehicles. Then the vehicle (Duel: your role, then the vehicle; Race:
the course, then the vehicle). Then Fly. Duel and Race also have a lobby: a room code, peer
status and ready.

**In flight.** A thin top bar shows mode · scenario · vehicle · view · video · wind · timer/score.
It fades after 3 s in FPV and comes back on mouse movement or a menu key. In the corners:

- bottom right: the minimap PiP (M toggles it);
- bottom left: the dials (today's instruments.js: throttle, altitude, climb, speed, compass,
  attitude, with the stick meters).

Both hide in FPV, where the OSD is the instrument panel. The quick keys: V cycles views, 1–6 jump
to a view, R resets, B switches vehicle, D shows the dynamics overlay (Chupito), Esc pauses.

**Pause/Options (Esc)**, in tabs:

| Tab | Holds |
|---|---|
| Vehicle | switch vehicle (respawns at start), reset |
| View | default view, chase distance, spotter position, FOV for the non-FPV views |
| Video | analog/digital, VTX power, camera tilt override, OSD on/off |
| Wind | speed, direction, turbulence level, gusts on/off, "gust now" |
| Controls | controller profile, calibration wizard, channel map, expo/rates for the truck, keyboard |
| Inputs | every hotkey, and each vehicle's sticks and switches on a radio, gamepad and keyboard (`ui/inputs.ts`, which the HUD's key strip also reads) |
| Graphics | map source and keys (Google, Cesium ion, Esri), quality, frame cap |
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

- `input/radio.ts` samples `navigator.getGamepads()` at 250 Hz with a timer (the page is in
  front while flying) and emits an Observable of channels, -1..1, "natural" for every firmware
  (stick forward +1); `flight.ts` applies each firmware's conventions (ArduPilot's pitch) on the way
  out.
- Chrome on Linux may not expose a radio until a stick moves, or all its axes. So the server also
  reads it through Linux itself (`rc/evdev.py`) and streams its axes; the page takes whichever
  source sees the most axes, through the same profiles and calibration.
- A profile (`content/controllers/*.toml`) matches a gamepad by its USB vendor and product id,
  and maps axes and buttons to AETR plus AUX1–AUX8 with inversions. We ship LiteRadio 2 (from
  today's radio.py), TX15 and T8L. EdgeTX radios in USB joystick mode order the axes by their
  mixer, so their profiles assume EdgeTX's default joystick setup and say so.
- The calibration wizard records centre and endpoints and which way each stick and switch moves
  ("move the throttle to full", as driveradio.js does). It saves to the server's prefs, keyed by
  the gamepad's id, so the result survives a browser reset.
- The keyboard and an Xbox-style gamepad are fallbacks, mainly for the truck.
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
- Every flight is recorded to `logs/flights/` (today's FlightRecorder, generalised).

## 15. Decided in review (2026-10-29)

1. The UI says "Scenarios"; "Mission" is kept for ArduPilot missions.
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
