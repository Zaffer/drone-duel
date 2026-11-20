# Drone Duels: roadmap

After v1 (DESIGN.md §17). Roughly in priority order within each group.

## Vehicles and firmware

- **Betaflight's 5" as a firmware choice on the 5" quad's card**, not a second card: it is a
  vehicle of its own today (`quad5-betaflight`).
- A battery for Betaflight SITL: its simulator protocol carries none, so it reads no pack.
- **ArduPilot Rover on the truck.** Rapier stays the physics (tile collisions). A lockstep link
  between the page and Rover SITL's JSON backend, for practising rover missions.
- **Our own 5" build** as a second SkyFlow-Dynamics parameter row (750 g, 6S, 2207 1950 KV), if
  racer5in flies too differently.
- More airframes: 7" long range, a cinewhoop, an ArduPlane fixed wing, an ArduPilot VTOL.
- INAV's `inav_xitl` battery datarefs, so the Chupito's pack is simulated, not a fixed 16.8 V.

## Swarm

- **Bring back the drone swarm** from Synetic-Labs/airmetal-swarm, branch `airmetal` (strip the
  AirMetal branding): the leader-follower launcher, ArduPilot FOLLOW, the geofence upload, the
  Logitech switch panel, the terminal panel and `rehearse`. It returns as a mode ("Swarm") with
  AreaXO as its place, using the vehicle slots (DESIGN.md §3).
- Swarm strike in Duel: the pilot flies the swarm against the truck, as on `main`.

## Visualisation

- Dynamics overlay for the quad: per-rotor thrust and drag, inflow, frame drag, ground effect,
  from the plant's terms.
- Dynamics overlay for the truck: tyre forces and suspension.
- Digital video link effects: breakup, bitrate drop and latency for DJI, Walksnail and HDZero.
- DVR: record the FPV feed. Replays from flight records, with free cameras.

## Play

- In-game race course editor (place, drag and rotate gates on the map; save to TOML).
- Ghost of your best lap in a solo time trial.
- More than two players; spectators.
- More modes: tag, capture the flag, payload drop.
- More locations.

## ArduPilot practice

- In-game mission planner: draw waypoints and upload them over MAVLink.
- Failure scripts: failures to practise (GPS loss, failsafe, low battery, wind shear).
- INAV missions over MSP, drawn like ArduPilot's.

## The web version

- **The radio over WebHID**, read in the simulator's worker (dedicated workers, Chrome 131+):
  the LiteRadio 2's CH5 and CH8, which Chrome's Gamepad API zeroes, so its arm switch works in
  the web version, and no radio's sticks pass through the page's thread.
- A battery for Betaflight in the page: the model has the pack; feed its volts and amps to a
  virtual sensor (the native SITL's simulator protocol carries none either).
- Shoot Down: the target flown by the worker on Betaflight's switches (POSHOLD, as the server
  flies it), and hits (the model's damage, ported).
- The quad's dynamics overlay and the turbulence's rotation, with the server's.
- Upstream the web target to cudaflight (the patch is 70 lines and the harness one file), and
  move this repository's skyflow-dynamics pin to a commit with the C backend, so
  `duels gen-web` owns `web/wasm/sfd.c`.
- INAV for the browser (its SITL has no lockstep target yet), for the Chupito.

## Controllers and platforms

- Test and tune more radio profiles; a shared profile format others can contribute.
- Windows (WSL2) and macOS.
- Gamepads read in the server too (SDL2), as the radio already is (`rc/pilot.py`), so no
  controller's sticks pass through the page's busy thread (DESIGN.md §3.1).

## Hardware

- Live mode: the Map/COP view against real aircraft through `router/field.conf`. The design
  keeps the ground-station side on telemetry only, so it needs no change (DESIGN.md §2.2).

## Found while building v1

- The quad: ground effect (SkyFlow-Dynamics has it only as a candidate term), and the
  turbulence's rotation (`Weather.rates`), which the wing takes and the quad does not yet.
- The quad's STABILIZE and ACRO tunes: only LOITER has been flown in the tests.
- INAV's own OSD font for the analog Chupito (its analog OSD is the Betaflight-compatible mode,
  in Betaflight's glyphs, so a few INAV symbols show as `?`).
- INAV's climb rate reads low: SITL's fake GPS gives it no vertical speed.
- Plans: send each item's params (`PlanItem`), so fence circles have their radius, and the
  mission's current item, so the Map view can tell flown, active and next legs apart.
- Radio profiles for the TX15 and T8L are EdgeTX's and ELRS's defaults, untested on the real
  radios; the LiteRadio 2's was checked in Chrome and through Linux. Chrome lists its CH5 and CH8
  axes but never moves them, so the page flies it on the server's reading (`rc/evdev.py`).
- A ghost's model is drawn without the silhouette for now (it broke Cesium's frustums when
  combined with a model loaded before its first pose).
